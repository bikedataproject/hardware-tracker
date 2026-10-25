#!/usr/bin/env python3
"""
BikeTracker BLE diagnostic CLI.

A test harness for the firmware's BLE interface that does not depend on the phone app.
Speaks the same batch protocol the app uses, so a successful download here proves the
device side end to end.

Install:
    python3 -m venv ~/.venvs/bt && ~/.venvs/bt/bin/pip install bleak cryptography

Usage:
    python diag.py scan        # list BLE devices, flag the tracker
    python diag.py status      # paired state, tracker state, stored point count
    python diag.py pair        # generate a key, store it on device and locally
    python diag.py download    # fetch + decode all points
    python diag.py clear       # wipe stored points (device refuses while TRACKING)
    python diag.py watch       # poll state/count every 2s
"""
import argparse
import asyncio
import os
import secrets
import struct
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

from bleak import BleakClient, BleakScanner
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_BATCH   = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHAR_COUNT   = "8ca20d91-5a20-4d5b-a7f1-7c4e3a1c2b3d"
CHAR_CLEAR   = "9da30e02-6b31-4e6c-b802-8d5f4b2d3c4e"
CHAR_PAIR    = "a1b2c3d4-1234-5678-9abc-def012345678"
CHAR_STATUS  = "b2c3d4e5-2345-6789-abcd-ef0123456789"
CHAR_STATE   = "c3d4e5f6-3456-789a-bcde-f01234567890"
CHAR_SEEK    = "d4e5f607-4567-89ab-cdef-012345678901"

NAMES = ("BikeTracker", "BikeTracker (Setup)")
KEY_FILE = Path.home() / ".biketracker.key"

# The firmware stores time as seconds since 2000-01-01, not the Unix epoch.
EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)


async def find_device(timeout=8.0):
    """Return as soon as the tracker is seen rather than waiting out the full timeout."""
    print(f"Scanning for {NAMES[0]} / {NAMES[1]} (up to {timeout:.0f}s)...")
    t0 = time.monotonic()
    dev = await BleakScanner.find_device_by_filter(
        lambda d, adv: d.name in NAMES, timeout=timeout
    )
    if dev is None:
        print("No tracker found. A sleeping tracker does not advertise — tap it to wake.")
        return None
    print(f"Found: {dev.name} @ {dev.address}  ({time.monotonic() - t0:.1f}s)")
    return dev


async def read_str(client, uuid):
    data = await client.read_gatt_char(uuid)
    return data.decode("utf-8", errors="replace").strip("\x00")


def load_key():
    return KEY_FILE.read_bytes() if KEY_FILE.exists() else None


def save_key(key: bytes):
    KEY_FILE.write_bytes(key)
    os.chmod(KEY_FILE, 0o600)


def decrypt(data: bytes, key: bytes) -> bytes:
    """AES-128-ECB with PKCS7 padding, matching the firmware's encryptBlock()."""
    dec = Cipher(algorithms.AES(key), modes.ECB()).decryptor()
    plain = dec.update(data) + dec.finalize()
    pad = plain[-1]
    return plain[:-pad] if 0 < pad <= 16 else plain


def decode_batch(plain: bytes):
    """Decode one batch into [(lat, lng, datetime)].

    Wire format, matching BatchReadCallbacks in the firmware:
        header     startIndex (u16 BE), count (u16 BE)
        reference  lat (f32 LE), lng (f32 LE), epoch (u32 LE)
        deltas     dlat (i16 LE), dlng (i16 LE), dt (u16 LE)   x (count-1)
    Deltas are in microdegrees and accumulate from the reference.
    """
    if len(plain) < 4:
        return []
    start_index, count = struct.unpack_from(">HH", plain, 0)
    pos = 4
    if count <= 0 or pos + 12 > len(plain):
        return []

    lat, lng, epoch = struct.unpack_from("<ffI", plain, pos)
    pos += 12
    out = [(lat, lng, EPOCH_2000 + timedelta(seconds=epoch))]

    for _ in range(count - 1):
        if pos + 6 > len(plain):
            break
        dlat, dlng, dt = struct.unpack_from("<hhH", plain, pos)
        pos += 6
        lat += dlat / 1_000_000.0
        lng += dlng / 1_000_000.0
        epoch += dt
        out.append((lat, lng, EPOCH_2000 + timedelta(seconds=epoch)))

    return out


# --- commands ---

async def cmd_scan(_args):
    print("Scanning all BLE devices for 8s...")
    results = await BleakScanner.discover(timeout=8.0, return_adv=True)
    for d, adv in sorted(results.values(), key=lambda da: -(da[1].rssi or -999)):
        marker = "  <-- tracker" if d.name in NAMES else ""
        print(f"  {d.address}  RSSI={adv.rssi:>4}  {d.name or '(no name)'}{marker}")


async def cmd_status(_args):
    dev = await find_device()
    if not dev:
        return
    async with BleakClient(dev) as c:
        print(f"  status: {await read_str(c, CHAR_STATUS)}")
        print(f"  state:  {await read_str(c, CHAR_STATE)}")
        print(f"  count:  {await read_str(c, CHAR_COUNT)}")


async def cmd_pair(_args):
    dev = await find_device()
    if not dev:
        return
    async with BleakClient(dev) as c:
        if await read_str(c, CHAR_STATUS) == "PAIRED":
            print("Already paired. Factory-reset first (hold BOOT 5s).")
            return
        key = secrets.token_bytes(16)
        await c.write_gatt_char(CHAR_PAIR, key, response=True)
        save_key(key)
        print(f"Paired. Key saved to {KEY_FILE}")


async def cmd_download(_args):
    key = load_key()
    if key is None:
        print(f"No key at {KEY_FILE}. Run `pair` first.")
        return
    dev = await find_device()
    if not dev:
        return

    async with BleakClient(dev) as c:
        count_str = await read_str(c, CHAR_COUNT)
        if count_str == "LOCKED":
            print("Device LOCKED — key mismatch?")
            return
        expected = int(count_str)
        print(f"Device reports {expected} points. Downloading...")

        # Start from 0 regardless of where a previous run left the cursor.
        await c.write_gatt_char(CHAR_SEEK, b"0", response=True)

        points, batches, t0 = [], 0, time.monotonic()
        while True:
            raw = await c.read_gatt_char(CHAR_BATCH)
            # DONE/LOCKED are sent as short plaintext, never encrypted.
            if len(raw) <= 6:
                tag = raw.decode("ascii", errors="replace")
                if tag.startswith("DONE"):
                    break
                print(f"Unexpected marker: {tag}")
                return
            batch = decode_batch(decrypt(raw, key))
            if not batch:
                print("Empty/undecodable batch — wrong key?")
                return
            points.extend(batch)
            batches += 1
            if len(points) >= expected:
                break

        elapsed = time.monotonic() - t0

    print(f"\n{len(points)} points in {batches} batches, {elapsed:.1f}s")
    if points:
        for lat, lng, ts in points[:3]:
            print(f"  {ts.isoformat()}  {lat:.6f}, {lng:.6f}")
        if len(points) > 6:
            print("  ...")
        for lat, lng, ts in points[-3:]:
            print(f"  {ts.isoformat()}  {lat:.6f}, {lng:.6f}")

        out = Path.cwd() / "points.csv"
        with open(out, "w") as f:
            f.write("timestamp,lat,lng\n")
            for lat, lng, ts in points:
                f.write(f"{ts.isoformat()},{lat:.6f},{lng:.6f}\n")
        print(f"\n-> {out}")

        if len(points) != expected:
            print(f"WARNING: expected {expected}, decoded {len(points)}")


async def cmd_clear(_args):
    dev = await find_device()
    if not dev:
        return
    async with BleakClient(dev) as c:
        await c.write_gatt_char(CHAR_CLEAR, b"\x01", response=True)
        print(f"Clear sent. count now: {await read_str(c, CHAR_COUNT)}")


async def cmd_watch(_args):
    print("Polling every 2s. Ctrl+C to stop.\n")
    prev = None
    while True:
        try:
            dev = await BleakScanner.find_device_by_filter(
                lambda d, adv: d.name in NAMES, timeout=4.0)
            if not dev:
                print(f"[{time.strftime('%H:%M:%S')}] not advertising (asleep, or BLE down while riding)")
                await asyncio.sleep(2)
                continue
            async with BleakClient(dev, timeout=10.0) as c:
                while True:
                    line = f"{await read_str(c, CHAR_STATE):<10} count={await read_str(c, CHAR_COUNT)}"
                    if line != prev:
                        print(f"[{time.strftime('%H:%M:%S')}] {line}")
                        prev = line
                    await asyncio.sleep(2)
        except KeyboardInterrupt:
            return
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] {type(e).__name__}: {e}")
            await asyncio.sleep(2)


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("scan", "status", "pair", "download", "clear", "watch"):
        sub.add_parser(name)
    args = p.parse_args()
    try:
        asyncio.run(globals()[f"cmd_{args.cmd}"](args))
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
