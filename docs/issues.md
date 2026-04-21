# Known Issues & Future Work

## Multi-device pairing selection
When multiple BikeTracker devices are in BLE range during pairing, the app picks the first one found with no way for the user to choose.

**Current behavior:** First device named "BikeTracker" or "BikeTracker (Setup)" is auto-paired.

**Expected:** Show a list of discovered devices with identifier (e.g. last 4 chars of MAC address) and signal strength (RSSI) so the user can pick the right one.

**Priority:** Low — not an issue for single-device users. Needed for group/fleet scenarios or when replacing a tracker.
