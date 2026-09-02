#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/scan_mode"
BASE="https://raw.githubusercontent.com/Livox-SDK/livox_laser_simulation/main/scan_mode"
mkdir -p "$DEST"
for f in mid360.csv avia.csv horizon.csv mid40.csv mid70.csv tele.csv HAP.csv; do
  echo "fetch $f"
  curl -fsSL "$BASE/$f" -o "$DEST/$f"
done
echo "wrote $DEST"
