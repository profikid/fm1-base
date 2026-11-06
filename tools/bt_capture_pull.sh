#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only OR MPL-2.0
# Read the SBC frames the FM-1 sent, and say what a decoder makes of them.
#
# The FM-1's Bluetooth audio stack keeps the first frames of the stream (bt/br/bt_acl.c ba_capture: four windows of a
# second, eight frames each) and reads them out over USB-MIDI (tools/fm1_remote.py bta capture / bta capture-frame).
# This script pulls those frames off the device and decodes them with AOSP's SBC decoder (the one a speaker uses),
# so we can see whether both channels come back.
#
#   sh tools/bt_capture_pull.sh              the frames of window 0
#   sh tools/bt_capture_pull.sh 1            window 1
#   sh tools/bt_capture_pull.sh 0 8          window 0, eight frames
#
# The FM-1 must be connected over USB-MIDI and streaming over Bluetooth (the capture is armed when the encoder
# starts). The dump lands in /tmp/bt_capture.hex; the verdict is printed here.
set -e
cd "$(dirname "$0")/.."

WIN="${1:-0}"
FRAMES="${2:-8}"
DUMP="${DUMP:-/tmp/bt_capture.hex}"
REMOTE="python3 tools/fm1_remote.py"

echo "== the capture's own line: the mode, the bitpool, the frames per window =="
$REMOTE bta capture

: > "$DUMP"
i=0
while [ "$i" -lt "$FRAMES" ]; do
    line="$($REMOTE bta capture-frame "$i" "$WIN" | sed -n 's/^bta: //p' | tr -d '\r' | head -1)"
    case "$line" in
        *[!0-9A-Fa-f]*|"") echo "frame $i of window $WIN: nothing captured"; i=$((i + 1)) ;;
        *) echo "$line" >> "$DUMP"; echo "frame $i of window $WIN: ${#line} hex digits"; i=$((i + 1)) ;;
    esac
done

echo
echo "== what a decoder makes of those frames =="
python3 tools/bt_capture_decode.py "$DUMP"
