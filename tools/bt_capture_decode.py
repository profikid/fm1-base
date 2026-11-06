#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only OR MPL-2.0
"""Decode the FM-1's captured SBC frames, the way a speaker's decoder reads them.

The FM-1's Bluetooth audio stack can capture the SBC frames it sends (bt/br/bt_acl.c ba_capture) and read them
out over USB-MIDI (tools/fm1_remote.py bta capture / bta capture-frame). Those frames are the bytes a speaker's
decoder gets: with this script one can check, off the hardware, what a decoder makes of them, and whether both
channels come back.

  python3 tools/bt_capture_decode.py dump.hex

The decoder is AOSP's SBC decoder (tests/bt/ref/sbc_dec, Apache-2.0, the one bt_sbc_test.c uses), built on the
fly into tools/bt_capture_decode.c. It reads the channel mode out of every frame's own header (A2DP 12.9, byte 1:
bits 4-3), so the mode in the header and the mode in the SET_CONFIGURATION must agree, or the sink's decoder is
wrong for the stream (the user, 2026-11-06: only the right channel of the speaker sounded, on Polly, LOOM-8 and
Optimist alike).

The input is the hex the FM-1 printed: one line per frame, the bytes as hex without a separator. Lines that are
not hex (the mode/bitpool line, for instance) are skipped.
"""
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.normpath(os.path.join(HERE, ".."))
DECODER_SRC = os.path.join(HERE, "bt_capture_decode.c")
DECODER_DIR = os.path.join(BASE, "tests", "bt", "ref", "sbc_dec")


def build_decoder():
    """The decoder as a program: AOSP's SBC decoder (Apache-2.0) with our own main()."""
    out = os.path.join(tempfile.gettempdir(), "bt_capture_decode")
    cmd = ["cc", "-O2", "-w", "-I", DECODER_DIR, "-o", out, DECODER_SRC]
    cmd += [os.path.join(DECODER_DIR, f) for f in sorted(os.listdir(DECODER_DIR)) if f.endswith(".c")]
    cmd += ["-lm"]
    subprocess.run(cmd, check=True)
    return out


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    with open(argv[1]) as f:
        dump = f.read()
    decoder = build_decoder()
    p = subprocess.run([decoder], input=dump, capture_output=True, text=True)
    sys.stdout.write(p.stdout)
    if p.stderr:
        sys.stderr.write(p.stderr)
    return p.returncode


if __name__ == "__main__":
    sys.exit(main(sys.argv))
