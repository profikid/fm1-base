/* SPDX-License-Identifier: GPL-3.0-only OR MPL-2.0 */
/* Decode the FM-1's captured SBC frames: what a speaker's decoder makes of them.
 *
 * The FM-1's Bluetooth audio stack can capture the SBC frames it sends (bt/br/bt_acl.c ba_capture) and read them out
 * over USB-MIDI (tools/fm1_remote.py bta capture / bta capture-frame). Those frames are the bytes a speaker's
 * decoder gets. This program decodes them with AOSP's SBC decoder (tests/bt/ref/sbc_dec, Apache-2.0, the one
 * bt_sbc_test.c uses), so one can see whether both channels come back.
 *
 * The decoder reads the channel mode out of every frame's own header (A2DP 12.9, byte 1: bits 4-3), not out of
 * the configuration that was agreed. A frame that says joint stereo is decoded as joint stereo, whatever the sink
 * was configured for. So a frame set that does not agree with the SET_CONFIGURATION leaves the sink's decoder
 * wrong for the stream (the user, 2026-11-06: only the right channel of the speaker sounded, on Polly, LOOM-8 and
 * Optimist alike).
 *
 * The input is the hex the FM-1 printed: one line per frame, the bytes as hex without a separator. Lines that
 * are not hex (the mode/bitpool line, for instance) are skipped.
 *
 * Build and run (tools/bt_capture_decode.sh does this):
 *   cc -O2 -w -I tests/bt/ref/sbc_dec -o /tmp/dec tools/bt_capture_decode.c tests/bt/ref/sbc_dec/*.c -lm
 *   /tmp/dec < dump.hex
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "oi_codec_sbc.h"

#define MAXF 1024u                                  /* a frame is at most 260 B (A2DP 12.9) */

static int hexval(int c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

int main(int argc, char **argv)
{
    static uint32_t dec_data[CODEC_DATA_WORDS(2, SBC_CODEC_FAST_FILTER_BUFFERS)];
    OI_CODEC_SBC_DECODER_CONTEXT dc;
    static uint8_t bytes[MAXF];
    static int16_t pcm[512];
    char line[2400];
    uint32_t frame = 0, refused = 0, silent_l = 0, silent_r = 0;

    (void)argc;
    (void)argv;
    while (fgets(line, (int)sizeof line, stdin)) {
        uint32_t n = 0, i, len = (uint32_t)strlen(line), got;
        const OI_BYTE *p;
        uint32_t left, pb;
        double sl = 0, sr = 0;

        for (i = 0; i + 1u < len && n < MAXF; i += 2) {    /* the hex, two digits a byte */
            int h = hexval(line[i]), l = hexval(line[i + 1u]);
            if (h < 0 || l < 0)
                break;
            bytes[n++] = (uint8_t)((h << 4) | l);
        }
        if (n < 4u)
            continue;                                       /* not a frame */
        if (OI_CODEC_SBC_DecoderReset(&dc, dec_data, sizeof dec_data, 2, 2, FALSE) != OI_OK) {
            printf("frame %u: the decoder would not start\n", frame);
            frame++;
            refused++;
            continue;
        }
        p = bytes;
        left = n;
        pb = (uint32_t)sizeof pcm;
        if (OI_CODEC_SBC_DecodeFrame(&dc, &p, &left, pcm, &pb) != OI_OK) {
            printf("frame %u: refused (the decoder took the frame for nothing)\n", frame);
            frame++;
            refused++;
            continue;
        }
        got = pb / (2u * sizeof(int16_t));                  /* the samples per channel */
        for (i = 0; i < got; i++) {
            sl += (double)pcm[2u * i] * pcm[2u * i];
            sr += (double)pcm[2u * i + 1u] * pcm[2u * i + 1u];
        }
        sl = sqrt(sl / (got ? got : 1u));
        sr = sqrt(sr / (got ? got : 1u));
        printf("frame %u: L rms %6.0f, R rms %6.0f%s\n", frame, sl, sr, (sl < 1.0 && sr < 1.0) ? "  (both silent)" : "");
        if (sl < 1.0)
            silent_l++;
        if (sr < 1.0)
            silent_r++;
        frame++;
    }
    printf("%u frames, %u refused, %u with the left channel silent, %u with the right channel silent\n",
           frame, refused, silent_l, silent_r);
    return 0;
}
