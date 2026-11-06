/* SPDX-License-Identifier: GPL-3.0-only OR MPL-2.0 */
/* Make the frames the decode-tool reads: the base's own encoder, in both modes.
 *
 * tools/bt_capture_decode.py decodes the frames the FM-1 sent (bt/br/bt_acl.c ba_capture). This program makes
 * such frames, so the tool can be checked without the hardware: the base's encoder at a bitpool, in the mode
 * asked for, written as hex, one frame a line (the form the FM-1's readout gives).
 *
 *   cc -O2 -w -o /tmp/mkframes tools/bt_capture_frames.c -lm
 *   /tmp/mkframes 46 1 > dump.hex     (joint stereo)
 *   /tmp/mkframes 46 0 > dump.hex     (plain stereo)
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../bt/br/bt_sbc.c"

#define NFR 40u                                   /* the frames written */
#define SAMPLES 128u

int main(int argc, char **argv)
{
    static struct bt_sbc_ctx c;
    static int16_t pcm[SAMPLES * 2];
    static uint8_t frame[260];
    uint32_t bitpool = 46, js = 1, fr, i;
    static const char hex[] = "0123456789ABCDEF";

    if (argc > 1)
        bitpool = (uint32_t)atoi(argv[1]);
    if (argc > 2)
        js = (uint32_t)atoi(argv[2]);
    bt_sbc_init(&c, 44100, (uint8_t)bitpool, (uint8_t)js);
    for (fr = 0; fr < NFR; fr++) {
        uint32_t len, z;
        for (i = 0; i < SAMPLES; i++) {
            pcm[2u * i] = (int16_t)(12000.0 * sin(2.0 * M_PI * 1000.0 * (fr * SAMPLES + i) / 44100.0));
            pcm[2u * i + 1u] = (int16_t)(12000.0 * sin(2.0 * M_PI * 440.0 * (fr * SAMPLES + i) / 44100.0));
        }
        len = bt_sbc_encode(&c, pcm, frame);
        for (z = 0; z < len; z++) {
            putchar(hex[frame[z] >> 4]);
            putchar(hex[frame[z] & 15u]);
        }
        putchar('\n');
    }
    return 0;
}
