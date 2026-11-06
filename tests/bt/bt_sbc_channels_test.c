/* SPDX-License-Identifier: GPL-3.0-only OR MPL-2.0 */
/* The SBC encoder keeps both channels apart: one channel silent, the other not.
 *
 * bt_sbc_test.c encodes two tones (1 kHz left, 440 Hz right) and reads them back, so it proves the channels are not
 * mixed up. It does not prove that a channel that is silent stays silent: a square wave that is the same on both sides
 * (bt_sbc_mode_test.c) does too. A channel that is silent on one side and not on the other is the case that matters:
 * the user, 2026-11-06, heard only the right channel of the speaker, on Polly, LOOM-8 and Optimist alike, and the
 * app's own L/R test (a 440 Hz beep left, 880 Hz right) makes exactly that signal.
 *
 * So: the left channel silent, the right one a tone, and back the other way round. The frames go through the base's
 * encoder (bt/br/bt_sbc.c, AOSP's behind our interface) and AOSP's decoder (tests/bt/ref/sbc_dec, the one
 * bt_sbc_test.c uses), the way a speaker's decoder reads them. Every mode (joint stereo and plain stereo) and every
 * bitpool the FM-1 uses are checked: the channel that was silent must come back silent, and the one that was not must
 * come back with its tone.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "../../bt/br/bt_sbc.c"
#include "ref/sbc_dec/oi_codec_sbc.h"

#define SBC_STEREO 2
#define SBC_JOINT_STEREO 3

static int fails;
static void check(const char *what, int ok)
{
    printf("bt_sbc_channels: %-88s %s\n", what, ok ? "ok" : "FAIL");
    fails += !ok;
}

#define SECS 1u
#define NFR (SECS * 44100u / 128u)
static int16_t pcm_in[NFR * 256u], pcm_out[NFR * 256u];

/* the amplitude of f in x */
static double tone(const int16_t *x, uint32_t n, uint32_t stride, double f)
{
    double c = 0, s = 0;
    uint32_t i;
    for (i = 0; i < n; i++)
        c += x[i * stride] * cos(2 * M_PI * f * i / 44100.0), s += x[i * stride] * sin(2 * M_PI * f * i / 44100.0);
    return 2 * hypot(c, s) / n;
}

/* the tone in the right channel (the left one silent), or the other way round; encoded and decoded back.
 * *amp_l and *amp_r the two amplitudes back. */
static int round_trip(uint8_t bitpool, uint8_t js, int tone_on_right, double *amp_l, double *amp_r, uint32_t *bad)
{
    static struct bt_sbc_ctx c;
    static uint8_t stream[NFR * 128u];
    static uint32_t dec_data[CODEC_DATA_WORDS(2, SBC_CODEC_FAST_FILTER_BUFFERS)];
    OI_CODEC_SBC_DECODER_CONTEXT dc;
    uint32_t fr, len = 0, at = 0, got = 0;
    const OI_BYTE *p;
    uint32_t left;
    bt_sbc_init(&c, 44100, bitpool, js);
    for (fr = 0; fr < NFR; fr++) {
        uint32_t i;
        for (i = 0; i < 128u; i++) {
            uint32_t k = fr * 128u + i;
            int16_t v = (int16_t)(12000.0 * sin(2 * M_PI * 1000.0 * k / 44100.0));
            pcm_in[2u * k] = tone_on_right ? 0 : v;       /* one channel silent, the other the tone */
            pcm_in[2u * k + 1u] = tone_on_right ? v : 0;
        }
        len = bt_sbc_encode(&c, pcm_in + fr * 256u, stream + at);
        at += len;
    }
    *bad = 0;
    if (OI_CODEC_SBC_DecoderReset(&dc, dec_data, sizeof dec_data, 2, 2, FALSE) != OI_OK)
        return 0;
    p = stream;
    left = at;
    while (left > 0u && got < NFR) {
        uint32_t pb = 256u * sizeof(int16_t);
        if (OI_CODEC_SBC_DecodeFrame(&dc, &p, &left, pcm_out + got * 256u, &pb) != OI_OK) {
            (*bad)++;
            break;
        }
        got++;
    }
    {
        uint32_t skip = 2048u, n = (NFR - 16u) * 128u - skip;   /* (the filters' delay settled) */
        *amp_l = tone(pcm_out + 2u * skip, n, 2, 1000.0);
        *amp_r = tone(pcm_out + 2u * skip + 1u, n, 2, 1000.0);
    }
    return got == NFR && !*bad;
}

int main(void)
{
    static const struct { uint8_t bitpool, js; } SETS[] = {{53, 0}, {53, 1}, {46, 0}, {46, 1}, {35, 0}, {35, 1},
                                                          {28, 0}, {28, 1}, {2, 0}, {2, 1}};
    uint32_t i;
    char what[160];
    double al, ar;
    uint32_t bad;
    for (i = 0; i < sizeof SETS / sizeof SETS[0]; i++) {
        int ok;
        ok = round_trip(SETS[i].bitpool, SETS[i].js, 1, &al, &ar, &bad);
        snprintf(what, sizeof what, "bitpool %u %s, tone on the right: the decoder takes all %u frames (%u refused)",
                 SETS[i].bitpool, SETS[i].js ? "joint stereo" : "stereo", NFR, bad);
        check(what, ok);
        snprintf(what, sizeof what, "... the silent channel stays silent (L %4.0f) and the tone comes back (R %5.0f of 12000)",
                 al, ar);
        check(what, al < 120.0 && fabs(ar - 12000.0) < 400.0);
        ok = round_trip(SETS[i].bitpool, SETS[i].js, 0, &al, &ar, &bad);
        snprintf(what, sizeof what, "bitpool %u %s, tone on the left: the decoder takes all %u frames (%u refused)",
                 SETS[i].bitpool, SETS[i].js ? "joint stereo" : "stereo", NFR, bad);
        check(what, ok);
        snprintf(what, sizeof what, "... the silent channel stays silent (R %4.0f) and the tone comes back (L %5.0f of 12000)",
                 ar, al);
        check(what, ar < 120.0 && fabs(al - 12000.0) < 400.0);
    }
    printf("bt_sbc_channels: %d failed\n", fails);
    return fails != 0;
}
