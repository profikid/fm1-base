/* SPDX-License-Identifier: GPL-3.0-only OR MPL-2.0 */
/* The SBC encoder's mode: the one we agreed with the device.
 *
 * bt_acl.c bt_audio_feed set the encoder up with a fixed joint stereo, whatever was agreed:
 *
 *     bt_sbc_init(&ba_sbc, 44100, bitpool, 1);
 *
 * while the mode we agreed lives in ba.conf[0] (A2DP 1.3 4.3, the SBC capability's byte 0: bit 0 joint
 * stereo, bit 1 stereo). A device that was told "stereo" and receives joint-stereo frames reads the
 * wrong channel layout.
 *
 * The encoder must use the agreed mode, and the frame it writes must say so: the frame's own header
 * (A2DP 12.9, byte 1: bits 4-3 the channel mode) is what the decoder reads, so a frame that says joint
 * stereo is decoded as joint stereo, whatever the decoder was configured for.
 *
 * This test checks that the encoder's mode follows the agreement, and that the frames it writes carry
 * that mode in their header (the decoder reads the frame, not the configuration).
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "../../bt/br/bt_sbc.c"
#include "ref/sbc_dec/oi_codec_sbc.h"

#define SBC_STEREO 2
#define SBC_JOINT_STEREO 3

static int fails;
static void check(const char *what, int ok)
{
    printf("bt_sbc_mode: %-84s %s\n", what, ok ? "ok" : "FAIL");
    fails += !ok;
}

/* the mode in a frame's own header (A2DP 12.9: byte 1, bits 4-3) */
static uint8_t frame_mode(const uint8_t *f)
{
    return (uint8_t)((f[1] >> 2) & 0x03u);
}

/* the mode we agreed, out of the configuration (ba.conf[0]: bit 0 joint stereo, bit 1 stereo): the base's own
 * function (bt_acl.c ba_sbc_agreed_mode), copied here (it is static in a translation unit with the whole stack,
 * and the test does not link the stack) */
static uint8_t agreed_mode(uint8_t conf0)
{
    if (conf0 & 0x02u)
        return 2u;                                  /* plain stereo */
    if (conf0 & 0x01u)
        return 1u;                                  /* joint stereo */
    return 0u;                                      /* none */
}

int main(void)
{
    static struct bt_sbc_ctx c;
    static int16_t pcm[128 * 2];
    uint8_t frame[128];
    uint32_t n, i;

    for (i = 0; i < 128u * 2u; i++)
        pcm[i] = (int16_t)((i & 1u) ? 8000 : -8000);   /* a square wave, left and right alike */

    /* the device offers both modes: plain stereo is agreed (bt_avdtp.c bv_sbc_choose) */
    bt_sbc_init(&c, 44100, 46u, agreed_mode(0x22u) == 1u ? 1u : 0u);
    n = bt_sbc_encode(&c, pcm, frame);
    check("the agreed plain stereo: the encoder's mode is plain stereo",
          c.p.s16ChannelMode == SBC_STEREO);
    check("... and the frame says so in its header",
          n > 4u && frame_mode(frame) == SBC_STEREO);
    check("... the frame is one byte shorter than a joint-stereo frame (no join byte)",
          n == bt_sbc_frame_len(46u, 0u));

    /* the device offers joint stereo only: joint stereo is agreed */
    bt_sbc_init(&c, 44100, 46u, agreed_mode(0x21u) == 1u ? 1u : 0u);
    n = bt_sbc_encode(&c, pcm, frame);
    check("the agreed joint stereo: the encoder's mode is joint stereo",
          c.p.s16ChannelMode == SBC_JOINT_STEREO);
    check("... and the frame says so in its header",
          n > 4u && frame_mode(frame) == SBC_JOINT_STEREO);
    check("... the frame is a byte longer (the join byte)",
          n == bt_sbc_frame_len(46u, 1u));

    /* the encoder's mode and the frame's header must never disagree */
    {
        static const uint8_t confs[4] = {0x21u, 0x22u, 0x23u, 0x20u};
        uint32_t k, bad = 0;
        for (k = 0; k < 4u; k++) {
            bt_sbc_init(&c, 44100, 46u, agreed_mode(confs[k]) == 1u ? 1u : 0u);
            n = bt_sbc_encode(&c, pcm, frame);
            if (n < 4u || frame_mode(frame) != c.p.s16ChannelMode)
                bad++;
        }
        check("the encoder's mode and the frame's header always agree", bad == 0);
    }

    /* the mode we agreed: plain stereo first (bt_avdtp.c bv_sbc_choose), joint stereo only when the sink has no
     * plain stereo, and nothing when nothing was configured */
    check("the agreed mode: plain stereo when the sink has both", agreed_mode(0x22u) == 2u);
    check("... joint stereo when the sink has only that", agreed_mode(0x21u) == 1u);
    check("... plain stereo when the sink has both and joint stereo", agreed_mode(0x23u) == 2u);
    check("... none when nothing was configured", agreed_mode(0x00u) == 0u);

    printf(fails ? "bt_sbc_mode_test: %d FAILED\n" : "bt_sbc_mode_test: all ok\n", fails);
    return fails != 0;
}
