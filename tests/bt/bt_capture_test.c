/* SPDX-License-Identifier: GPL-3.0-only OR MPL-2.0 */
/* The SBC capture: the frames the FM-1 sends, kept to be read out and decoded.
 *
 * bt/br/bt_acl.c ba_capture keeps the first frames of the stream (four windows of one second, eight frames
 * each) so that one can check off the hardware what a speaker's decoder gets (the user, 2026-11-06: only the
 * right channel of the speaker sounded, on Polly, LOOM-8 and Optimist alike). The capture is armed when the
 * encoder starts and read out over USB-MIDI (device/bt_own_audio.c bta_frame, tools/fm1_remote.py
 * bta capture / bta capture-frame).
 *
 * This test checks that the capture takes the frames as they go on the air (the bytes, not a copy that
 * differs), that it stops at its window, and that the readout gives those bytes back as hex.
 *
 * The capture's own code is copied here (it is static in bt_acl.c, one translation unit with the whole
 * stack, and the test does not link the stack): the copy is checked against bt_acl.c by a test that
 * compares the two files' capture blocks.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define BA_CAP_PHASES 4u
#define BA_CAP_FRAMES 8u
#define BA_CAP_MAX 128u
#define IN_POOL

struct ba_capture {
    volatile uint8_t active, ready;
    uint32_t samples;
    uint8_t count[BA_CAP_PHASES];
    uint8_t len[BA_CAP_PHASES][BA_CAP_FRAMES];
    uint8_t data[BA_CAP_PHASES][BA_CAP_FRAMES][128];
};
static struct ba_capture ba_cap;

static void ba_capture_arm(void)
{
    uint32_t p, f;
    ba_cap.active = 0;
    ba_cap.ready = 0;
    ba_cap.samples = 0;
    for (p = 0; p < BA_CAP_PHASES; p++) {
        ba_cap.count[p] = 0;
        for (f = 0; f < BA_CAP_FRAMES; f++)
            ba_cap.len[p][f] = 0;
    }
    ba_cap.active = 1;
}

static uint32_t ba_capture_frame_hex(uint32_t phase, uint32_t frame, uint8_t *out, uint32_t n)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t z, w = 0;
    if (phase >= BA_CAP_PHASES || frame >= BA_CAP_FRAMES || frame >= ba_cap.count[phase])
        return 0;
    for (z = 0; z < ba_cap.len[phase][frame] && w + 2u < n; z++) {
        uint8_t b = ba_cap.data[phase][frame][z];
        out[w++] = (uint8_t)hex[b >> 4];
        out[w++] = (uint8_t)hex[b & 15u];
    }
    return w;
}

static uint32_t ba_capture_count(uint32_t phase)
{
    return phase < BA_CAP_PHASES ? ba_cap.count[phase] : 0u;
}

/* the capture as bt_acl.c runs it: the frame goes in, the window fills up, then it stops */
static void ba_capture_take(const uint8_t *frame, uint32_t slen)
{
    if (ba_cap.active) {
        uint32_t phase = ba_cap.samples / 44100u;
        if (phase < BA_CAP_PHASES && ba_cap.count[phase] < BA_CAP_FRAMES) {
            uint32_t ci = ba_cap.count[phase], z;
            ba_cap.len[phase][ci] = (uint8_t)(slen > 128u ? 128u : slen);
            for (z = 0; z < ba_cap.len[phase][ci]; z++)
                ba_cap.data[phase][ci][z] = frame[z];
            ba_cap.count[phase]++;
        }
        ba_cap.samples += 128u;
        if (ba_cap.samples >= BA_CAP_PHASES * 44100u) {
            ba_cap.active = 0;
            ba_cap.ready = 1;
        }
    }
}

static int fails;
static void check(const char *what, int ok)
{
    printf("bt_capture_test: %-84s %s\n", what, ok ? "ok" : "FAIL");
    fails += !ok;
}

int main(void)
{
    uint8_t frame[256], hex[300];
    uint32_t i, n, w;

    /* nothing captured before it is armed */
    check("nothing captured before it is armed", ba_capture_frame_hex(0, 0, hex, sizeof hex) == 0);

    ba_capture_arm();
    check("armed: the capture is not ready yet", !ba_cap.ready && ba_cap.active);

    /* the first frame of the first window */
    for (i = 0; i < 105u; i++)
        frame[i] = (uint8_t)(i + 1u);
    ba_capture_take(frame, 105u);
    check("the first frame is kept", ba_capture_count(0) == 1);
    w = ba_capture_frame_hex(0, 0, hex, sizeof hex);
    check("the readout gives the frame's bytes back, as hex",
          w == 210u && hex[0] == '0' && hex[1] == '1' && hex[2] == '0' && hex[3] == '2' &&
          hex[208] == '6' && hex[209] == '9');

    /* a frame that is not there yet */
    check("a frame that was not captured has nothing to read",
          ba_capture_frame_hex(0, 1, hex, sizeof hex) == 0);
    check("a window that was not captured has nothing to read",
          ba_capture_frame_hex(3, 0, hex, sizeof hex) == 0);

    /* the window holds at most eight frames */
    for (i = 1; i < 12u; i++)
        ba_capture_take(frame, 105u);
    check("the window holds at most eight frames", ba_capture_count(0) == BA_CAP_FRAMES);

    /* the second window: a second of audio later. A frame is 128 samples, so the second window starts at the
     * first frame that reaches 44100 samples (345 frames: 344 whole ones and a part). */
    {
        uint32_t first = (44100u + 127u) / 128u;   /* the first frame of the second window */
        for (i = 12u; i < first; i++)             /* the rest of the first window's own frames */
            ba_capture_take(frame, 105u);
        for (i = 0; i < 8u; i++)                  /* the second window's eight frames */
            ba_capture_take(frame, 105u);
    }
    check("the second window holds its own frames", ba_capture_count(1) == 8u);

    /* the whole capture: four windows, then it stops */
    for (i = 0; i < 4u * 44100u / 128u + 16u; i++)
        ba_capture_take(frame, 105u);
    check("the capture is ready after its four windows", ba_cap.ready && !ba_cap.active);
    check("every window holds eight frames",
          ba_capture_count(0) == 8u && ba_capture_count(1) == 8u &&
          ba_capture_count(2) == 8u && ba_capture_count(3) == 8u);

    /* after it is done, nothing more is taken */
    for (i = 0; i < 100u; i++)
        ba_capture_take(frame, 105u);
    check("nothing more is taken after it is ready", ba_cap.count[3] == 8u);

    /* arming again starts over */
    ba_capture_arm();
    check("arming again starts over", !ba_cap.ready && ba_cap.count[0] == 0 && ba_cap.count[1] == 0);

    /* a frame longer than the capture's buffer is cut, not overflowed */
    ba_capture_arm();
    for (i = 0; i < 200u; i++)
        frame[i] = (uint8_t)i;
    ba_capture_take(frame, 200u);
    check("a frame longer than the capture's buffer is cut", ba_cap.len[0][0] == 128u);
    n = ba_capture_frame_hex(0, 0, hex, sizeof hex);
    check("... and the readout gives the cut frame back", n == 256u);

    printf(fails ? "bt_capture_test: %d FAILED\n" : "bt_capture_test: all ok\n", fails);
    return fails != 0;
}
