/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Charles Vestal */
/* The USB microphone capture (device/usb_mic.c) against a model of the SIE and of a USB Audio Class microphone:
 * the descriptors parsed, the isochronous IN endpoint armed, the packets in the ring in order, the app's read out
 * of it, a disconnect flushing the ring, and a short or oversized packet harmless.
 * The model is the SIE's registers and a device that answers the control requests (as bsp/tests/usb_host_test.c does
 * for the MIDI host): the controller itself needs a bench, see docs/USB_AUDIO.md. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#include "../bsp/chip/fm1_usb.h"
#pragma GCC diagnostic pop

/* ------------------------------------------------- the SIE and the mic --- */
static uint8_t regs[4][32], common[16], index_reg, rsp[512];
static uint32_t rsp_len, rsp_pos;
static void *ep_radr[5];
static uint32_t line_state;                       /* the cable: 0 nothing, 2 a device */
static uint8_t bus_dead;
static uint32_t setup_requests;
static uint8_t pend_addr;
static uint8_t mic_addr, mic_cfgval, mic_alt, mic_seen_cfg, mic_seen_if;
static uint8_t iso_q[8][1024];
static uint16_t iso_len[8];
static uint32_t iso_head, iso_tail, iso_packets;

/* the microphone's configuration descriptor: an audio control interface, an audio streaming interface with the PCM
 * format in its first alternate setting and the isochronous IN endpoint in the second */
static uint8_t mic_cfg[96];
static uint16_t mic_cfg_len;

static void mic_descriptors(void)
{
    static const uint8_t body[] = {
        9, 2, 0, 0, 2, 1, 0, 0x80, 50,                   /* the configuration: the value 1 */
        9, 4, 0, 0, 0, 1, 1, 0, 0,                      /* interface 0: the audio control */
        9, 0x24, 1, 0, 1, 0x0A, 0, 0, 0,                /* the audio control header */
        9, 4, 1, 0, 0, 1, 2, 0, 0,                      /* interface 1: the audio streaming, alternate 0 */
        9, 0x24, 1, 1, 1, 0, 0, 0, 0,                   /* the format configuration: one format */
        12, 0x24, 2, 1, 1, 2, 16, 1, 1, 0x44, 0xAC, 0,  /* the format: PCM, 16 bits, one channel, 44100 */
        9, 4, 1, 1, 1, 1, 2, 0, 0,                      /* interface 1, alternate 1: the endpoint */
        9, 5, 0x81, 1, 0, 2, 1, 0, 0,                   /* the isochronous IN endpoint 1, 512 bytes */
    };
    uint16_t n = (uint16_t)sizeof body;
    memcpy(mic_cfg, body, n);
    mic_cfg[2] = (uint8_t)n;
    mic_cfg[3] = (uint8_t)(n >> 8);
    mic_cfg_len = n;
}

/* the device that answers the control requests */
static int answer(const uint8_t *s)
{
    uint8_t rt = s[0], rq = s[1];
    uint16_t val = (uint16_t)(s[2] | s[3] << 8), idx = (uint16_t)(s[4] | s[5] << 8), len = (uint16_t)(s[6] | s[7] << 8);
    uint32_t n = 0;
    rsp_len = rsp_pos = 0;
    if (rt == 0x80 && rq == 6 && (val >> 8) == 1) {         /* the device descriptor */
        uint8_t dd[18] = {18, 1, 0x00, 0x02, 0x00, 0, 0, 64, 0x09, 0x12, 1, 0, 0, 1, 1, 2, 0, 1};
        memcpy(rsp, dd, 18);
        n = 18;
    } else if (rt == 0x80 && rq == 6 && (val >> 8) == 2) {  /* the configuration descriptor */
        memcpy(rsp, mic_cfg, mic_cfg_len);
        n = mic_cfg_len;
    } else if (rt == 0x00 && rq == 5) {                     /* SET_ADDRESS */
        pend_addr = (uint8_t)val;
    } else if (rt == 0x00 && rq == 9) {                     /* SET_CONFIGURATION */
        mic_cfgval = (uint8_t)val;
        mic_seen_cfg++;
    } else if (rt == 0x01 && rq == 11) {                    /* SET_INTERFACE */
        mic_alt = (uint8_t)val;
        mic_seen_if++;
        (void)idx;
    } else
        return 0;
    rsp_len = n < len ? n : len;
    return 1;
}

/* an isochronous packet the microphone sends */
static void iso_push(const uint8_t *p, uint32_t n)
{
    if (iso_tail - iso_head >= 8u || n > sizeof iso_q[0])
        return;
    memcpy(iso_q[iso_tail % 8], p, n);
    iso_len[iso_tail % 8] = (uint16_t)n;
    iso_tail++;
}

static void iso_deliver(void)                     /* the packet lands in EP1's buffer */
{
    if (iso_head == iso_tail || !ep_radr[1])
        return;
    memcpy(ep_radr[1], iso_q[iso_head % 8], iso_len[iso_head % 8]);
    regs[1][22] = (uint8_t)iso_len[iso_head % 8];
    regs[1][23] = (uint8_t)(iso_len[iso_head % 8] >> 8);
    regs[1][20] = 1;                              /* RxPktRdy */
    iso_head++;
    iso_packets++;
}

static void mock_write(uint32_t r, uint32_t v)
{
    if (r == 14) {                                /* INDEX */
        index_reg = (uint8_t)v;
    } else if (r == 1 && (v & 8u)) {              /* POWER: a bus reset */
        mic_addr = 0;
        common[r] = (uint8_t)v;
    } else if (r < 16) {
        common[r] = (uint8_t)v;
    } else if (index_reg == 0 && r == 17) {       /* EP0's CSR (the host) */
        if (v == 0x0A) {                          /* the setup packet */
            uint8_t s[8];
            setup_requests++;
            memcpy(s, ep_radr[0], 8);
            if (bus_dead || !answer(s))
                regs[0][17] = 0x10;               /* no answer */
            else
                regs[0][17] = 0;
        } else if (v == 0x20) {                   /* an IN data stage */
            uint32_t n = rsp_len - rsp_pos;
            if (n > 64u)
                n = 64u;
            memcpy(ep_radr[0], rsp + rsp_pos, n);
            rsp_pos += n;
            regs[0][22] = (uint8_t)n;
            regs[0][17] = 1;
        } else if (v == 0x60) {                   /* the status stage */
            regs[0][22] = 0;
            regs[0][17] = bus_dead ? 0x10 : 1;
            mic_addr = pend_addr;
        } else if (v == 0x42) {
            regs[0][17] = bus_dead ? 0x10 : 0;
        } else {
            regs[0][17] = (uint8_t)v;
        }
    } else if (index_reg == 1 && r == 20) {       /* EP1's RXCSR1 */
        if (v == 0x20) {                          /* ReqPkt: the microphone's packet may land now */
            regs[1][20] = 0x20;
            iso_deliver();
        } else if (v == 0x10) {                   /* the packet is released */
            regs[1][20] &= (uint8_t)~1u;
        } else {
            regs[1][20] = (uint8_t)v;
        }
    } else {
        regs[index_reg][r] = (uint8_t)v;
    }
}
static uint8_t last_read;
static void mock_rd(uint32_t r) { last_read = (uint8_t)r; }
static uint32_t mock_dt(void)
{
    uint32_t r = last_read;
    if (r == 15)
        return 0x40u;                             /* DEVCTL: the host, full speed */
    return r < 16 ? common[r] : regs[index_reg][r];
}
static void mock_enable(uint32_t eps) { (void)eps; }
static void mock_rxbuf(uint32_t ep, void *p) { ep_radr[ep] = p; }
static void mock_send(uint32_t ep, void *p, uint32_t n) { (void)ep; (void)p; (void)n; }
static void mock_ep0_send(void *p, uint32_t n) { mock_send(0, p, n); }
static void mock_ep0_buf(void *p) { ep_radr[0] = p; }
static void mock_nop(void) {}
static void mock_off(void) {}
static void mock_attach(void *p) { (void)p; }
static void mock_host_attach(void *p) { ep_radr[0] = p; }
static uint32_t mock_one(void) { return 1; }
static uint32_t mock_zero(void) { return 0; }
static uint32_t mock_line(void) { return line_state; }
static void fm1_delay_ms(uint32_t ms) { (void)ms; }

#define fm1_usb_ep_enable mock_enable
#define fm1_usb_ep_txbuf mock_rxbuf
#define fm1_usb_ep_rxbuf mock_rxbuf
#define fm1_usb_ep_send mock_send
#define fm1_usb_ep0_send mock_ep0_send
#define fm1_usb_ep0_buf mock_ep0_buf
#define fm1_usb_ep4_rxbuf(p) mock_rxbuf(4, p)
#define fm1_usb_ep4_send(p, n) mock_send(4, p, n)
#define fm1_usb_rx_sync mock_nop
#define fm1_usb_sie_on mock_one
#define fm1_usb_sie_done mock_one
#define fm1_usb_sie_wr_start mock_write
#define fm1_usb_sie_rd_start mock_rd
#define fm1_usb_sie_data mock_dt
#define fm1_usb_reset mock_nop
#define fm1_usb_off mock_off
#define fm1_usb_attach mock_attach
#define fm1_usb_host_attach mock_host_attach
#define fm1_usb_line mock_line
#define fm1_usb_sof_take mock_zero
#define fm1_wdt_feed() (fm1_ms++)                  /* the host code's waits run on this clock */
#define RING_PUBLISH() ((void)0)
#define FELUCCA_UAC 0
#define FELUCCA_CDC 0
#define FELUCCA_OTA 0
#define FELUCCA_USB_HOST 1
#define FELUCCA_UAC_48K 0
#define FELUCCA_UAC_OUT 0
#define HALF_FRAMES 128u
#define PLAT_HOST                                /* the pool is empty on the host */
#include "../include/plat.h"
#include "../bsp/chip/usb.c"
#define FM1B_USB_HOST 1
#include "../device/usb_mic.c"

/* the platform's five (device/plat_fm1.c), the same one-liners over the capture: the test drives the app's names */
int plat_mic_state(void) { return umic_state(); }
uint32_t plat_mic_rate(void) { return umic_rate(); }
int plat_mic_channels(void) { return umic_channels(); }
int plat_mic_bits(void) { return umic_bits(); }
int plat_mic_read(int32_t *dst, uint32_t n) { return umic_read(dst, n); }

/* ------------------------------------------------------------ the test --- */
static void reset_model(void)
{
    memset(regs, 0, sizeof regs);
    memset(common, 0, sizeof common);
    index_reg = 0;
    rsp_len = rsp_pos = 0;
    setup_requests = 0;
    mic_addr = mic_cfgval = mic_alt = mic_seen_cfg = mic_seen_if = 0;
    iso_head = iso_tail = iso_packets = 0;
    line_state = 2;                               /* the microphone is plugged in */
    bus_dead = 0;
    mic_descriptors();
    umic.armed = 0;
    umic.tried = 0;
    umic.fault = 0;
    umic.state = PLAT_MIC_NONE;
    umic.rate = 0;
    umic.channels = 0;
    umic.bits = 0;
    umic_w = umic_r = 0;
}

/* the host session: the bsp driver looks at the device (it finds no MIDI in it), then the capture takes it over */
static void bring_up(void)
{
    int k;
    usb_host_switch(1);
    for (k = 0; k < 50 && !uh.tried; k++) {
        fm1_ms += 20;
        usb_host_task(fm1_ms);
    }
    assert(uh.on && uh.tried && !uh.run);          /* the driver looked and found no MIDI */
    for (k = 0; k < 20 && !umic.armed; k++) {
        fm1_ms += 20;
        usb_mic_task(fm1_ms);
    }
    assert(umic.armed);
}

/* the packets into the ring: the driver takes one per poll */
static void pump(int n)
{
    int k;
    for (k = 0; k < n; k++)
        usb_mic_poll();
}

static void test_descriptors_and_arm(void)
{
    reset_model();
    bring_up();
    /* the configuration: the device was set up and its streaming interface selected */
    assert(mic_seen_cfg == 1 && mic_cfgval == 1);
    assert(mic_seen_if == 1 && mic_alt == 1);
    /* the format, as the AS format descriptor has it */
    assert(plat_mic_rate() == 44100u);
    assert(plat_mic_channels() == 1);
    assert(plat_mic_bits() == 16);
    assert(plat_mic_state() == PLAT_MIC_CONNECTED);
    /* the isochronous IN endpoint: the ISO bit in RXCSR2 and the endpoint type 1 in RXTYPE */
    assert(regs[1][21] & 0x40u);
    assert((regs[1][26] >> 4) == 1u);
    assert((regs[1][26] & 0x0Fu) == 1u);           /* the endpoint 1 */
    assert(regs[1][19] == 255u);                   /* RXMAXP: the endpoint's packet size, the register's maximum */
    puts("USB mic: the UAC descriptors parsed, the isochronous IN endpoint armed: OK");
}

static void test_packets(void)
{
    uint8_t pkt[64];
    int32_t out[1200];
    uint32_t i;
    reset_model();
    bring_up();
    for (i = 0; i < 16; i++) {                    /* 16 samples, 16-bit, in order */
        int16_t v = (int16_t)(1000 + (int32_t)i);
        pkt[i * 2] = (uint8_t)v;
        pkt[i * 2 + 1] = (uint8_t)((uint16_t)v >> 8);
    }
    iso_push(pkt, 32);
    pump(4);
    assert(iso_packets == 1);
    assert(plat_mic_state() == PLAT_MIC_STREAMING);
    assert(plat_mic_read(out, 16) == 16);
    for (i = 0; i < 16; i++)
        assert(out[i] == 1000 + (int32_t)i);
    assert(plat_mic_read(out, 4) == 0);            /* nothing more: the read never waits */
    /* the ring wraps: the order is kept across the end of the buffer */
    for (i = 0; i < 1000; i++) {
        int16_t v = (int16_t)i;
        pkt[0] = (uint8_t)v;
        pkt[1] = (uint8_t)((uint16_t)v >> 8);
        umic_put(pkt, 2);
    }
    assert(umic_w - umic_r == 1000);
    assert(plat_mic_read(out, 32) == 32);
    for (i = 0; i < 32; i++)
        assert(out[i] == (int32_t)i);
    assert(plat_mic_read(out, 1100) == 968);       /* what is in the ring, no more */
    for (i = 0; i < 100; i++) {                    /* 100 more: the write pointer passes the end of the ring */
        int16_t v = (int16_t)(1000 + i);
        pkt[0] = (uint8_t)v;
        pkt[1] = (uint8_t)((uint16_t)v >> 8);
        umic_put(pkt, 2);
    }
    assert(umic_w - umic_r == 100);
    assert(plat_mic_read(out, 100) == 100);
    for (i = 0; i < 100; i++)
        assert(out[i] == (int32_t)(1000 + i));
    puts("USB mic: the packets in the ring in order, the app's read, the ring's wrap: OK");
}

static void test_short_and_oversized(void)
{
    uint8_t pkt[2048];
    int32_t out[8];
    uint32_t drops;
    reset_model();
    bring_up();
    pkt[0] = 1;
    pkt[1] = 0;
    umic_put(pkt, 2);                             /* one sample: the ring takes it */
    drops = umic.drops;
    umic_put(pkt, 1);                             /* half a sample: dropped */
    assert(umic.drops == drops + 1);
    umic_put(pkt, 2048);                          /* more than a packet: dropped, nothing read out of bounds */
    assert(umic.drops == drops + 2);
    assert(plat_mic_read(out, 8) == 1);           /* only the whole sample is in the ring */
    assert(out[0] == 1);
    puts("USB mic: a short and an oversized packet leave the ring alone: OK");
}

static void test_disconnect(void)
{
    uint8_t pkt[8];
    int32_t out[8];
    reset_model();
    bring_up();
    pkt[0] = 7;
    pkt[1] = 0;
    iso_push(pkt, 2);
    pump(4);
    assert(plat_mic_state() == PLAT_MIC_STREAMING);
    assert(plat_mic_read(out, 4) == 1);
    line_state = 0;                               /* the cable is out */
    pump(20);
    assert(plat_mic_state() == PLAT_MIC_NONE);
    assert(plat_mic_read(out, 4) == 0);           /* the ring is flushed */
    /* and it comes back */
    line_state = 2;
    for (int k = 0; k < 50 && !umic.armed; k++) {
        fm1_ms += 20;
        usb_host_task(fm1_ms);
        usb_mic_task(fm1_ms);
    }
    assert(umic.armed && plat_mic_state() == PLAT_MIC_CONNECTED);
    puts("USB mic: the disconnect flushes the ring and the state, the reconnect takes the device again: OK");
}

static void test_fault(void)
{
    int32_t out[8];
    reset_model();
    bring_up();
    regs[1][20] = 0x40;                           /* a stall on the endpoint */
    pump(2);
    assert(plat_mic_state() == PLAT_MIC_ERROR);
    assert(plat_mic_read(out, 4) == 0);
    puts("USB mic: a stall on the endpoint is an error, not a stream: OK");
}

int main(void)
{
    reset_model();
    test_descriptors_and_arm();
    test_packets();
    test_short_and_oversized();
    test_disconnect();
    test_fault();
    puts("USB microphone capture: OK");
    return 0;
}
