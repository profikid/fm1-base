/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Charles Vestal */
/* The USB microphone capture: the FM-1 as the USB host, a USB Audio Class microphone streaming PCM in.
 * Part of the unity build (device/unity.c), after usb.c (the SIE helpers and the host driver) and before plat_fm1.c
 * (the platform's API).
 *
 * The bsp's host driver (bsp/chip/usb_host.c) is a USB-MIDI driver: it reads the configuration descriptor and keeps
 * only a USB-MIDI streaming interface, and its endpoint service is the bulk/interrupt request-and-release handshake.
 * An isochronous microphone is none of that, so this file does the microphone's part of the host session: it walks
 * the configuration descriptor for an AudioControl and an AudioStreaming interface, takes the isochronous IN endpoint
 * and the PCM format, sets the configuration and the alternate setting, and arms host EP1 for isochronous reception
 * (RXCSR2 bit 6, the bit the device driver's EP4 sets for its isochronous OUT path). The packets go into a
 * lock-free ring the app reads (plat_mic_read).
 *
 *   main loop (usb_mic_task)  the host session is the bsp driver's: it resets the bus, reads the descriptor and
 *                      knows the hubs. This waits until the driver has looked at the device and found no MIDI in
 *                      it (the driver leaves the port alone then) and takes the session over: the descriptor walk,
 *                      the configuration, the arming. A microphone behind a powered hub (the FM-1 gives the port
 *                      no power) is found on the hub's ports.
 *   TIMER5 (usb_mic_poll)  once per USB frame the isochronous packet in EP1 goes into the ring. Never waits.
 *
 * Without FM1B_USB_HOST (the port is never a host) the API stays and reports no microphone.
 * The isochronous reception itself is not verified on hardware: see docs/USB_AUDIO.md. */

#ifndef FM1B_USB_HOST
#define FM1B_USB_HOST 0
#endif

#if FM1B_USB_HOST

/* the ring: 1024 samples (a power of two), the producer the TIMER5 ISR, the consumer the app's audio interrupt */
#define UMIC_RING 1024u
#define UMIC_RING_MASK (UMIC_RING - 1u)
/* an isochronous packet at full speed: 1023 bytes at most */
#define UMIC_PKT 1024u
/* the configuration descriptor we keep */
#define UMIC_CFG 512u
/* the address of the device on the root port */
#define UMIC_ROOT_ADDR 1u
/* a run of SIE errors before the capture gives up */
#define UMIC_MAX_ERRORS 16u
/* the idle samples (2 kHz) that mean the device is gone */
#define UMIC_IDLE 16u

static int32_t umic_ring[UMIC_RING] PLAT_POOL;
static volatile uint32_t umic_w, umic_r;             /* the ring's write (the ISR) and read (the app) */
static uint8_t umic_rx[UMIC_PKT] __attribute__((aligned(4)));
static uint8_t umic_cfg[UMIC_CFG];

static struct {
    volatile uint8_t state;                          /* PLAT_MIC_* */
    uint8_t armed, tried, fault, errors, idle;        /* the session's flags */
    uint8_t channels, bits;
    uint8_t addr, mps, iface, alt, ep;               /* the device and its endpoint */
    uint16_t ep_mps;                                 /* the endpoint's maximum packet size */
    uint8_t hub_addr, hub_mps, hub_ports;             /* the hub it sits behind, if any */
    uint32_t rate;
    uint32_t pkts, drops, samples;                    /* what came in, what was dropped */
    uint32_t retry_ms;
} umic;

/* ------------------------------------------------- the configuration --- */

/* What the microphone is: the streaming interface, its alternate setting, the isochronous IN endpoint and the PCM
 * format it offers. */
typedef struct {
    uint8_t cfg;                                     /* bConfigurationValue */
    uint8_t iface, alt;                              /* the streaming interface and its alternate setting */
    uint8_t ep;                                      /* the isochronous IN endpoint */
    uint16_t mps;                                    /* its maximum packet size */
    uint8_t channels, bits;
    uint32_t rate;
} umic_dev_t;

/* Walk the configuration descriptor: an AudioControl interface (class 1, subclass 1) and an AudioStreaming interface
 * (class 1, subclass 2) with an isochronous IN endpoint (the endpoint type 1), and the PCM format (the class-specific
 * FORMAT_TYPE) of that streaming interface. 1 = a microphone, 0 = not one. */
static int umic_parse(const uint8_t *d, uint32_t len, umic_dev_t *o)
{
    uint32_t i = 0;
    uint8_t cfg = 0, ac = 0, ifnum = 0, alt = 0, in_as = 0;
    uint8_t ep = 0, ep_if = 0, ep_alt = 0;
    uint16_t ep_mps = 0;
    uint32_t rate = 0, ch = 0, bits = 0;
    int fmt = 0;
    while (i + 1u < len) {
        uint8_t bl = d[i], ty = d[i + 1];
        if (bl < 2u || i + bl > len)
            break;
        if (ty == 2u) {                              /* the configuration descriptor */
            cfg = d[i + 5];
        } else if (ty == 4u) {                       /* an interface descriptor */
            ifnum = d[i + 2];
            alt = d[i + 3];
            in_as = (d[i + 5] == 1u && d[i + 6] == 2u);
            if (d[i + 5] == 1u && d[i + 6] == 1u)
                ac = 1;                              /* the audio control interface */
        } else if (ty == 5u && in_as && !ep && (d[i + 2] & 0x80u)) {    /* an endpoint, IN */
            if ((d[i + 3] & 3u) == 1u) {             /* isochronous */
                ep = (uint8_t)(d[i + 2] & 0x0Fu);
                ep_mps = (uint16_t)(d[i + 4] | d[i + 5] << 8);
                ep_if = ifnum;
                ep_alt = alt;
            }
        }
        i += bl;
    }
    if (!ac || !ep)
        return 0;
    i = 0;
    ifnum = 0;
    in_as = 0;
    while (i + 1u < len) {                           /* the PCM format of that interface */
        uint8_t bl = d[i], ty = d[i + 1];
        if (bl < 2u || i + bl > len)
            break;
        if (ty == 4u) {
            ifnum = d[i + 2];
            in_as = (d[i + 5] == 1u && d[i + 6] == 2u);
        } else if (ty == 0x24u && in_as && ifnum == ep_if && d[i + 2] == 2u && bl >= 12u && d[i + 8]) {
            /* a class-specific descriptor, the FORMAT_TYPE (subtype 2) */
            if (d[i + 3] == 1u && d[i + 4] == 1u) {  /* PCM */
                bits = d[i + 6];
                ch = d[i + 7];
                rate = (uint32_t)(d[i + 9] | d[i + 10] << 8 | d[i + 11] << 16);
                fmt = 1;
                break;
            }
        }
        i += bl;
    }
    if (!fmt)
        return 0;
    o->cfg = cfg;
    o->iface = ep_if;
    o->alt = ep_alt;
    o->ep = ep;
    o->mps = ep_mps;
    o->channels = (uint8_t)ch;
    o->bits = (uint8_t)bits;
    o->rate = rate;
    return 1;
}

/* ---------------------------------------------------------- the ring --- */

/* The packet's samples into the ring. A packet that does not hold whole frames, or a ring that is full, is dropped:
 * the stream stays frame aligned and the ring keeps its order. */
static void umic_put(const uint8_t *p, uint32_t n)
{
    uint32_t w = umic_w, room = UMIC_RING - (w - umic_r), bps, ch, frame, frames, i, s;
    if (n > UMIC_PKT) {                              /* more than an isochronous packet can hold */
        umic.drops++;
        return;
    }
    bps = (uint32_t)umic.bits / 8u;
    ch = umic.channels;
    if ((umic.bits != 16u && umic.bits != 24u) || ch < 1u || ch > 2u)
        return;
    frame = bps * ch;
    if (n < frame) {                                /* a short packet: no whole frame in it */
        umic.drops++;
        return;
    }
    frames = n / frame;
    if (frames * ch > room) {                       /* the ring is full: the packet is dropped */
        umic.drops++;
        return;
    }
    for (i = 0; i < frames; i++) {
        for (s = 0; s < ch; s++) {
            const uint8_t *b = p + i * frame + s * bps;
            int32_t v;
            if (bps == 2u)
                v = (int16_t)(uint16_t)(b[0] | (uint16_t)b[1] << 8);
            else {
                v = (int32_t)(b[0] | (uint16_t)b[1] << 8 | (uint16_t)b[2] << 16);
                if (v & 0x800000)
                    v -= 0x1000000;                 /* the sign of the 24th bit */
            }
            umic_ring[(w + i * ch + s) & UMIC_RING_MASK] = v;
        }
    }
    RING_PUBLISH();                                 /* the samples are in before the count says so */
    umic_w = w + frames * ch;
    umic.pkts++;
    umic.samples += frames * ch;
}

/* The app's read: what the device received, up to n samples. It never waits: what is not there is not
 * there. Only whole frames go out: the capture is a stream of frames, and a half frame would swap
 * the channels of the next read. */
int umic_read(int32_t *dst, uint32_t n)
{
    uint32_t r = umic_r, w = umic_w, avail = w - r, first, ch = umic.channels ? umic.channels : 1u;
    if (n > avail)
        n = avail;
    n -= n % ch;                          /* whole frames only */
    if (!n)
        return 0;
    first = UMIC_RING - (r & UMIC_RING_MASK);
    if (first > n)
        first = n;
    memcpy(dst, &umic_ring[r & UMIC_RING_MASK], first * sizeof(int32_t));
    if (n > first)
        memcpy(dst + first, &umic_ring[0], (n - first) * sizeof(int32_t));
    RING_PUBLISH();
    umic_r = r + n;
    return (int)n;
}

/* ------------------------------------------------------- the session --- */

/* The device is gone: the ring is emptied and the state goes back to none. */
static void umic_stop(void)
{
    umic.armed = 0;
    umic.errors = 0;
    umic.idle = 0;
    umic.tried = 0;
    umic_r = umic_w;
    RING_PUBLISH();
    umic.state = PLAT_MIC_NONE;
    umic.rate = 0;
    umic.channels = 0;
    umic.bits = 0;
}

/* A fault on the endpoint: the state says so, the ring is emptied (its samples are stale) and the main loop looks
 * for the device again. */
static void umic_fault(void)
{
    umic.armed = 0;
    umic.fault = 1;
    umic.tried = 0;
    umic_r = umic_w;
    RING_PUBLISH();
    umic.state = PLAT_MIC_ERROR;
}

/* The device at address 0 (just reset): give it an address, read its configuration, and if it is a microphone set it
 * up. 1 = found, 0 = not one. */
static int umic_probe(uint8_t addr)
{
    uint8_t d8[8];
    uint32_t total;
    umic_dev_t dev;
    int r;
    r = uh_ctl(0, 0x80, 6, 0x0100, 0, 8, d8, 8);    /* GET_DESCRIPTOR(device), the first 8 bytes */
    if (r < 8)
        return 0;
    umic.mps = (uint8_t)(d8[7] == 16u || d8[7] == 32u || d8[7] == 64u ? d8[7] : 8u);
    if (uh_ctl(0, 0x00, 5, addr, 0, 0, 0, umic.mps) < 0)      /* SET_ADDRESS */
        return 0;
    uh_wait(10);                                    /* the device settles to its new address */
    r = uh_ctl(addr, 0x80, 6, 0x0200, 0, 9, umic_cfg, umic.mps);
    if (r < 9)
        return 0;
    total = (uint32_t)(umic_cfg[2] | umic_cfg[3] << 8);
    if (total > sizeof umic_cfg)
        total = sizeof umic_cfg;
    if (total > 9u) {
        r = uh_ctl(addr, 0x80, 6, 0x0200, 0, (uint16_t)total, umic_cfg, umic.mps);
        if (r < 9)
            return 0;
        total = (uint32_t)r;
    }
    if (!umic_parse(umic_cfg, total, &dev))
        return 0;
    if (dev.rate != 44100u && dev.rate != 48000u)
        return 0;                                   /* a rate the platform does not carry */
    if (dev.channels < 1u || dev.channels > 2u || (dev.bits != 16u && dev.bits != 24u))
        return 0;
    if (uh_ctl(addr, 0x00, 9, dev.cfg, 0, 0, 0, umic.mps) < 0)          /* SET_CONFIGURATION */
        return 0;
    if (dev.alt && uh_ctl(addr, 0x01, 11, dev.alt, dev.iface, 0, 0, umic.mps) < 0)   /* SET_INTERFACE */
        return 0;
    umic.addr = addr;
    umic.iface = dev.iface;
    umic.alt = dev.alt;
    umic.ep = dev.ep;
    umic.ep_mps = dev.mps;
    umic.rate = dev.rate;
    umic.channels = dev.channels;
    umic.bits = dev.bits;
    return 1;
}

/* Arm host EP1 for the microphone: the bsp driver's sequence for its MIDI endpoint, with the isochronous bit in
 * RXCSR2 and the endpoint type 1 (isochronous) in RXTYPE. */
static void umic_arm(void)
{
    sie_wr(S_FADDR, umic.addr);
    sie_wr(S_INDEX, 1);
    fm1_usb_ep_rxbuf(1, umic_rx);
    sie_wr(S_RXMAXP, umic.ep_mps > 255u ? 0xFFu : (uint32_t)umic.ep_mps);   /* RXMAXP is a byte */
    sie_wr(S_RXCSR1, 0x90);                         /* clear the data toggle, flush the FIFO */
    sie_wr(S_RXCSR2, 0);
    sie_wr(S_RXTYPE, (uint32_t)(1u << 4 | umic.ep));  /* the endpoint type 1: isochronous */
    sie_wr(S_RXINTERVAL, 0);                       /* isochronous: no interval */
    sie_wr(S_RXCSR2, 0x40);                        /* the isochronous mode */
    sie_wr(S_RXCSR1, 0x20);                        /* ask for the first packet */
    fm1_usb_ep_enable(1u << 1);
    umic.errors = 0;
    umic.idle = 0;
    umic.armed = 1;
    umic.state = PLAT_MIC_CONNECTED;
}

/* A microphone behind a hub (the FM-1 gives the port no power: a powered hub it is): reset the port and probe what
 * is on it. 1 = a microphone was found there. */
static int umic_hub_port(uint8_t p)
{
    uint8_t b[4];
    uint32_t t;
    if (uh_ctl(umic.hub_addr, 0x23, 3, 4, p, 0, 0, umic.hub_mps) < 0)     /* SET_FEATURE(PORT_RESET) */
        return 0;
    t = fm1_ms;
    for (;;) {
        if (uh_ctl(umic.hub_addr, 0xA3, 0, 0, p, 4, b, umic.hub_mps) >= 4 && (b[2] & 0x10u))   /* C_PORT_RESET */
            break;
        if (fm1_ms - t > 500u)
            return 0;
        uh_wait(10);
    }
    uh_ctl(umic.hub_addr, 0x23, 1, 20, p, 0, 0, umic.hub_mps);           /* CLEAR_FEATURE(C_PORT_RESET) */
    if (uh_ctl(umic.hub_addr, 0xA3, 0, 0, p, 4, b, umic.hub_mps) < 4 || !(b[0] & 0x02u))
        return 0;                                   /* the port is not enabled */
    uh_wait(20);                                    /* reset recovery */
    return umic_probe((uint8_t)(UH_FIRST_ADDR + p));
}

/* The microphone's part of the host session: the device on the root port, or on a port of the hub the bsp driver
 * found. */
static void umic_try(void)
{
    uint8_t b[4], p;
    if (uh.hub_addr) {                              /* the root device is a hub: the microphone is on one of its ports */
        umic.hub_addr = uh.hub_addr;
        umic.hub_mps = uh.hub_mps;
        umic.hub_ports = uh.hub_ports;
        for (p = 1; p <= umic.hub_ports; p++) {
            if (!uh_hub_status(p, b))
                break;
            if (!(b[0] & 1u) || (b[1] & 2u))        /* nothing there, or low speed */
                continue;
            if (umic_hub_port(p)) {
                umic_arm();
                uh.hub_addr = 0;                    /* the driver's idle-hub poll would touch EP0 while this ISR
                                                    * takes EP1: it stops while the microphone streams */
                return;
            }
        }
        return;
    }
    sie_wr(S_POWER, 0x08u);                         /* bus reset: the device is at address 0 again */
    uh_wait(30);
    sie_wr(S_POWER, 0);
    uh_wait(30);
    if (umic_probe(UMIC_ROOT_ADDR))
        umic_arm();
}

/* The main loop: the host session, then the microphone's part of it. */
void usb_mic_task(uint32_t now)
{
    if (!uh.on || uh.run) {                          /* the port is not a host, or a MIDI controller has it */
        if (umic.state != PLAT_MIC_NONE || umic.armed)
            umic_stop();
        return;
    }
    if (umic.armed)                                  /* the microphone streams: the ISR watches the line */
        return;
    if ((int32_t)(now - umic.retry_ms) < 0)
        return;                                      /* too soon after the last attempt */
    if (umic.fault) {                                /* a fault: look for the device again, after a moment */
        umic.fault = 0;
        umic.retry_ms = now + 500u;
        return;
    }
    if (!uh.tried || umic.tried)                     /* the driver has not looked at the device yet, or we have */
        return;
    umic.tried = 1;
    umic_try();
    if (!umic.armed) {                               /* no microphone there: look again in a second */
        umic.tried = 0;
        umic.retry_ms = now + 1000u;
    }
}

/* The TIMER5 ISR: the isochronous packet in EP1 into the ring, once per USB frame. It never waits. */
void usb_mic_poll(void)
{
    uint32_t csr, n;
    if (!umic.armed)
        return;
    if (!fm1_usb_line()) {                          /* the device is gone */
        if (++umic.idle >= UMIC_IDLE)
            umic_stop();
        return;
    }
    umic.idle = 0;
    sie_wr(S_INDEX, 1);
    csr = sie_rd(S_RXCSR1);
    if (csr & 0x40u) {                              /* RxStall: the endpoint is gone */
        umic_fault();
        return;
    }
    if (csr & 0x0Cu) {                              /* Error / DataError: no usable answer */
        if (++umic.errors >= UMIC_MAX_ERRORS) {
            umic_fault();
            return;
        }
        sie_wr(S_RXCSR1, 0x20u);                    /* the flags clear and the request goes again */
        return;
    }
    if (!(csr & 0x01u)) {                           /* no packet yet */
        sie_wr(S_RXCSR1, 0x20u);                    /* ask for the next one */
        return;
    }
    n = sie_rd(S_RXCOUNT1) | sie_rd(S_RXCOUNT2) << 8;
    if (n > UMIC_PKT)
        n = UMIC_PKT;
    fm1_usb_rx_sync();
    umic_put(umic_rx, n);
    sie_wr(S_RXCSR1, 0x10u);                        /* release the packet */
    sie_wr(S_RXCSR1, 0x20u);                        /* and ask for the next one */
    if (umic.state == PLAT_MIC_CONNECTED)
        umic.state = PLAT_MIC_STREAMING;
}

/* ------------------------------------------------------- the platform --- */

int umic_state(void) { return umic.state; }
uint32_t umic_rate(void) { return umic.state == PLAT_MIC_NONE ? 0u : umic.rate; }
int umic_channels(void) { return umic.state == PLAT_MIC_NONE ? 0 : umic.channels; }
int umic_bits(void) { return umic.state == PLAT_MIC_NONE ? 0 : umic.bits; }

#else /* FM1B_USB_HOST: the port is never a host, so there is no microphone */

/* The API stays: it reports no microphone. */
int umic_state(void) { return PLAT_MIC_NONE; }
uint32_t umic_rate(void) { return 0u; }
int umic_channels(void) { return 0; }
int umic_bits(void) { return 0; }
int umic_read(int32_t *dst, uint32_t n) { (void)dst; (void)n; return 0; }
void usb_mic_task(uint32_t now) { (void)now; }
void usb_mic_poll(void) {}

#endif /* FM1B_USB_HOST */
