# USB audio: the microphone capture, the host port and the power

**The PRD's Phase 1.** ANATOMIX VOXINATOR takes its input from an external USB microphone plugged into the
FM-1's USB port. Before any of the vocal engine can be written, the PRD asks six questions. This file answers
them with the evidence, and says plainly what still needs a physical FM-1.

Short version: **the AC791N's USB controller can act as a host and its SIE already speaks isochronous
endpoints, so a USB Audio Class microphone is feasible. The FM-1's port drives no VBUS, so the microphone must
be powered from outside — a powered hub is required. The capture is written and model-tested; the silicon is
not verified.**

What landed, all in the platform layer (`base/`), nothing in `bsp/`:

| Where | What |
|---|---|
| `include/plat.h` | the API: `plat_mic_state`, `plat_mic_rate`, `plat_mic_channels`, `plat_mic_bits`, `plat_mic_read` |
| `device/usb_mic.c` | the capture: the UAC descriptor walk, the isochronous IN endpoint, the ring, the state machine |
| `device/plat_fm1.c` | the platform's five functions over the capture |
| `device/unity.c` | the capture in the unity build, after `usb.c` and before `plat_fm1.c` |
| `device/main_fm1.c` | the capture's main-loop task and its TIMER5 service |
| `host/plat_host.c` | the simulator: `wavin FILE` feeds the app the way the microphone would |
| `tests/usb_mic_test.c` | the capture against a model of the SIE and of a UAC microphone |
| `docs/USB_AUDIO.md` | this file |

The app sees nothing but the five functions. The simulator's `wavin` is the same data path as the device's, so
the app can be written and tested before the hardware exists.

---

## 1. Does the AC791N USB controller support USB host functionality?

**Yes.** The controller has a host-mode SIE with its own register file, and fm1-base already drives it.

| Evidence | Where |
|---|---|
| The host mode's register set (`DEVCTL`, `TXTYPE`, `TXINTERVAL`, `RXTYPE`, `RXINTERVAL`) | `bsp/chip/usb_host.c:29` |
| The host PHY set-up (D+/D- as inputs with their pull-downs, no pull-ups, the PHY on, the SIE out of reset) | `bsp/chip/fm1_usb.h`, `fm1_usb_host_attach()` |
| The register sequences follow the AC79 SDK's own USB-host code (`usb_h_sie_init`, `usb_h_chirp_and_reset`, `usb0_h_ep_read`, `usb_ctlXfer`) | `bsp/chip/usb_host.c` (the header comment) |
| A working host port: USB-MIDI controllers and synths, directly or behind one hub | `bsp/docs/USB_HOST.md` |

The host port boots as a *device* (so the update paths always work) and the application asks for host mode from
the main loop (`plat_usb_host(1)`). The microphone capture needs that: the port is either the computer's or the
microphone's, never both.

## 2. Does the FM-1's USB connector expose the required host-mode hardware?

**Yes, the same connector does both.** USB 2.0's OTG-style A/B roles share the connector; the FM-1's USB port
drives the PHY as an A-device when it is the host. The board needs no change for the data path.

The one thing the board does **not** have is a VBUS driver — see §4.

## 3. Can USB Audio Class microphones be enumerated?

**In software, yes — the code exists and is tested against the SIE model. On silicon, not yet.**

The port was MIDI-only: `uh_parse_midi()` (`bsp/chip/usb_host_desc.h`) accepts only endpoint types 2 (bulk)
and 3 (interrupt), and the RX service loop uses the request/release handshake that isochronous endpoints do
not use. `bsp/` is a submodule with its own history, so the microphone's part of the host session is written in
the platform layer instead: `device/usb_mic.c`, in the same compilation unit as `usb.c` (so it reaches the SIE
helpers and the host driver's own statics).

The capture does this:

* the configuration descriptor is walked for an **AudioControl** interface (class 1, subclass 1) and an
  **AudioStreaming** interface (class 1, subclass 2) with an **isochronous IN endpoint** (the endpoint type 1),
  in one of its alternate settings;
* the **AS format descriptor** (`FORMAT_TYPE`) gives the sample rate, the channel count and the bits per
  sample: 1 or 2 channels, 16 or 24 bits, and a rate of 44100 or 48000;
* the device is configured (`SET_CONFIGURATION`) and its streaming interface selected (`SET_INTERFACE`);
* the **isochronous IN endpoint is armed with the SIE's ISO-mode bit, `S_RXCSR2` bit 6** (`0x40`)**, the same
  bit the device-side USB-audio OUT path already uses** (`bsp/chip/usb.c`, `uo_ep4_reset()`). The silicon and the
  register file already spoke isochronous endpoints; only the host driver never enabled them;
* the endpoint is serviced **once per USB frame** in the TIMER5 interrupt (2 kHz), and the PCM lands in a
  **lock-free single-producer / single-consumer ring** (1024 samples, in the pool).

**The platform's API** (`include/plat.h`, implemented in `device/plat_fm1.c` over the capture):

```c
enum { PLAT_MIC_NONE = 0, PLAT_MIC_CONNECTED = 1, PLAT_MIC_STREAMING = 2, PLAT_MIC_ERROR = 3 };
int plat_mic_state(void);
uint32_t plat_mic_rate(void);       /* the device's sample rate (44100 or 48000); 0 = no device */
int plat_mic_channels(void);        /* 1 or 2 */
int plat_mic_bits(void);            /* 16 or 24 */
int plat_mic_read(int32_t *dst, uint32_t n);   /* up to n samples, interleaved as the device sends them; the count read */
```

`plat_mic_read` is called from the app's audio interrupt: it is a bounded copy out of the ring, never a wait,
never an allocation and never a flash read.

**How the two drivers share the port.** The host session belongs to the bsp's MIDI driver: it resets the bus,
reads the descriptor and knows the hubs. The capture waits until that driver has looked at the device and found
no MIDI in it (a MIDI controller keeps the MIDI path, and the microphone stays `PLAT_MIC_NONE`), and then takes
the session over: the descriptor walk, the configuration, the arming. A microphone behind a hub is found on the
hub's ports (the port is reset and the device on it probed, as the MIDI driver does for its own devices). While
the microphone streams, the driver's idle-hub poll is stood down: it would touch EP0 while the TIMER5 interrupt
owns EP1, and the SIE's index register is shared.

**What the capture does not do.** It cannot *change* the device's rate: the host driver has control transfers
with an IN data stage only, so there is no way to send the `SET_CUR` that picks a sampling frequency. The
capture therefore takes the rate the AS format descriptor offers first (the device's default at power-up) and
carries 44100 and 48000. A device whose first rate is something else is refused, not misread.

## 4. Can microphone power be supplied?

**No. This is the hard constraint of Phase 1, and it is a hardware fact, not a software one.**

`bsp/chip/fm1_usb.h` says it outright, in the host-mode set-up:

> *"Nothing here drives VBUS: the port needs power from outside."*

There is no USB power switch in the FM-1's board files, and the "VBUS" bit in `USB_CON0` is the controller's
*VBUS-valid* status input, not a power driver.

**Consequence:** a bus-powered USB microphone plugged straight into the FM-1 **will not work** — it gets no
5 V. The instrument needs one of:

1. **A powered USB hub** between the microphone and the FM-1 (the host driver supports one hub, up to seven
   ports, and it is the arrangement the existing MIDI host already documents). *This is the recommended route:
   it is one cheap accessory and it needs no hardware change.*
2. A microphone with its own power and a self-powered USB interface — rare, and many of them still want VBUS
   for the PHY.
3. A hardware revision of the FM-1 with a VBUS switch and the current limit for it. Out of scope for the
   firmware, and the board is not ours to change.

The base's own MIDI host documentation already tells its users the same thing ("The PHY setup does not supply
5 V VBUS. Use a powered hub or powered adapter"), so this is the platform's established position.

## 5. Can isochronous USB audio transfers be received reliably?

**The mechanism is implemented and exercised in the SIE model. The silicon is untested.**

What the model proves (`tests/usb_mic_test.c` drives the production capture against a modelled SIE and a
modelled UAC microphone, under ASan/UBSan):

* the descriptor walk finds the microphone's streaming interface, its isochronous IN endpoint and its PCM format;
* the endpoint is armed with the ISO bit (`S_RXCSR2 & 0x40`) and the endpoint type 1 in `S_RXTYPE`;
* the packets reach the ring in order, and `plat_mic_read` hands them to the app in order;
* the ring survives a short packet (no whole frame in it), an oversized packet and its own wrap;
* a disconnect flushes the ring and the state returns to `PLAT_MIC_NONE`; a reconnect takes the device again;
* a stall on the endpoint is an error (`PLAT_MIC_ERROR`), not a stream.

What the model cannot prove, and what a person with an FM-1 must test: see **NOT VERIFIED ON HARDWARE** below.

## 6. Can the firmware process incoming PCM without disrupting the existing audio engine?

**Yes, by construction.**

* The capture is a ring buffer written by the USB host's timer interrupt and read by the app's audio
  interrupt. The two sides never block each other: the reader takes whatever is there, an empty ring means
  silence, and a full ring drops the packet instead of overwriting (the drop is counted).
* The app's conversion stage (the device's rate and depth to the engine's 44.1 kHz float) is the app's own
  code; the platform delivers the samples as the device sent them (16-bit stays in ±32767, 24-bit in ±8388607).
* The render's budget is unchanged: the engine's block time is unchanged by the capture, and the platform's
  `plat_cpu_pct()` / `plat_xruns()` show it.

---

## NOT VERIFIED ON HARDWARE

Everything below needs an FM-1, a **powered** hub and a USB Audio Class microphone. None of it can be settled
by the model in `tests/usb_mic_test.c`.

**The setup**

1. Build a firmware with `usb_host = true` in `fm1.toml` (`FM1B_USB_HOST=1`), flash it into the FM-1.
2. Plug a **powered** USB hub into the FM-1's USB port and a UAC microphone into the hub.
3. Put the port into host mode (the app's `plat_usb_host(1)`; the app's own switch drives it) and watch the
   app's INPUT screen: the microphone's state, its rate, its channels and its bits.

**The steps**

| # | What | What must happen |
|---|---|---|
| 1 | The power | The microphone lights up (if not: the hub is not powering its ports, or the cable is charge-only). |
| 2 | The enumeration | The state goes `NONE` → `CONNECTED` within a second, and the rate, the channels and the bits are the microphone's own. |
| 3 | The stream | The state goes to `STREAMING`; sing into the microphone and the input's level moves. |
| 4 | The rate | A 48 kHz microphone is resampled to 44.1 kHz without an audible pitch error (a sustained note against a reference). |
| 5 | The endpoint's packet size | If the microphone's packets are larger than 255 bytes (the usual 512 or 1023), the SIE's `RXMAXP` is a **byte** and the capture can only take 255 of them. Either the packets arrive whole, or the audio breaks — this is the first thing to look at if the stream is noisy. |
| 6 | The per-frame service | The capture asks for the next packet after every one it takes (`S_RXCSR1` `ReqPkt`). If the SIE wants that only once, in isochronous mode, the stream stops after the first packet — the symptom is one short burst of audio and then silence. |
| 7 | The endurance | Leave it running for ten minutes: no clicks, no `plat_xruns()`, and the render's share of the CPU (`plat_cpu_pct()`) unchanged. |
| 8 | The unplug | Pull the microphone mid-word: the state goes to `NONE`, the input to silence, and the app falls back to its idle. Plug it back in: the state returns to `STREAMING` within a second. |
| 9 | The hub | Repeat 1-8 behind the hub (the capture finds the microphone on the hub's ports). One hub level only; a nested hub is not supported. |
| 10 | The interference | With the microphone running, the FM-1 is a USB *device* to a computer and a USB *host* to the microphone at the same time — impossible on one port: the computer's connection and the host mode are exclusive. |

**What the model cannot check at all:** the real silicon's packet delivery, the USB clock's jitter against the
FM-1's 44.1 kHz audio clock, the bus errors and retries on the wire, and the interrupt deadline when the
TIMER5 service and the ALNK0 render meet.

---

## The honest summary

| PRD question | Answer |
|---|---|
| 1. USB host functionality | Yes |
| 2. Host-mode hardware on the connector | Yes (same connector) |
| 3. UAC microphones enumerated | The code exists and is model-tested; the silicon is unverified |
| 4. Microphone power | **No — a powered hub is required** |
| 5. Isochronous transfers | Implemented; model-tested only |
| 6. Without disrupting the audio engine | Yes, by construction (a ring, never a wait) |

Phase 1 is therefore **conditionally passed**: the firmware work is done and testable on this machine; the
hardware acceptance needs an FM-1, a powered hub and a microphone.
