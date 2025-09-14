# RX Device Replica — Protocol & Code Documentation

This document explains the structure and behavior of the RX-device replica implemented in **`main.c`** (cleaned-up version), including protocol details, buffers, the RX frame parser, and each function’s role.

---

## Table of Contents

- [Overview](#overview)
- [Wire Protocol](#wire-protocol)
  - [Frame Layout](#frame-layout)
  - [App → RX Commands](#app--rx-commands)
  - [RX → App Replies](#rx--app-replies)
- [Buffers, Limits, and Sizing](#buffers-limits-and-sizing)
- [Source Layout](#source-layout)
- [Runtime Flow](#runtime-flow)
- [Function Reference](#function-reference)
- [Typical Sequences](#typical-sequences)
- [Error Handling & Safety](#error-handling--safety)
- [Porting & Build Notes](#porting--build-notes)
- [Extending the Stub](#extending-the-stub)

---

## Overview

The program emulates an **RX device** that talks to an **App** over **UART0** and (optionally) to **slave LED boards** over **UART1**.

- **App → RX (`GROUP = 0x85`)**: poll/heartbeat, upload-map, LED control, LED reset, relay control.  
- **RX → App (`GROUP = 0x00`)**: status frames (variable-length, reflecting the current configured connectors).

Internally the RX replica:

- Stores the **configured connector list** (`cfg_conn`, length `cfg_count`) upon **Upload-Map**.  
- Replies to **Poll** with a **Status** frame that includes one **per-connector presence byte** (`0x05` = present, `0x00` = absent) for each configured connector. Presence is currently driven by a static `alive_list`.  
- Applies **LED control** via placeholders (`led_on/off`), ready to be wired to UART1 later.  
- Applies **Relay set** via actual GPIO writes (implemented in `board.c`).

---

## Wire Protocol

### Frame Layout

All frames use this layout (both directions):

```
[0x27] [LEN] [GROUP] [RX_ID] [SUBCMD] [PAYLOAD ...] [0x16]
```

- `0x27` = start (SOF)  
- `LEN`   = number of bytes from `[GROUP]` through the **last payload byte** (excludes SOF, LEN, END)  
- `GROUP` = `0x85` (App→RX) or `0x00` (RX→App)  
- `RX_ID` = which RX unit (this code uses `0x01`)  
- `SUBCMD` = command code  
- `0x16` = end (END_BYTE)

### App → RX Commands

| Subcmd | Hex  | Meaning                      | Payload (App→RX)                         | Notes |
|:------:|:----:|------------------------------|------------------------------------------|------|
| Poll   | `00` | Heartbeat / status request   | `00`                                     | RX replies with Status |
| LED    | `02` | LED control                  | `[state] [connector] [led#] [00 00 00]`  | Only first 3 are used by stub |
| Map    | `04` | Upload connector map         | `N [conn1 state1] … [connN stateN]`      | Keep `state==0x01` only |
| Reset  | `3A` | LED reset / clear LEDs       | `00`                                     | Clears LEDs in real device (no-op here) |
| Relay  | `06` | Relay set                    | `[relay#] [0/1]`                         | Drives GPIO relays |

**Upload-Map length rule:**  
`LEN = 4 + 2*N` (for `N` connector pairs).

### RX → App Replies

**Status (`SC_STATUS = 0x0A`)**

```
27 (N+7) 00 RX_ID 0A  flags  N  S1 S2 ... SN  00 00  16
```

- `N` = number of **configured** connectors (post Upload-Map), capped by `MAX_CFG`.
- `Si` = per-connector presence (`0x05` present / `0x00` absent).  
- Length: `LEN = N + 7`, on-wire total: `LEN + 3 = N + 10`.

---

## Buffers, Limits, and Sizing

```c
#define MAX_CFG         31                 // hardware max
#define RX_LEN_MAX      (4 + 2*MAX_CFG)    // largest incoming LEN (Upload-Map)
#define TX_FRAME_MAX    (MAX_CFG + 10)     // largest outgoing status frame on wire
```

- **Why `RX_LEN_MAX`**: Upload-Map is largest incoming (`LEN = 4 + 2*N`), worst `N=31` → `LEN=66`.  
- **Why `TX_FRAME_MAX`**: Status is largest outgoing (on-wire bytes = `N + 10`, worst `N=31` → `41`).

The RX frame parser rejects frames with `LEN == 0` or `LEN > RX_LEN_MAX` and resets on overflow.

---

## Source Layout

- **`board_sysinit.c`** — Pin mux and clocking (UART0 P0.2/P0.3, UART1 P2.0/P2.1, relay GPIOs).  
- **`board.c`** — GPIO relay setup and control (`Board_Relays_Init`, `Board_Relay_Set`).  
- **`main.c`** — Protocol, parser, command handlers, and UART runtime loop.

---

## Runtime Flow

1. **UART0 ISR** ingests bytes from the App and assembles frames with a tiny state machine.  
2. On a full, valid frame, **`dispatch_body()`** routes by `SUBCMD`.  
3. The stub updates state (`cfg_conn/cfg_count`), toggles relays/LEDs, and queues **status** frames as needed.  
4. The **main loop** sends any queued reply via `Chip_UART_SendBlocking` and returns to idle (`__WFI`).

---

## Function Reference

### Presence & LED helpers

- **`bool is_alive(uint8_t conn)`**  
  Returns whether the given connector is reported as present (`0x05`) using the static `k_alive_list`.

- **`void led_on(uint8_t con, uint8_t led)` / `void led_off(uint8_t con, uint8_t led)`**  
  Placeholders for slave LED bus control (wire to UART1 later).

### Outgoing status

- **`uint8_t build_status_frame(uint8_t *dst, uint8_t cap)`**  
  Builds a full **Status** frame into `dst`. Returns on-wire byte count (including SOF/LEN/END), or `0` if it won’t fit.

- **`void send_status(void)`**  
  Builds Status into the global TX buffer and marks it for sending in the main loop.

### Command handlers

- **`void handle_upload_map(const uint8_t *pay, uint8_t paylen)`**  
  Parses Upload-Map: `N` followed by `N` pairs `[connector, state]`.  
  Keeps only entries with `state == 0x01`, up to `MAX_CFG`, preserving order.  
  Sends Status immediately after updating `cfg_conn/cfg_count`.

- **`void handle_led_ctrl(const uint8_t *pay, uint8_t paylen)`**  
  Parses LED control: `[state][connector][led]` and calls the LED placeholder.

- **`void handle_led_reset(const uint8_t *pay, uint8_t paylen)`**  
  Placeholder for global LED clear (`0x3A`). No-op in stub.

- **`void handle_relay_set(const uint8_t *pay, uint8_t paylen)`**  
  Parses `[relay#][0/1]` and calls `Board_Relay_Set` (drives GPIO).

### Parser & ISR

- **`void rx_ingest(uint8_t b)`**  
  Byte-by-byte parser FSM with four states: `WAIT_SOF → WAIT_LEN → COLLECT_BODY → WAIT_END`.  
  Resets on overflow or invalid length.

- **`void dispatch_body(const uint8_t *p, uint8_t len)`**  
  Validates `GROUP`/`RX_ID`, switches on `SUBCMD`, and calls the appropriate handler.  
  For `SC_POLL`, sends a Status.

- **`void HANDLER_APP(void)`**  
  UART0 ISR: drains RX FIFO and feeds `rx_ingest`.

### Main

- **`int main(void)`**  
  System/board init, UART0/UART1 setup, IRQ enable, and the TX-queue flush loop.

---

## Typical Sequences

### First boot (no map configured)
- **App → RX**: Poll — `27 04 85 01 00 00 16`  
- **RX → App**: Status with `N=0` — `27 07 00 01 0A 00 00 00 00 16`

### Upload map (e.g., connectors 1..3 enabled)
- **App → RX**: Upload-Map `SC=0x04` with `N=3`, pairs `[1,1][2,1][3,1]`  
- **RX**: Stores `cfg_conn={1,2,3}`, `cfg_count=3`; **immediately** sends Status with `N=3` and presence bytes from `is_alive()`.

### Poll after map
- **App → RX**: Poll  
- **RX → App**: Status (`N=3`, `S1..S3`).

### LED control
- **App → RX**: `SC=0x02`, payload `[state][conn][led]`  
- **RX**: Calls LED placeholder (`led_on/off`).

### Relay control
- **App → RX**: `SC=0x06`, payload `[relay#][0/1]`  
- **RX**: `Board_Relay_Set()` toggles the relay GPIO.

---

## Error Handling & Safety

- **Length validation**: Frames with `LEN==0` or `LEN>RX_LEN_MAX` are discarded early.  
- **Overflow guard**: If the body exceeds buffer capacity, the parser resets.  
- **Connector cap**: Only the first `MAX_CFG` enabled connectors are stored; extra pairs are ignored.  
- **Buffer sizing**: Chosen to handle worst-case Upload-Map and Status frames without overflow.

---

## Porting & Build Notes

- **UART pins**: Ensure `board_sysinit.c` muxes UART0 and UART1 correctly (e.g., `P0.2/P0.3` for UART0; `P2.0/P2.1` for UART1 on FUNC2 — verify for your board).  
- **Relays**: `board.c` maps relays to the given GPIOs and initializes them OFF.  
- **Clocks**: `Board_SetupClocking()` configures a 100 MHz core (adjust if needed).

---

## Extending the Stub

- **Real presence detection**: Replace `is_alive()` with actual checks (e.g., ping slaves via UART1) and maintain a dynamic presence table.  
- **LED reset behavior**: Implement true LED clearing in `handle_led_reset()`.  
- **Acks/Errors**: Add explicit ack/nack frames for robustness, if required.  
- **CRC**: For stronger integrity, add a checksum/CRC and validate in the parser.

---

*End of document.*
