# RX↔APP Protocol — Revised Spec (Upload‑Map LEN‑tolerant)

This document describes the byte-level protocol between the **App ⇄ RX** device and the **RX ⇄ Slave** bus, matching the current firmware behavior—including tolerant parsing for the **Upload‑Map** command where the App transmits an **incorrect LEN**.

---

## 1) Transport & Framing

**Physical links**
- **UART0 (APP⇄RX)**: 19 200 baud, 8‑N‑1  
- **UART1 (RX⇄Slaves)**: 9 600 baud, 8‑N‑1

**Frame format (both directions)**
```
+------+-----+--------- ... ---------+------+
| SOF  | LEN |      BODY (LEN bytes) | END  |
+------+-----+------------------------+------+
 0x27   1 B          LEN bytes         0x16
```
- **SOF**: `0x27`  
- **LEN**: Number of bytes from the first BODY byte through the last BODY byte (excludes SOF/END)  
- **END**: `0x16`  
- **Checksum**: none

**BODY layout (APP⇄RX)**
```
+-------+------+----+-----------...
| GROUP |  ID  | SC |  PAYLOAD
+-------+------+----+-----------...
```
- `GROUP` = `0x85` (App→RX), `0x00` (RX→App)  
- `ID`    = `0x01` (your RX device id)  
- `SC`    = “sub‑command” selector (see §4)

---

## 2) Heartbeat (RX→App)

Sent by RX **after every valid App→RX command** and may also be sent periodically by the application layer.

**Semantics**
- Reports *presence* and *limit* state for the **configured connectors** (from Upload‑Map).
- Includes a **global FLAGS byte** (push‑button status).

**Format**
```
SOF LEN  GROUP ID  SC   FLAGS  N    S1  S2 ... SN   00 00  END
27  (N+7) 00   01  0A   FF?    N   Si … Si        (reserved)
```
- `GROUP=0x00`, `ID=0x01`, `SC=0x0A`  
- `FLAGS` (1 B): **global status**
  - `0x00` = normal
  - `0x02` = **push‑button asserted** (see §5)
- `N` (1 B): number of connectors (equals “enabled count” in Upload‑Map)  
- `Si` (N B): **per‑connector status** in the **order** provided by Upload‑Map  
  - `0x00` = not alive (no reply in last committed round)  
  - `0x05` = alive (limit **not** triggered)  
  - `0x07` = alive **and** limit triggered

> Heartbeat **doesn’t clear snapshots** when LED streaming starts/stops. Snapshots are committed only at poll‑round boundaries (see §6). While streaming, the targeted connector’s status can still update immediately from LED‑ON replies.

**Example (N=3, 1&2 alive, 3 dead, no push‑button):**
```
27 0A 00 01 0A 00 03 05 07 00 00 00 16
          ^FLAGS=00  ^N=3  S1=05 S2=07 S3=00
```

---

## 3) Slave Status (Slave→RX on UART1)

Slaves reply to poll and to LED‑ON with:
```
27 27 03 0A <ADDR> <STATE> 16
```
- `<ADDR>`: connector address (1..31)  
- `<STATE>`:
  - `0x01` = idle / limit **not** triggered (but still “alive”)  
  - `0x03` = limit **triggered** (alive)

**RX update logic**
- Set `alive` for `<ADDR>` when `<STATE> != 0x00`  
- Set/clear `triggered` for `<ADDR>` based on `<STATE>`  
- If **LED streaming is active**, apply these updates **immediately** to the committed snapshot for that connector.

---

## 4) App→RX Commands (SC codes)

### 4.1 `SC=0x04` — Upload‑Map (configure polling set & order)

Select which connectors (1..31) are in service and define their **poll/heartbeat order**.

**Conceptual (correct) payload**
```
PAYLOAD:  N  (CONN1  STATE1) (CONN2 STATE2) ... (CONNN STATEN)
          1B   1B     1B       1B     1B           1B
STATEi: 0x01 = enabled; any other value = disabled
LEN should be 4 + 2*N
```

**Actual behavior (App bug tolerated):**  
For `SC=0x04` **only**, RX **ignores LEN** and reads until `END=0x16`, then parses:
- `N = payload[0]`
- read `N` pairs `(CONN, STATE)`
- **Enable** any `CONN` whose `STATE==0x01`
- **Order** in RX = the order the pairs appear

**Examples from App (as‑sent)**

- N=3:
```
27 0A 85 01 04 03 01 01 02 01 03 01 16
      
```
Parsed → enabled: 1,2,3; order `[1,2,3]`.

- N=5:
```
27 0E 85 01 04 05 01 01 02 01 03 01 04 01 05 01 16
      
```
Parsed → enabled: 1,2,3,4,5; order `[1,2,3,4,5]`.

---

### 4.2 `SC=0x02` — LED Control

Two payload forms under the same SC:

#### (A) **Simple LED (slave board)**
```
PAYLOAD:  STATE  CONN  LED
          1B     1B    1B
STATE: 0x01 = ON (kept alive periodically), 0x00 = OFF (single broadcast OFF)
```
- `STATE=0x01` → start/maintain a **LED keep‑alive job** (RIT will periodically send `LED_ON` to `<CONN>/<LED>`).  
- `STATE=0x00` → stop that job, then **one** broadcast OFF on UART1.

**On‑wire to slave (UART1)**
- `LED_ON`: `27 97 05 85 <CONN> 02 01 <LED> 16`  
- `OFF (broadcast)`: `27 97 04 85 FF 03 00 16`

#### (B) **Addressable WS2812B (B1 on P3.25/P3.26)**

App frame examples you provided:
```
27 0C 85 01 02 00 00 00 02 <BUS> <LED#> 00 00 00 16
                   ^            ^BUS  ^LED#
```
- `BUS=0x01` → B1 (P3.25) (P3.26) `BUS=0x0N` → for other   
- `LED#` = 1‑based LED index on that strip  
- RX updates its **WS2812B framebuffer** and schedules a flush in the display service window.  
- The simple LED **OFF broadcast** also clears addressable LEDs for consistency.

*(The three padding `0x00` values are ignored.)*

---

### 4.3 `SC=0x3A` — LED Reset (all simple LEDs OFF)
```
PAYLOAD: —
Effect : Stop all LED keep‑alive jobs, issue one broadcast OFF.
```
**App example (full frame):**
```
27 04 85 01 3A 00 16
```

---

### 4.4 `SC=0x06` — Relay Set
```
PAYLOAD: RELAY  STATE
         1B     1B
STATE: 0x01=ON, else OFF

Turn on R1 27 05 85 01 06 01 01 16
Turn off R2 27 05 85 01 06 02 00 16
```

---

### 4.5 `SC=0x00` — Poll/No‑op
- No payload needed; RX replies with a heartbeat (see §2).

---

### 4.6 `SC=0x09` — Reset Push‑Button Flag
```
PAYLOAD: 00
Effect : Set Heartbeat FLAGS back to 0x00 (clears 0x02).
Example: 27 04 85 01 09 00 16
```
(Upload‑Map also clears the push‑button flag—see §5.)

---

## 5) Push‑Button Flag (P2[3], interrupt‑driven)

- **Hardware**: `P2[3]` with pull‑up; falling‑edge IRQ asserts a **latch**.  
- **On press**: RX sets the **Heartbeat FLAGS** byte to `0x02` (persists across heartbeats).  
- **To clear**: App sends **SC=0x09** (`27 04 85 01 09 00 16`) **or** Upload‑Map (SC=0x04); RX then sets FLAGS to `0x00`.

**Heartbeat example with push‑button asserted (N=2, both alive & not triggered):**
```
27 09 00 01 0A 02 02 05 05 00 00 16
          ^FLAGS=02 ^N=2  S1=05 S2=05
```

---

## 6) Polling & Snapshot Commit (avoid “momentary 0”)

- RX performs **round‑robin polling** of connectors in the **exact Upload‑Map order**: `c1, c2, …, cN`.  
- A **round** = one pass through all `N`.  
- At the **start of a new round**, RX **commits** the previous round’s accumulated replies into:
  - `g_alive_mask`  (presence)
  - `g_triggered_mask` (limit‑triggered; only meaningful where alive)
- **LED streaming active** ⇒ **pause general polling**, but **do not clear** snapshots.  
  - The targeted connector continues to update **immediately** via its LED‑ON replies.
- When streaming ends (LED‑OFF), polling **resumes from the first configured connector**; snapshots remain until the next round commit.

> If the App samples exactly between the last streamed reply and the next poll commit, it may momentarily read stale info for non‑streamed nodes. Firmware minimizes this; the App can re‑sample on `SC=0x3A/0x02` completion if needed.

---

## 7) Command Quick‑Reference

| Direction | Purpose                   | SC   | Typical BODY (hex)                             | Notes |
|---|---|---|---|---|
| App→RX | Upload‑Map                  | 0x04 | `85 01 04 N (c1 01) … (cN 01)`                | reads N and starts roundrobin on UART1 for con alive state. |
| App→RX | LED Control (simple)        | 0x02 | `85 01 02 STATE CONN LED`                      | `STATE: 01=ON, 00=OFF` |
| App→RX | LED Control (WS2812B)       | 0x02 | `85 01 02 00 00 00 02 BUS LED# 00 00 00`      | `BUS: 1=B1 for GPIO led`; paddings ignored |
| App→RX | LED Reset (all OFF)         | 0x3A | `85 01 3A` + `00`                              | Stops jobs, broadcast OFF led to all |
| App→RX | Relay Set                   | 0x06 | `85 01 06 RELAY STATE`                         | `STATE: 01=ON, else OFF` |
| App→RX | Poll/No‑op                  | 0x00 | `85 01 00` + `00`                              | RX replies heartbeat |
| App→RX | Reset Push‑Button Flag      | 0x09 | `85 01 09 00`                                   | Heartbeat FLAGS ← `0x00` ie reset sw2 state from 02 to 00|
| RX→App | Heartbeat                   | 0x0A | `00 01 0A FLAGS N S1…SN 00 00`                 | `Si: 00/05/07`; FLAGS `00`/`02` Si 00 dead con, 05 alive, 07 limit triggered|
| Slave→RX | Status reply              | —    | `27 27 03 0A <ADDR> <STATE> 16`                | if state is 01 it is alive if 03 limit sw is triggered |
| RX→Slave | Get Status                | —    | `27 97 05 85 <con> 00 00 00 16`                | con = 1,2,3... n and send this hex in UART1 and wait for reply |

*(For full frames, wrap BODY with `27 <LEN> … 16`. For Upload‑Map, the App’s LEN is ignored by RX.)*

---

## 8) Worked Examples

### A) Upload‑Map (App sends wrong LEN), then Heartbeat
App (N=3: `1,2,3` enabled):
```
27 09 85 01 04 03 01 01 02 01 03 01 16
```
RX tolerates LEN → order `[1,2,3]`.

Heartbeat (1 alive/no‑limit, 2 alive+limit, 3 dead; no push‑button):
```
27 0A 00 01 0A 00 03 05 07 00 00 00 16
```

### B) LED ON simple, then OFF
LED ON (state=1) for `conn=2, led=3`:
```
27 06 85 01 02 01 02 03 16
```
RX streams; slave 2 replies; heartbeat shows S2 `05/07` accordingly.

LED OFF (state=0) for `conn=2, led=3`:
```
27 06 85 01 02 00 02 03 16
```
RX stops job, one broadcast OFF; polling resumes at first connector, snapshots preserved until next commit.

### C) WS2812B (B1, LED#3 on)
```
27 0C 85 01 02 00 00 00 02 01 03 00 00 00 16
                   ^            ^BUS=1   ^LED#=3
```

### D) Push‑button assert & clear
- Press P2[3] → Heartbeat `FLAGS=0x02`
- Clear via App:
```
27 04 85 01 09 00 16
```
Next heartbeats: `FLAGS=0x00`.

---

## 9) Timing Summary (firmware behavior)

- **RIT scheduler** (~70–100 ms tick)
  - Sends keep‑alive `LED_ON` frames for active simple‑LED jobs (per‑job rate‑limited).
  - **Any** LED job active ⇒ **pause** general polling; snapshots **not** cleared. Targeted connector continues to update.
  - When LED jobs stop, polling **restarts from first connector**; previous snapshot persists until next round commit.
- **Poll commit**: At the **start** of a new round, RX commits the previous round’s accumulations.

---

## 10) Robustness Notes

- **Upload‑Map compatibility**: Only for `SC=0x04`, RX **ignores LEN** and parses to `END`. All other commands honor LEN.  
- **No ghost clears**: LED OFF / LED Reset don’t zero the heartbeat.  
- **Immediate updates while streaming**: Currently streamed connector’s status updates as soon as its reply is parsed.

