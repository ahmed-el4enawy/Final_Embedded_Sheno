# SPI IPC Packet Definition

> **Submission Requirement (Section 6):** "Packet Definition: Diagram of your 8-byte SPI frame."

## 8-Byte Frame Layout (Discussion Ready)

```text
       Byte 0    Byte 1      Byte 2      Byte 3    Byte 4    Byte 5    Byte 6    Byte 7
      +--------+-----------+-----------+---------+---------+---------+---------+---------+
      | Header |State|Floor| Dir|Cabin |Assigned |  Hall   |  Flags  |Sequence |Checksum |
      |  0xA5  |[7:4][3:0] |[7:4][3:0] |  Calls  |  Calls  |         |         |  (XOR)  |
      +--------+-----------+-----------+---------+---------+---------+---------+---------+
```

## Technical Rationale

| Design Choice | Purpose / Rationale |
|---------------|---------------------|
| **XOR Checksum** | **Data Integrity**: SPI has no built-in error checking. The XOR checksum detects single-bit flips caused by EMI noise on breadboard/jumper wires, preventing the FSM from reacting to corrupted floor requests. |
| **Sequence Counter** | **Stale Data Rejection**: Prevents the system from processing the same command twice if the Master/Slave desyncs or if the logic analyzer glitches. It ensures each received frame is "fresh". |
| **8-Byte Fixed Frame** | **Determinism**: A fixed length ensures the interrupt load on the CPU is constant and predictable. It avoids the overhead of a length byte and simplifies buffer management in bare-metal memory. |
| **SPI Full Duplex** | **Maximum Throughput**: Allows the Master to send Dispatcher assignments while simultaneously receiving the Slave's state in a single clock sequence (8 bytes total), halving the bus occupation time. |

## Per-Byte Detail

### Byte 0 — Header (Sync Marker)

```
Always 0xA5. Frames with buf[0] != 0xA5 are rejected.
Used for frame synchronization on the SPI bus.
```

### Byte 1 — State | Floor

```
  Bit:  7   6   5   4   3   2   1   0
       ┌───┬───┬───┬───┬───┬───┬───┬───┐
       │    State     │   Floor (1-4)   │
       └───┴───┴───┴───┴───┴───┴───┴───┘
         Upper nibble    Lower nibble

  State encoding:
    0 = IDLE          3 = DOORS_OPEN
    1 = MOVING_UP     4 = EMERGENCY
    2 = MOVING_DOWN   5 = DOOR_CLOSING
```

### Byte 2 — Direction | Cabin Requests

```
  Bit:  7   6   5   4   3   2   1   0
       ┌───┬───┬───┬───┬───┬───┬───┬───┐
       │  Direction   │  Cabin Reqs     │
       └───┴───┴───┴───┴───┴───┴───┴───┘
         Upper nibble    Lower nibble

  Direction: 0=NONE, 1=UP, 2=DOWN
  Cabin Reqs: bit0=F1, bit1=F2, bit2=F3, bit3=F4
```

### Byte 3 — Assigned Calls (Floor Bitmask)

```
  Bit:  7   6   5   4   3   2   1   0
       ┌───┬───┬───┬───┬───┬───┬───┬───┐
       │  reserved    │ F4│ F3│ F2│ F1│
       └───┴───┴───┴───┴───┴───┴───┴───┘

  Hall calls assigned to THIS elevator by the Dispatcher.
```

### Byte 4 — Hall Calls (Directional Bitmask)

```
  Bit:  7   6   5   4   3   2   1   0
       ┌───┬───┬───┬───┬───┬───┬───┬───┐
       │ reserved │ D4│ U3│ D3│ U2│ D2│ U1│
       └───┴───┴───┴───┴───┴───┴───┴───┘

  Master→Slave: new hall call assignments for Slave B.
  6 directional calls across 4 floors.

  HALL_U1 = bit 0    Floor 1 Up
  HALL_D2 = bit 1    Floor 2 Down
  HALL_U2 = bit 2    Floor 2 Up
  HALL_D3 = bit 3    Floor 3 Down
  HALL_U3 = bit 4    Floor 3 Up
  HALL_D4 = bit 5    Floor 4 Down
```

### Byte 5 — Flags

```
  Bit:  7   6   5   4   3   2   1   0
       ┌───┬───┬───┬───┬───┬───┬───┬───┐
       │    reserved       │DOR│CMF│EMG│
       └───┴───┴───┴───┴───┴───┴───┴───┘

  EMG (bit 0) = FLAG_EMERGENCY   — Emergency stop active
  CMF (bit 1) = FLAG_COMM_FAULT  — SPI communication fault
  DOR (bit 2) = FLAG_DOOR_OPEN   — Doors currently open
```

### Byte 6 — Sequence Counter

```
  8-bit rolling counter (0–255).
  Incremented by transmitter on each frame.
  Receiver drops frames where sequence == spiLastRxSeq (duplicate).
  Invalidated to 0xFF on comm-fault entry to prevent wrap-around collision.
```

### Byte 7 — XOR Checksum

```
  buf[7] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6]

  Frames with incorrect checksum are silently rejected.
  Checksum failures are counted in spiChecksumErrors for telemetry.
```

## Packing / Unpacking Code

```c
/* Pack (Spi_Protocol.c) */
buf[0] = 0xA5;                                           // Header
buf[1] = (state << 4) | (currentFloor & 0x0F);          // State|Floor
buf[2] = (direction << 4) | (cabinRequests & 0x0F);     // Dir|Cabin
buf[3] = assignedCalls;                                   // Assigned
buf[4] = hallCalls;                                       // Hall
buf[5] = flags;                                           // Flags
buf[6] = sequence;                                        // Sequence
buf[7] = XOR(buf[0..6]);                                 // Checksum

/* Unpack — validates header (0xA5) and checksum before extracting fields */
```

## Example: Elevator A Moving UP at Floor 2, Cabin Request for Floor 4

```
Byte:  0     1     2     3     4     5     6     7
     ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
     │ A5  │ 12  │ 18  │ 00  │ 00  │ 00  │ 03  │ BE  │
     └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
       │     │     │     │     │     │     │     └─ XOR checksum = 0xBE
       │     │     │     │     │     │     └─ Sequence #3
       │     │     │     │     │     └─ No flags
       │     │     │     │     └─ No hall calls pending
       │     │     │     └─ No assigned hall calls
       │     │     └─ Dir=UP(1)<<4 | CabinReqs=F4(0x08) = 0x18
       │     └─ State=MOVING_UP(1)<<4 | Floor=2(0x02) = 0x12
       └─ Header 0xA5
```

## Frame Integrity

| Check | Action on Failure |
|-------|-------------------|
| `buf[0] != 0xA5` | Frame rejected (not a valid frame) |
| `buf[7] != XOR(buf[0..6])` | Frame rejected (corrupted in transit) |
| `sequence == spiLastRxSeq` | Frame rejected (duplicate) |
