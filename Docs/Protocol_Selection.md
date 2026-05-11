# Protocol Selection & Justification

The dual-elevator IPC (Inter-Process Communication) link uses **SPI (Serial Peripheral Interface)**. Below is the justification for this choice compared to other common protocols.

## Comparison Table

| Protocol | Full Duplex | Synchronous | Max Speed (Typical) | Complexity | Verdict |
|----------|-------------|-------------|---------------------|------------|---------|
| **UART** | Yes | No | 115.2 kbps | Low | **Rejected**: Asynchronous nature makes frame synchronization fragile without a start/stop protocol. No hardware clock. |
| **I2C**  | No | Yes | 400 kbps | Medium | **Rejected**: Half-duplex. Master-polling overhead increases latency. Slower than SPI. |
| **SPI**  | **Yes** | **Yes** | **10+ Mbps** | **Medium** | **Selected**: Full-duplex allows simultaneous state exchange. Hardware clock (SCK) ensures deterministic timing. |

## Rationale for SPI

1. **Full-Duplex Efficiency**: Since the Master (Board A) needs to send assignments to the Slave (Board B) at the same time the Slave sends its status to the Master, SPI allows both 8-byte frames to be exchanged in a single transaction.
2. **Determinism**: SPI is a clocked protocol. There is no "baud rate drift" or sampling jitter. This is critical for high-reliability embedded systems like elevators.
3. **Hardware Support**: The STM32F401 SPI peripheral supports non-blocking interrupt-driven transfers (RXNE/TXE), allowing the CPU to perform FSM logic while the data moves over the wire.

## Frame Integrity Measures

To ensure the safety of the elevator system, the following measures are implemented over the SPI layer:
* **Sync Header (0xA5)**: Detects frame misalignment.
* **XOR Checksum**: Detects bit-flips or electrical noise on the jumpers.
* **Sequence Counting**: Prevents duplicate processing of stale frames.
* **Hardware CS (NSS)**: Explicitly defines the start and end of the 8-byte IPC packet.
