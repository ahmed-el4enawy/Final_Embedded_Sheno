/**
 * Spi_Protocol.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Fixed-length 8-byte SPI IPC frame definition,
 *  packing / unpacking, and checksum helpers.
 *
 *  [REDESIGN] Added targetFloor field and ACK flag for reliable comms.
 */

#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include "Std_Types.h"

/* ============================================================ */
/*  Frame constants                                              */
/* ============================================================ */
#define SPI_FRAME_LEN        8U
#define SPI_FRAME_HEADER     0xA5U

/* ============================================================ */
/*  Direction / call encoding                                    */
/* ============================================================ */
#define DIR_NONE    0U
#define DIR_UP      1U
#define DIR_DOWN    2U

/* ============================================================ */
/*  Elevator state encoding  (fits in 4 bits)                   */
/* ============================================================ */
#define PKT_STATE_IDLE          0U
#define PKT_STATE_MOVING_UP     1U
#define PKT_STATE_MOVING_DOWN   2U
#define PKT_STATE_DOORS_OPEN    3U
#define PKT_STATE_EMERGENCY     4U
#define PKT_STATE_DOOR_CLOSING  5U

/* ============================================================ */
/*  Flag bits  (byte 5)                                         */
/* ============================================================ */
#define FLAG_EMERGENCY      (1U << 0)
#define FLAG_COMM_FAULT     (1U << 1)
#define FLAG_DOOR_OPEN      (1U << 2)
#define FLAG_ACK            (1U << 3)   /* [REDESIGN] ACK for last received frame */

/* ============================================================ */
/*  Hallway call bitmask encoding (6 calls → 6 bits)            */
/*  Bit 0 = U1, Bit 1 = D2, Bit 2 = U2,                       */
/*  Bit 3 = D3, Bit 4 = U3, Bit 5 = D4                        */
/* ============================================================ */
#define HALL_U1   (1U << 0)
#define HALL_D2   (1U << 1)
#define HALL_U2   (1U << 2)
#define HALL_D3   (1U << 3)
#define HALL_U3   (1U << 4)
#define HALL_D4   (1U << 5)

/* ============================================================ */
/*  Parsed frame structure                                       */
/* ============================================================ */
/**
 * Frame layout (8 bytes):
 *  [0] Header           0xA5
 *  [1] State|Floor       upper nibble = FSM state, lower = current floor (1-4)
 *  [2] Direction|Target  upper nibble = direction, lower = targetFloor (0-4) [REDESIGN]
 *  [3] CabinRequests     bitmask of cabin requests (bit0=F1..bit3=F4)
 *  [4] AssignedCalls     floor bitmask (master→slave: assignments)
 *  [5] Flags             emergency, comm-fault, door-open, ACK
 *  [6] Sequence          rolling counter for freshness
 *  [7] Checksum          XOR of bytes 0..6
 */
typedef struct {
    uint8 state;           /* PKT_STATE_* */
    uint8 currentFloor;    /* 1 – 4       */
    uint8 direction;       /* DIR_*       */
    uint8 targetFloor;     /* [REDESIGN] 0=none, 1-4 = committed target */
    uint8 cabinRequests;   /* bitmask     */
    uint8 assignedCalls;   /* bitmask     */
    uint8 flags;           /* FLAG_*      */
    uint8 sequence;
} SpiFrameData;

/* ============================================================ */
/*  API                                                          */
/* ============================================================ */

/**
 * @brief  Pack a SpiFrameData into an 8-byte raw buffer.
 */
void SpiProto_Pack(const SpiFrameData *data, uint8 *buf);

/**
 * @brief  Unpack an 8-byte raw buffer into a SpiFrameData.
 * @return TRUE if header and checksum are valid, FALSE otherwise.
 */
boolean SpiProto_Unpack(const uint8 *buf, SpiFrameData *data);

/**
 * @brief  Compute XOR checksum of buf[0..len-1].
 */
uint8 SpiProto_Checksum(const uint8 *buf, uint8 len);

#endif /* SPI_PROTOCOL_H */
