/**
 * Spi_Protocol.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Pack / Unpack / Checksum for the 8-byte SPI IPC frame.
 *
 *  [REDESIGN] Byte[2] now carries direction|targetFloor instead
 *  of direction|cabinRequests.  Byte[3] carries cabinRequests.
 */

#include "Spi_Protocol.h"

/* ------------------------------------------------------------------ */
uint8 SpiProto_Checksum(const uint8 *buf, uint8 len) {
    uint8 cs = 0;
    uint8 i;
    for (i = 0; i < len; i++) {
        cs ^= buf[i];
    }
    return cs;
}

/* ------------------------------------------------------------------ */
void SpiProto_Pack(const SpiFrameData *data, uint8 *buf) {
    buf[0] = SPI_FRAME_HEADER;
    buf[1] = (uint8)((data->state << 4) | (data->currentFloor & 0x0F));
    buf[2] = (uint8)((data->direction << 4) | (data->targetFloor & 0x0F));
    buf[3] = data->cabinRequests;
    buf[4] = data->assignedCalls;
    buf[5] = data->flags;
    buf[6] = data->sequence;
    buf[7] = SpiProto_Checksum(buf, 7);
}

/* ------------------------------------------------------------------ */
boolean SpiProto_Unpack(const uint8 *buf, SpiFrameData *data) {
    /* Validate header */
    if (buf[0] != SPI_FRAME_HEADER) {
        return FALSE;
    }

    /* Validate checksum */
    uint8 expected = SpiProto_Checksum(buf, 7);
    if (buf[7] != expected) {
        return FALSE;
    }

    data->state         = (buf[1] >> 4) & 0x0F;
    data->currentFloor  =  buf[1]       & 0x0F;
    data->direction     = (buf[2] >> 4) & 0x0F;
    data->targetFloor   =  buf[2]       & 0x0F;
    data->cabinRequests =  buf[3];
    data->assignedCalls =  buf[4];
    data->flags         =  buf[5];
    data->sequence      =  buf[6];

    return TRUE;
}
