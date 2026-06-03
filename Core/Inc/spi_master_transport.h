#ifndef SPI_MASTER_TRANSPORT_H
#define SPI_MASTER_TRANSPORT_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// KONFIGURATION (Muss mit ESP32 config.h übereinstimmen)
// ============================================================================
#define ACTOR_OBS_DIM               4
#define ACTION_DIM                  1

#define SPI_CMD_STATE               0xA1u
#define SPI_CMD_ACTION_REQ          0xA2u

#define SPI_STATE_FRAME_SIZE        20  // 1(cmd)+1(seq)+16(obs)+1(chk)+1(pad)
#define SPI_ACTION_FRAME_SIZE       8   // 1(cmd)+1(seq)+4(act)+1(chk)+1(pad)

// ============================================================================
// FRAME STRUKTUREN
// ============================================================================
#pragma pack(push, 1)

typedef union {
    uint8_t raw[SPI_STATE_FRAME_SIZE];
    struct {
        uint8_t  cmd;
        uint8_t  seq;
        float    obs[ACTOR_OBS_DIM];
        uint8_t  checksum;
        uint8_t  _pad[1];
    } fields;
} spi_state_frame_t;

typedef union {
    uint8_t raw[SPI_ACTION_FRAME_SIZE];
    struct {
        uint8_t  cmd;
        uint8_t  seq;
        float    action[ACTION_DIM];
        uint8_t  checksum;
        uint8_t  _pad[1];
    } fields;
} spi_action_frame_t;

#pragma pack(pop)

// ============================================================================
// API
// ============================================================================

// Sendet die Observationen an den ESP32 und triggert die Inferenz
HAL_StatusTypeDef SPI_Master_SendState(const float *obs, uint8_t seq);

// Holt synchron und ohne Wartezeit die Action. Darf nur nach dem Interrupt aufgerufen werden.
HAL_StatusTypeDef SPI_Master_FetchAction(float *action, uint8_t expected_seq);

#endif // SPI_MASTER_TRANSPORT_H