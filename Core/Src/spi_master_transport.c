#include "spi_master_transport.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2; // Von CubeMX in main.c generiert

static uint8_t compute_checksum(const uint8_t *buf, size_t len)
{
    uint8_t xor = 0;
    for (size_t i = 0; i < len; i++) {
        xor ^= buf[i];
    }
    return xor;
}

HAL_StatusTypeDef SPI_Master_SendState(const float *obs, uint8_t seq)
{
    spi_state_frame_t tx_frame = {0};

    // Frame zusammenbauen
    tx_frame.fields.cmd = SPI_CMD_STATE;
    tx_frame.fields.seq = seq;
    memcpy(tx_frame.fields.obs, obs, sizeof(float) * ACTOR_OBS_DIM);
    
    // Checksum berechnen (alles außer checksum und padding)
    tx_frame.fields.checksum = compute_checksum(tx_frame.raw, offsetof(spi_state_frame_t, fields.checksum));

    // CS Low (Aktiv)
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    
    // Transmit (Blockierend für kurze Payload ausreichend performant)
    HAL_StatusTypeDef res = HAL_SPI_Transmit(&hspi2, tx_frame.raw, SPI_STATE_FRAME_SIZE, 10);
    
    // CS High (Inaktiv)
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    return res;
}

HAL_StatusTypeDef SPI_Master_GetAction(float *action, uint8_t expected_seq, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    
    // Warten bis ESP32 Handshake (GPIO5) auf HIGH zieht
    while (HAL_GPIO_ReadPin(SPI_HANDSHAKE_GPIO_Port, SPI_HANDSHAKE_Pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }

    // Dummy TX-Buffer senden, um den Takt für MISO zu erzeugen. 
    // Erstes Byte ist das Kommando, falls der Slave den Input decodiert.
    uint8_t dummy_tx[SPI_ACTION_FRAME_SIZE] = {0};
    dummy_tx[0] = SPI_CMD_ACTION_REQ;

    spi_action_frame_t rx_frame = {0};

    // CS Low
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

    // Vollduplex-Transfer: Master taktet, Slave schiebt s_tx_buf raus
    HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(&hspi2, dummy_tx, rx_frame.raw, SPI_ACTION_FRAME_SIZE, 10);

    // CS High
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    if (res != HAL_OK) return res;

    // Validierung: Cmd und Sequence Number
    if (rx_frame.fields.cmd != SPI_CMD_ACTION_REQ || rx_frame.fields.seq != expected_seq) {
        return HAL_ERROR;
    }

    // Validierung: Checksum
    uint8_t expected_cs = compute_checksum(rx_frame.raw, offsetof(spi_action_frame_t, fields.checksum));
    if (rx_frame.fields.checksum != expected_cs) {
        return HAL_ERROR; // Checksummenfehler
    }

    // Nutzdaten kopieren
    memcpy(action, rx_frame.fields.action, sizeof(float) * ACTION_DIM);

    return HAL_OK;
}