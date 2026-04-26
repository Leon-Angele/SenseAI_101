/**
 ******************************************************************************
 * @file           : servo_protocol.c
 * @brief          : Waveshare ST/SC Serial Bus Servo Protocol Implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "servo_protocol.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define SCS_MIN_RESPONSE_LENGTH     6   // Header(2) + ID + Length + Error + Checksum

/* Private variables ---------------------------------------------------------*/
static ServoProtocol_t *g_protocol = NULL;  // Global reference for callbacks

/* Private function prototypes -----------------------------------------------*/
static ServoStatus_t Servo_SendPacket(ServoProtocol_t *protocol, 
                                       const SCS_Packet_t *packet, 
                                       uint8_t param_count);
static ServoStatus_t Servo_WaitResponse(ServoProtocol_t *protocol, 
                                         uint8_t expected_length);
static void Servo_BuildPacket(SCS_Packet_t *packet, uint8_t id, 
                               uint8_t instruction, const uint8_t *params, 
                               uint8_t param_count);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize servo protocol
 */
ServoStatus_t Servo_Init(ServoProtocol_t *protocol, UART_HandleTypeDef *huart)
{
    if (protocol == NULL || huart == NULL) {
        return SERVO_ERROR_NOT_INITIALIZED;
    }
    
    memset(protocol, 0, sizeof(ServoProtocol_t));
    
    protocol->huart = huart;
    protocol->hdma_tx = huart->hdmatx;
    protocol->hdma_rx = huart->hdmarx;
    protocol->state = SERVO_STATE_IDLE;
    protocol->timeout_ms = SCS_TIMEOUT_MS;
    protocol->initialized = true;
    
    g_protocol = protocol;  // Store global reference for callbacks
    
    /* DE pin not configured by default */
    protocol->de_gpio = NULL;
    protocol->de_pin = 0;
    protocol->de_enabled = false;
    
    return SERVO_OK;
}

/**
 * @brief Ping a servo
 */
ServoStatus_t Servo_Ping(ServoProtocol_t *protocol, uint8_t id)
{
    if (!protocol->initialized) {
        return SERVO_ERROR_NOT_INITIALIZED;
    }
    
    if (protocol->state != SERVO_STATE_IDLE) {
        return SERVO_ERROR_BUSY;
    }
    
    if (id == 0 || id > SCS_MAX_SERVOS) {
        return SERVO_ERROR_INVALID_ID;
    }
    
    SCS_Packet_t packet;
    Servo_BuildPacket(&packet, id, SCS_PING, NULL, 0);

    /* Send packet via DMA and request to receive expected response length
       This function is non-blocking: it starts the transfer and returns.
       The caller should check protocol->state or use Servo_Process() and
       inspect protocol->rx_buffer when protocol->state == SERVO_STATE_RX_COMPLETE. */
    ServoStatus_t status = Servo_SendPacket(protocol, &packet, 0);
    if (status != SERVO_OK) {
        return status;
    }

    /* Expect 6 bytes response: Header(2) + ID + Length + Error + Checksum */
    protocol->rx_expected_length = 6;
    return SERVO_OK;
}

/**
 * @brief Write position to servo
 */
ServoStatus_t Servo_WritePosition(ServoProtocol_t *protocol, uint8_t id, 
                                   uint16_t position, uint16_t time)
{
    if (!protocol->initialized) {
        return SERVO_ERROR_NOT_INITIALIZED;
    }
    
    if (protocol->state != SERVO_STATE_IDLE) {
        return SERVO_ERROR_BUSY;
    }
    
    if (id == 0 || id > SCS_MAX_SERVOS) {
        return SERVO_ERROR_INVALID_ID;
    }
    
    // Clamp position to valid range
    if (position > SCS_POSITION_MAX) {
        position = SCS_POSITION_MAX;
    }
    
    // Build parameter array: Address, Position_L, Position_H, Time_L, Time_H
    uint8_t params[5];
    params[0] = SCS_ADDR_GOAL_POSITION;
    params[1] = (uint8_t)(position & 0xFF);         // Position low byte
    params[2] = (uint8_t)((position >> 8) & 0xFF);  // Position high byte
    params[3] = (uint8_t)(time & 0xFF);             // Time low byte
    params[4] = (uint8_t)((time >> 8) & 0xFF);      // Time high byte
    
    SCS_Packet_t packet;
    Servo_BuildPacket(&packet, id, SCS_WRITE, params, 5);
    
    return Servo_SendPacket(protocol, &packet, 5);
}

/**
 * @brief Read current position from servo
 */
ServoStatus_t Servo_ReadPosition(ServoProtocol_t *protocol, uint8_t id, uint16_t *position)
{
    if (!protocol->initialized) {
        return SERVO_ERROR_NOT_INITIALIZED;
    }
    
    if (protocol->state != SERVO_STATE_IDLE) {
        return SERVO_ERROR_BUSY;
    }
    
    if (id == 0 || id > SCS_MAX_SERVOS || position == NULL) {
        return SERVO_ERROR_INVALID_ID;
    }
    
    // Build parameter array: Address, Length
    uint8_t params[2];
    params[0] = SCS_ADDR_PRESENT_POSITION;
    params[1] = 2;  // Read 2 bytes (position is 16-bit)
    
    SCS_Packet_t packet;
    Servo_BuildPacket(&packet, id, SCS_READ, params, 2);
    
    ServoStatus_t status = Servo_SendPacket(protocol, &packet, 2);
    if (status != SERVO_OK) {
        return status;
    }
    
    // Wait for response: Header(2) + ID + Length + Error + Data(2) + Checksum = 8 bytes
    status = Servo_WaitResponse(protocol, 8);
    if (status != SERVO_OK) {
        return status;
    }
    
    // Extract position from response (bytes 5 and 6)
    *position = (uint16_t)(protocol->rx_buffer[5] | (protocol->rx_buffer[6] << 8));
    
    return SERVO_OK;
}

/**
 * @brief Sync write position to multiple servos
 */
ServoStatus_t Servo_SyncWritePosition(ServoProtocol_t *protocol, 
                                       const uint8_t *ids, 
                                       const uint16_t *positions,
                                       const uint16_t *times,
                                       uint8_t count)
{
    if (!protocol->initialized) {
        return SERVO_ERROR_NOT_INITIALIZED;
    }
    
    if (protocol->state != SERVO_STATE_IDLE) {
        return SERVO_ERROR_BUSY;
    }
    
    if (count == 0 || count > SCS_MAX_SERVOS || ids == NULL || positions == NULL) {
        return SERVO_ERROR_INVALID_ID;
    }
    
    SCS_Packet_t packet;
    packet.header1 = SCS_HEADER;
    packet.header2 = SCS_HEADER;
    packet.id = SCS_BROADCAST_ID;
    packet.instruction = SCS_SYNC_WRITE;
    
    // Parameters: Address, Data_Length, [ID, Data...]...
    uint8_t param_idx = 0;
    packet.params[param_idx++] = SCS_ADDR_GOAL_POSITION;  // Starting address
    packet.params[param_idx++] = 4;  // Data length per servo (Position + Time = 4 bytes)
    
    for (uint8_t i = 0; i < count; i++) {
        uint16_t pos = positions[i];
        uint16_t t = (times != NULL) ? times[i] : 0;
        
        // Clamp position
        if (pos > SCS_POSITION_MAX) {
            pos = SCS_POSITION_MAX;
        }
        
        packet.params[param_idx++] = ids[i];
        packet.params[param_idx++] = (uint8_t)(pos & 0xFF);
        packet.params[param_idx++] = (uint8_t)((pos >> 8) & 0xFF);
        packet.params[param_idx++] = (uint8_t)(t & 0xFF);
        packet.params[param_idx++] = (uint8_t)((t >> 8) & 0xFF);
    }
    
    packet.length = param_idx + 2;  // params + instruction + checksum
    
    // Calculate checksum
    uint8_t checksum = 0;
    checksum = ~(packet.id + packet.length + packet.instruction);
    for (uint8_t i = 0; i < param_idx; i++) {
        checksum = ~(~checksum + packet.params[i]);
    }
    packet.checksum = checksum;
    
    // Send packet (no response expected for sync write)
    return Servo_SendPacket(protocol, &packet, param_idx);
}

/**
 * @brief Enable/disable servo torque
 */
ServoStatus_t Servo_SetTorqueEnable(ServoProtocol_t *protocol, uint8_t id, bool enable)
{
    uint8_t params[2];
    params[0] = SCS_ADDR_TORQUE_ENABLE;
    params[1] = enable ? 1 : 0;
    
    return Servo_Write(protocol, id, SCS_ADDR_TORQUE_ENABLE, &params[1], 1);
}

/**
 * @brief Generic write to servo memory
 */
ServoStatus_t Servo_Write(ServoProtocol_t *protocol, uint8_t id, 
                           uint8_t address, const uint8_t *data, uint8_t length)
{
    if (!protocol->initialized) {
        return SERVO_ERROR_NOT_INITIALIZED;
    }
    
    if (protocol->state != SERVO_STATE_IDLE) {
        return SERVO_ERROR_BUSY;
    }
    
    if (id == 0 || id > SCS_MAX_SERVOS || data == NULL || length == 0) {
        return SERVO_ERROR_INVALID_ID;
    }
    
    uint8_t params[SCS_MAX_PACKET_SIZE - 6];
    params[0] = address;
    memcpy(&params[1], data, length);
    
    SCS_Packet_t packet;
    Servo_BuildPacket(&packet, id, SCS_WRITE, params, length + 1);
    
    return Servo_SendPacket(protocol, &packet, length + 1);
}

/**
 * @brief Generic read from servo memory
 */
ServoStatus_t Servo_Read(ServoProtocol_t *protocol, uint8_t id, 
                          uint8_t address, uint8_t *data, uint8_t length)
{
    if (!protocol->initialized) {
        return SERVO_ERROR_NOT_INITIALIZED;
    }
    
    if (protocol->state != SERVO_STATE_IDLE) {
        return SERVO_ERROR_BUSY;
    }
    
    if (id == 0 || id > SCS_MAX_SERVOS || data == NULL || length == 0) {
        return SERVO_ERROR_INVALID_ID;
    }
    
    uint8_t params[2];
    params[0] = address;
    params[1] = length;
    
    SCS_Packet_t packet;
    Servo_BuildPacket(&packet, id, SCS_READ, params, 2);
    
    ServoStatus_t status = Servo_SendPacket(protocol, &packet, 2);
    if (status != SERVO_OK) {
        return status;
    }
    
    // Wait for response
    uint8_t expected_length = 6 + length;  // Header(2) + ID + Length + Error + Data + Checksum
    status = Servo_WaitResponse(protocol, expected_length);
    if (status != SERVO_OK) {
        return status;
    }
    
    // Extract data from response
    memcpy(data, &protocol->rx_buffer[5], length);
    
    return SERVO_OK;
}

/**
 * @brief Process servo protocol state machine
 */
void Servo_Process(ServoProtocol_t *protocol)
{
    if (!protocol->initialized) {
        return;
    }
    
    // Check for timeout
    if (protocol->state == SERVO_STATE_RX_BUSY) {
        uint32_t elapsed = HAL_GetTick() - protocol->last_tx_time;
        if (elapsed > protocol->timeout_ms) {
            protocol->state = SERVO_STATE_ERROR;
            HAL_UART_AbortReceive(protocol->huart);
        }
    }
}

/**
 * @brief UART TX complete callback
 */
void Servo_TxCpltCallback(ServoProtocol_t *protocol)
{
    if (protocol == NULL) {
        return;
    }
    
    // Transmission complete, ready to receive or go idle
    if (protocol->rx_expected_length > 0) {
        protocol->state = SERVO_STATE_RX_BUSY;
        // Start DMA receive
        HAL_UART_Receive_DMA(protocol->huart, protocol->rx_buffer, protocol->rx_expected_length);
        /* If DE pin configured, disable transmitter (set LOW) once switching to RX */
        if (protocol->de_enabled && protocol->de_gpio != NULL) {
            HAL_GPIO_WritePin(protocol->de_gpio, protocol->de_pin, GPIO_PIN_RESET);
        }
    } else {
        protocol->state = SERVO_STATE_IDLE;
        if (protocol->de_enabled && protocol->de_gpio != NULL) {
            HAL_GPIO_WritePin(protocol->de_gpio, protocol->de_pin, GPIO_PIN_RESET);
        }
    }
}

/**
 * @brief UART RX complete callback
 */
void Servo_RxCpltCallback(ServoProtocol_t *protocol)
{
    if (protocol == NULL) {
        return;
    }
    
    protocol->state = SERVO_STATE_RX_COMPLETE;
    protocol->rx_length = protocol->rx_expected_length;
}

/**
 * @brief UART Error callback
 */
void Servo_ErrorCallback(ServoProtocol_t *protocol)
{
    if (protocol == NULL) {
        return;
    }
    
    protocol->state = SERVO_STATE_ERROR;
    HAL_UART_AbortTransmit(protocol->huart);
    HAL_UART_AbortReceive(protocol->huart);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Build SCS protocol packet
 */
static void Servo_BuildPacket(SCS_Packet_t *packet, uint8_t id, 
                               uint8_t instruction, const uint8_t *params, 
                               uint8_t param_count)
{
    packet->header1 = SCS_HEADER;
    packet->header2 = SCS_HEADER;
    packet->id = id;
    packet->length = param_count + 2;  // params + instruction + checksum
    packet->instruction = instruction;
    
    if (params != NULL && param_count > 0) {
        memcpy(packet->params, params, param_count);
    }
    
    // Calculate checksum: ~(ID + Length + Instruction + Param0 + ... + ParamN)
    uint8_t checksum = 0;
    checksum = ~(id + packet->length + instruction);
    for (uint8_t i = 0; i < param_count; i++) {
        checksum = ~(~checksum + params[i]);
    }
    packet->checksum = checksum;
}

/**
 * @brief Send packet via DMA
 */
static ServoStatus_t Servo_SendPacket(ServoProtocol_t *protocol, 
                                       const SCS_Packet_t *packet, 
                                       uint8_t param_count)
{
    if (protocol->state != SERVO_STATE_IDLE) {
        return SERVO_ERROR_BUSY;
    }
    
    // Copy packet to TX buffer
    protocol->tx_buffer[0] = packet->header1;
    protocol->tx_buffer[1] = packet->header2;
    protocol->tx_buffer[2] = packet->id;
    protocol->tx_buffer[3] = packet->length;
    protocol->tx_buffer[4] = packet->instruction;
    
    if (param_count > 0) {
        memcpy(&protocol->tx_buffer[5], packet->params, param_count);
    }
    
    protocol->tx_buffer[5 + param_count] = packet->checksum;
    protocol->tx_length = 6 + param_count;
    
    protocol->last_tx_time = HAL_GetTick();
    protocol->state = SERVO_STATE_TX_BUSY;
    
    /* If DE pin configured, enable transmitter (set HIGH) before sending */
    if (protocol->de_enabled && protocol->de_gpio != NULL) {
        HAL_GPIO_WritePin(protocol->de_gpio, protocol->de_pin, GPIO_PIN_SET);
    }

    /* Send via DMA */
    HAL_StatusTypeDef hal_status = HAL_UART_Transmit_DMA(protocol->huart, 
                                                           protocol->tx_buffer, 
                                                           protocol->tx_length);
    
    if (hal_status != HAL_OK) {
        protocol->state = SERVO_STATE_IDLE;
        return SERVO_ERROR_HAL;
    }
    
    return SERVO_OK;
}

/**
 * @brief Wait for servo response
 */
static ServoStatus_t Servo_WaitResponse(ServoProtocol_t *protocol, 
                                         uint8_t expected_length)
{
    protocol->rx_expected_length = expected_length;
    protocol->rx_length = 0;
    
    // Wait for TX to complete and RX to start
    uint32_t start_time = HAL_GetTick();
    while (protocol->state == SERVO_STATE_TX_BUSY) {
        if (HAL_GetTick() - start_time > protocol->timeout_ms) {
            protocol->state = SERVO_STATE_IDLE;
            return SERVO_ERROR_TIMEOUT;
        }
    }
    
    // Wait for RX to complete
    start_time = HAL_GetTick();
    while (protocol->state == SERVO_STATE_RX_BUSY) {
        if (HAL_GetTick() - start_time > protocol->timeout_ms) {
            protocol->state = SERVO_STATE_IDLE;
            HAL_UART_AbortReceive(protocol->huart);
            return SERVO_ERROR_TIMEOUT;
        }
    }
    
    if (protocol->state == SERVO_STATE_ERROR) {
        protocol->state = SERVO_STATE_IDLE;
        return SERVO_ERROR_HAL;
    }
    
    // Verify checksum
    if (!SCS_VerifyChecksum(protocol->rx_buffer, protocol->rx_length)) {
        protocol->state = SERVO_STATE_IDLE;
        return SERVO_ERROR_CHECKSUM;
    }
    
    protocol->state = SERVO_STATE_IDLE;
    return SERVO_OK;
}

/**
 * @brief Calculate SCS protocol checksum
 */
uint8_t SCS_CalculateChecksum(const uint8_t *packet, uint8_t length)
{
    // Checksum = ~(ID + Length + Instruction + Param0 + ... + ParamN)
    // Start from byte 2 (ID), exclude headers and checksum
    uint8_t checksum = 0;
    for (uint8_t i = 2; i < length - 1; i++) {
        checksum = ~(~checksum + packet[i]);
    }
    return checksum;
}

/**
 * @brief Verify packet checksum
 */
bool SCS_VerifyChecksum(const uint8_t *packet, uint8_t length)
{
    if (length < SCS_MIN_RESPONSE_LENGTH) {
        return false;
    }
    
    uint8_t calculated = SCS_CalculateChecksum(packet, length);
    uint8_t received = packet[length - 1];
    
    return (calculated == received);
}

void Servo_SetDirectionPin(ServoProtocol_t *protocol, GPIO_TypeDef *gpio, uint16_t pin)
{
    if (protocol == NULL) return;
    protocol->de_gpio = gpio;
    protocol->de_pin = pin;
    protocol->de_enabled = true;
    /* Ensure pin is low (receive) by default */
    HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
}

/* HAL Callback Wrappers -----------------------------------------------------*/

/**
 * @brief HAL UART TX Complete Callback
 * User must call Servo_TxCpltCallback from this function
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (g_protocol != NULL && huart == g_protocol->huart) {
        Servo_TxCpltCallback(g_protocol);
    }
}

/**
 * @brief HAL UART RX Complete Callback
 * User must call Servo_RxCpltCallback from this function
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (g_protocol != NULL && huart == g_protocol->huart) {
        Servo_RxCpltCallback(g_protocol);
    }
}

/**
 * @brief HAL UART Error Callback
 * User must call Servo_ErrorCallback from this function
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (g_protocol != NULL && huart == g_protocol->huart) {
        Servo_ErrorCallback(g_protocol);
    }
}
