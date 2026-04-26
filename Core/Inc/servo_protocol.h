/**
 ******************************************************************************
 * @file           : servo_protocol.h
 * @brief          : Waveshare ST/SC Serial Bus Servo Protocol (SCS Series)
 ******************************************************************************
 * @attention
 *
 * SCS Series Protocol Implementation for STM32 HAL
 * Compatible with Feetech STS/SCS servos (Waveshare branded)
 * Communication: Half-Duplex UART, 1Mbps, 8N1
 *
 ******************************************************************************
 */

#ifndef __SERVO_PROTOCOL_H
#define __SERVO_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
#define SCS_BAUDRATE                1000000
#define SCS_MAX_SERVOS              6
#define SCS_TIMEOUT_MS              100

/* SCS Protocol Constants */
#define SCS_HEADER                  0xFF
#define SCS_BROADCAST_ID            0xFE

/* SCS Instruction Set */
#define SCS_PING                    0x01
#define SCS_READ                    0x02
#define SCS_WRITE                   0x03
#define SCS_REG_WRITE               0x04
#define SCS_ACTION                  0x05
#define SCS_SYNC_WRITE              0x83

/* SCS Control Table Addresses (STS3215) */
#define SCS_ADDR_MODEL              3
#define SCS_ADDR_ID                 5
#define SCS_ADDR_BAUD_RATE          6
#define SCS_ADDR_MIN_ANGLE_LIMIT    9
#define SCS_ADDR_MAX_ANGLE_LIMIT    11
#define SCS_ADDR_MAX_TEMP_LIMIT     13
#define SCS_ADDR_MAX_VOLTAGE        14
#define SCS_ADDR_MIN_VOLTAGE        15
#define SCS_ADDR_MAX_TORQUE         16
#define SCS_ADDR_UNLOAD_CONDITION   19
#define SCS_ADDR_LED_ALARM          20
#define SCS_ADDR_P_COEFFICIENT      21
#define SCS_ADDR_D_COEFFICIENT      22
#define SCS_ADDR_I_COEFFICIENT      23
#define SCS_ADDR_MIN_STARTUP_FORCE  24
#define SCS_ADDR_MODE               33
#define SCS_ADDR_TORQUE_ENABLE      40
#define SCS_ADDR_ACCELERATION       41
#define SCS_ADDR_GOAL_POSITION      42
#define SCS_ADDR_GOAL_TIME          44
#define SCS_ADDR_GOAL_SPEED         46
#define SCS_ADDR_TORQUE_LIMIT       48
#define SCS_ADDR_LOCK               55
#define SCS_ADDR_PRESENT_POSITION   56
#define SCS_ADDR_PRESENT_SPEED      58
#define SCS_ADDR_PRESENT_LOAD       60
#define SCS_ADDR_PRESENT_VOLTAGE    62
#define SCS_ADDR_PRESENT_TEMP       63
#define SCS_ADDR_MOVE_STATUS        66
#define SCS_ADDR_PRESENT_CURRENT    69

/* Servo Position Range (12-bit) */
#define SCS_POSITION_MIN            0
#define SCS_POSITION_MAX            4095
#define SCS_POSITION_CENTER         2048

/* Max packet size */
#define SCS_MAX_PACKET_SIZE         64

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SCS Servo ID enumeration
 */
typedef enum {
    SERVO_SHOULDER_PAN  = 1,
    SERVO_SHOULDER_LIFT = 2,
    SERVO_ELBOW_FLEX    = 3,
    SERVO_WRIST_FLEX    = 4,
    SERVO_WRIST_ROLL    = 5,
    SERVO_GRIPPER       = 6
} ServoID_t;

/**
 * @brief SCS Packet structure
 */
typedef struct {
    uint8_t header1;        // 0xFF
    uint8_t header2;        // 0xFF
    uint8_t id;             // Servo ID (0-253, 0xFE=broadcast)
    uint8_t length;         // Packet length (parameters + 2)
    uint8_t instruction;    // Instruction byte
    uint8_t params[SCS_MAX_PACKET_SIZE - 6];  // Parameters
    uint8_t checksum;       // Checksum byte
} __attribute__((packed)) SCS_Packet_t;

/**
 * @brief Servo communication state
 */
typedef enum {
    SERVO_STATE_IDLE = 0,
    SERVO_STATE_TX_BUSY,
    SERVO_STATE_RX_BUSY,
    SERVO_STATE_RX_COMPLETE,
    SERVO_STATE_ERROR
} ServoState_t;

/**
 * @brief Servo protocol context
 */
typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_tx;
    DMA_HandleTypeDef  *hdma_rx;
    /* Optional half-duplex direction control pin (DE/RE) */
    GPIO_TypeDef *de_gpio;  /* GPIO port for direction enable */
    uint16_t de_pin;        /* GPIO pin for direction enable */
    bool de_enabled;        /* true if DE pin configured */
    
    uint8_t tx_buffer[SCS_MAX_PACKET_SIZE];
    uint8_t rx_buffer[SCS_MAX_PACKET_SIZE];
    
    uint16_t tx_length;
    uint16_t rx_length;
    uint16_t rx_expected_length;
    
    ServoState_t state;
    uint32_t last_tx_time;
    uint32_t timeout_ms;
    
    bool initialized;
} ServoProtocol_t;

/**
 * @brief Servo status return codes
 */
typedef enum {
    SERVO_OK = 0,
    SERVO_ERROR_TIMEOUT,
    SERVO_ERROR_CHECKSUM,
    SERVO_ERROR_INVALID_ID,
    SERVO_ERROR_BUSY,
    SERVO_ERROR_NOT_INITIALIZED,
    SERVO_ERROR_HAL
} ServoStatus_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize servo protocol
 * @param protocol Pointer to protocol context
 * @param huart UART handle (must be configured for half-duplex)
 * @retval ServoStatus_t
 */
ServoStatus_t Servo_Init(ServoProtocol_t *protocol, UART_HandleTypeDef *huart);

/**
 * @brief Configure optional direction (DE) pin for half-duplex transceiver
 * @param protocol Pointer to protocol context
 * @param gpio GPIO port (e.g., GPIOA) for DE pin
 * @param pin GPIO pin (e.g., GPIO_PIN_8)
 */
void Servo_SetDirectionPin(ServoProtocol_t *protocol, GPIO_TypeDef *gpio, uint16_t pin);

/**
 * @brief Ping a servo to check if it's connected
 * @param protocol Pointer to protocol context
 * @param id Servo ID
 * @retval ServoStatus_t
 */
ServoStatus_t Servo_Ping(ServoProtocol_t *protocol, uint8_t id);

/**
 * @brief Write position to servo
 * @param protocol Pointer to protocol context
 * @param id Servo ID
 * @param position Target position (0-4095)
 * @param time Movement time in milliseconds (0 = max speed)
 * @retval ServoStatus_t
 */
ServoStatus_t Servo_WritePosition(ServoProtocol_t *protocol, uint8_t id, 
                                   uint16_t position, uint16_t time);

/**
 * @brief Read current position from servo
 * @param protocol Pointer to protocol context
 * @param id Servo ID
 * @param position Pointer to store read position
 * @retval ServoStatus_t
 */
ServoStatus_t Servo_ReadPosition(ServoProtocol_t *protocol, uint8_t id, uint16_t *position);

/**
 * @brief Write multiple servo positions simultaneously (sync write)
 * @param protocol Pointer to protocol context
 * @param ids Array of servo IDs
 * @param positions Array of target positions
 * @param times Array of movement times (can be NULL for max speed)
 * @param count Number of servos to control
 * @retval ServoStatus_t
 */
ServoStatus_t Servo_SyncWritePosition(ServoProtocol_t *protocol, 
                                       const uint8_t *ids, 
                                       const uint16_t *positions,
                                       const uint16_t *times,
                                       uint8_t count);

/**
 * @brief Enable/disable servo torque
 * @param protocol Pointer to protocol context
 * @param id Servo ID
 * @param enable true to enable torque, false to disable
 * @retval ServoStatus_t
 */
ServoStatus_t Servo_SetTorqueEnable(ServoProtocol_t *protocol, uint8_t id, bool enable);

/**
 * @brief Write generic data to servo memory
 * @param protocol Pointer to protocol context
 * @param id Servo ID
 * @param address Memory address
 * @param data Pointer to data to write
 * @param length Number of bytes to write
 * @retval ServoStatus_t
 */
ServoStatus_t Servo_Write(ServoProtocol_t *protocol, uint8_t id, 
                           uint8_t address, const uint8_t *data, uint8_t length);

/**
 * @brief Read generic data from servo memory
 * @param protocol Pointer to protocol context
 * @param id Servo ID
 * @param address Memory address
 * @param data Pointer to buffer for read data
 * @param length Number of bytes to read
 * @retval ServoStatus_t
 */
ServoStatus_t Servo_Read(ServoProtocol_t *protocol, uint8_t id, 
                          uint8_t address, uint8_t *data, uint8_t length);

/**
 * @brief Process servo protocol state machine (call from main loop)
 * @param protocol Pointer to protocol context
 */
void Servo_Process(ServoProtocol_t *protocol);

/**
 * @brief UART TX complete callback (call from HAL_UART_TxCpltCallback)
 * @param protocol Pointer to protocol context
 */
void Servo_TxCpltCallback(ServoProtocol_t *protocol);

/**
 * @brief UART RX complete callback (call from HAL_UART_RxCpltCallback)
 * @param protocol Pointer to protocol context
 */
void Servo_RxCpltCallback(ServoProtocol_t *protocol);

/**
 * @brief UART Error callback (call from HAL_UART_ErrorCallback)
 * @param protocol Pointer to protocol context
 */
void Servo_ErrorCallback(ServoProtocol_t *protocol);

/* Internal functions (exposed for testing) ----------------------------------*/

/**
 * @brief Calculate SCS protocol checksum
 * @param packet Pointer to packet (excluding checksum byte)
 * @param length Packet length (including headers, excluding checksum)
 * @retval Calculated checksum byte
 */
uint8_t SCS_CalculateChecksum(const uint8_t *packet, uint8_t length);

/**
 * @brief Verify packet checksum
 * @param packet Pointer to complete packet (including checksum)
 * @param length Total packet length
 * @retval true if checksum is valid
 */
bool SCS_VerifyChecksum(const uint8_t *packet, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_PROTOCOL_H */
