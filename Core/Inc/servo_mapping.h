/**
 ******************************************************************************
 * @file           : servo_mapping.h
 * @brief          : Servo Calibration and Angle Mapping for SO-101
 ******************************************************************************
 * @attention
 *
 * Maps between joint angles (radians) and servo positions (0-4095)
 * Handles calibration offsets, drive modes, and angle inversions
 *
 * Calibration data from: so101/calibration.json
 * - homing_offset: [2030, 3023, -1049, -2058, -2057, -2452]
 * - drive_mode: [1, 1, 0, 0, 0, 0] (1=inverted direction)
 * - Angle inversions for joints [0, 1, 4] as per feetech_arm.py
 *
 ******************************************************************************
 */

#ifndef __SERVO_MAPPING_H
#define __SERVO_MAPPING_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "kinematics.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

/* Number of servos */
#define NUM_SERVOS              6

/* Servo position range (12-bit) */
#define SERVO_POS_MIN           0
#define SERVO_POS_MAX           4095
#define SERVO_POS_CENTER        2048

/* Degrees per servo unit (for degree-mode servos) */
/* Full range: 0-4095 corresponds to ~240 degrees */
#define SERVO_UNITS_PER_DEGREE  (4096.0f / 240.0f)  // ~17.067

/* Linear servo scaling (for gripper) */
#define SERVO_LINEAR_MIN_PERCENT    0.0f
#define SERVO_LINEAR_MAX_PERCENT    100.0f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Servo calibration mode
 */
typedef enum {
    CALIB_MODE_DEGREE = 0,  // Rotary joint (in degrees)
    CALIB_MODE_LINEAR = 1   // Linear joint (in percentage, e.g., gripper)
} CalibMode_t;

/**
 * @brief Servo calibration data
 */
typedef struct {
    int16_t homing_offset;  // Calibration offset (servo units)
    uint8_t drive_mode;     // 0=normal, 1=inverted
    int16_t start_pos;      // Start position for calibration
    int16_t end_pos;        // End position for calibration
    CalibMode_t calib_mode; // Degree or linear mode
} ServoCalib_t;

/**
 * @brief Full arm servo positions (all 6 servos)
 */
typedef struct {
    uint16_t pos[NUM_SERVOS];  // Servo positions [0-4095]
} ServoPositions_t;

/**
 * @brief Full arm joint angles (radians, for all joints including base rotation)
 */
typedef struct {
    float angles[NUM_SERVOS];  // Joint angles in radians
} FullArmAngles_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize servo mapping module with calibration data
 */
void ServoMapping_Init(void);

/**
 * @brief Get calibration data for a specific servo
 * @param servo_idx Servo index (0-5)
 * @param calib Output calibration structure
 * @retval true if successful
 */
bool ServoMapping_GetCalibration(uint8_t servo_idx, ServoCalib_t *calib);

/**
 * @brief Convert joint angle (radians) to servo position (0-4095)
 * @param servo_idx Servo index (0-5)
 * @param angle_rad Joint angle in radians
 * @retval Servo position (0-4095), clamped to valid range
 */
uint16_t ServoMapping_RadToPosition(uint8_t servo_idx, float angle_rad);

/**
 * @brief Convert servo position to joint angle (radians)
 * @param servo_idx Servo index (0-5)
 * @param position Servo position (0-4095)
 * @retval Joint angle in radians
 */
float ServoMapping_PositionToRad(uint8_t servo_idx, uint16_t position);

/**
 * @brief Convert IK joint angles (4-DOF) to full arm servo positions (6 servos)
 * @param ik_angles 4-DOF joint angles from IK solver
 * @param base_rotation Base rotation angle (joint 0, radians)
 * @param gripper_percent Gripper opening percentage (0-100)
 * @param servo_pos Output servo positions for all 6 servos
 */
void ServoMapping_IKToServoPositions(const JointAngles_t *ik_angles,
                                      float base_rotation,
                                      float gripper_percent,
                                      ServoPositions_t *servo_pos);

/**
 * @brief Convert full arm servo positions to joint angles
 * @param servo_pos Servo positions for all 6 servos
 * @param arm_angles Output joint angles in radians
 */
void ServoMapping_ServoPositionsToAngles(const ServoPositions_t *servo_pos,
                                          FullArmAngles_t *arm_angles);

/**
 * @brief Apply angle inversions as per Python reference (joints 0, 1, 4)
 * @param angles Full arm angles (modified in place)
 */
void ServoMapping_ApplyAngleInversions(FullArmAngles_t *angles);

/**
 * @brief Remove angle inversions (for reading feedback)
 * @param angles Full arm angles (modified in place)
 */
void ServoMapping_RemoveAngleInversions(FullArmAngles_t *angles);

/**
 * @brief Set gripper position (percentage)
 * @param percent Gripper opening percentage (0=closed, 100=open)
 * @retval Servo position (0-4095)
 */
uint16_t ServoMapping_SetGripperPercent(float percent);

/**
 * @brief Get gripper percentage from servo position
 * @param position Servo position (0-4095)
 * @retval Gripper opening percentage (0-100)
 */
float ServoMapping_GetGripperPercent(uint16_t position);

/**
 * @brief Clamp servo position to valid range
 * @param position Input position
 * @retval Clamped position (0-4095)
 */
uint16_t ServoMapping_ClampPosition(int32_t position);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_MAPPING_H */
