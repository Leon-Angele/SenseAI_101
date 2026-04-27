/**
 ******************************************************************************
 * @file           : servo_mapping.c
 * @brief          : Servo Calibration and Angle Mapping Implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "servo_mapping.h"
#include <math.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define PI                      3.14159265358979f
#define DEG_TO_RAD              (PI / 180.0f)
#define RAD_TO_DEG              (180.0f / PI)

/* Private variables ---------------------------------------------------------*/

/**
 * @brief Calibration data from MinMax Range of Motion_Servos.json
 * 
 * Updated for analytical kinematics with horizontal (q=0) as reference pose.
 * Physical measurements for horizontal extended position (q1=0, q2=0, q3=0):
 *   - Servo 2: 3100 (Shoulder Lift)
 *   - Servo 3: 1000 (Elbow Flex)
 *   - Servo 4: 2000 (Wrist Pitch)
 * 
 * Order: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
 * Indices: [0, 1, 2, 3, 4, 5]
*/
static const ServoCalib_t servo_calibration[NUM_SERVOS] = {
    // ID 1: Shoulder Pan (Base rotation, center: ~2250)
    { .homing_offset = 250,  .range_min = 1100, .range_max = 3569, .calib_mode = CALIB_MODE_DEGREE },
    
    // ID 2: Shoulder Lift (Horizontal q1=0 -> pos=1200)
    // Calculation: 1200 = 2048 + (-848)
    { .homing_offset = -848, .range_min = 871,  .range_max = 3341, .calib_mode = CALIB_MODE_DEGREE, .drive_mode = 0 },
    
    // ID 3: Elbow Flex (Straight q2=0 -> pos=1000)
    // Calculation: 1000 = 2048 + (-1048)
    { .homing_offset = -1048, .range_min = 815,  .range_max = 3024, .calib_mode = CALIB_MODE_DEGREE, .drive_mode = 0 },
    
    // ID 4: Wrist Pitch (Straight q3=0 -> pos=2000)
    // Calculation: 2000 = 2048 + (-48)
    { .homing_offset = -48,  .range_min = 774,  .range_max = 3141, .calib_mode = CALIB_MODE_DEGREE, .drive_mode = 0 },
    
    // ID 5: Wrist Roll (Center, unchanged)
    { .homing_offset = -1,   .range_min = 0,    .range_max = 4095, .calib_mode = CALIB_MODE_DEGREE },
    
    // ID 6: Gripper (Linear mode, offset ignored, uses start/end positions)
    { .homing_offset = 0,    .start_pos = 1601, .end_pos = 3097, .range_min = 1601, .range_max = 3097, .calib_mode = CALIB_MODE_LINEAR }
};

/* Private function prototypes -----------------------------------------------*/
static float ClampFloat(float value, float min, float max);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize servo mapping module
 */
void ServoMapping_Init(void)
{
    // Currently no initialization needed
    // Calibration data is statically defined
}

/**
 * @brief Get calibration data for a specific servo
 */
bool ServoMapping_GetCalibration(uint8_t servo_idx, ServoCalib_t *calib)
{
    if (servo_idx >= NUM_SERVOS || calib == NULL) {
        return false;
    }
    
    *calib = servo_calibration[servo_idx];
    return true;
}

/**
 * @brief Convert joint angle (radians) to servo position
 */
uint16_t ServoMapping_RadToPosition(uint8_t servo_idx, float angle_rad)
{
    if (servo_idx >= NUM_SERVOS) {
        return SERVO_POS_CENTER;
    }
    
    const ServoCalib_t *calib = &servo_calibration[servo_idx];
    
    if (calib->calib_mode == CALIB_MODE_LINEAR) {
        // For linear mode (gripper), angle_rad is treated as percentage (0-1)
        float percent = angle_rad * 100.0f;
        return ServoMapping_SetGripperPercent(percent);
    }
    
    // Convert radians to degrees
    float angle_deg = angle_rad * RAD_TO_DEG;
    
    // Apply drive mode inversion if needed
    if (calib->drive_mode == 1) {
        angle_deg = -angle_deg;
    }
    
    // Convert degrees to servo units
    // Center position (0 degrees) is at SERVO_POS_CENTER
    float servo_units = angle_deg * SERVO_UNITS_PER_DEGREE;
    
    // Add calibration offset and center position
    int32_t position = (int32_t)(SERVO_POS_CENTER + servo_units + calib->homing_offset);
    
    // Clamp to valid range
    return ServoMapping_ClampPosition(servo_idx, position);
}

/**
 * @brief Convert servo position to joint angle (radians)
 */
float ServoMapping_PositionToRad(uint8_t servo_idx, uint16_t position)
{
    if (servo_idx >= NUM_SERVOS) {
        return 0.0f;
    }
    
    const ServoCalib_t *calib = &servo_calibration[servo_idx];
    
    if (calib->calib_mode == CALIB_MODE_LINEAR) {
        // For linear mode (gripper), return percentage as fraction
        float percent = ServoMapping_GetGripperPercent(position);
        return percent / 100.0f;
    }
    
    // Remove calibration offset
    int32_t servo_units = (int32_t)position - SERVO_POS_CENTER - calib->homing_offset;
    
    // Convert servo units to degrees
    float angle_deg = (float)servo_units / SERVO_UNITS_PER_DEGREE;
    
    // Apply drive mode inversion if needed
    if (calib->drive_mode == 1) {
        angle_deg = -angle_deg;
    }
    
    // Convert degrees to radians
    return angle_deg * DEG_TO_RAD;
}

/**
 * @brief Convert IK joint angles to full servo positions
 */
void ServoMapping_IKToServoPositions(const JointAngles_t *ik_angles,
                                      float base_rotation,
                                      float gripper_percent,
                                      ServoPositions_t *servo_pos)
{
    if (ik_angles == NULL || servo_pos == NULL) {
        return;
    }
    
    // Create full arm angles structure
    FullArmAngles_t full_angles;
    
    // Map IK joints to full arm:
    // IK Joint 0 -> Arm Joint 1 (Shoulder Lift)
    // IK Joint 1 -> Arm Joint 2 (Elbow Flex)
    // IK Joint 2 -> Arm Joint 3 (Wrist Pitch)
    // IK Joint 3 -> Arm Joint 4 (Wrist Roll)
    
    full_angles.angles[0] = base_rotation;      // Base rotation (not from IK)
    full_angles.angles[1] = ik_angles->q[0];    // Shoulder Lift
    full_angles.angles[2] = ik_angles->q[1];    // Elbow Flex
    full_angles.angles[3] = ik_angles->q[2];    // Wrist Pitch
    full_angles.angles[4] = ik_angles->q[3];    // Wrist Roll
    full_angles.angles[5] = gripper_percent / 100.0f;  // Gripper (as fraction)
    
    // Apply angle inversions as per Python reference
    // Keep only joints 0 and 4 inverted (base and wrist roll)
    full_angles.angles[0] = -full_angles.angles[0];
    full_angles.angles[4] = -full_angles.angles[4];
    
    // Convert each angle to servo position
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servo_pos->pos[i] = ServoMapping_RadToPosition(i, full_angles.angles[i]);
    }
}

/**
 * @brief Convert servo positions to joint angles
 */
void ServoMapping_ServoPositionsToAngles(const ServoPositions_t *servo_pos,
                                          FullArmAngles_t *arm_angles)
{
    if (servo_pos == NULL || arm_angles == NULL) {
        return;
    }
    
    // Convert each servo position to angle
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        arm_angles->angles[i] = ServoMapping_PositionToRad(i, servo_pos->pos[i]);
    }
    
    // Remove angle inversions (reverse the inversion applied in IKToServoPositions)
    arm_angles->angles[0] = -arm_angles->angles[0];
    arm_angles->angles[4] = -arm_angles->angles[4];
}

/**
 * @brief Apply angle inversions (for command)
 */
void ServoMapping_ApplyAngleInversions(FullArmAngles_t *angles)
{
    if (angles == NULL) {
        return;
    }
    
    // Invert joints 0 and 4 (base and wrist roll)
    angles->angles[0] = -angles->angles[0];
    angles->angles[4] = -angles->angles[4];
}

/**
 * @brief Remove angle inversions (for feedback)
 */
void ServoMapping_RemoveAngleInversions(FullArmAngles_t *angles)
{
    if (angles == NULL) {
        return;
    }
    
    // Remove inversion (same operation as apply since it's negation)
    angles->angles[0] = -angles->angles[0];
    angles->angles[4] = -angles->angles[4];
}

/**
 * @brief Set gripper position from percentage
 */
uint16_t ServoMapping_SetGripperPercent(float percent)
{
    // Clamp to 0-100%
    percent = ClampFloat(percent, 0.0f, 100.0f);
    
    const ServoCalib_t *calib = &servo_calibration[5];  // Gripper is servo 5
    
    // Map percentage to servo range
    // 0% = closed (start_pos), 100% = open (end_pos)
    float range = (float)(calib->end_pos - calib->start_pos);
    int32_t position = calib->start_pos + (int32_t)((percent / 100.0f) * range);
    
    return ServoMapping_ClampPosition(5, position);
}

/**
 * @brief Get gripper percentage from servo position
 */
float ServoMapping_GetGripperPercent(uint16_t position)
{
    const ServoCalib_t *calib = &servo_calibration[5];  // Gripper is servo 5
    
    float range = (float)(calib->end_pos - calib->start_pos);
    if (fabsf(range) < 0.001f) {
        return 0.0f;
    }
    
    float percent = ((float)position - (float)calib->start_pos) / range * 100.0f;
    
    return ClampFloat(percent, 0.0f, 100.0f);
}

/**
 * @brief Clamp servo position to valid range
 */
uint16_t ServoMapping_ClampPosition(uint8_t servo_idx, int32_t position)
{
    if (servo_idx >= NUM_SERVOS) {
        // Fallback to global range if invalid index
        if (position < SERVO_POS_MIN) {
            return SERVO_POS_MIN;
        } else if (position > SERVO_POS_MAX) {
            return SERVO_POS_MAX;
        }
        return (uint16_t)position;
    }
    
    const ServoCalib_t *calib = &servo_calibration[servo_idx];
    
    // Clamp to servo-specific range
    if (position < calib->range_min) {
        return calib->range_min;
    } else if (position > calib->range_max) {
        return calib->range_max;
    }
    return (uint16_t)position;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Clamp float value to range
 */
static float ClampFloat(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
