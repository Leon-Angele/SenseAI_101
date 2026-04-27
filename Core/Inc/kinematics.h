/**
 ******************************************************************************
 * @file           : kinematics.h
 * @brief          : SO-101 Robot Arm Kinematics (Forward & Inverse)
 ******************************************************************************
 * @attention
 *
 * Analytical kinematics for SO-101 4-DOF arm using Elementary Transform
 * Sequence (ETS) for forward kinematics and closed-form geometric solution
 * for inverse kinematics (law of cosines + wrist decoupling).
 * 
 * Uses CMSIS-DSP for matrix operations and FPU optimization when available.
 *
 * Joint Configuration:
 * - Joint 0: Base Pan (Rotation) - Handled separately in trajectory planning
 * - Joint 1: Shoulder Lift (Pitch) - q[0] in IK
 * - Joint 2: Elbow Flex (Pitch) - q[1] in IK
 * - Joint 3: Wrist Pitch - q[2] in IK
 * - Joint 4: Wrist Roll - q[3] in IK (not used in position IK)
 * - Joint 5: Gripper - Not used in arm IK
 *
 * Reference Pose (q = 0 for all joints):
 * - Arm extended horizontally forward
 *
 ******************************************************************************
 */

#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#ifdef USE_CMSIS_DSP
#include "arm_math.h"
#else
#include <math.h>
#endif

/* Exported constants --------------------------------------------------------*/

/* Number of joints for IK computation (4-DOF: joints 1-4) */
#define IK_NUM_JOINTS           4

/* Total number of arm joints (excluding gripper) */
#define ARM_NUM_JOINTS          5

/* Link Lengths for SO-101 (in meters) - Elementary Transform Sequence */
/* Measured from physical robot and approximated from DH parameters */
#define L1_BASE_HEIGHT          0.055f      // Base to shoulder pitch axis
#define L2_UPPER_ARM            0.116f      // Shoulder to elbow
#define L3_FOREARM              0.135f      // Elbow to wrist pitch
#define L4_WRIST_TCP            0.060f      // Wrist pitch to tool center point

/* Joint Limits (in radians) - from robot.py qlim */
#define JOINT_LIMIT_MIN_1       -0.2f       // Shoulder Lift
#define JOINT_LIMIT_MAX_1       3.14159f
#define JOINT_LIMIT_MIN_2       -1.5f       // Elbow Flex
#define JOINT_LIMIT_MAX_2       1.5f
#define JOINT_LIMIT_MIN_3       -3.14159f   // Wrist Pitch
#define JOINT_LIMIT_MAX_3       3.14159f
#define JOINT_LIMIT_MIN_4       -3.14159f   // Wrist Roll
#define JOINT_LIMIT_MAX_4       3.14159f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 4x4 Homogeneous transformation matrix
 */
typedef struct {
    float data[16];  // Column-major order
} Mat4x4_t;

/**
 * @brief Joint angle vector (4-DOF for IK)
 */
typedef struct {
    float q[IK_NUM_JOINTS];  // Joint angles in radians
} JointAngles_t;

/**
 * @brief Full arm joint angles (5-DOF)
 */
typedef struct {
    float q[ARM_NUM_JOINTS];  // All joint angles
} ArmJointAngles_t;

/**
 * @brief 6D Cartesian pose (X, Y, Z, Roll, Pitch, Yaw)
 */
typedef struct {
    float x;        // Position X (meters)
    float y;        // Position Y (meters)
    float z;        // Position Z (meters)
    float roll;     // Orientation Roll (radians)
    float pitch;    // Orientation Pitch (radians)
    float yaw;      // Orientation Yaw (radians)
} CartesianPose_t;

/**
 * @brief IK solution result
 */
typedef struct {
    JointAngles_t q;        // Computed joint angles
    bool success;           // True if IK converged
    uint8_t iterations;     // Number of iterations used
    uint8_t searches;       // Number of search attempts
    float residual;     // Final position error (meters)
} IK_Solution_t;

/**
 * @brief IK solver configuration (deprecated, kept for API compatibility)
 * @note Analytical IK does not use iterations or tolerances.
 *       The enforce_limits flag is still respected.
 */
typedef struct {
    uint8_t max_iterations; // Deprecated (not used in analytical IK)
    uint8_t max_searches;   // Deprecated (not used in analytical IK)
    float tolerance;        // Deprecated (not used in analytical IK)
    float lambda;           // Deprecated (not used in analytical IK)
    bool enforce_limits;    // Enforce joint limits (still used)
} IK_Config_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize kinematics module
 */
void Kinematics_Init(void);

/**
 * @brief Compute forward kinematics (joint angles -> Cartesian pose)
 * @param q Joint angles (4-DOF)
 * @param pose Output Cartesian pose
 * @retval true if successful
 */
bool FK_Compute(const JointAngles_t *q, CartesianPose_t *pose);

/**
 * @brief Compute forward kinematics transformation matrix
 * @param q Joint angles (4-DOF)
 * @param T Output 4x4 transformation matrix
 * @retval true if successful
 */
bool FK_ComputeMatrix(const JointAngles_t *q, Mat4x4_t *T);

/**
 * @brief Compute inverse kinematics (Cartesian pose -> joint angles)
 * @param target_pose Desired end-effector pose
 * @param q_init Initial joint guess (deprecated, not used in analytical IK)
 * @param config IK solver configuration (NULL for defaults, only enforce_limits is used)
 * @param solution Output IK solution
 * @retval true if IK succeeded (target reachable)
 */
bool IK_Compute(const CartesianPose_t *target_pose, 
                const JointAngles_t *q_init,
                const IK_Config_t *config,
                IK_Solution_t *solution);

/**
 * @brief Check if joint angles are within limits
 * @param q Joint angles to check
 * @retval true if all joints are within limits
 */
bool Kinematics_CheckJointLimits(const JointAngles_t *q);

/**
 * @brief Clamp joint angles to valid range
 * @param q Joint angles to clamp (modified in place)
 */
void Kinematics_ClampJointLimits(JointAngles_t *q);

/**
 * @brief Normalize angle to [-π, π]
 * @param angle Input angle (radians)
 * @retval Normalized angle
 */
float Kinematics_NormalizeAngle(float angle);

/**
 * @brief Get default IK configuration
 * @param config Output configuration structure
 */
void IK_GetDefaultConfig(IK_Config_t *config);

/* Internal helper functions (exposed for testing) ---------------------------*/

/**
 * @brief Multiply two 4x4 matrices: C = A * B
 * @param A First matrix
 * @param B Second matrix
 * @param C Output matrix
 */
void Mat4x4_Multiply(const Mat4x4_t *A, const Mat4x4_t *B, Mat4x4_t *C);

/**
 * @brief Extract position and orientation from transformation matrix
 * @param T Transformation matrix
 * @param pose Output Cartesian pose
 */
void Mat4x4_ToPose(const Mat4x4_t *T, CartesianPose_t *pose);

/**
 * @brief Build transformation matrix from pose
 * @param pose Cartesian pose
 * @param T Output transformation matrix
 */
void Pose_ToMat4x4(const CartesianPose_t *pose, Mat4x4_t *T);

/**
 * @brief Compute angle-axis error between two transformation matrices
 * @param T_current Current transformation
 * @param T_desired Desired transformation
 * @param error Output 6D error vector [dx, dy, dz, wx, wy, wz]
 */
void Compute_AngleAxisError(const Mat4x4_t *T_current, 
                             const Mat4x4_t *T_desired,
                             float error[6]);

#ifdef __cplusplus
}
#endif

#endif /* __KINEMATICS_H */
