/**
 ******************************************************************************
 * @file           : kinematics.h
 * @brief          : SO-101 Robot Arm Kinematics (Forward & Inverse)
 ******************************************************************************
 * @attention
 *
 * Denavit-Hartenberg based kinematics for SO-101 6-DOF arm
 * Uses CMSIS-DSP for matrix operations and FPU optimization
 *
 * Joint Configuration:
 * - Joint 0: Shoulder Pan (Rotation) - Not used in 4-DOF IK
 * - Joint 1: Shoulder Lift (Pitch)
 * - Joint 2: Elbow Flex
 * - Joint 3: Wrist Pitch
 * - Joint 4: Wrist Roll - Not used in position IK (orientation only)
 * - Joint 5: Gripper - Not used in arm IK
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

/* DH Parameters for SO-101 (in meters) - from robot.py */
/* Joint 1 (Shoulder Lift): tx=0.02943, tz=0.05504, Ry */
#define DH_J1_TX                0.02943f
#define DH_J1_TZ                0.05504f

/* Joint 2 (Elbow Flex): tx=0.1127, tz=-0.02798, Ry */
#define DH_J2_TX                0.1127f
#define DH_J2_TZ                -0.02798f

/* Joint 3 (Wrist Pitch): tx=0.13504, tz=0.00519, Ry */
#define DH_J3_TX                0.13504f
#define DH_J3_TZ                0.00519f

/* Joint 4 (Wrist Roll): tx=0.0593, tz=0.00996, Rx */
#define DH_J4_TX                0.0593f
#define DH_J4_TZ                0.00996f

/* Joint Limits (in radians) - from robot.py qlim */
#define JOINT_LIMIT_MIN_1       -0.2f       // Shoulder Lift
#define JOINT_LIMIT_MAX_1       3.14159f
#define JOINT_LIMIT_MIN_2       -1.5f       // Elbow Flex
#define JOINT_LIMIT_MAX_2       1.5f
#define JOINT_LIMIT_MIN_3       -3.14159f   // Wrist Pitch
#define JOINT_LIMIT_MAX_3       3.14159f
#define JOINT_LIMIT_MIN_4       -3.14159f   // Wrist Roll
#define JOINT_LIMIT_MAX_4       3.14159f

/* IK Solver Parameters */
#define IK_MAX_ITERATIONS       10          // Max iterations per attempt
#define IK_MAX_SEARCHES         2           // Max random restarts
#define IK_TOLERANCE            1e-3f       // Convergence tolerance (meters)
#define IK_DAMPING_LAMBDA       0.01f       // LM damping factor
#define IK_MAX_JOINT_DELTA      0.1f        // Max joint change per iteration (rad)

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
 * @brief 6xN Jacobian matrix
 */
typedef struct {
    float data[6 * IK_NUM_JOINTS];  // 6 rows x 4 columns
} Jacobian_t;

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
 * @brief IK solver configuration
 */
typedef struct {
    uint8_t max_iterations; // Max iterations per search
    uint8_t max_searches;   // Max search attempts
    float tolerance;    // Convergence tolerance
    float lambda;       // Damping factor for LM
    bool enforce_limits;    // Enforce joint limits
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
 * @param q_init Initial joint guess (can be current position)
 * @param config IK solver configuration (NULL for defaults)
 * @param solution Output IK solution
 * @retval true if IK converged
 */
bool IK_Compute(const CartesianPose_t *target_pose, 
                const JointAngles_t *q_init,
                const IK_Config_t *config,
                IK_Solution_t *solution);

/**
 * @brief Compute Jacobian matrix at given joint configuration
 * @param q Joint angles (4-DOF)
 * @param J Output Jacobian matrix (6x4)
 * @retval true if successful
 */
bool Jacobian_Compute(const JointAngles_t *q, Jacobian_t *J);

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

/**
 * @brief Compute manipulability measure
 * @param J Jacobian matrix
 * @retval Manipulability index (higher is better)
 */
float Kinematics_Manipulability(const Jacobian_t *J);

/* Internal helper functions (exposed for testing) ---------------------------*/

/**
 * @brief Build homogeneous transformation matrix from DH parameters
 * @param tx Translation along X
 * @param tz Translation along Z
 * @param theta Rotation angle (radians)
 * @param axis Rotation axis ('x' or 'y')
 * @param T Output transformation matrix
 */
void Build_Transform(float tx, float tz, float theta, char axis, Mat4x4_t *T);

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
