/**
 ******************************************************************************
 * @file           : kinematics.c
 * @brief          : SO-101 Robot Arm Kinematics Implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "kinematics.h"
#include <string.h>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
#define PI                      3.14159265358979f
#define TWO_PI                  6.28318530717959f
#define EPSILON                 1e-6f
#define JACOBIAN_DELTA          1e-4f  // Finite difference step for Jacobian

/* Private variables ---------------------------------------------------------*/
static bool initialized = false;

/* Private function prototypes -----------------------------------------------*/
static void SetIdentityMatrix(Mat4x4_t *T);
static void RotationMatrix_X(float angle, float R[9]);
static void RotationMatrix_Y(float angle, float R[9]);
static void RotationMatrix_Z(float angle, float R[9]);
static void ExtractEulerAngles(const float R[9], float *roll, float *pitch, float *yaw);
static bool IK_Solve_LM(const CartesianPose_t *target, const JointAngles_t *q0, 
                        const IK_Config_t *cfg, IK_Solution_t *sol);
static void Jacobian_Numerical(const JointAngles_t *q, Jacobian_t *J);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize kinematics module
 */
void Kinematics_Init(void)
{
    if (initialized) {
        return;
    }
    
    initialized = true;
}

/**
 * @brief Compute forward kinematics
 */
bool FK_Compute(const JointAngles_t *q, CartesianPose_t *pose)
{
    if (q == NULL || pose == NULL) {
        return false;
    }
    
    Mat4x4_t T;
    if (!FK_ComputeMatrix(q, &T)) {
        return false;
    }
    
    Mat4x4_ToPose(&T, pose);
    return true;
}

/**
 * @brief Compute forward kinematics transformation matrix
 */
bool FK_ComputeMatrix(const JointAngles_t *q, Mat4x4_t *T)
{
    if (q == NULL || T == NULL) {
        return false;
    }
    
    Mat4x4_t T1, T2, T3, T4, T_temp, T_result;
    
    // Build transformation for each joint using DH parameters
    // Joint 1: Shoulder Lift (Ry)
    Build_Transform(DH_J1_TX, DH_J1_TZ, q->q[0], 'y', &T1);
    
    // Joint 2: Elbow Flex (Ry)
    Build_Transform(DH_J2_TX, DH_J2_TZ, q->q[1], 'y', &T2);
    
    // Joint 3: Wrist Pitch (Ry)
    Build_Transform(DH_J3_TX, DH_J3_TZ, q->q[2], 'y', &T3);
    
    // Joint 4: Wrist Roll (Rx)
    Build_Transform(DH_J4_TX, DH_J4_TZ, q->q[3], 'x', &T4);
    
    // Chain transformations: T = T1 * T2 * T3 * T4
    Mat4x4_Multiply(&T1, &T2, &T_temp);
    Mat4x4_Multiply(&T_temp, &T3, &T_result);
    Mat4x4_Multiply(&T_result, &T4, T);
    
    return true;
}

/**
 * @brief Compute inverse kinematics
 */
bool IK_Compute(const CartesianPose_t *target_pose, 
                const JointAngles_t *q_init,
                const IK_Config_t *config,
                IK_Solution_t *solution)
{
    if (target_pose == NULL || solution == NULL) {
        return false;
    }
    
    // Use default config if not provided
    IK_Config_t cfg;
    if (config == NULL) {
        IK_GetDefaultConfig(&cfg);
    } else {
        cfg = *config;
    }
    
    // Initialize solution
    memset(solution, 0, sizeof(IK_Solution_t));
    
    // Use provided initial guess or default mid-range position
    JointAngles_t q0;
    if (q_init != NULL) {
        q0 = *q_init;
    } else {
        // Default initial guess (mid-range for most joints)
        q0.q[0] = 1.5f;   // Shoulder lift
        q0.q[1] = 0.0f;   // Elbow
        q0.q[2] = 0.0f;   // Wrist pitch
        q0.q[3] = 0.0f;   // Wrist roll
    }
    
    // Solve using Levenberg-Marquardt
    return IK_Solve_LM(target_pose, &q0, &cfg, solution);
}

/**
 * @brief Compute Jacobian matrix
 */
bool Jacobian_Compute(const JointAngles_t *q, Jacobian_t *J)
{
    if (q == NULL || J == NULL) {
        return false;
    }
    
    Jacobian_Numerical(q, J);
    return true;
}

/**
 * @brief Check joint limits
 */
bool Kinematics_CheckJointLimits(const JointAngles_t *q)
{
    if (q == NULL) {
        return false;
    }
    
    const float limits_min[IK_NUM_JOINTS] = {
        JOINT_LIMIT_MIN_1, JOINT_LIMIT_MIN_2, JOINT_LIMIT_MIN_3, JOINT_LIMIT_MIN_4
    };
    const float limits_max[IK_NUM_JOINTS] = {
        JOINT_LIMIT_MAX_1, JOINT_LIMIT_MAX_2, JOINT_LIMIT_MAX_3, JOINT_LIMIT_MAX_4
    };
    
    for (uint8_t i = 0; i < IK_NUM_JOINTS; i++) {
        if (q->q[i] < limits_min[i] || q->q[i] > limits_max[i]) {
            return false;
        }
    }
    
    return true;
}

/**
 * @brief Clamp joint angles to limits
 */
void Kinematics_ClampJointLimits(JointAngles_t *q)
{
    if (q == NULL) {
        return;
    }
    
    const float limits_min[IK_NUM_JOINTS] = {
        JOINT_LIMIT_MIN_1, JOINT_LIMIT_MIN_2, JOINT_LIMIT_MIN_3, JOINT_LIMIT_MIN_4
    };
    const float limits_max[IK_NUM_JOINTS] = {
        JOINT_LIMIT_MAX_1, JOINT_LIMIT_MAX_2, JOINT_LIMIT_MAX_3, JOINT_LIMIT_MAX_4
    };
    
    for (uint8_t i = 0; i < IK_NUM_JOINTS; i++) {
        if (q->q[i] < limits_min[i]) {
            q->q[i] = limits_min[i];
        } else if (q->q[i] > limits_max[i]) {
            q->q[i] = limits_max[i];
        }
    }
}

/**
 * @brief Normalize angle to [-π, π]
 */
float Kinematics_NormalizeAngle(float angle)
{
    while (angle > PI) {
        angle -= TWO_PI;
    }
    while (angle < -PI) {
        angle += TWO_PI;
    }
    return angle;
}

/**
 * @brief Get default IK configuration
 */
void IK_GetDefaultConfig(IK_Config_t *config)
{
    if (config == NULL) {
        return;
    }
    
    config->max_iterations = IK_MAX_ITERATIONS;
    config->max_searches = IK_MAX_SEARCHES;
    config->tolerance = IK_TOLERANCE;
    config->lambda = IK_DAMPING_LAMBDA;
    config->enforce_limits = true;
}

/**
 * @brief Compute manipulability
 */
float Kinematics_Manipulability(const Jacobian_t *J)
{
    if (J == NULL) {
        return 0.0f;
    }
    
    // For simplicity, return a placeholder
    // Full implementation would compute sqrt(det(J * J^T))
    return 1.0f;
}

/* Internal helper functions -------------------------------------------------*/

/**
 * @brief Build transformation matrix from DH parameters
 */
void Build_Transform(float tx, float tz, float theta, char axis, Mat4x4_t *T)
{
    if (T == NULL) {
        return;
    }
    
    SetIdentityMatrix(T);
    
    /* Use CMSIS-DSP trig functions when enabled for FPU-optimized math */
#ifdef USE_CMSIS_DSP
    float c = arm_cos_f32(theta);
    float s = arm_sin_f32(theta);
#else
    float c = cosf(theta);
    float s = sinf(theta);
#endif
    
    if (axis == 'y') {
        // Rotation around Y axis
        // R = [c  0  s]
        //     [0  1  0]
        //     [-s 0  c]
        T->data[0] = c;   T->data[4] = 0.0f; T->data[8]  = s;    T->data[12] = tx;
        T->data[1] = 0.0f; T->data[5] = 1.0f; T->data[9]  = 0.0f; T->data[13] = 0.0f;
        T->data[2] = -s;  T->data[6] = 0.0f; T->data[10] = c;    T->data[14] = tz;
        T->data[3] = 0.0f; T->data[7] = 0.0f; T->data[11] = 0.0f; T->data[15] = 1.0f;
    } else if (axis == 'x') {
        // Rotation around X axis
        // R = [1  0   0]
        //     [0  c  -s]
        //     [0  s   c]
        T->data[0] = 1.0f; T->data[4] = 0.0f; T->data[8]  = 0.0f; T->data[12] = tx;
        T->data[1] = 0.0f; T->data[5] = c;    T->data[9]  = -s;   T->data[13] = 0.0f;
        T->data[2] = 0.0f; T->data[6] = s;    T->data[10] = c;    T->data[14] = tz;
        T->data[3] = 0.0f; T->data[7] = 0.0f; T->data[11] = 0.0f; T->data[15] = 1.0f;
    }
}

/**
 * @brief Multiply two 4x4 matrices
 */
void Mat4x4_Multiply(const Mat4x4_t *A, const Mat4x4_t *B, Mat4x4_t *C)
{
    if (A == NULL || B == NULL || C == NULL) {
        return;
    }
    
    Mat4x4_t temp;
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            temp.data[i + j*4] = 0.0f;
            for (int k = 0; k < 4; k++) {
                temp.data[i + j*4] += A->data[i + k*4] * B->data[k + j*4];
            }
        }
    }
    
    memcpy(C, &temp, sizeof(Mat4x4_t));
}

/**
 * @brief Extract pose from transformation matrix
 */
void Mat4x4_ToPose(const Mat4x4_t *T, CartesianPose_t *pose)
{
    if (T == NULL || pose == NULL) {
        return;
    }
    
    // Extract position
    pose->x = T->data[12];
    pose->y = T->data[13];
    pose->z = T->data[14];
    
    // Extract rotation matrix
    float R[9];
    R[0] = T->data[0]; R[3] = T->data[4]; R[6] = T->data[8];
    R[1] = T->data[1]; R[4] = T->data[5]; R[7] = T->data[9];
    R[2] = T->data[2]; R[5] = T->data[6]; R[8] = T->data[10];
    
    // Extract Euler angles
    ExtractEulerAngles(R, &pose->roll, &pose->pitch, &pose->yaw);
}

/**
 * @brief Build transformation matrix from pose
 */
void Pose_ToMat4x4(const CartesianPose_t *pose, Mat4x4_t *T)
{
    if (pose == NULL || T == NULL) {
        return;
    }
    
    float Rx[9], Ry[9], Rz[9], Rtemp[9], R[9];
    
    // Build rotation matrices
    RotationMatrix_X(pose->roll, Rx);
    RotationMatrix_Y(pose->pitch, Ry);
    RotationMatrix_Z(pose->yaw, Rz);
    
    // Combine: R = Rz * Ry * Rx
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Rtemp[i*3 + j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                Rtemp[i*3 + j] += Ry[i*3 + k] * Rx[k*3 + j];
            }
        }
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i*3 + j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                R[i*3 + j] += Rz[i*3 + k] * Rtemp[k*3 + j];
            }
        }
    }
    
    // Build transformation matrix
    T->data[0] = R[0]; T->data[4] = R[3]; T->data[8]  = R[6]; T->data[12] = pose->x;
    T->data[1] = R[1]; T->data[5] = R[4]; T->data[9]  = R[7]; T->data[13] = pose->y;
    T->data[2] = R[2]; T->data[6] = R[5]; T->data[10] = R[8]; T->data[14] = pose->z;
    T->data[3] = 0.0f; T->data[7] = 0.0f; T->data[11] = 0.0f; T->data[15] = 1.0f;
}

/**
 * @brief Compute angle-axis error
 */
void Compute_AngleAxisError(const Mat4x4_t *T_current, 
                             const Mat4x4_t *T_desired,
                             float error[6])
{
    if (T_current == NULL || T_desired == NULL || error == NULL) {
        return;
    }
    
    // Position error
    error[0] = T_desired->data[12] - T_current->data[12];  // dx
    error[1] = T_desired->data[13] - T_current->data[13];  // dy
    error[2] = T_desired->data[14] - T_current->data[14];  // dz
    
    // Orientation error (simplified - angle difference)
    // Extract rotation matrices
    float R_curr[9], R_des[9];
    R_curr[0] = T_current->data[0]; R_curr[3] = T_current->data[4]; R_curr[6] = T_current->data[8];
    R_curr[1] = T_current->data[1]; R_curr[4] = T_current->data[5]; R_curr[7] = T_current->data[9];
    R_curr[2] = T_current->data[2]; R_curr[5] = T_current->data[6]; R_curr[8] = T_current->data[10];
    
    R_des[0] = T_desired->data[0]; R_des[3] = T_desired->data[4]; R_des[6] = T_desired->data[8];
    R_des[1] = T_desired->data[1]; R_des[4] = T_desired->data[5]; R_des[7] = T_desired->data[9];
    R_des[2] = T_desired->data[2]; R_des[5] = T_desired->data[6]; R_des[8] = T_desired->data[10];
    
    // Simplified orientation error (for 4-DOF, focus on position)
    error[3] = 0.0f;  // Roll error (not controlled in 4-DOF)
    error[4] = 0.0f;  // Pitch error
    error[5] = 0.0f;  // Yaw error
}

/**
 * @brief IK solver using Levenberg-Marquardt
 */
static bool IK_Solve_LM(const CartesianPose_t *target, const JointAngles_t *q0, 
                        const IK_Config_t *cfg, IK_Solution_t *sol)
{
    JointAngles_t q_current = *q0;
    Mat4x4_t T_target, T_current;
    float error[6];
    Jacobian_t J;
    
    // Build target transformation matrix
    Pose_ToMat4x4(target, &T_target);
    
    sol->searches = 0;
    sol->iterations = 0;
    
    // Main search loop
    for (uint8_t search = 0; search < cfg->max_searches; search++) {
        sol->searches = search + 1;
        
        // Iteration loop
        for (uint8_t iter = 0; iter < cfg->max_iterations; iter++) {
            sol->iterations++;
            
            // Compute current end-effector pose
            FK_ComputeMatrix(&q_current, &T_current);
            
            // Compute error
            Compute_AngleAxisError(&T_current, &T_target, error);
            
            // Compute residual (position error only for 4-DOF)
                    float residual_sq = error[0]*error[0] + error[1]*error[1] + error[2]*error[2];
                    sol->residual = sqrtf(residual_sq);
            
            // Check convergence
            if (sol->residual < cfg->tolerance) {
                sol->success = true;
                sol->q = q_current;
                
                // Normalize angles
                for (uint8_t i = 0; i < IK_NUM_JOINTS; i++) {
                    sol->q.q[i] = Kinematics_NormalizeAngle(sol->q.q[i]);
                }
                
                // Check limits if enforced
                if (cfg->enforce_limits && !Kinematics_CheckJointLimits(&sol->q)) {
                    sol->success = false;
                    break;
                }
                
                return true;
            }
            
            // Compute Jacobian
            Jacobian_Numerical(&q_current, &J);
            
            // Compute update: dq = J^T * error (simplified, should use pseudoinverse)
            // For now, use simple gradient descent with damping
            for (uint8_t i = 0; i < IK_NUM_JOINTS; i++) {
                float grad = 0.0f;
                for (uint8_t j = 0; j < 3; j++) {  // Only position components
                    grad += J.data[j * IK_NUM_JOINTS + i] * error[j];
                }
                
                float dq = grad * cfg->lambda;
                
                // Limit joint change per iteration
                if (dq > IK_MAX_JOINT_DELTA) dq = IK_MAX_JOINT_DELTA;
                if (dq < -IK_MAX_JOINT_DELTA) dq = -IK_MAX_JOINT_DELTA;
                
                q_current.q[i] += dq;
            }
            
            // Clamp to limits during iteration
            if (cfg->enforce_limits) {
                Kinematics_ClampJointLimits(&q_current);
            }
        }
        
        // If not converged, try random restart
            if (search < cfg->max_searches - 1) {
            q_current.q[0] = ((float)rand() / RAND_MAX) * 3.34159f - 0.2f;
            q_current.q[1] = ((float)rand() / RAND_MAX) * 3.0f - 1.5f;
            q_current.q[2] = ((float)rand() / RAND_MAX) * 6.28318f - 3.14159f;
            q_current.q[3] = ((float)rand() / RAND_MAX) * 6.28318f - 3.14159f;
        }
    }
    
    sol->success = false;
    sol->q = q_current;
    return false;
}

/**
 * @brief Compute Jacobian using numerical differentiation
 */
static void Jacobian_Numerical(const JointAngles_t *q, Jacobian_t *J)
{
    CartesianPose_t pose_base, pose_perturbed;
    JointAngles_t q_perturbed;
    
    // Get base pose
    FK_Compute(q, &pose_base);
    
    // Compute numerical derivatives for each joint
    for (uint8_t i = 0; i < IK_NUM_JOINTS; i++) {
        q_perturbed = *q;
        q_perturbed.q[i] += JACOBIAN_DELTA;
        
        FK_Compute(&q_perturbed, &pose_perturbed);
        
        // J[0-2, i] = d(position) / dq[i]
        J->data[0 * IK_NUM_JOINTS + i] = (pose_perturbed.x - pose_base.x) / JACOBIAN_DELTA;
        J->data[1 * IK_NUM_JOINTS + i] = (pose_perturbed.y - pose_base.y) / JACOBIAN_DELTA;
        J->data[2 * IK_NUM_JOINTS + i] = (pose_perturbed.z - pose_base.z) / JACOBIAN_DELTA;
        
        // J[3-5, i] = d(orientation) / dq[i] (simplified for 4-DOF)
        J->data[3 * IK_NUM_JOINTS + i] = 0.0f;
        J->data[4 * IK_NUM_JOINTS + i] = 0.0f;
        J->data[5 * IK_NUM_JOINTS + i] = 0.0f;
    }
}

/* Private helper functions --------------------------------------------------*/

static void SetIdentityMatrix(Mat4x4_t *T)
{
    memset(T->data, 0, sizeof(T->data));
    T->data[0] = 1.0f;
    T->data[5] = 1.0f;
    T->data[10] = 1.0f;
    T->data[15] = 1.0f;
}

static void RotationMatrix_X(float angle, float R[9])
{
    float c = cosf(angle);
    float s = sinf(angle);
    R[0] = 1.0f; R[3] = 0.0f; R[6] = 0.0f;
    R[1] = 0.0f; R[4] = c;    R[7] = -s;
    R[2] = 0.0f; R[5] = s;    R[8] = c;
}

static void RotationMatrix_Y(float angle, float R[9])
{
    float c = cosf(angle);
    float s = sinf(angle);
    R[0] = c;    R[3] = 0.0f; R[6] = s;
    R[1] = 0.0f; R[4] = 1.0f; R[7] = 0.0f;
    R[2] = -s;   R[5] = 0.0f; R[8] = c;
}

static void RotationMatrix_Z(float angle, float R[9])
{
    float c = cosf(angle);
    float s = sinf(angle);
    R[0] = c;    R[3] = -s;   R[6] = 0.0f;
    R[1] = s;    R[4] = c;    R[7] = 0.0f;
    R[2] = 0.0f; R[5] = 0.0f; R[8] = 1.0f;
}

static void ExtractEulerAngles(const float R[9], float *roll, float *pitch, float *yaw)
{
    /* Extract XYZ Euler angles from rotation matrix */
    float sy = sqrtf(R[0]*R[0] + R[1]*R[1]);
    bool singular = sy < EPSILON;

    if (!singular) {
        *roll = atan2f(R[7], R[8]);
        *pitch = atan2f(-R[6], sy);
        *yaw = atan2f(R[3], R[0]);
    } else {
        *roll = atan2f(-R[5], R[4]);
        *pitch = atan2f(-R[6], sy);
        *yaw = 0.0f;
    }
}
