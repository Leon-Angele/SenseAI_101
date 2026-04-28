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

/* Private variables ---------------------------------------------------------*/
static bool initialized = false;

/* Private function prototypes -----------------------------------------------*/
static void SetIdentityMatrix(Mat4x4_t *T);
static void RotationMatrix_X(float angle, float R[9]);
static void RotationMatrix_Y(float angle, float R[9]);
static void RotationMatrix_Z(float angle, float R[9]);
static void ExtractEulerAngles(const float R[9], float *roll, float *pitch, float *yaw);
static void BuildTranslation(float tx, float ty, float tz, Mat4x4_t *T);
static void BuildRotationY(float angle, Mat4x4_t *T);

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
    
    /* Elementary Transform Sequence (ETS) for SO-101
     * Chain: T = Tz(L1) * Ry(q1) * Tx(L2) * Ry(q2) * Tx(L3) * Ry(q3) * Tx(L4)
     * 
     * Joints:
     *  q[0] = Shoulder Lift (Ry)
     *  q[1] = Elbow Flex (Ry)
     *  q[2] = Wrist Pitch (Ry)
     *  q[3] = Wrist Roll (Rx) - not used in this 4-DOF position kinematics
     */
    
    Mat4x4_t T_base, T_rot, T_trans, T_temp1, T_temp2;
    
    // Start with base translation: Tx(L1_X_OFFSET) * Tz(L1_BASE_HEIGHT)
    BuildTranslation(L1_X_OFFSET, 0.0f, L1_BASE_HEIGHT, &T_base);
    
    // Apply shoulder lift: Ry(q1). q=0 is UP, q>0 is FORWARD
    BuildRotationY((PI/2.0f) - q->q[0], &T_rot);
    Mat4x4_Multiply(&T_base, &T_rot, &T_temp1);
    
    // Apply upper arm translation
    BuildTranslation(L2_UPPER_ARM, 0.0f, L2_Z_OFFSET, &T_trans);
    Mat4x4_Multiply(&T_temp1, &T_trans, &T_temp2);
    
    // Apply elbow flex: Negative q2 folds DOWN
    BuildRotationY(q->q[1], &T_rot);
    Mat4x4_Multiply(&T_temp2, &T_rot, &T_temp1);
    
    // Apply forearm translation
    BuildTranslation(L3_FOREARM, 0.0f, 0.0f, &T_trans);
    Mat4x4_Multiply(&T_temp1, &T_trans, &T_temp2);
    
    // Apply wrist pitch: Maintain orientation logic
    BuildRotationY(q->q[2], &T_rot);
    Mat4x4_Multiply(&T_temp2, &T_rot, &T_temp1);
    
    // Apply wrist to TCP translation: Tx(L4_WRIST_TCP)
    BuildTranslation(L4_WRIST_TCP, 0.0f, 0.0f, &T_trans);
    Mat4x4_Multiply(&T_temp1, &T_trans, T);
    
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
    
    // Initialize solution
    memset(solution, 0, sizeof(IK_Solution_t));
    solution->success = false;
    solution->iterations = 1;  // Analytical IK is non-iterative
    solution->searches = 1;
    solution->residual = 0.0f;
    
    // Use default config if not provided
    bool enforce_limits = true;
    if (config != NULL) {
        enforce_limits = config->enforce_limits;
    }
    
    /* Analytical Inverse Kinematics using Geometric Approach
     * 
     * This is a closed-form solution for a 4-DOF arm with:
     * - All revolute joints rotating about Y-axis (pitch joints)
     * - Serial chain: Base -> Shoulder -> Elbow -> Wrist -> TCP
     * 
     * Strategy:
     * 1. Wrist decoupling: Calculate wrist position from target and pitch
     * 2. 2D projection: Reduce to planar 2-link problem (shoulder + elbow)
     * 3. Law of cosines: Solve for elbow angle
     * 4. Trigonometry: Solve for shoulder angle
     * 5. Pitch preservation: Calculate wrist angle to maintain end-effector orientation
     */
    
    float x = target_pose->x;
    float y = target_pose->y;
    float z = target_pose->z;
    float target_pitch = target_pose->pitch;
    
    // Joint angles
    float q1, q2, q3;
    
    // Step 1: Calculate base rotation (not used in 4-DOF IK, set to 0)
    // Note: Base pan is handled separately in trajectory planning
    // For now, we work in the 2D plane defined by the target
    
    // Step 2: Project to 2D (calculate radial distance in XY plane)
    float r_xy = sqrtf(x*x + y*y);
    
    // If target is directly above/below (r_xy ≈ 0), set base angle to 0
    // In practice, trajectory planner should handle base rotation separately
    
    // Step 3: Wrist decoupling
    // Calculate wrist position by subtracting wrist-to-TCP offset
    // Wrist offset depends on target pitch angle
#ifdef USE_CMSIS_DSP
    float cos_pitch = arm_cos_f32(target_pitch);
    float sin_pitch = arm_sin_f32(target_pitch);
#else
    float cos_pitch = cosf(target_pitch);
    float sin_pitch = sinf(target_pitch);
#endif
    
    // R_wrist reduced by static base X-offset so arm can tilt backward when r_xy is small
    float r_wrist = r_xy - L4_WRIST_TCP * cos_pitch - L1_X_OFFSET;
    float z_wrist = z - L4_WRIST_TCP * sin_pitch - L1_BASE_HEIGHT;

    // Account for fixed Z-offset at upper arm when projecting to shoulder frame
    // The vertical offset L2_Z_OFFSET is part of the static geometry
    
    // Step 4: Check reachability
    // Distance from shoulder to wrist in 2D
    float d = sqrtf(r_wrist*r_wrist + z_wrist*z_wrist);

    // Effective upper-arm length accounting for vertical offset
    float L2_eff = sqrtf(L2_UPPER_ARM * L2_UPPER_ARM + L2_Z_OFFSET * L2_Z_OFFSET);

    float reach_max = L2_eff + L3_FOREARM;
    float reach_min = fabsf(L2_eff - L3_FOREARM);
    
    if (d > reach_max + EPSILON || d < reach_min - EPSILON) {
        // Target is unreachable
        solution->success = false;
        solution->residual = (d > reach_max) ? (d - reach_max) : (reach_min - d);
        return false;
    }
    
    // Clamp d to valid range (handle numerical errors)
    if (d > reach_max) d = reach_max;
    if (d < reach_min) d = reach_min;
    
    // Step 5: Solve for elbow angle using law of cosines
    float cos_gamma = (d*d - L2_eff*L2_eff - L3_FOREARM*L3_FOREARM) / (2.0f * L2_eff * L3_FOREARM);
    if (cos_gamma > 1.0f) cos_gamma = 1.0f;
    if (cos_gamma < -1.0f) cos_gamma = -1.0f;
    
    // Calculate positive geometric folding angle
#ifdef USE_CMSIS_DSP
    float gamma_eff = acosf(cos_gamma);
#else
    float gamma_eff = acosf(cos_gamma);
#endif
    
    // Step 6: Solve for shoulder angle
#ifdef USE_CMSIS_DSP
    float sin_gamma = arm_sin_f32(gamma_eff);
    float cos_gamma_val = arm_cos_f32(gamma_eff);
#else
    float sin_gamma = sinf(gamma_eff);
    float cos_gamma_val = cosf(gamma_eff);
#endif
    
    float alpha = atan2f(z_wrist, r_wrist);
    float beta = atan2f(L3_FOREARM * sin_gamma, L2_eff + L3_FOREARM * cos_gamma_val);
    float offset_angle = atan2f(L2_Z_OFFSET, L2_UPPER_ARM);
    
    // Absolute angle of upper arm from X-axis
    float theta1 = alpha + beta - offset_angle;
    
    // Map to joint angles based on physical convention
    // q1: 0 = Vertical, Positive = Forward
    q1 = (PI/2.0f) - theta1;
    
    // q2: 0 = Straight, Negative = Fold Down/Inward
    // CRITICAL FIX: Das Vorzeichen des Offsets muss positiv sein!
    q2 = -gamma_eff + offset_angle;
    
    // Step 7: Solve for wrist pitch to maintain end-effector orientation
    q3 = target_pitch - theta1 - q2;
    
    // Step 8: Normalize angles to [-π, π]
    q1 = Kinematics_NormalizeAngle(q1);
    q2 = Kinematics_NormalizeAngle(q2);
    q3 = Kinematics_NormalizeAngle(q3);
    
    // Step 9: Store solution
    solution->q.q[0] = q1;  // Shoulder lift
    solution->q.q[1] = q2;  // Elbow flex
    solution->q.q[2] = q3;  // Wrist pitch
    solution->q.q[3] = 0.0f;  // Wrist roll (not used in position IK)
    
    // Step 10: Check joint limits
    if (enforce_limits) {
        if (!Kinematics_CheckJointLimits(&solution->q)) {
            solution->success = false;
            return false;
        }
    }
    
    // Success!
    solution->success = true;
    solution->residual = 0.0f;  // Analytical solution has zero error
    
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
    
    // Note: These parameters are deprecated for analytical IK
    // They are kept for API compatibility only
    config->max_iterations = 1;      // Not used (analytical IK is non-iterative)
    config->max_searches = 1;        // Not used
    config->tolerance = 0.0f;        // Not used (analytical IK has zero error)
    config->lambda = 0.0f;           // Not used (no damping needed)
    config->enforce_limits = true;   // Still used
}

/* Internal helper functions -------------------------------------------------*/

/**
 * @brief Build translation matrix
 */
static void BuildTranslation(float tx, float ty, float tz, Mat4x4_t *T)
{
    if (T == NULL) {
        return;
    }
    
    SetIdentityMatrix(T);
    T->data[12] = tx;
    T->data[13] = ty;
    T->data[14] = tz;
}

/**
 * @brief Build rotation matrix around Y axis
 */
static void BuildRotationY(float angle, Mat4x4_t *T)
{
    if (T == NULL) {
        return;
    }
    
    /* Use CMSIS-DSP trig functions when enabled for FPU-optimized math */
#ifdef USE_CMSIS_DSP
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
#else
    float c = cosf(angle);
    float s = sinf(angle);
#endif
    
    SetIdentityMatrix(T);
    
    // Rotation around Y axis
    // R = [ c  0  s]
    //     [ 0  1  0]
    //     [-s  0  c]
    T->data[0] = c;
    T->data[2] = -s;
    T->data[8] = s;
    T->data[10] = c;
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
    // Simplified orientation error (for 4-DOF, focus on position)
    error[3] = 0.0f;  // Roll error (not controlled in 4-DOF)
    error[4] = 0.0f;  // Pitch error
    error[5] = 0.0f;  // Yaw error
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
