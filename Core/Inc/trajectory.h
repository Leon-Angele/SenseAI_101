/**
 ******************************************************************************
 * @file           : trajectory.h
 * @brief          : Trajectory Planning and Smooth Motion Control
 ******************************************************************************
 * @attention
 *
 * Timer-based trajectory interpolation for smooth, non-blocking motion
 * Designed for 20-50Hz control loop with CPU headroom for Cube.AI
 *
 * Features:
 * - Linear interpolation between waypoints
 * - Trapezoidal velocity profile support
 * - Non-blocking state machine
 * - Position command queue
 *
 ******************************************************************************
 */

#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "kinematics.h"
#include "servo_mapping.h"
#include "servo_protocol.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

/* Trajectory control loop frequency */
#define TRAJECTORY_FREQ_HZ      50      // 50Hz = 20ms period
#define TRAJECTORY_PERIOD_MS    (1000 / TRAJECTORY_FREQ_HZ)

/* Command queue size */
#define TRAJECTORY_QUEUE_SIZE   8

/* Maximum velocity (rad/s per joint) */
#define MAX_JOINT_VELOCITY      1.0f    // 1 rad/s (~57 deg/s)

/* Maximum acceleration (rad/s² per joint) */
#define MAX_JOINT_ACCEL         2.0f    // 2 rad/s²

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Trajectory controller state
 */
typedef enum {
    TRAJ_STATE_IDLE = 0,
    TRAJ_STATE_COMPUTING_IK,
    TRAJ_STATE_INTERPOLATING,
    TRAJ_STATE_EXECUTING,
    TRAJ_STATE_ERROR
} TrajectoryState_t;

/**
 * @brief Motion command type
 */
typedef enum {
    CMD_TYPE_JOINT_ANGLES = 0,  // Direct joint angle command
    CMD_TYPE_CARTESIAN_POSE,    // Cartesian pose (requires IK)
    CMD_TYPE_GRIPPER_ONLY       // Gripper command only
} CommandType_t;

/**
 * @brief Motion command
 */
typedef struct {
    CommandType_t type;
    
    union {
        struct {
            JointAngles_t angles;       // For CMD_TYPE_JOINT_ANGLES
            float base_rotation;    // Base rotation (joint 0)
        } joint_cmd;
        
        struct {
            CartesianPose_t pose;       // For CMD_TYPE_CARTESIAN_POSE
            float base_rotation;    // Base rotation (joint 0)
        } cartesian_cmd;
        
        struct {
            float percent;          // For CMD_TYPE_GRIPPER_ONLY
        } gripper_cmd;
    };
    
    float gripper_percent;          // Gripper opening (0-100)
    uint16_t duration_ms;               // Motion duration (0 = immediate)
    bool wait_for_completion;           // Block until complete
} MotionCommand_t;

/**
 * @brief Trajectory interpolator state
 */
typedef struct {
    JointAngles_t q_start;      // Starting joint angles
    JointAngles_t q_target;     // Target joint angles
    JointAngles_t q_current;    // Current joint angles
    
    float base_start;       // Starting base rotation
    float base_target;      // Target base rotation
    float base_current;     // Current base rotation
    
    float gripper_start;    // Starting gripper position
    float gripper_target;   // Target gripper position
    float gripper_current;  // Current gripper position
    
    uint32_t start_time;        // Interpolation start time (ms)
    uint32_t duration;          // Total duration (ms)
    uint32_t elapsed;           // Elapsed time (ms)
    
    float interpolation_factor;  // 0.0 to 1.0
} TrajectoryInterpolator_t;

/**
 * @brief Trajectory controller context
 */
typedef struct {
    TrajectoryState_t state;
    ServoProtocol_t *servo_protocol;
    
    TrajectoryInterpolator_t interpolator;
    
    MotionCommand_t command_queue[TRAJECTORY_QUEUE_SIZE];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
    
    IK_Solution_t last_ik_solution;
    bool ik_in_progress;
    
    uint32_t last_update_time;
    uint32_t update_count;
    
    bool initialized;
    bool enabled;
} TrajectoryController_t;

/**
 * @brief Trajectory status
 */
typedef enum {
    TRAJ_OK = 0,
    TRAJ_ERROR_NOT_INITIALIZED,
    TRAJ_ERROR_QUEUE_FULL,
    TRAJ_ERROR_IK_FAILED,
    TRAJ_ERROR_SERVO_COMM,
    TRAJ_ERROR_BUSY
} TrajectoryStatus_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize trajectory controller
 * @param controller Pointer to controller context
 * @param servo_protocol Pointer to initialized servo protocol
 * @retval TrajectoryStatus_t
 */
TrajectoryStatus_t Trajectory_Init(TrajectoryController_t *controller,
                                    ServoProtocol_t *servo_protocol);

/**
 * @brief Enable/disable trajectory controller
 * @param controller Pointer to controller context
 * @param enable true to enable, false to disable
 */
void Trajectory_Enable(TrajectoryController_t *controller, bool enable);

/**
 * @brief Add motion command to queue
 * @param controller Pointer to controller context
 * @param cmd Pointer to motion command
 * @retval TrajectoryStatus_t
 */
TrajectoryStatus_t Trajectory_AddCommand(TrajectoryController_t *controller,
                                          const MotionCommand_t *cmd);

/**
 * @brief Move to Cartesian pose (adds command to queue)
 * @param controller Pointer to controller context
 * @param pose Target Cartesian pose
 * @param base_rotation Base rotation angle (radians)
 * @param gripper_percent Gripper opening (0-100)
 * @param duration_ms Motion duration (0 = immediate)
 * @retval TrajectoryStatus_t
 */
TrajectoryStatus_t Trajectory_MoveToPose(TrajectoryController_t *controller,
                                          const CartesianPose_t *pose,
                                          float base_rotation,
                                          float gripper_percent,
                                          uint16_t duration_ms);

/**
 * @brief Move to joint angles (adds command to queue)
 * @param controller Pointer to controller context
 * @param angles Target joint angles (4-DOF)
 * @param base_rotation Base rotation angle (radians)
 * @param gripper_percent Gripper opening (0-100)
 * @param duration_ms Motion duration (0 = immediate)
 * @retval TrajectoryStatus_t
 */
TrajectoryStatus_t Trajectory_MoveToJoints(TrajectoryController_t *controller,
                                            const JointAngles_t *angles,
                                            float base_rotation,
                                            float gripper_percent,
                                            uint16_t duration_ms);

/**
 * @brief Set gripper position only
 * @param controller Pointer to controller context
 * @param percent Gripper opening (0-100)
 * @param duration_ms Motion duration (0 = immediate)
 * @retval TrajectoryStatus_t
 */
TrajectoryStatus_t Trajectory_SetGripper(TrajectoryController_t *controller,
                                          float percent,
                                          uint16_t duration_ms);

/**
 * @brief Emergency stop - clear queue and halt motion
 * @param controller Pointer to controller context
 */
void Trajectory_EmergencyStop(TrajectoryController_t *controller);

/**
 * @brief Update trajectory controller (call from timer ISR at TRAJECTORY_FREQ_HZ)
 * @param controller Pointer to controller context
 * @retval TrajectoryStatus_t
 */
TrajectoryStatus_t Trajectory_Update(TrajectoryController_t *controller);

/**
 * @brief Check if trajectory is idle (no motion in progress)
 * @param controller Pointer to controller context
 * @retval true if idle
 */
bool Trajectory_IsIdle(const TrajectoryController_t *controller);

/**
 * @brief Get current joint positions
 * @param controller Pointer to controller context
 * @param angles Output current joint angles
 * @param base_rotation Output current base rotation
 * @param gripper_percent Output current gripper position
 */
void Trajectory_GetCurrentPosition(const TrajectoryController_t *controller,
                                    JointAngles_t *angles,
                                    float *base_rotation,
                                    float *gripper_percent);

/**
 * @brief Get number of commands in queue
 * @param controller Pointer to controller context
 * @retval Number of queued commands
 */
uint8_t Trajectory_GetQueueCount(const TrajectoryController_t *controller);

/**
 * @brief Clear command queue
 * @param controller Pointer to controller context
 */
void Trajectory_ClearQueue(TrajectoryController_t *controller);

/* Helper functions (exposed for testing) ------------------------------------*/

/**
 * @brief Linear interpolation between two values
 * @param start Start value
 * @param end End value
 * @param t Interpolation factor (0.0 to 1.0)
 * @retval Interpolated value
 */
float Lerp(float start, float end, float t);

/**
 * @brief S-curve (smoothstep) interpolation
 * @param t Input value (0.0 to 1.0)
 * @retval Smoothed value (0.0 to 1.0)
 */
float Smoothstep(float t);

#ifdef __cplusplus
}
#endif

#endif /* __TRAJECTORY_H */
