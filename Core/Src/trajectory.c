/**
 ******************************************************************************
 * @file           : trajectory.c
 * @brief          : Trajectory Planning and Motion Control Implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "trajectory.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define MIN_DURATION_MS     20      // Minimum motion duration (one update cycle)
#define MAX_DURATION_MS     10000   // Maximum motion duration (10 seconds)

/* Private function prototypes -----------------------------------------------*/
static bool Trajectory_PopCommand(TrajectoryController_t *ctrl, MotionCommand_t *cmd);
static void Trajectory_StartInterpolation(TrajectoryController_t *ctrl, const MotionCommand_t *cmd);
static void Trajectory_UpdateInterpolation(TrajectoryController_t *ctrl);
static void Trajectory_ExecuteMotion(TrajectoryController_t *ctrl);
static TrajectoryStatus_t Trajectory_ComputeIK(TrajectoryController_t *ctrl, const CartesianPose_t *pose);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize trajectory controller
 */
TrajectoryStatus_t Trajectory_Init(TrajectoryController_t *controller,
                                    ServoProtocol_t *servo_protocol)
{
    if (controller == NULL || servo_protocol == NULL) {
        return TRAJ_ERROR_NOT_INITIALIZED;
    }
    
    memset(controller, 0, sizeof(TrajectoryController_t));
    
    controller->servo_protocol = servo_protocol;
    controller->state = TRAJ_STATE_IDLE;
    controller->queue_head = 0;
    controller->queue_tail = 0;
    controller->queue_count = 0;
    controller->enabled = false;
    controller->initialized = true;
    
    // Initialize to safe mid-range position
    controller->interpolator.q_current.q[0] = 0.0f;
    controller->interpolator.q_current.q[1] = 0.0f;
    controller->interpolator.q_current.q[2] = 0.0f;
    controller->interpolator.q_current.q[3] = 0.0f;
    controller->interpolator.base_current = 0.0f;
    controller->interpolator.gripper_current = 0.0f;
    
    return TRAJ_OK;
}

/**
 * @brief Enable/disable trajectory controller
 */
void Trajectory_Enable(TrajectoryController_t *controller, bool enable)
{
    if (controller != NULL && controller->initialized) {
        controller->enabled = enable;
        
        if (!enable) {
            // Stop motion when disabled
            controller->state = TRAJ_STATE_IDLE;
        }
    }
}

/**
 * @brief Add motion command to queue
 */
TrajectoryStatus_t Trajectory_AddCommand(TrajectoryController_t *controller,
                                          const MotionCommand_t *cmd)
{
    if (!controller->initialized) {
        return TRAJ_ERROR_NOT_INITIALIZED;
    }
    
    if (cmd == NULL) {
        return TRAJ_ERROR_BUSY;
    }
    
    // Check if queue is full
    if (controller->queue_count >= TRAJECTORY_QUEUE_SIZE) {
        return TRAJ_ERROR_QUEUE_FULL;
    }
    
    // Add command to queue
    controller->command_queue[controller->queue_tail] = *cmd;
    controller->queue_tail = (controller->queue_tail + 1) % TRAJECTORY_QUEUE_SIZE;
    controller->queue_count++;
    
    return TRAJ_OK;
}

/**
 * @brief Move to Cartesian pose
 */
TrajectoryStatus_t Trajectory_MoveToPose(TrajectoryController_t *controller,
                                          const CartesianPose_t *pose,
                                          float base_rotation,
                                          float gripper_percent,
                                          uint16_t duration_ms)
{
    if (!controller->initialized || pose == NULL) {
        return TRAJ_ERROR_NOT_INITIALIZED;
    }
    
    MotionCommand_t cmd;
    cmd.type = CMD_TYPE_CARTESIAN_POSE;
    cmd.cartesian_cmd.pose = *pose;
    // AUTO-BASE: If caller passed base_rotation==0.0f, compute from X/Y when available
    if (base_rotation == 0.0f && (fabsf(pose->x) > 0.001f || fabsf(pose->y) > 0.001f)) {
        cmd.cartesian_cmd.base_rotation = atan2f(pose->y, pose->x);
    } else {
        cmd.cartesian_cmd.base_rotation = base_rotation;
    }
    cmd.gripper_percent = gripper_percent;
    cmd.duration_ms = duration_ms;
    cmd.wait_for_completion = false;
    
    return Trajectory_AddCommand(controller, &cmd);
}

/**
 * @brief Move to joint angles
 */
TrajectoryStatus_t Trajectory_MoveToJoints(TrajectoryController_t *controller,
                                            const JointAngles_t *angles,
                                            float base_rotation,
                                            float gripper_percent,
                                            uint16_t duration_ms)
{
    if (!controller->initialized || angles == NULL) {
        return TRAJ_ERROR_NOT_INITIALIZED;
    }
    
    MotionCommand_t cmd;
    cmd.type = CMD_TYPE_JOINT_ANGLES;
    cmd.joint_cmd.angles = *angles;
    cmd.joint_cmd.base_rotation = base_rotation;
    cmd.gripper_percent = gripper_percent;
    cmd.duration_ms = duration_ms;
    cmd.wait_for_completion = false;
    
    return Trajectory_AddCommand(controller, &cmd);
}

/**
 * @brief Set gripper position
 */
TrajectoryStatus_t Trajectory_SetGripper(TrajectoryController_t *controller,
                                          float percent,
                                          uint16_t duration_ms)
{
    if (!controller->initialized) {
        return TRAJ_ERROR_NOT_INITIALIZED;
    }
    
    MotionCommand_t cmd;
    cmd.type = CMD_TYPE_GRIPPER_ONLY;
    cmd.gripper_cmd.percent = percent;
    cmd.gripper_percent = percent;
    cmd.duration_ms = duration_ms;
    cmd.wait_for_completion = false;
    
    return Trajectory_AddCommand(controller, &cmd);
}

/**
 * @brief Emergency stop
 */
void Trajectory_EmergencyStop(TrajectoryController_t *controller)
{
    if (!controller->initialized) {
        return;
    }
    
    // Clear queue
    Trajectory_ClearQueue(controller);
    
    // Stop current motion
    controller->state = TRAJ_STATE_IDLE;
    controller->ik_in_progress = false;
    
    // Disable servos (optional - comment out to maintain position)
    // for (uint8_t i = 1; i <= 6; i++) {
    //     Servo_SetTorqueEnable(controller->servo_protocol, i, false);
    // }
}

/**
 * @brief Update trajectory controller (call from timer ISR)
 */
TrajectoryStatus_t Trajectory_Update(TrajectoryController_t *controller)
{
    if (!controller->initialized || !controller->enabled) {
        return TRAJ_ERROR_NOT_INITIALIZED;
    }
    
    controller->update_count++;
    controller->last_update_time = HAL_GetTick();
    
    // State machine
    switch (controller->state) {
        case TRAJ_STATE_IDLE:
            // Check if there are commands in queue
            if (controller->queue_count > 0) {
                MotionCommand_t cmd;
                if (Trajectory_PopCommand(controller, &cmd)) {
                    
                    if (cmd.type == CMD_TYPE_CARTESIAN_POSE) {
                        // Compute IK synchronously (analytical IK is fast, no separate state needed)
                        TrajectoryStatus_t status = Trajectory_ComputeIK(controller, &cmd.cartesian_cmd.pose);
                        
                        if (status == TRAJ_OK && controller->last_ik_solution.success) {
                            // IK succeeded, prepare joint command and start interpolation
                            MotionCommand_t joint_cmd;
                            joint_cmd.type = CMD_TYPE_JOINT_ANGLES;
                            joint_cmd.joint_cmd.angles = controller->last_ik_solution.q;
                            joint_cmd.joint_cmd.base_rotation = cmd.cartesian_cmd.base_rotation;
                            joint_cmd.gripper_percent = cmd.gripper_percent;
                            joint_cmd.duration_ms = cmd.duration_ms;
                            
                            Trajectory_StartInterpolation(controller, &joint_cmd);
                            controller->state = TRAJ_STATE_INTERPOLATING;
                        } else {
                            // IK failed - target unreachable
                            // Stay in IDLE, command is discarded
                            // Optional: Log error or increment error counter
                            controller->state = TRAJ_STATE_IDLE;
                        }
                    } else {
                        // Direct joint command or gripper, start interpolation immediately
                        Trajectory_StartInterpolation(controller, &cmd);
                        controller->state = TRAJ_STATE_INTERPOLATING;
                    }
                }
            }
            break;
            
        case TRAJ_STATE_INTERPOLATING:
            // Update interpolation
            Trajectory_UpdateInterpolation(controller);
            
            // Execute motion
            Trajectory_ExecuteMotion(controller);
            
            // Check if interpolation is complete
            if (controller->interpolator.interpolation_factor >= 1.0f) {
                controller->state = TRAJ_STATE_IDLE;
            }
            break;
            
        case TRAJ_STATE_EXECUTING:
            // Execution state (reserved for future use)
            controller->state = TRAJ_STATE_IDLE;
            break;
            
        case TRAJ_STATE_ERROR:
            // Clear error and go to idle
            controller->state = TRAJ_STATE_IDLE;
            return TRAJ_ERROR_IK_FAILED;
            
        default:
            controller->state = TRAJ_STATE_IDLE;
            break;
    }
    
    return TRAJ_OK;
}

/**
 * @brief Check if trajectory is idle
 */
bool Trajectory_IsIdle(const TrajectoryController_t *controller)
{
    if (!controller->initialized) {
        return true;
    }
    
    return (controller->state == TRAJ_STATE_IDLE && controller->queue_count == 0);
}

/**
 * @brief Get current joint positions
 */
void Trajectory_GetCurrentPosition(const TrajectoryController_t *controller,
                                    JointAngles_t *angles,
                                    float *base_rotation,
                                    float *gripper_percent)
{
    if (!controller->initialized) {
        return;
    }
    
    if (angles != NULL) {
        *angles = controller->interpolator.q_current;
    }
    
    if (base_rotation != NULL) {
        *base_rotation = controller->interpolator.base_current;
    }
    
    if (gripper_percent != NULL) {
        *gripper_percent = controller->interpolator.gripper_current;
    }
}

/**
 * @brief Get queue count
 */
uint8_t Trajectory_GetQueueCount(const TrajectoryController_t *controller)
{
    if (!controller->initialized) {
        return 0;
    }
    
    return controller->queue_count;
}

/**
 * @brief Clear command queue
 */
void Trajectory_ClearQueue(TrajectoryController_t *controller)
{
    if (!controller->initialized) {
        return;
    }
    
    controller->queue_head = 0;
    controller->queue_tail = 0;
    controller->queue_count = 0;
}

/* Helper functions ----------------------------------------------------------*/

/**
 * @brief Linear interpolation
 */
float Lerp(float start, float end, float t)
{
    if (t <= 0.0f) return start;
    if (t >= 1.0f) return end;
    return start + (end - start) * t;
}

/**
 * @brief S-curve interpolation (smoothstep)
 */
float Smoothstep(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Pop command from queue
 */
static bool Trajectory_PopCommand(TrajectoryController_t *ctrl, MotionCommand_t *cmd)
{
    if (ctrl->queue_count == 0) {
        return false;
    }
    
    *cmd = ctrl->command_queue[ctrl->queue_head];
    ctrl->queue_head = (ctrl->queue_head + 1) % TRAJECTORY_QUEUE_SIZE;
    ctrl->queue_count--;
    
    return true;
}

/**
 * @brief Start interpolation
 */
static void Trajectory_StartInterpolation(TrajectoryController_t *ctrl, const MotionCommand_t *cmd)
{
    // Set starting position to current position
    ctrl->interpolator.q_start = ctrl->interpolator.q_current;
    ctrl->interpolator.base_start = ctrl->interpolator.base_current;
    ctrl->interpolator.gripper_start = ctrl->interpolator.gripper_current;
    
    // Set target position
    if (cmd->type == CMD_TYPE_JOINT_ANGLES) {
        ctrl->interpolator.q_target = cmd->joint_cmd.angles;
        ctrl->interpolator.base_target = cmd->joint_cmd.base_rotation;
        ctrl->interpolator.gripper_target = cmd->gripper_percent;
    } else if (cmd->type == CMD_TYPE_GRIPPER_ONLY) {
        // Keep current joint angles, only change gripper
        ctrl->interpolator.q_target = ctrl->interpolator.q_current;
        ctrl->interpolator.base_target = ctrl->interpolator.base_current;
        ctrl->interpolator.gripper_target = cmd->gripper_cmd.percent;
    }
    
    // Set duration
    ctrl->interpolator.duration = cmd->duration_ms;
    if (ctrl->interpolator.duration < MIN_DURATION_MS) {
        ctrl->interpolator.duration = MIN_DURATION_MS;
    }
    
    // Reset interpolation state
    ctrl->interpolator.start_time = HAL_GetTick();
    ctrl->interpolator.elapsed = 0;
    ctrl->interpolator.interpolation_factor = 0.0f;
}

/**
 * @brief Update interpolation
 */
static void Trajectory_UpdateInterpolation(TrajectoryController_t *ctrl)
{
    uint32_t current_time = HAL_GetTick();
    ctrl->interpolator.elapsed = current_time - ctrl->interpolator.start_time;
    
    // Compute interpolation factor (0.0 to 1.0)
    if (ctrl->interpolator.duration > 0) {
        ctrl->interpolator.interpolation_factor = 
            (float)ctrl->interpolator.elapsed / (float)ctrl->interpolator.duration;
    } else {
        ctrl->interpolator.interpolation_factor = 1.0f;
    }
    
    // Clamp to [0, 1]
    if (ctrl->interpolator.interpolation_factor > 1.0f) {
        ctrl->interpolator.interpolation_factor = 1.0f;
    }
    
    // Apply smoothstep for smoother motion
    float t = Smoothstep(ctrl->interpolator.interpolation_factor);
    
    // Interpolate joint angles
    for (uint8_t i = 0; i < IK_NUM_JOINTS; i++) {
        ctrl->interpolator.q_current.q[i] = Lerp(
            ctrl->interpolator.q_start.q[i],
            ctrl->interpolator.q_target.q[i],
            t
        );
    }
    
    // Interpolate base rotation
    ctrl->interpolator.base_current = Lerp(
        ctrl->interpolator.base_start,
        ctrl->interpolator.base_target,
        t
    );
    
    // Interpolate gripper
    ctrl->interpolator.gripper_current = Lerp(
        ctrl->interpolator.gripper_start,
        ctrl->interpolator.gripper_target,
        t
    );
}

/**
 * @brief Execute motion (send to servos)
 */
static void Trajectory_ExecuteMotion(TrajectoryController_t *ctrl)
{
    // Convert joint angles to servo positions
    ServoPositions_t servo_pos;
    ServoMapping_IKToServoPositions(
        &ctrl->interpolator.q_current,
        ctrl->interpolator.base_current,
        ctrl->interpolator.gripper_current,
        &servo_pos
    );
    
    // Send positions to all servos using sync write
    uint8_t ids[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};
    
    Servo_SyncWritePosition(
        ctrl->servo_protocol,
        ids,
        servo_pos.pos,
        NULL,  // No time specification (use default speed)
        NUM_SERVOS
    );
}

/**
 * @brief Compute IK for Cartesian pose
 */
static TrajectoryStatus_t Trajectory_ComputeIK(TrajectoryController_t *ctrl, const CartesianPose_t *pose)
{
    IK_Config_t config;
    IK_GetDefaultConfig(&config);
    
    // Use current joint angles as initial guess
    CartesianPose_t test_pose = *pose;
    bool success = IK_Compute(
        &test_pose,
        &ctrl->interpolator.q_current,
        &config,
        &ctrl->last_ik_solution
    );

    // If initial attempt failed, search for a usable pitch angle
    if (!success || !ctrl->last_ik_solution.success) {
        // Try pitches from -90° to +45° in 0.1 rad steps
        for (float p = -1.57f; p <= 0.8f; p += 0.1f) {
            test_pose.pitch = p;
            success = IK_Compute(&test_pose, &ctrl->interpolator.q_current, &config, &ctrl->last_ik_solution);
            if (success && ctrl->last_ik_solution.success) {
                break; // found a working pitch
            }
        }
    }

    if (!success || !ctrl->last_ik_solution.success) {
        return TRAJ_ERROR_IK_FAILED;
    }

    return TRAJ_OK;
}
