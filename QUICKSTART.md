# SO-101 Robot Arm Control - Quick Start Guide

## Project Structure

```
SenseAI_101/
├── Core/
│   ├── Inc/
│   │   ├── servo_protocol.h      ← SCS protocol interface
│   │   ├── kinematics.h          ← IK/FK interface
│   │   ├── servo_mapping.h       ← Calibration & angle mapping
│   │   ├── trajectory.h          ← Motion planning interface
│   │   ├── main.h                ← Main application header
│   │   └── stm32f4xx_hal_conf.h  ← HAL configuration
│   │
│   ├── Src/
│   │   ├── servo_protocol.c      ← SCS protocol implementation
│   │   ├── kinematics.c          ← IK/FK implementation
│   │   ├── servo_mapping.c       ← Calibration implementation
│   │   ├── trajectory.c          ← Motion planning implementation
│   │   ├── main.c                ← Main application (MODIFIED)
│   │   └── stm32f4xx_it.c        ← Interrupt handlers
│   │
│   └── so101_IK/                 ← Python reference implementation
│       ├── src/
│       │   ├── robot.py          ← DH parameters reference
│       │   └── feetech_arm.py    ← Servo mapping reference
│       └── so101/
│           ├── calibration.json  ← Servo calibration data
│           └── so101.urdf        ← Joint limits
│
├── Drivers/                      ← STM32 HAL drivers
├── CMakeLists.txt
└── README.md                     ← Full documentation
```

---

## 5-Minute Getting Started

### Step 1: Hardware Setup
```
STM32 NUCLEO-F446RE
    PA9 (UART1) ──→ Servo Data Line
    GND ──────────→ Servo GND
    
External 7.4V PSU ──→ Servo VCC (5A min)
```

### Step 2: Flash Firmware
1. Open project in STM32CubeIDE
2. Build (Ctrl+B)
3. Flash to board (F11)

### Step 3: Power On
1. Connect USB to STM32 (for programming/debug)
2. Connect 7.4V to servos
3. Watch LD2 LED:
   - Blinks during servo ping (6x rapid blinks)
   - Slow blink during operation (0.5s interval)

### Step 4: Verify Operation
Arm should:
1. Move to home position (mid-range)
2. Wait 3 seconds
3. Execute demo trajectory (4 poses in loop)

---

## Code Examples

### Minimal Example (No IK)
```c
#include "main.h"
#include "servo_protocol.h"
#include "trajectory.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_UART1_Init();
    
    // Initialize
    Servo_Init(&servo_protocol, &huart1);
    Trajectory_Init(&trajectory_controller, &servo_protocol);
    Trajectory_Enable(&trajectory_controller, true);
    
    // Move to joint angles
    JointAngles_t target = {1.5f, 0.5f, -0.3f, 0.0f};
    Trajectory_MoveToJoints(&trajectory_controller, &target, 
                            0.0f, 50.0f, 2000);
    
    while (1) {
        Trajectory_Update(&trajectory_controller);
        HAL_Delay(20);  // 50Hz
    }
}
```

### Cartesian Control Example
```c
// Move end-effector to X=20cm, Y=5cm, Z=15cm
CartesianPose_t pose = {
    .x = 0.20f, .y = 0.05f, .z = 0.15f,
    .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f
};

Trajectory_MoveToPose(&trajectory_controller, &pose, 
                      0.0f,    // Base rotation
                      80.0f,   // Gripper 80% open
                      2000);   // 2s duration
```

---

## Testing Checklist

- [ ] **Servo Ping Test**: All 6 servos respond to ping
- [ ] **Home Position**: Arm moves to mid-range safely
- [ ] **Single Joint**: Move one joint at a time
- [ ] **Cartesian Move**: IK solves and arm reaches target
- [ ] **Smooth Motion**: No jerky movements (check 50Hz update)
- [ ] **Gripper Control**: Opens/closes correctly
- [ ] **Emergency Stop**: Can halt motion safely

---

## Common Issues & Quick Fixes

| Issue | Quick Fix |
|-------|-----------|
| No servo response | Check baud rate = 1000000, verify power supply |
| IK fails | Target out of reach, try closer position |
| Jerky motion | Ensure Trajectory_Update() at 50Hz |
| Wrong position | Verify calibration data matches your servos |
| Arm doesn't move | Check torque enable, verify UART half-duplex |

---

## Key Parameters to Tune

### For Your Specific Arm
Located in: `servo_mapping.c`
```c
static const ServoCalib_t servo_calibration[NUM_SERVOS] = {
    // Adjust homing_offset if arm doesn't center correctly
    { .homing_offset = 2030, ... },  // Servo 0
    { .homing_offset = 3023, ... },  // Servo 1
    // ...
};
```

### For IK Performance
Located in: `kinematics.h`
```c
#define IK_MAX_ITERATIONS       10    // ↑ = slower, more accurate
#define IK_TOLERANCE            1e-3f // ↓ = more precise, slower
#define IK_DAMPING_LAMBDA       0.01f // ↑ = more stable, slower
```

### For Motion Smoothness
Located in: `trajectory.h`
```c
#define TRAJECTORY_FREQ_HZ      50    // Control loop frequency
#define MAX_JOINT_VELOCITY      1.0f  // rad/s (↓ = smoother)
```

---

## Next Steps

1. **Test with your arm**: Adjust calibration if needed
2. **Tune IK parameters**: Balance speed vs accuracy
3. **Create custom trajectories**: See examples in main.c
4. **Add safety limits**: Implement workspace boundaries
5. **Integrate Cube.AI**: Add neural network inference

---

## Module Dependencies

```
main.c
  ├─→ trajectory.h
  │     ├─→ kinematics.h (IK/FK)
  │     ├─→ servo_mapping.h (calibration)
  │     └─→ servo_protocol.h (UART)
  │           └─→ STM32 HAL (DMA, UART)
  │
  └─→ CMSIS-DSP (arm_math.h)
```

---

## Performance Targets

✓ IK computation: <10ms (target: 5-8ms)  
✓ Control loop: 50Hz (20ms period)  
✓ Servo latency: <20ms  
✓ CPU utilization: <30% (70% free for Cube.AI)  
✓ Position accuracy: ±1mm (with good calibration)  

---

## Debugging Tips

### Enable Debug Output (optional)
Add to `main.c`:
```c
// In main loop
if (Trajectory_Update(&trajectory_controller) == TRAJ_ERROR_IK_FAILED) {
    // IK failed, print debug info via UART2
    printf("IK failed, residual: %f\r\n", 
           trajectory_controller.last_ik_solution.residual);
}
```

### Monitor Servo Status
```c
// Read current servo positions
uint16_t pos;
for (uint8_t i = 1; i <= 6; i++) {
    if (Servo_ReadPosition(&servo_protocol, i, &pos) == SERVO_OK) {
        printf("Servo %d: %d\r\n", i, pos);
    }
}
```

---

**Ready to control your robot arm! 🚀**

For detailed API reference, see [README.md](README.md)
