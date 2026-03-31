# Dobot Magician Python API Programming Guide

This document is a detailed guide to programming the Dobot Magician robot arm using the provided 64-bit Python API wrapper (`DobotDllType.py`). The library uses `ctypes` to interface with the `DobotDll.dll` C-library, allowing you to establish a connection, send movement commands, and interact with accessories like a linear rail.

---

## 1. Setup and Initialization

Every program controlling the Dobot must start by importing the wrapper, loading the dynamic link library (DLL), and establishing a connection over a serial port.

### Basic Connection Boilerplate
```python
import DobotDllType as dType

# Load the DLL and get the CDLL object
api = dType.load()

# Connect to the Dobot (Specify your COM port correctly, e.g., "COM9")
# Baudrate for the Dobot Magician is usually 115200
result = dType.ConnectDobot(api, "COM9", 115200)
state = result[0]

# Check if connection was successful
if state == dType.DobotConnect.DobotConnect_NoError:
    print("Dobot Connected Successfully!")
else:
    print("Failed to Connect.")
```

> Ensure the Python execution environment matches the DLL architecture (64-bit Python for a 64-bit DLL). If the connection fails, verify the COM port via your Device Manager.

---

## 2. The Command Queue Architecture

The Dobot controller works using a **Command Queue**. Instead of Python waiting synchronously for the arm to physically finish moving before going to the next line of code, you push "Queued" commands (by passing `isQueued=1`) rapidly to the controller. You then start the execution of the queue on the robot side, and make Python wait until the last command finishes.

### Typical Workflow:
1. Clear any old pending commands.
2. Queue motion parameters (speed, acceleration).
3. Queue movement commands.
4. Tell the robot to start executing the queue.
5. Poll the robot's current execution index until it matches the index of your last queued command.
6. Stop execution and cleanly disconnect.

```python
# 1. Clear old queue
dType.SetQueuedCmdClear(api)

# ... (Queue up parameters and movements here) ...
# Example: lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200, 0, 0, 0, isQueued=1)[0]

# 2. Start executing the queue
dType.SetQueuedCmdStartExec(api)

# 3. Wait until the robot reaches the `lastIndex`
while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
    dType.dSleep(100) # Sleep 100ms between checks

# 4. Stop execution
dType.SetQueuedCmdStopExec(api)
```

> **IMPORTANT**: If you don't call `SetQueuedCmdStartExec(api)`, the robot will receive the commands but will just sit idle!

---

## 3. Reading the Robot's Position

You can query the current position of the robot's end effector. This is useful for debugging or relative movements.

```python
# Returns [x, y, z, r, joint1Angle, joint2Angle, joint3Angle, joint4Angle]
position = dType.GetPose(api)

print(f"X: {position[0]:.2f} mm")
print(f"Y: {position[1]:.2f} mm")
print(f"Z: {position[2]:.2f} mm")
print(f"R (Rotation): {position[3]:.2f} degrees")

# If using a linear rail, you can get the rail coordinate
rail_pos = dType.GetPoseL(api)
print(f"Rail Position: {rail_pos[0]:.2f} mm")
```

---

## 4. Configuring Motion Speeds

Before sending movement commands, you must configure the speeds and accelerations. 
* **Joint Params**: Dictates the speed of individual motors.
* **Common Params**: Dictates the general velocity and acceleration ratios.

```python
# Set velocity and acceleration for joints 1-4
# dType.SetPTPJointParams(api, j1_vel, j2_vel, j3_vel, j4_vel, j1_acc, j2_acc, j3_acc, j4_acc, isQueued=1)
dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)

# Set general coordinate velocity and acceleration ratio (0-100)
# dType.SetPTPCommonParams(api, velocityRatio, accelerationRatio, isQueued=1)
dType.SetPTPCommonParams(api, 100, 100, isQueued=1)
```

---

## 5. Basic Point-to-Point (PTP) Movements

Point-to-Point (PTP) is the primary method of moving the arm. You use `dType.SetPTPCmd`. 

### PTP Modes
There are multiple ways to move the arm:
* `dType.PTPMode.PTPJUMPXYZMode`: The arm lifts up along the Z-axis, moves horizontally, and drops down (JUMP). Excellent for pick-and-place without hitting obstacles.
* `dType.PTPMode.PTPMOVJXYZMode`: Joint movement to a coordinate. The trajectory is nonlinear (it curves). This is the fastest way to get from A to B.
* `dType.PTPMode.PTPMOVLXYZMode`: Linear movement. The end-effector moves in a perfectly straight line to the destination. Useful for drawing or precision work.

### Example: Moving in a Square
```python
dType.SetQueuedCmdClear(api)

# Let's say we start at X=200, Y=0, Z=0
# Move to 200, 50
dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200, 50, 0, 0, isQueued=1)
# Move to 250, 50
dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 250, 50, 0, 0, isQueued=1)
# Move to 250, -50
dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 250, -50, 0, 0, isQueued=1)
# Move to 200, -50
lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200, -50, 0, 0, isQueued=1)[0]

dType.SetQueuedCmdStartExec(api)
while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
    dType.dSleep(100)
dType.SetQueuedCmdStopExec(api)
```

---

## 6. Using the Linear Rail Accessory

If your Dobot is mounted on a sliding base (Linear Rail), you need to use a slightly different set of commands to synchronize the arm's movement with the rail's movement.

**1. Enable the rail and set speeds:**
```python
# Enable the rail accessory (version=1 is standard)
dType.SetDeviceWithL(api, True, version=1)

# Set linear rail speeds and accelerations
dType.SetPTPLParams(api, velocity=100, acceleration=100, isQueued=1)
```

**2. Send moves with the `SetPTPWithLCmd` function:**
This command includes an extra parameter at the end for the `L` (Rail) position.

```python
# mode, x, y, z, r, L, isQueued
# Move Arm to (X:300, Y:0, Z:0) while moving the rail to position 300
lastIndex = dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVJXYZMode, 300, 0, 0, 0, 300, isQueued=1)[0]
```

> **TIP**: If you are drawing continuous complex geometry (like a circle) using the linear rail, start the queue *before* the loop using `SetQueuedCmdStartExec(api)`. This prevents the command queue buffer from filling up completely if the calculation involves hundreds of tiny coordinate steps.

---

## 7. Clean Disconnect

Always ensure you disconnect gracefully to free up the COM port for the next run. Failing to call `DisconnectDobot` can result in a locked COM port that requires a physical restart of the arm or the python terminal.

```python
dType.DisconnectDobot(api)
print("Dobot Disconnected")
```
