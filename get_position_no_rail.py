import threading
import DobotDllType as dType

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

# Load Dll and get the CDLL object
api = dType.load()

# Connect Dobot
state = dType.ConnectDobot(api, "COM3", 115200)[0]
print("Connect status:", CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):
    
    # It's still good practice to clear the queue just in case
    dType.SetQueuedCmdClear(api)

    # 1. Ask for the position immediately (No queue parameters!)
    # This returns a list: [x, y, z, r, joint1, joint2, joint3, joint4]
    position = dType.GetPose(api)
    
    # 2. Print it out cleanly
    print("\n--- Current Robot Position ---")
    print(f"X: {position[0]:.2f} mm")
    print(f"Y: {position[1]:.2f} mm")
    print(f"Z: {position[2]:.2f} mm")
    print(f"R: {position[3]:.2f} degrees\n")

    # NOTICE: We removed SetQueuedCmdStartExec and the while loop 
    # because we aren't asking the robot to move!

# Disconnect Dobot
dType.DisconnectDobot(api)
print("Dobot Disconnected")