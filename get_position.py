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
state = dType.ConnectDobot(api, "COM9", 115200)[0]
print("Connect status:", CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):
    
    # Clear the queue just in case
    dType.SetQueuedCmdClear(api)

    # 1. Ask for the arm's position
    # Returns [x, y, z, r, joint1, joint2, joint3, joint4]
    position = dType.GetPose(api)
    
    # 2. Ask for the linear rail's position
    # Returns a list containing the L coordinate: [l]
    rail_position = dType.GetPoseL(api)
    
    # 3. Print everything out cleanly
    print("\n--- Current Robot & Rail Position ---")
    print(f"X: {position[0]:.2f} mm")
    print(f"Y: {position[1]:.2f} mm")
    print(f"Z: {position[2]:.2f} mm")
    print(f"R: {position[3]:.2f} degrees")
    print(f"L (Rail): {rail_position[0]:.2f} mm\n")

# Disconnect Dobot
dType.DisconnectDobot(api)
print("Dobot Disconnected")