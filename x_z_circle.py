import math
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
    
    # Clean Command Queued
    dType.SetQueuedCmdClear(api)
    
    # 1. Set general motion speeds
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued=1)

    # 2. Enable and configure the linear rail
    dType.SetDeviceWithL(api, True, version=1) 
    dType.SetPTPLParams(api, 100, 100, isQueued=1)
    
   # ... (Setup code above stays the same: SetDeviceWithL, etc.) ...
    
    # --- CIRCLE PARAMETERS ---
    radius = 100        
    center_l = 250     
    center_z = 50      
    fixed_x = 200      
    fixed_y = 0        
    steps = 36         
    
    # 1. Start the execution engine BEFORE the loop!
    # Now the robot will "eat" commands as fast as we serve them.
    dType.SetQueuedCmdStartExec(api)

    print("Calculating and queuing circle coordinates...")
    
    # --- DRAWING LOOP ---
    for i in range(steps + 1):
        angle = math.radians(i * (360 / steps))
        
        l_pos = center_l + radius * math.cos(angle)
        z_pos = center_z + radius * math.sin(angle)
        
        if i == 0:
            lastIndex = dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVJXYZMode, fixed_x, fixed_y, z_pos, 0, l_pos, isQueued=1)[0]
        else:
            lastIndex = dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVJXYZMode, fixed_x, fixed_y, z_pos, 0, l_pos, isQueued=1)[0]

    # Wait for the robot to finish the entire circle
    print("Drawing circle in the X-Z plane...")
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)
        
    print("Drawing complete.")

    # Stop execution
    dType.SetQueuedCmdStopExec(api)

# Disconnect Dobot
dType.DisconnectDobot(api)
print("Dobot Disconnected")