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
    
    # 1. Set general motion parameters for the arm
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)

    # 2. Enable and configure the linear rail
    dType.SetDeviceWithL(api, True, version=1) 
    dType.SetPTPLParams(api, 99, 99, isQueued=1)
    
    # 3. Issue the combined Point-to-Point and Rail move command
    # We capture the 'lastIndex' here so the script waits for this exact move to finish.
    # Mode 1 is dType.PTPMode.PTPMOVJXYZMode
    lastIndex = dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVJXYZMode, 300, 0, 0, 0, 300, isQueued=1)[0]
    for i in range(0, 5):
        if i % 2 == 0:
            offset = 50
        else:
            offset = -50
        lastIndex = dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200 + offset, 0, 0, 0, 500 - offset, isQueued = 1)[0]

    # Start to Execute Command Queue
    dType.SetQueuedCmdStartExec(api)

    # Wait for Executing Last Command 
    print("Moving arm and rail...")
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)
        
    print("Movement complete.")

    # Stop to Execute Command Queued
    dType.SetQueuedCmdStopExec(api)

# Disconnect Dobot
dType.DisconnectDobot(api)
print("Dobot Disconnected")