import threading
import DobotDllType as dType

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#将dll读取到内存中并获取对应的CDLL实例
#Load Dll and get the CDLL object
api = dType.load()
#建立与dobot的连接

#Connect Dobot
state = dType.ConnectDobot(api, "COM3", 115200)[0]
print("Connect status:",CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):
    
    #清空队列
    #Clean Command Queued
    dType.SetQueuedCmdClear(api)
    
    #设置运动参数
    #Async Motion Params Setting
    dType.SetHOMEParams(api, 200, 0, 0, 0, isQueued = 1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)

    #回零
    #Async Home
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)

        #Move rail
    # 1. Enable the linear rail
    # True = enabled. 'version' depends on your hardware (often 0 or 1).
    dType.SetDeviceWithL(api, True, version=1)    
    
    # 2. Set the velocity and acceleration for the linear rail specifically
    # We add isQueued=1 so this command stays in sync with your other queued movements
    dType.SetPTPLParams(api, 99, 99, isQueued=1)
    
    # 3. Issue the combined Point-to-Point and Rail move command
    # Parameters: api, mode, X, Y, Z, R, L (Rail Position), isQueued
    lastIndex = dType.SetPTPWithLCmd(api, 1, 200, 0, 0, 0, 200, isQueued=1)[0]

    #开始执行指令队列
    #Start to Execute Command Queue
    dType.SetQueuedCmdStartExec(api)

    #如果还未完成指令队列则等待
    #Wait for Executing Last Command 
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)

    #停止执行指令
    #Stop to Execute Command Queued
    dType.SetQueuedCmdStopExec(api)

#断开连接
#Disconnect Dobot
dType.DisconnectDobot(api)
print("Dobot Disconnected")
