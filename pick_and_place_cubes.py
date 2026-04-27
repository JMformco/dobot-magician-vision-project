import sys
import os
import time
import cv2
import numpy as np
from ctypes import *

# Dobot imports
import DobotDllType as dType

# Attempt to import Hikrobot Camera SDK
try:
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "MvImport"))
    from MvCameraControl_class import *
except ImportError as e:
    print(f"\n[ERROR] Hikrobot Library Error: {e}")
    sys.exit()

# HSV Limits for Red and Blue
COLOR_RANGES = {
    'red': [
        (np.array([0, 100, 100]), np.array([10, 255, 255])),
        (np.array([160, 100, 100]), np.array([180, 255, 255]))
    ],
    'blue': [
        (np.array([100, 100, 100]), np.array([130, 255, 255]))
    ]
}

CALIBRATION_RAIL_POS = 200
SAFE_Z = 50.0
CUBE_Z = 7.0 # Suction cup down, tuned for picking up 25mm cubes

def setup_dobot():
    api = dType.load()
    print("Connecting to Dobot...")
    state = dType.ConnectDobot(api, "COM3", 115200)[0]
    if state == dType.DobotConnect.DobotConnect_NoError:
        print("Connected!")
        dType.SetQueuedCmdClear(api)
        
        # Home the robot
        print("Homing robot...")
        dType.SetHOMEParams(api, 200, 0, 0, 0, isQueued=1)
        dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
        dType.SetPTPCommonParams(api, 100, 100, isQueued=1)
        
        lastIndex = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
        
        dType.SetQueuedCmdStartExec(api)
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)
        dType.SetQueuedCmdStopExec(api)
        print("Homing complete.")
        
        # Setup Rail
        dType.SetDeviceWithL(api, True, version=1)
        dType.SetPTPLParams(api, 99, 99, isQueued=1)
        
        # Move to Calibration Rail Pos
        print(f"Moving rail to {CALIBRATION_RAIL_POS}...")
        dType.SetQueuedCmdClear(api)
        lastIndex = dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200, 0, SAFE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)[0]
        dType.SetQueuedCmdStartExec(api)
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)
        dType.SetQueuedCmdStopExec(api)
        print("Rail ready.")
        
        # Reset suction cup to off
        dType.SetQueuedCmdClear(api)
        lastIndex = dType.SetEndEffectorSuctionCup(api, True, False, isQueued=1)[0]
        dType.SetQueuedCmdStartExec(api)
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(50)
        dType.SetQueuedCmdStopExec(api)
        
        return api
    return None

def apply_color_mask(hsv_frame, color_name):
    mask = None
    ranges = COLOR_RANGES.get(color_name, [])
    for (lower, upper) in ranges:
        curr_mask = cv2.inRange(hsv_frame, lower, upper)
        if mask is None:
            mask = curr_mask
        else:
            mask = cv2.add(mask, curr_mask)
            
    if mask is not None:
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def main():
    # 1. Connect and Home Dobot
    api = setup_dobot()
    if not api: return
    
    # 2. Load Matrix
    matrix_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_matrix.npy")
    if not os.path.exists(matrix_path):
        print("Error: calibration_matrix.npy not found! Please run the calibration script first.")
        dType.DisconnectDobot(api)
        return
    calibration_matrix = np.load(matrix_path)
    
    # 3. Setup Camera
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
    MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if deviceList.nDeviceNum == 0:
        print("Error: No Hikrobot camera found.")
        dType.DisconnectDobot(api)
        return
        
    cam = MvCamera()
    stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
    cam.MV_CC_CreateHandle(stDeviceList)
    cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    
    nPacketSize = cam.MV_CC_GetOptimalPacketSize()
    if nPacketSize > 0:
        cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
        
    cam.MV_CC_SetEnumValue("TriggerMode", 0)
    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
    cam.MV_CC_GetIntValue("PayloadSize", stParam)
    payload_size = stParam.nCurValue
    data_buf = (c_ubyte * payload_size)()
    
    cam.MV_CC_StartGrabbing()
    stbInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stbInfo), 0, sizeof(stbInfo))
    
    print("\n--- Running Pick and Place ---")
    print("Looking for Red or Blue cubes...")
    print("Press 'q' in the camera window to exit.")
    
    # Tracking variables
    stable_color = None
    stable_center = None
    stable_start_time = 0
    stable_duration = 2.0
    stable_radius = 20 # pixels
    
    try:
        while True:
            ret = cam.MV_CC_GetOneFrameTimeout(data_buf, payload_size, stbInfo, 1000)
            if ret == 0:
                nparr = np.frombuffer(data_buf, dtype=np.uint8, count=payload_size)
                if stbInfo.enPixelType == PixelType_Gvsp_RGB8_Packed:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth, 3))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif stbInfo.enPixelType == PixelType_Gvsp_BayerRG8:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth))
                    frame = cv2.cvtColor(frame, cv2.COLOR_BayerRG2BGR)
                else:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth, 3))
                    
                blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
                
                detected_color = None
                detected_center = None
                max_area = 0
                
                # Check both red and blue
                for color in ['red', 'blue']:
                    mask = apply_color_mask(hsv, color)
                    if mask is not None:
                        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        for c in contours:
                            area = cv2.contourArea(c)
                            if area > 1000 and area > max_area:
                                M = cv2.moments(c)
                                if M["m00"] > 0:
                                    max_area = area
                                    detected_color = color
                                    detected_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                # Update Stability Logic
                if detected_center:
                    # Draw detection indicator
                    cv2.circle(frame, detected_center, 15, (0, 255, 0), 3)
                    
                    if stable_color == detected_color and stable_center:
                        dist = np.linalg.norm(np.array(stable_center) - np.array(detected_center))
                        if dist < stable_radius:
                            elapsed = time.time() - stable_start_time
                            
                            # Draw Timer
                            cv2.putText(frame, f"{stable_color.capitalize()}: {elapsed:.1f}s", 
                                        (detected_center[0] - 50, detected_center[1] - 30), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 0), 4)
                            
                            if elapsed >= stable_duration:
                                # TRIGGER PICK AND PLACE
                                print(f"Triggering Pick and Place for {detected_color} cube!")
                                
                                # Convert click pt (pixel) to robot pt
                                click_pt = np.array([[[detected_center[0], detected_center[1]]]], dtype=np.float32)
                                robot_pt = cv2.perspectiveTransform(click_pt, calibration_matrix)
                                
                                target_x = robot_pt[0][0][0]
                                target_y = robot_pt[0][0][1]
                                
                                drop_x, drop_y, drop_z = 0, 200, 0
                                drop_r = 800 if detected_color == 'red' else 600
                                
                                dType.SetQueuedCmdClear(api)
                                
                                # Pick up Sequence
                                dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVLXYZMode, target_x, target_y, SAFE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)
                                dType.SetEndEffectorSuctionCup(api, True, True, isQueued=1) # Enable suction
                                dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVLXYZMode, target_x, target_y, CUBE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)
                                
                                # Wait for grab using WAITCmd (Wait 500 ms)
                                dType.SetWAITCmd(api, 500, isQueued=1) 
                                
                                # Lift up
                                dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVLXYZMode, target_x, target_y, SAFE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)
                                
                                # Drop off Sequence
                                dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVLXYZMode, drop_x, drop_y, SAFE_Z, 0, drop_r, isQueued=1)
                                dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVLXYZMode, drop_x, drop_y, drop_z, 0, drop_r, isQueued=1)
                                
                                # Release
                                dType.SetEndEffectorSuctionCup(api, True, False, isQueued=1) # Disable suction
                                dType.SetWAITCmd(api, 500, isQueued=1)
                                
                                # Lift up and Return to default position
                                dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVLXYZMode, drop_x, drop_y, SAFE_Z, 0, drop_r, isQueued=1)
                                lastIndex = dType.SetPTPWithLCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200, 0, SAFE_Z, 0, CALIBRATION_RAIL_POS, isQueued=1)[0]
                                
                                print("Executing movement...")
                                dType.SetQueuedCmdStartExec(api)
                                while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                                    dType.dSleep(100)
                                dType.SetQueuedCmdStopExec(api)
                                print("Movement finished. Resuming detection.")
                                
                                # Reset timers so it doesn't immediately re-trigger
                                stable_start_time = time.time()
                                detected_center = None
                                stable_center = None
                        else:
                            # Moved outside radius, reset timer
                            stable_center = detected_center
                            stable_start_time = time.time()
                    else:
                        # New color or tracking lost
                        stable_color = detected_color
                        stable_center = detected_center
                        stable_start_time = time.time()
                else:
                    stable_color = None
                    stable_center = None
                    
                display_frame = cv2.resize(frame, (960, 720))
                cv2.imshow("Pick and Place Vision", display_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        print("Cleaning up...")
        cam.MV_CC_StopGrabbing()
        cam.MV_CC_CloseDevice()
        cam.MV_CC_DestroyHandle()
        cv2.destroyAllWindows()
        
        # Turn off suction and disconnect
        dType.SetQueuedCmdClear(api)
        lastIndex = dType.SetEndEffectorSuctionCup(api, True, False, isQueued=1)[0]
        dType.SetQueuedCmdStartExec(api)
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(50)
        dType.SetQueuedCmdStopExec(api)
        
        dType.DisconnectDobot(api)

if __name__ == "__main__":
    main()
