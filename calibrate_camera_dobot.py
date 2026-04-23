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

# Global state
calibration_points_camera = []
calibration_points_robot = []
calibration_matrix = None
STATE = "CALIBRATION" # "CALIBRATION" or "TESTING"

# Camera feed scale factor
RENDER_SCALE = 0.4 # Scale down for display if frame is too large, but we must record clicks related to original scale
ORIGINAL_FRAME_SIZE = None

api = None # Dobot API

def mouse_callback(event, x, y, flags, param):
    global STATE, calibration_points_camera, calibration_points_robot, calibration_matrix, api, ORIGINAL_FRAME_SIZE
    
    if event == cv2.EVENT_LBUTTONDBLCLK:
        # Scale back the click coordinates to original resolution!
        orig_x = int(x / RENDER_SCALE)
        orig_y = int(y / RENDER_SCALE)
        
        if STATE == "CALIBRATION":
            # Get Dobot Current Pose
            pose = dType.GetPose(api)
            robot_x = pose[0]
            robot_y = pose[1]
            
            calibration_points_camera.append([orig_x, orig_y])
            calibration_points_robot.append([robot_x, robot_y])
            
            pt_idx = len(calibration_points_camera)
            print(f"Point {pt_idx}/4 Saved! Camera: ({orig_x}, {orig_y}) -> Robot: ({robot_x:.2f}, {robot_y:.2f})")
            
            if pt_idx == 4:
                # Calculate matrix
                src_pts = np.float32(calibration_points_camera)
                dst_pts = np.float32(calibration_points_robot)
                
                calibration_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
                np.save("calibration_matrix.npy", calibration_matrix)
                print("=========================================")
                print("Calibration Complete! Matrix saved to 'calibration_matrix.npy'")
                print("STATE: TESTING MODE")
                print("Double-click anywhere to move the Dobot to that location!")
                print("=========================================")
                STATE = "TESTING"
                
        elif STATE == "TESTING":
            if calibration_matrix is not None:
                # Transform pixel coordinate to robot coordinate
                click_pt = np.array([[[orig_x, orig_y]]], dtype=np.float32)
                robot_pt = cv2.perspectiveTransform(click_pt, calibration_matrix)
                
                target_x = robot_pt[0][0][0]
                target_y = robot_pt[0][0][1]
                target_z = -15.0 # Default table height
                
                print(f"Testing: Clicked ({orig_x}, {orig_y}) -> Target Robot X:{target_x:.2f}, Y:{target_y:.2f}")
                
                # Command Dobot
                dType.SetQueuedCmdClear(api)
                
                # Lift up
                pose = dType.GetPose(api)
                safe_z = 50.0
                dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, pose[0], pose[1], safe_z, pose[3], isQueued=1)
                
                # Move above target
                dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, target_x, target_y, safe_z, pose[3], isQueued=1)
                
                # Lower to target (Wait for completion here using last_index?)
                # Actually, wait for movement loop is better, but since it's queued, we just fire it.
                dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, target_x, target_y, target_z, pose[3], isQueued=1)
                
                dType.SetQueuedCmdStartExec(api)
                
                print("Moving Dobot...")

def setup_dobot():
    global api
    # Initialize Dobot
    print("Connecting to Dobot...")
    api = dType.load()
    state = dType.ConnectDobot(api, "COM3", 115200)[0]
    if state == dType.DobotConnect.DobotConnect_NoError:
        print(" -> Dobot Connected Successfully!")
        dType.SetQueuedCmdClear(api)
        dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
        dType.SetPTPCommonParams(api, 100, 100, isQueued=1)
        dType.SetQueuedCmdStartExec(api)
        return True
    else:
        print(" -> Error: Failed to Connect Dobot on COM3")
        return False

def main():
    global ORIGINAL_FRAME_SIZE, api
    
    if not setup_dobot():
        sys.exit()

    # 1. Enumerate and connect Hikrobot camera
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
    
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if ret != 0 or deviceList.nDeviceNum == 0:
        print("Error: No Hikrobot camera found!")
        sys.exit()

    print(f"Found {deviceList.nDeviceNum} Hikrobot camera. Connecting...")
    cam = MvCamera()
    stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
    
    if cam.MV_CC_CreateHandle(stDeviceList) != 0:
        print("Failed to create handle.")
        sys.exit()
    if cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0) != 0:
        print("Failed to open device.")
        sys.exit()
        
    nPacketSize = cam.MV_CC_GetOptimalPacketSize()
    if nPacketSize > 0:
        cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)

    cam.MV_CC_SetEnumValue("TriggerMode", 0)

    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
    cam.MV_CC_GetIntValue("PayloadSize", stParam)
    payload_size = stParam.nCurValue
    data_buf = (c_ubyte * payload_size)()

    if cam.MV_CC_StartGrabbing() != 0:
        print("Failed to start grabbing.")
        sys.exit()

    stbInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stbInfo), 0, sizeof(stbInfo))

    # OpenCV window configuration
    cv2.namedWindow("Dobot Vision Calibration")
    cv2.setMouseCallback("Dobot Vision Calibration", mouse_callback)

    print("\n-----------------------------------------")
    print("CALIBRATION INSTRUCTIONS:")
    print("1. Unlock the Dobot arm by holding the button on its forearm.")
    print("2. Move the Dobot tip to a point on the table (e.g. top-left corner).")
    print("3. Double-click exactly on the Dobot's tip in the video feed below.")
    print("4. Repeat this for 4 different points forming a rectangle/square on the workspace.")
    print("Press 'q' at any time to exit.")
    print("-----------------------------------------\n")

    while True:
        ret = cam.MV_CC_GetOneFrameTimeout(data_buf, payload_size, stbInfo, 1000)
        
        if ret == 0:
            nparr = np.frombuffer(data_buf, dtype=np.uint8, count=payload_size)
            try:
                if stbInfo.enPixelType == PixelType_Gvsp_RGB8_Packed:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth, 3))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif stbInfo.enPixelType == PixelType_Gvsp_BayerRG8:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth))
                    frame = cv2.cvtColor(frame, cv2.COLOR_BayerRG2BGR)
                else:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth, 3))
            except ValueError:
                print("Error parsing frame.")
                break
            
            ORIGINAL_FRAME_SIZE = (stbInfo.nWidth, stbInfo.nHeight)

            # Draw calibration overlay and info
            # The Hikrobot frames are huge (usually 2448x2048), we scale them
            display_frame = cv2.resize(frame, (int(frame.shape[1] * RENDER_SCALE), int(frame.shape[0] * RENDER_SCALE)))
            
            # Fetch current Dobot position to show it on screen continuously
            pose = dType.GetPose(api)
            info_text = f"Dobot Pos -> X:{pose[0]:.1f} Y:{pose[1]:.1f} Z:{pose[2]:.1f}"
            
            state_text = f"STATE: {STATE}"
            if STATE == "CALIBRATION":
                state_text += f" (Pts: {len(calibration_points_camera)}/4)"
                
            # Put text directly safely
            cv2.putText(display_frame, state_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(display_frame, info_text, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # Draw clicked points
            for pt in calibration_points_camera:
                scaled_pt = (int(pt[0] * RENDER_SCALE), int(pt[1] * RENDER_SCALE))
                cv2.circle(display_frame, scaled_pt, 5, (0, 255, 0), -1)

            cv2.imshow("Dobot Vision Calibration", display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    print("Shutting down...")
    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    cv2.destroyAllWindows()
    dType.DisconnectDobot(api)

if __name__ == "__main__":
    main()
