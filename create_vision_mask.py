import sys
import os
import cv2
import numpy as np
from ctypes import *

# Attempt to import Hikrobot Camera SDK
try:
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "MvImport"))
    from MvCameraControl_class import *
except ImportError as e:
    print(f"\n[ERROR] Hikrobot Library Error: {e}")
    sys.exit()

RENDER_SCALE = 0.4
ORIGINAL_FRAME_SIZE = None
mask_points = []
STATE = "DRAWING" # DRAWING, SAVED

def mouse_callback(event, x, y, flags, param):
    global mask_points, STATE, RENDER_SCALE
    
    if event == cv2.EVENT_LBUTTONDOWN and STATE in ["DRAWING", "SAVED"]:
        STATE = "DRAWING"
        rotated_x = int(x / RENDER_SCALE)
        rotated_y = int(y / RENDER_SCALE)
        
        orig_x = rotated_y
        orig_y = ORIGINAL_FRAME_SIZE[1] - rotated_x
        
        mask_points.append([orig_x, orig_y])
        print(f"Added point: ({orig_x}, {orig_y})")

def main():
    global ORIGINAL_FRAME_SIZE, mask_points, STATE

    # Enumerate and connect Hikrobot camera
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

    cv2.namedWindow("Create Vision Mask")
    cv2.setMouseCallback("Create Vision Mask", mouse_callback)

    print("\n-----------------------------------------")
    print("VISION MASK CREATION:")
    print("1. Click on the image to add points for your ROI polygon.")
    print("2. Press 'c' to clear the current points.")
    print("3. Press 's' to save the mask to 'vision_mask.npy'.")
    print("4. Press 'q' to quit.")
    print("-----------------------------------------\n")

    while True:
        # Use byref for data_buf to prevent crashes as seen in the newer scripts
        ret = cam.MV_CC_GetOneFrameTimeout(byref(data_buf), payload_size, stbInfo, 1000)
        
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
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth, -1))
            except ValueError:
                print("Error parsing frame.")
                break
            
            ORIGINAL_FRAME_SIZE = (stbInfo.nWidth, stbInfo.nHeight)

            rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            display_frame = cv2.resize(rotated_frame, (int(rotated_frame.shape[1] * RENDER_SCALE), int(rotated_frame.shape[0] * RENDER_SCALE)))
            
            # Draw lines and points
            if len(mask_points) > 0:
                mapped_points = []
                for p in mask_points:
                    orig_x, orig_y = p[0], p[1]
                    rot_x = ORIGINAL_FRAME_SIZE[1] - orig_y
                    rot_y = orig_x
                    mapped_points.append([rot_x, rot_y])

                pts = np.array([[int(p[0]*RENDER_SCALE), int(p[1]*RENDER_SCALE)] for p in mapped_points], np.int32)
                pts = pts.reshape((-1, 1, 2))
                # Draw lines between points
                cv2.polylines(display_frame, [pts], isClosed=False, color=(0, 255, 0), thickness=2)
                if len(mask_points) > 2:
                    # Draw a dashed line from last point to first point to show closure
                    cv2.line(display_frame, tuple(pts[-1][0]), tuple(pts[0][0]), (0, 100, 0), 1, cv2.LINE_AA)
                
                # Draw points
                for pt in pts:
                    cv2.circle(display_frame, tuple(pt[0]), 4, (0, 0, 255), -1)

            state_text = f"STATE: {STATE}"
            cv2.putText(display_frame, state_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(display_frame, "Click to add points, 's' to save, 'c' to clear, 'q' to quit", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.imshow("Create Vision Mask", display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                mask_points = []
                STATE = "DRAWING"
                print("Points cleared.")
            elif key == ord('s'):
                if len(mask_points) >= 3:
                    np.save("vision_mask.npy", np.array(mask_points))
                    STATE = "SAVED"
                    print("Mask saved to 'vision_mask.npy'!")
                else:
                    print("Need at least 3 points to save a polygon mask.")

    print("Shutting down...")
    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
