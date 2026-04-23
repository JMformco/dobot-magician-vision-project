import cv2
import numpy as np
import sys
from ctypes import *

# Attempt to import Hikrobot Camera SDK
try:
    # Important: Add the 'MvImport' folder to the system path so internal module imports work
    import os
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "MvImport"))
    from MvCameraControl_class import *
except ImportError as e:
    print(f"\n[ERROR] Hikrobot Library Error: {e}")
    print("Please copy the 'MvImport' folder from:")
    print("C:\\Program Files (x86)\\MVS\\Development\\python\\")
    print("and paste it right next to this script in your project folder.")
    sys.exit()

def main():
    # 1. Enumerate available cameras
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
    
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if ret != 0 or deviceList.nDeviceNum == 0:
        print("Error: No Hikrobot camera found!")
        sys.exit()

    print(f"Found {deviceList.nDeviceNum} Hikrobot camera(s). Connecting to the first one...")

    # Create the camera control object
    cam = MvCamera()
    
    # 2. Select the first camera and create its handle
    stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print(f"Failed to create handle! ret[0x{ret:x}]")
        sys.exit()
    print(f"  -> Handle created OK", flush=True)

    # 3. Open the camera connection
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print(f"Failed to open device! ret[0x{ret:x}]")
        sys.exit()
    print(f"  -> Device opened OK", flush=True)

    # For GigE cameras: set the optimal packet size for the network adapter
    # This is CRITICAL — without it, frames won't arrive if jumbo frames aren't enabled
    nPacketSize = cam.MV_CC_GetOptimalPacketSize()
    if nPacketSize > 0:
        ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
        if ret == 0:
            print(f"  -> GigE packet size set to {nPacketSize}", flush=True)
        else:
            print(f"  -> Warning: Could not set packet size (ret[0x{ret:x}])", flush=True)
    else:
        print(f"  -> Not a GigE camera or could not determine packet size", flush=True)

    # Make sure trigger mode is OFF (free-run mode) so frames flow continuously
    ret = cam.MV_CC_SetEnumValue("TriggerMode", 0)
    if ret == 0:
        print(f"  -> Trigger mode set to OFF (free-run)", flush=True)
    else:
        print(f"  -> Warning: Could not set trigger mode (ret[0x{ret:x}])", flush=True)

    # Find the payload size so we can allocate enough memory for a raw frame
    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
    ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
    if ret != 0:
        print(f"Failed to get PayloadSize! ret[0x{ret:x}]")
        sys.exit()
    
    payload_size = stParam.nCurValue
    print(f"  -> PayloadSize: {payload_size} bytes", flush=True)
    # Create the buffer in RAM to intercept the camera frame
    data_buf = (c_ubyte * payload_size)()

    # 4. Start continuously grabbing frames
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print(f"Failed to start grabbing! ret[0x{ret:x}]")
        sys.exit()
    print(f"  -> Grabbing started OK", flush=True)

    stbInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stbInfo), 0, sizeof(stbInfo))

    print("Tracking Red Color on Hikrobot Camera. Press 'q' to quit.", flush=True)

    frame_count = 0
    fail_count = 0
    while True:
        # 5. Ask the Hikrobot camera for a single frame (Wait max 1000ms)
        if frame_count == 0 and fail_count == 0:
            print("  -> Waiting for first frame...", flush=True)
        ret = cam.MV_CC_GetOneFrameTimeout(data_buf, payload_size, stbInfo, 1000)
        
        if ret == 0:
            frame_count += 1
            if frame_count == 1:
                print(f"First frame received! Size: {stbInfo.nWidth}x{stbInfo.nHeight}, PixelType: 0x{stbInfo.enPixelType:x}")

            # Successfully got a frame! We must convert raw bytes into a format OpenCV understands.
            # Convert pointer buffer to Numpy Array
            nparr = np.frombuffer(data_buf, dtype=np.uint8, count=payload_size)
            
            try:
                # Based on pixel formats (Mono/RGB/Bayer), reshape accordingly.
                # Here we assume a color camera outputting common RGB/BGR or Bayer formats.
                # If your picture looks weird/distorted, you might need to change the Pixel Format inside the MVS Client.
                if stbInfo.enPixelType == PixelType_Gvsp_RGB8_Packed:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth, 3))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) # OpenCV uses BGR natively
                elif stbInfo.enPixelType == PixelType_Gvsp_BayerRG8:
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth))
                    frame = cv2.cvtColor(frame, cv2.COLOR_BayerRG2BGR)
                else:
                    # Default assumption: 3-channel color image
                    frame = nparr.reshape((stbInfo.nHeight, stbInfo.nWidth, 3))

            except ValueError:
                print("Error: Could not convert camera format. Please open MVS Client and set Pixel Format to RGB8 or BayerRG8.")
                break

            # -----------------------------------------------------
            # --- START RED TRACKING LOGIC (Exactly same as before) ---
            # -----------------------------------------------------
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            lower_red_1 = np.array([0, 20, 20])
            upper_red_1 = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)

            lower_red_2 = np.array([170, 20, 20])
            upper_red_2 = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

            mask = mask1 + mask2
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                
                M = cv2.moments(c)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])

                    if radius > 10:
                        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"X:{center_x} Y:{center_y}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Show the camera feed
            # Note: Hikrobot frames are huge (e.g. 2448x2048). We might want to shrink the window preview.
            display_frame = cv2.resize(frame, (800, 600))
            display_mask = cv2.resize(mask, (800, 600))
            
            cv2.imshow("Hikrobot - Red Tracker", display_frame)
            cv2.imshow("Hikrobot - Red Mask", display_mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            fail_count += 1
            print(f"Frame grab failed! ret[0x{ret:x}] (attempt {fail_count})", flush=True)
            if fail_count >= 10:
                print("Too many consecutive failures. Exiting.", flush=True)
                break

    # -----------------------------------------------------
    # --- PROPER CLEANUP (Crucial for Hikrobot memory) ----
    # -----------------------------------------------------
    print("Safely shutting down connection to camera...")
    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
