import cv2
import numpy as np
import sys
from ctypes import *

# Attempt to import Hikrobot Camera SDK
try:
    import os
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "MvImport"))
    from MvCameraControl_class import *
except ImportError as e:
    print(f"\n[ERROR] Hikrobot Library Error: {e}")
    print("Please copy the 'MvImport' folder from:")
    print("C:\\Program Files (x86)\\MVS\\Development\\python\\")
    print("and paste it right next to this script in your project folder.")
    sys.exit()

# Dictionary containing HSV ranges for different colors
# Format: 'color_name': [(lower_bound, upper_bound), ...]
COLOR_RANGES = {
    'red': [
        (np.array([0, 30, 30]), np.array([10, 255, 255])),
        (np.array([160, 30, 30]), np.array([180, 255, 255]))
    ],
    'green': [
        (np.array([35, 30, 30]), np.array([85, 255, 255]))
    ],
    'blue': [
        (np.array([100, 30, 30]), np.array([130, 255, 255]))
    ],
    'yellow': [
        (np.array([20, 30, 30]), np.array([35, 255, 255]))
    ],
    'white': [
        # White has varying hue, low saturation, high brightness
        (np.array([0, 0, 50]), np.array([180, 40, 255]))
    ]
}

def get_shape(contour):
    """
    Analyzes a contour to determining its shape and dimensions.
    Returns: shape_name (str), dimensions (str)
    """
    shape = "Unknown"
    dimensions = ""
    # Perimeter
    peri = cv2.arcLength(contour, True)
    # Approximate the polygon with a precision proportional to the perimeter
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    
    vertices = len(approx)
    
    if vertices == 3:
        shape = "Triangle"
        _, _, w, h = cv2.boundingRect(approx)
        dimensions = f"{w}x{h}"
        
    elif vertices == 4:
        _, _, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        # A square will have an aspect ratio that is approximately equal to one
        if 0.90 <= aspect_ratio <= 1.10:
            shape = "Square"
        else:
            shape = "Rectangle"
        dimensions = f"{w}x{h}"
        
    else:
        # For > 4 vertices, we assume it's a circle if it fills a bounding circle well
        area = cv2.contourArea(contour)
        if area > 0:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            circle_area = np.pi * (radius ** 2)
            # Check if contour area is reasonably close to corresponding perfect circle area
            if circle_area > 0 and 0.7 <= (area / circle_area) <= 1.25:
                shape = "Circle"
                dimensions = f"R:{int(radius)}"
            else:
                shape = "Polygon"
                _, _, w, h = cv2.boundingRect(approx)
                dimensions = f"{w}x{h}"
                
    return shape, dimensions

def apply_color_mask(hsv_frame, color_name):
    """
    Applies the HSV limits for the given color name and returns the mask.
    """
    mask = None
    ranges = COLOR_RANGES.get(color_name, [])
    
    for (lower, upper) in ranges:
        curr_mask = cv2.inRange(hsv_frame, lower, upper)
        if mask is None:
            mask = curr_mask
        else:
            mask = cv2.add(mask, curr_mask)
            
    if mask is not None:
        # Morphological operations to remove noise
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
    return mask

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

    # Optimal packet size
    nPacketSize = cam.MV_CC_GetOptimalPacketSize()
    if nPacketSize > 0:
        ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)

    # Free-run mode
    ret = cam.MV_CC_SetEnumValue("TriggerMode", 0)

    # Payload size
    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
    ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
    if ret != 0:
        print(f"Failed to get PayloadSize! ret[0x{ret:x}]")
        sys.exit()
    
    payload_size = stParam.nCurValue
    data_buf = (c_ubyte * payload_size)()

    # 4. Start grabbing frames
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print(f"Failed to start grabbing! ret[0x{ret:x}]")
        sys.exit()
    print(f"  -> Grabbing started OK", flush=True)

    stbInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stbInfo), 0, sizeof(stbInfo))

    print("\n---------------------------------------------------------")
    print("Shape and Color Tracker Running.")
    print("Press the corresponding key to switch the tracked color:")
    print("  'r' = Red")
    print("  'g' = Green")
    print("  'b' = Blue")
    print("  'y' = Yellow")
    print("  'w' = White")
    print("Press 'q' to quit.")
    print("---------------------------------------------------------", flush=True)

    active_color = 'red' # Default start color
    fail_count = 0

    while True:
        ret = cam.MV_CC_GetOneFrameTimeout(data_buf, payload_size, stbInfo, 1000)
        
        if ret == 0:
            fail_count = 0
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
                print("Error: Could not convert camera format. Exiting.")
                break

            # -----------------------------------------------------
            # --- VISION PROCESSING LOGIC ---
            # -----------------------------------------------------
            
            # Reduce noise 
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # Get the mask for the currently active color
            mask = apply_color_mask(hsv, active_color)

            if mask is not None:
                # Find contours
                contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for c in contours:
                    # Ignore tiny contours (noise)
                    if cv2.contourArea(c) > 1000:
                        
                        # 1. Identify Shape and Dimensions
                        shape_name, dimensions = get_shape(c)
                        
                        # 2. Find Geometrical Center
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])

                            # Draw outline
                            cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
                            
                            # Draw center
                            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                            
                            # Annotate Text (Shape, Dim, Center)
                            text_str = f"{shape_name} | {dimensions} | C:({cx},{cy})"
                            cv2.putText(frame, text_str, (cx - 50, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Draw currently active color on screen
            cv2.putText(frame, f"Tracking: {active_color.upper()}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 255), 3)

            # Resize for display so it fits nicely on a standard monitor
            display_frame = cv2.resize(frame, (960, 720))
            if mask is not None:
                display_mask = cv2.resize(mask, (960, 720))
                cv2.imshow("Mask View", display_mask)
            
            cv2.imshow("Hikrobot - Shape and Color Tracker", display_frame)

            # Keyboard logic
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                active_color = 'red'
            elif key == ord('g'):
                active_color = 'green'
            elif key == ord('b'):
                active_color = 'blue'
            elif key == ord('y'):
                active_color = 'yellow'
            elif key == ord('w'):
                active_color = 'white'

        else:
            fail_count += 1
            if fail_count >= 10:
                print("Too many consecutive camera failures. Exiting.", flush=True)
                break

    # Cleanup
    print("Safely shutting down connection to camera...")
    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
