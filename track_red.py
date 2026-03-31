import cv2
import numpy as np

def main():
    # 1. Attempt to open the USB webcam (1 is usually the first external USB camera)
    # If 1 doesn't work (for example if you have multiple virtual cameras), try changing it to 2 or 3
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("Error: Could not open the webcam.")
        return

    print("Tracking Red Color. Press 'q' to quit.")

    while True:
        # 2. Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab a frame from the webcam.")
            break

        # 3. Blur the frame to reduce noise and help tracking
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)

        # 4. Convert the color space from BGR (OpenCV default) to HSV
        # HSV translates lighting separate from color type, making it great for object tracking
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 5. Define boundaries for "Red" in the HSV color space
        # Red is tricky because it wraps around the HSV spectrum (0-10 and 170-180)
        
        # Lower red range
        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)

        # Upper red range
        lower_red_2 = np.array([170, 120, 70])
        upper_red_2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

        # Combine both ranges
        mask = mask1 + mask2

        # 6. Clean up the mask using morphological operations (removes tiny specks of false reds)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # 7. Find contours (outlines) of the red shapes from the mask
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 8. If we found any red shapes, calculate their centers
        if len(contours) > 0:
            # Grab the largest contour (the biggest red thing on screen)
            c = max(contours, key=cv2.contourArea)
            
            # Find the smallest circle that can enclose this contour
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # Calculate the "Image Moments" to strictly find the center of gravity of the shape
            M = cv2.moments(c)
            if M["m00"] > 0:
                # This explicitly gives us the (X, Y) pixel coordinate!
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])

                # Only draw if the object is reasonably large (filters out noise)
                if radius > 10:
                    # Draw a bounding circle showing we locked on
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    
                    # Draw a dot right on the exact center coordinate
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                    
                    # Display the pixel coordinate as text on the screen
                    text = f"X:{center_x} Y:{center_y}"
                    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 9. Show the live camera feed window
        cv2.imshow("Original Frame - Red Tracker", frame)
        
        # (Optional) Show the Black & White Mask to see what OpenCV "sees"
        cv2.imshow("Red Mask", mask)

        # 10. Break the loop if the user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 11. Clean everything up properly
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
