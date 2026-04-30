# Dobot Project Scripts Documentation

This document provides a step-by-step explanation of the purpose and execution flow of each Python script in the project directory.

---

### `pick_and_place_cubes_RGBY_node.py`
**Purpose:** Automates the process of detecting colored cubes (Red, Blue, Green, Yellow) and sorting them using a robotic arm with a linear rail, while simultaneously streaming the video feed to a web dashboard.
**Step-by-step:**
1. Initializes a Flask server in a separate background thread to stream the video feed via HTTP.
2. Connects to the Dobot arm, runs its homing sequence, and positions the linear rail to a default state.
3. Loads a calibration matrix (to translate pixel coordinates to real-world coordinates) and optionally loads a vision mask polygon.
4. Initializes the Hikrobot camera via its SDK and begins grabbing frames.
5. In the main loop, it captures a frame, converts it to HSV, and applies color masks to detect colored contours.
6. Filters the detections by area size, shape (approximating 4 vertices), and edge-touching logic to isolate individual cubes.
7. Requires a detection to remain stable in the same spot for 2 seconds before triggering an action.
8. Once stable, it transforms the camera center coordinates to physical robot coordinates and assigns a drop-off rail position based on the detected color.
9. Queues and executes Dobot commands to move to the cube, activate the suction cup, lift the cube, move along the rail, place the cube at the drop position, and return to default.
10. Resumes vision detection once the physical movement is complete.

---

### `calibrate_camera_dobot.py`
**Purpose:** Creates a perspective transformation matrix (`calibration_matrix.npy`) to mathematically map camera pixel coordinates to physical robot coordinates.
**Step-by-step:**
1. Connects to the Dobot arm and the Hikrobot camera.
2. Displays the live camera feed and awaits user interaction via mouse double-clicks.
3. When the user double-clicks on the feed, the script records the clicked pixel coordinate and simultaneously reads the current physical position of the Dobot arm's end effector.
4. Once 4 distinct points are recorded, it uses OpenCV (`cv2.getPerspectiveTransform`) to compute the perspective transformation matrix and saves it to disk.
5. Switches to a "Testing" mode where double-clicking any point on the camera feed automatically commands the Dobot to move to that physical location to verify accuracy.

---

### `create_vision_mask.py`
**Purpose:** Allows the user to manually draw a polygon on the camera feed to define a Region of Interest (ROI) mask, which is used to ignore anything outside the workspace during vision processing.
**Step-by-step:**
1. Connects to the Hikrobot camera and displays its live feed.
2. Registers mouse clicks to create a list of points forming a polygon.
3. Draws lines connecting the selected points on the screen to visually construct the mask.
4. Awaits keyboard input: 'c' to clear points, 's' to save the mask points to `vision_mask.npy`, or 'q' to quit.

---

### `detect_shapes_colors_mvs.py`
**Purpose:** A diagnostic computer vision tool to track objects of specific colors and determine their geometric shapes and physical dimensions using the Hikrobot camera.
**Step-by-step:**
1. Connects to the Hikrobot camera and starts capturing frames.
2. Loads the `vision_mask.npy` if available to limit the image processing to the workspace area.
3. In a loop, it captures a frame, blurs it to remove noise, and converts it to the HSV color space.
4. Applies an HSV color mask for the currently active color (which can be toggled via keyboard inputs 'r', 'g', 'b', 'y', 'w').
5. Finds contours in the masked image and filters out small noise.
6. Analyzes each contour's perimeter and vertices to determine its geometric shape (Triangle, Square, Rectangle, Circle, Polygon) and bounding dimensions.
7. Calculates the geometric center of the shape and overlays this information as text and tracking graphics on the video feed.

---

### `dobot_gui_control.py`
**Purpose:** Provides a comprehensive graphical user interface (GUI) to manually control, configure, and monitor the Dobot arm.
**Step-by-step:**
1. Uses the PySide6 framework to build a desktop window containing connection settings, axis controls, and configuration limits.
2. Establishes a serial connection to the Dobot via the user-selected COM port.
3. Runs a background timer that continuously polls the robot's current pose (X, Y, Z, R, L) and updates the UI displays.
4. Provides "Jog" buttons to manually move each axis incrementally.
5. Intercepts movement requests and validates them against predefined min/max axis limits to prevent hardware collisions.
6. Sends movement commands (`PTPMOVLXYZMode` and `SetPTPWithLCmd`) to the robot to execute the manual jogging.

---

### `get_position.py`
**Purpose:** A simple utility to retrieve and display the current physical coordinates of the robot and its linear rail.
**Step-by-step:**
1. Connects to the Dobot over the serial COM port.
2. Clears the command queue to ensure immediate execution.
3. Requests the robot's end effector pose (X, Y, Z, R).
4. Requests the linear rail position (L).
5. Prints the formatted coordinates to the terminal and disconnects.

---

### `x_z_circle.py`
**Purpose:** Demonstrates how to generate complex mathematical trajectories by making the Dobot draw a circle in the vertical X-Z plane.
**Step-by-step:**
1. Connects to the Dobot and enables the linear rail.
2. Configures general motion speeds and acceleration parameters.
3. Uses a loop and trigonometric functions (sine and cosine) to calculate 36 sequential coordinate points forming a circle.
4. Queues 36 individual point-to-point movement commands (`SetPTPWithLCmd`) to the Dobot.
5. Starts the command queue execution and waits in a loop until the robot finishes interpolating and drawing the entire circle.

---

### Basic Control Examples (`DobotControl_R200.py`, `DobotControl_adaptado.py`, `DobotControl_home.py`)
**Purpose:** Foundational scripts demonstrating basic connection, homing, and point-to-point movement logic.
**Step-by-step (General flow):**
1. Connects to the Dobot arm.
2. Triggers the homing sequence to reset the robot's internal coordinate system.
3. Enables the linear rail attachment.
4. Queues hardcoded point-to-point movement commands. For example, `_R200` commands the rail to move to position 200, while `_adaptado` tests smaller incremental arm movements.
5. Starts execution and waits for completion before disconnecting.

---

### `track_red.py`
**Purpose:** A simple computer vision test script that tracks red objects using a standard USB webcam.
**Step-by-step:**
1. Opens a video capture connection to the default USB webcam using OpenCV.
2. Captures frames continuously in a `while` loop.
3. Blurs and converts the frames to the HSV color space.
4. Applies a dual-range HSV mask to isolate red pixels (handling the hue wrap-around).
5. Finds contours, calculates the center using image moments, and draws a tracking circle on the screen.

---

### `track_red_mvs.py`
**Purpose:** Tracks red objects similar to `track_red.py`, but specifically implements the Hikrobot industrial camera SDK instead of relying on a standard webcam feed.
**Step-by-step:**
1. Initializes the Hikrobot SDK, scans for the camera, and opens a device handle.
2. Configures optimal network packet sizes and starts grabbing frames in free-run mode.
3. Receives raw byte buffers from the camera and reshapes them into OpenCV-compatible numpy arrays based on the detected pixel format (e.g., RGB8 or BayerRG8).
4. Applies the same HSV thresholding, contour finding, and center calculation logic as the standard webcam script.
5. Safely shuts down the SDK handle and releases camera memory upon exit.

---

### `DobotDllType.py`
**Purpose:** A Python wrapper library provided by Dobot that interfaces with the underlying C++ SDK (`DobotDll.dll`).
**Step-by-step:**
1. Uses Python's `ctypes` library to load the dynamic link library (`CDLL`).
2. Defines constants, enumerations, and data structures that perfectly match the C++ API.
3. Maps Python function calls to their corresponding C++ exported functions, automatically handling argument types and memory pointers.
