# Kinova Gen3 Vision-Guided Robotics System

An autonomous, vision-guided robotic manipulation system designed for the **Kinova Gen3** robotic arm. This project integrates real-time **AprilTag detection**, **3D coordinate transformations**, and **closed-loop control** to enable precise robot-to-target alignment.

**Sponsored by:** **MDA Space** Developed as a **Software Engineering Capstone Project**.

---

## 1. Overview

The system provides a full-stack solution for autonomous visual docking. By utilizing a wrist-mounted camera, the robot identifies environmental markers and adjusts its end-effector pose in real-time to match target orientations.

### Core Capabilities
* **Real-time Detection:** Identifies AprilTags using the robot's onboard tool camera.
* **Pose Estimation:** Calculates the 6D pose (position and orientation) of the tag relative to the camera.
* **Spatial Mapping:** Transforms camera-space coordinates into the robot's global base frame.
* **Autonomous Alignment:** Generates and executes Cartesian commands to align the tool with the target.

---

## 2. System Architecture

The pipeline processes visual data through several layers to translate pixels into physical motion.



### Data Pipeline
1.  **Image Capture:** The wrist camera streams frames of the workspace.
2.  **Detection:** The `pupil-apriltags` detector identifies tag IDs and pixel corner coordinates.
3.  **Calibration & PnP:** Using camera intrinsics ($f_x, f_y, c_x, c_y$), the system solves the **Perspective-n-Point (PnP)** problem to find the transformation $T_{cam}^{tag}$.
4.  **Coordinate Transformation:** The system resolves the kinematic chain to find the tag's position relative to the robot base:
    $$T_{base}^{tag} = T_{base}^{tool} \cdot T_{tool}^{cam} \cdot T_{cam}^{tag}$$
5.  **Control Loop:** The **Movement Controller** calculates the error between the current tool pose and the target tag pose, sending incremental updates via the Kinova Kortex API.

---

## 3. Key Components

### Vision Module
* Processes raw frames and filters noise.
* Handles 3D pose estimation and generates coordinate data.
* Provides visual debugging overlays, including coordinate axes, distance markers, and ID labels.

### Robot Interface
* Leverages the **Kinova Kortex API** for high-level control.
* Synchronizes real-time feedback of the robot's current Cartesian pose.
* Manages safety-rated motion commands to prevent collisions.

### Movement Controller
* Implements incremental alignment logic for high-precision docking.
* Enforces movement thresholds to prevent jitter and ensures the robot respects physical workspace limits.

---

## 4. Technical Stack

| Category | Technologies |
| :--- | :--- |
| **Sponsor** | **MDA Space** |
| **Languages** | Python |
| **Computer Vision** | OpenCV, NumPy, `pupil-apriltags` |
| **Robotics API** | Kinova Kortex API |
| **Math & Logic** | Spatial Transformations, PnP Solver |

---

## 5. Project Structure

```text
K3N/
├── vision/
│   ├── comp_vision.py          # Core detection logic
│   └── robot/
│       └── apriltag_viewer.py  # Debugging & visualization
├── movement/
│   └── auto_move.py            # Alignment & control loops
├── utilities/
│   └── DeviceConnection.py     # Kortex API connection handling
└── scripts/                    # Entry points and main control scripts
