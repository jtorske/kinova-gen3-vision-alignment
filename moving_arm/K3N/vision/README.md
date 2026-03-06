# AprilTagMVP – AprilTag Detection for Kinova Gen3

A toolkit for detecting AprilTags and computing pose coordinates for the Kinova Gen3 robotic arm.

**Two Viewers:**

- `offline_apriltag_viewer.py` – Works with any webcam (no robot required)
- `apriltag_viewer.py` – Full integration with Kinova Gen3 arm (computes Base→Tag pose)

---

## 📁 Repository Structure

```
AprilTagMVP/
├── common/
│   ├── __init__.py
│   ├── utils.py              # AprilTag detection & transformation utilities
│   ├── webcam_config.py      # Camera intrinsics, RTSP config, distortion coeffs
│   └── tool_cam_config.py    # Tool→Camera extrinsic calibration (R, t)
│
├── robot/
│   ├── __init__.py
│   ├── apriltag_viewer.py        # Full Kinova integration viewer
│   ├── offline_apriltag_viewer.py # Webcam-only viewer (no robot)
│   └── device_connection.py      # Kortex API connection helper
│
├── kortex_api-2.2.0.post31-py3-none-any.whl  # Kinova SDK wheel
├── session.csv                               # Sample output file
└── README.md
```

---

## 📦 Requirements

### Python

- **Python 3.11** (required)
- ❗ **Python 3.12 will NOT work** with Kortex API (protobuf incompatibility)

Verify your Python version:

```bash
py -3.11 --version
```

### Libraries

| Package           | Purpose                                |
| ----------------- | -------------------------------------- |
| `opencv-python`   | Image capture and visualization        |
| `numpy`           | Matrix operations and transformations  |
| `pupil-apriltags` | AprilTag detection and pose estimation |

### Kinova Kortex SDK (for robot viewer only)

Included in repo: `kortex_api-2.2.0.post31-py3-none-any.whl`

---

## 🧰 Installation

```bash
# 1. Clone/download the repository and navigate to the AprilTagMVP folder
cd AprilTagMVP

# 2. Install Python dependencies
py -3.11 -m pip install opencv-python numpy pupil-apriltags

# 3. Install Kortex API (for robot viewer only)
py -3.11 -m pip install kortex_api-2.2.0.post31-py3-none-any.whl
```

---

## 🎥 Offline AprilTag Viewer (No Robot)

Use this to test AprilTag detection with any webcam outside the lab. Operates in **camera frame only**.

### Run

```bash
py -3.11 -m robot.offline_apriltag_viewer
```

### Options

| Argument       | Description                           | Default        |
| -------------- | ------------------------------------- | -------------- |
| `--camera`     | Camera index                          | `0`            |
| `--width`      | Capture width (pixels)                | `1280`         |
| `--height`     | Capture height (pixels)               | `720`          |
| `--tag-size`   | AprilTag size in meters               | `0.05` (5 cm)  |
| `--tag-family` | AprilTag family                       | `tag36h11`     |
| `--fx`, `--fy` | Focal length override (pixels)        | Auto-estimated |
| `--cx`, `--cy` | Principal point override (pixels)     | Image center   |
| `--log`        | Log coordinates to console            | Off            |
| `--log-file`   | Log coordinates to CSV file           | None           |
| `--gt-x/y/z`   | Ground-truth position for error check | None           |

### Examples

```bash
# Default webcam
py -3.11 -m robot.offline_apriltag_viewer

# Use second camera with 10cm tag
py -3.11 -m robot.offline_apriltag_viewer --camera 1 --tag-size 0.10

# Log to CSV file
py -3.11 -m robot.offline_apriltag_viewer --log-file session.csv

# With known camera intrinsics
py -3.11 -m robot.offline_apriltag_viewer --fx 600 --fy 600 --cx 640 --cy 360
```

### Controls

| Key | Action |
| --- | ------ |
| `Q` | Quit   |

---

## 🤖 Kinova AprilTag Viewer (Robot Required)

Full integration with Kinova Gen3 arm. Streams from the robot's RTSP camera and computes:

- **Cam→Tag**: Tag position relative to camera
- **Base→Tag**: Tag position relative to robot base (for motion planning)

Uses calibrated camera intrinsics from `webcam_config.py` and Tool→Camera transform from `tool_cam_config.py`.

### Run

```bash
py -3.11 -m robot.apriltag_viewer --ip <ROBOT_IP> -u <USERNAME> -p <PASSWORD>
```

**All connection arguments are required** (no defaults for security).

### Options

| Argument           | Description                                   | Required |
| ------------------ | --------------------------------------------- | -------- |
| `--ip`             | Robot IP address                              | ✅ Yes   |
| `-u`, `--username` | Robot username                                | ✅ Yes   |
| `-p`, `--password` | Robot password                                | ✅ Yes   |
| `--log`            | Log coordinates to console                    | No       |
| `--log-file`       | Log coordinates to CSV file                   | No       |
| `--gt-x/y/z`       | Ground-truth Base→Tag position for validation | No       |

### Examples

```bash
# Basic usage
py -3.11 -m robot.apriltag_viewer --ip 192.168.1.10 -u admin -p admin

# With CSV logging
py -3.11 -m robot.apriltag_viewer --ip 192.168.1.10 -u admin -p admin --log-file session.csv

# With ground-truth validation
py -3.11 -m robot.apriltag_viewer --ip 192.168.1.10 -u admin -p admin --gt-x 0.5 --gt-y 0.0 --gt-z 0.3
```

### Controls

| Key | Action |
| --- | ------ |
| `Q` | Quit   |

---

## ⚙️ Configuration Files

### `common/webcam_config.py`

Contains camera intrinsics from checkerboard calibration:

| Parameter     | Description                            |
| ------------- | -------------------------------------- |
| `fx`, `fy`    | Focal lengths (pixels)                 |
| `cx`, `cy`    | Principal point (pixels)               |
| `dist_coeffs` | Distortion coefficients (k1-k3, p1-p2) |
| `tag_size`    | Default AprilTag size (0.132 m)        |
| `tag_family`  | Default tag family (`tag36h11`)        |

Also provides:

- `rtsp_url(ip)` – Builds RTSP URL for Kinova camera stream
- `undistort_frame(frame)` – Optional frame undistortion

### `common/tool_cam_config.py`

Contains the fixed **Tool→Camera** extrinsic transform:

- `R_tool_cam` – 3×3 rotation matrix
- `t_tool_cam` – Translation vector

These values are used to compute the full **Base→Tag** transform.

### `common/utils.py`

Utility functions for AprilTag detection and coordinate transforms:

- `create_detector()` – Initialize pupil-apriltags detector
- `detect_tags()` – Run detection with pose estimation
- `euler_xyz_to_R()` / `rotation_to_euler_xyz()` – Euler ↔ rotation matrix
- `compose_base_tag()` – Chain Base→Tool→Cam→Tag transforms

---

## 📊 Output Format

When using `--log-file`, data is saved as CSV with columns:

```
timestamp, tag_id, cam_x, cam_y, cam_z, cam_dist, base_x, base_y, base_z, base_dist, roll, pitch, yaw
```

All positions are in **meters**, angles in **degrees**.

---

## 🔧 Troubleshooting

| Issue                             | Solution                                                                              |
| --------------------------------- | ------------------------------------------------------------------------------------- |
| `ModuleNotFoundError: kortex_api` | Install the wheel: `py -3.11 -m pip install kortex_api-2.2.0.post31-py3-none-any.whl` |
| Kortex fails on Python 3.12+      | Use Python 3.11 only                                                                  |
| RTSP stream won't open            | Check robot IP and ensure you're on the same network                                  |
| No tags detected                  | Ensure adequate lighting and tag is flat/visible                                      |
| Poor pose accuracy                | Verify `tag_size` matches your physical tag                                           |
