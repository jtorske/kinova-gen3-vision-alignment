# 🚀 Autonomous Robotic Tool Change System

🎥 **Demo:**  
[![Autonomous Robotic Tool Change System](https://img.youtube.com/vi/CuOg0Ul9JbY/0.jpg)](https://www.youtube.com/watch?v=CuOg0Ul9JbY)

---

## 📌 Overview

This project presents a fully autonomous robotic tool change system developed as part of a **Multi-Disciplinary Engineering Capstone (2026)**.

The system enables a robotic arm to **identify, align with, attach, and utilize tools without human intervention** by integrating computer vision, control systems, and hardware design into a unified pipeline.

---

## ⚙️ Software System

### Deliverables
- Vision-based detection and localization using **AprilTags**
- Autonomous workflows for:
  - Tool identification  
  - Tool pickup and placement  
  - Sample collection and deposition  
- Fail-safe state handling for power loss or actuator faults  
- Modular and portable control interface compatible with the **Kinova Gen3 robotic arm**

---

## 👁️ Computer Vision Pipeline

The computer vision subsystem enables real-time pose estimation:

1. Capture image frames from the onboard camera  
2. Detect AprilTags and extract corner features  
3. Compute full **6-DOF pose** using calibrated camera parameters  
4. Convert pose into translation and rotation values  
5. Transform pose into usable quantities (yaw, pitch, roll)  
6. Output smoothed, real-time pose data for motion control  

👉 Final output: **3D pose of detected tag in camera frame**

---

## 🤖 Arm Control & Feedback

The control system executes autonomous robotic actions:

- Built using the **Kinova Kortex API**  
- Layered architecture for command execution and feedback handling  
- Centralized command system for:
  - Motion planning  
  - Execution  
  - State tracking  

### Key Features
- Real-time feedback via system flags and sensors  
- Continuous logging for diagnostics and error handling  
- Fail-safe mechanisms for safe operation  

---

## 🧠 System Pipeline

The system performs a complete perception-to-action sequence:

1. Locate tool using vision (AprilTags)  
2. Estimate pose and align the end effector  
3. Apply predefined transformation to tool position  
4. Secure tool via electromagnetic end effector  
5. Verify attachment using Hall-effect sensor feedback  
6. Execute task (e.g., regolith sample collection)  
7. Return and deposit tool and sample  

---

## 🛠️ Tech Stack

- **Programming:** Python  
- **Computer Vision:** OpenCV, AprilTags  
- **Robotics:** Kinova Gen3 (Kortex API)  
- **Control Systems:** Closed-loop feedback control  
- **Architecture:** Modular, layered control system  

---

## 🔌 Electrical System

- Embedded communication between sensors and control system  
- Hall-effect sensors for attachment verification  
- Electromagnetic latching mechanism  
- Custom interface board for robot integration  

---

## ⚙️ Mechanical System

- Custom-designed end effector and tool caddy  
- Magnetic latching mechanism for reliable attachment  
- Designed for durability under:
  - Thermal stress  
  - Repeated attachment cycles  
  - Misalignment tolerance  

---

## 📊 Results & Performance

### Achieved Capabilities
- Fully autonomous tool localization, pickup, and drop-off  
- Reliable magnetic latching mechanism  
- Integrated multi-system coordination  

### Performance Highlights
- Autonomous control using computer vision  
- Real-time system feedback and control  
- Robust tool attachment and handling  

---

## 🎯 Applications

- Space robotics (e.g., lunar regolith sampling)  
- Autonomous manufacturing systems  
- Hazardous or remote robotic operations  

---

## 👥 Contributors

### Software
- Jordan Torske  
- Ryan Pryor  
- Jaiden Reese Sanchez  
- Bruce Gillis  

### Electrical
- Jaden Savoia  

### Mechanical
- Cody Barth  

---

## 🏁 Summary

This project demonstrates a complete autonomous robotic system that integrates **perception, control, and hardware** into a seamless workflow. It highlights the effectiveness of combining computer vision and real-time control for precise and reliable robotic manipulation.

---
