Autonomous Mobile Robot (AMR) - Navigation & Control

📌 Project Overview
This project focuses on building an Autonomous Mobile Robot (AMR) capable of navigating indoor environments. The robot utilizes a combination of LiDAR for mapping, an ESP32 for central processing, and a fusion of IMU/Encoder data for precise movement control.
The goal is to provide a reliable solution for automated material transport in smart factories or hospitals.

🚀 Key Features
Autonomous Navigation: Path planning using the A (A-Star) algorithm*.
Sensor Fusion: Integrated MPU6050 (IMU) and Incremental Encoders for robust Odometry.
Obstacle Avoidance: Real-time 2D environment scanning via LiDAR.
Custom Electronics: Tailor-made PCB design to integrate all components efficiently.
High-Speed Processing: Optimized sampling time to minimize integration errors in positioning.

🛠 Hardware Architecture
The system is built on a layered chassis with the following core components:
Main Controller: ESP32 (WROOM-32).
Sensors: * RPLiDAR A1 (2D Scanning).
MPU6050 (6-Axis IMU).
Optical Encoders (Speed & Distance feedback).
Actuators: DC Motors with high-torque gearboxes + L298N/DRV8833 Drivers.
Power: 18650 Li-ion Battery Pack with BMS.
Tracking: MATLAB-based tags for ground truth path validation.

💻 Software & Algorithms
1. Motion Control
The robot uses a Nested PID Control loop:
Outer Loop: Heading control using IMU data.
Inner Loop: Velocity control for each wheel using Encoder feedback.

2. Navigation
Pathfinding: A* Algorithm operating on a grid map.
Localization: Odometry calculation based on differential drive kinematics, corrected by IMU heading.

3. Simulation & Validation
MATLAB: Used for simulating the robot's trajectory and validating the control logic against real-world sensor data.

⚠️ Challenges & Solutions
Odometry Drift: Solved by tightening the sampling time to ensure minimal gap between position updates, reducing cumulative integration errors.
Sensor Fusion: Implemented a specific logic for "Straight" vs "Turning" cases to handle noise from IMU and Encoder data effectively.

📈 Future Roadmap
[ ] Implement Dynamic Obstacle Avoidance (moving objects).
[ ] Integrate SLAM (Simultaneous Localization and Mapping) for unknown environments.
[ ] Add an AI Camera for object recognition and smarter docking.
