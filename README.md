⚠️ HEADS UP: THIS IS A WORK-IN-PROGRESS PROJECT, SO NOT ALL FEATURES ARE UP AND RUNNING YET! ⚠️
Autonomous Mobile Robot (AMR) - Navigation & Control
📌 What This Is All About
We're building an Autonomous Mobile Robot (AMR) designed to cruise around indoor spaces on its own. It maps out the room using LiDAR, uses an ESP32 as its main brain, and mixes IMU and encoder data to figure out exactly where it is and how to move. The end goal? Creating a solid, reliable bot for hauling materials around places like smart factories or hospitals.

🚀 Cool Features
Auto-Navigation: Plans out its route using the A* (A-Star) algorithm.

Sensor Fusion: Mashes up data from an MPU6050 (IMU) and incremental encoders for super robust odometry.

Dodging Obstacles: Scans the room in real-time with a 2D LiDAR to avoid bumping into things.

Custom Electronics: We designed our own PCBs to keep the wiring neat and everything running efficiently.

Fast Processing: We cranked up the sampling rate to keep integration errors in its positioning to an absolute minimum.

🛠️ The Hardware Setup
The whole system is built on a layered chassis. Here is the core gear:

The Brain: ESP32 (WROOM-32)

The Senses: * RPLiDAR A1 (for 2D scanning)

MPU6050 (6-Axis IMU)

Optical Encoders (to track speed and distance)

The Muscles: DC Motors with high-torque gearboxes, driven by L298N/DRV8833 boards.

The Juice: 18650 Li-ion Battery Pack hook up with a BMS.

Tracking: MATLAB-based tags to double-check our real-world path against the planned one.

💻 The Software & Math Stuff
1. Motion Control
The bot runs on a nested PID control loop:

Outer Loop: Keeps the heading straight using IMU data.

Inner Loop: Manages the speed of each wheel using feedback from the encoders.

2. Navigation

Pathfinding: Uses the A* algorithm on a grid-style map.

Localization: Calculates odometry using differential drive kinematics, then corrects it using the IMU's heading.

3. Simulation & Testing

MATLAB: We use this to simulate the bot's path and test our control logic against real-world sensor data.

🎮 How to Run It
Want to take it for a spin? Here is how:

Power it up: Turn on the robot and connect your device to the ESP32's WiFi network.

Launch the simulator: Fire up the Odometry_sim.py script on your computer.

<img width="1919" height="1060" alt="image" src="https://github.com/user-attachments/assets/f69db17c-25bf-4021-9fcf-4553e77ce0f1" />


Set a destination: Just click anywhere on the simulator screen to make the bot drive there automatically (point-to-point navigation).

<img width="1919" height="1065" alt="image" src="https://github.com/user-attachments/assets/698678e7-6253-4b40-9cb4-013e735f85d9" />


⚠️ Bumps in the Road & Fixes
Odometry Drift: We fixed this by tightening up the sampling time. Less gap between updates means less room for errors to stack up.

Sensor Fusion Noise: The IMU and encoders can get a bit noisy, so we wrote specific logic to handle "Straight" vs. "Turning" movements separately to filter the junk data out.

📈 What's Next? (Future Roadmap)
[ ] Add dynamic obstacle avoidance (so it can dodge moving people/objects).

[ ] Hook up SLAM (Simultaneous Localization and Mapping) so it can figure out totally unknown rooms.

[ ] Slap an AI camera on it to recognize objects and handle smart docking.
