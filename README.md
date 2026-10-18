# MCP-Project

FSM States:
-  Idle: Waiting for the start signal from the Kobuki.
-  Scan: Instructing the robot to scan surroundings using ultrasonic/LiDAR sensors.   
-  Navigate (Path A or B): Driving past bollards while avoiding obstacles and tracking odometry.   
-  Align & Deposit: Locating the blue deposit marker (D1 or D2) and aligning precisely ($100\text{ mm}$ away).   
-  Stop / Fault: Fail-safe state triggered by safety checks or bumpers.
