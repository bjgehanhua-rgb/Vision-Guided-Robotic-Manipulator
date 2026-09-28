\# ECE 486 Robotics Labs



A collection of five robotics laboratory projects completed for ECE 486 at the University of Waterloo.



The labs progressively develop a robotics software pipeline for the Dobot Magician, beginning with workspace validation and trajectory generation, then introducing forward and inverse kinematics, camera-to-robot calibration, computer vision, and autonomous manipulation.



\## Project Progression



\*\*Workspace \& Trajectory Planning → Forward Kinematics → Inverse Kinematics → Camera-to-Robot Transformation → Vision-Based Motion → Autonomous Pick-and-Place\*\*



\## Lab 1 — Workspace Validation \& Trajectory Planning



Implemented workspace and path validation for safe robot motion and generated Cartesian trajectories for the Dobot Magician.



Key components:

\- Cartesian workspace validation

\- Straight-line path sampling and safety checking

\- Safe robot motion commands

\- Circular trajectory generation

\- Target vs. measured trajectory comparison

\- Simulation and physical robot testing



!\[Trajectory Error](lab1-workspace-trajectory/results/error\_vs\_sample.png)



\[Source Code](lab1-workspace-trajectory/src/workspace\_trajectory.py) | \[Lab Report](lab1-workspace-trajectory/report/lab1\_report.pdf)



\## Lab 2 — Forward Kinematics



Developed and validated a forward kinematics model for the Dobot Magician.



Key components:

\- Joint-space representation

\- Analytical forward kinematics

\- End-effector position prediction

\- Validation using experimental data

\- Comparison between predicted and measured robot positions



\[Source Code](lab2-forward-kinematics/src/forward\_kinematics.py) | \[Lab Report](lab2-forward-kinematics/report/lab2\_report.pdf)



\## Lab 3 — Inverse Kinematics \& Coordinate Transformation



Extended the robot model with inverse kinematics and camera-to-robot coordinate transformation.



Key components:

\- Analytical inverse kinematics

\- Cartesian-to-joint-space conversion

\- IK validation

\- Camera-to-robot coordinate transformation

\- Transformation validation using experimental measurements



\[Source Code](lab3-inverse-kinematics/src/inverse\_kinematics.py) | \[Lab Report](lab3-inverse-kinematics/report/lab3\_report.pdf)



\## Lab 4 — Vision-Based Robot Motion



Integrated computer vision with robot control to enable motion toward visual targets.



Key components:

\- Camera calibration

\- ArUco marker detection

\- Camera-to-robot coordinate mapping

\- Vision-restricted workspace checking

\- Robot positioning based on detected markers

\- Physical robot validation



\[Source Code](lab4-vision-based-motion/src/vision\_based\_motion.py) | \[Lab Report](lab4-vision-based-motion/report/lab4\_report.pdf)



\## Lab 5 — Autonomous Vision-Based Pick-and-Place



Developed a vision-based manipulation pipeline for detecting and relocating objects using the Dobot Magician and a suction-cup end effector.



Key components:

\- HSV-based object segmentation

\- Contour detection and filtering

\- Pixel-to-robot coordinate transformation

\- ArUco marker target localization

\- Automated object pickup

\- Suction-cup control

\- Autonomous pick-and-place sequence



!\[Vision Detection](lab5-vision-pick-and-place/results/vision\_test\_image.png)



\[Source Code](lab5-vision-pick-and-place/src/vision\_pick\_and\_place.py) | \[Lab Report](lab5-vision-pick-and-place/report/lab5\_report.pdf)



\## Technologies



\- Python

\- NumPy

\- OpenCV

\- MuJoCo

\- Dobot Magician

\- ArUco markers

\- Computer Vision

\- Robot Kinematics

\- Coordinate Transformations

\- Autonomous Manipulation



\## Repository Structure



```text

ECE486-Robotics-Labs/

├── lab1-workspace-trajectory/

├── lab2-forward-kinematics/

├── lab3-inverse-kinematics/

├── lab4-vision-based-motion/

├── lab5-vision-pick-and-place/

├── .gitignore

└── README.md

```



\## Notes



The laboratory work was developed using course-provided starter code and robot interfaces. The implementations in this repository contain the completed algorithms, experimental logic, validation procedures, and project-specific modifications developed for the laboratory assignments.



\## Author



Hanhua Ge  

University of Waterloo

