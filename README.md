# RewardRRT
This project is based on ​​OMPL (Open Motion Planning Library)​​, integrating the ​​RewardRRT​​ algorithm into OMPL’s core planners and successfully compiling it into a dynamic link library (.so) under ​​Ubuntu 20.04 (ROS Noetic)​​.

## Key Features ✨ & Improvements:​​

-Integration of RewardRRT into BiTRRT​​
To avoid planner registration issues in OMPL, we  embedded ​​RewardRRT into ​​BiTRRT (Bidirectional T-RRT)​​. Users can directly invoke BiTRRT to utilize ​​RewardRRT​​ without additional setup.

-​​Cross-Platform Support​​
Tested on ​​Ubuntu 20.04 (ROS Noetic)​​ and ​​Ubuntu 18.04 (ROS Melodic)​​ for compatibility.
Provides dynamic link libraries (.so) and ROS interfaces for easy integration into robotic systems.

-Testing Scenarios & Data​​
Test data is in ​​Point Cloud format​​, simulating real-world robotic environments.
Includes ​​4 typical constrained scenarios​​ (e.g., pipe navigation, robotic arm grasping, high-obstacle environments) to validate algorithm performance in tight spaces.

​​-Open-Source Plan​​
The code has passed internal testing. Upon paper acceptance, we will ​​fully open-source​​ all implementations (C++ core, ROS wrappers, and test datasets).
！！Supports ​​custom reward functions​​ for path optimization in complex scenarios (e.g., grasping, obstacle avoidance).


## ​​Applications:​​

-1.Robotic path planning (mobile robots, manipulators)

-2.Optimal trajectory generation in complex environments

-3.Research & education (motion planning algorithms)
