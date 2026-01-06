# RewardRRT: Path Planning for Multi-Degree-of-Freedom Robots in Narrow Environments

This repository contains the implementation and resources related to the paper:

**"RewardRRT: Path Planning for Multi-Degree-of-Freedom Robots in Narrow Environments"**  
Qinhu Chen, Wenqiang Wang, Zeming Fan, Meilin Kang, Chuan Yu, and Ho Seok Ahn  
IEEE Robotics and Automation Letters, Vol. 11, No. 1, Jan. 2026  
[DOI: 10.1109/LRA.2025.3635448](https://doi.org/10.1109/LRA.2025.3635448)

---

## Overview

RewardRRT is a novel path planning algorithm designed for multi-degree-of-freedom (DOF) robots operating in narrow environments. It integrates concepts from **Rapidly-exploring Random Trees (RRT)** and **Reinforcement Learning (RL)** to guide tree expansion using a reward mechanism.  

Key features of RewardRRT:
- Treats each sampling tree as an agent and assigns a **reward value function** to each sampled state.
- Uses **cumulative rewards** and **reward increments** as the state space, predicted via a **linear Kalman Filter**.
- Implements a **bidirectional expansion strategy**, prioritizing the tree with lower predicted cumulative reward.
- Employs a **dynamic sampling bias** using a sigmoid function to improve convergence and success rate.
- Demonstrated on a **21-DOF wheeled humanoid robot** in both simulation and real-world indoor scenarios.

---

## Features & Advantages

- **High Planning Efficiency:** Up to 38% faster than baseline RRT algorithms in simulation.
- **High Success Rate:** Average planning success rate of 88.25%, outperforming the best-performing OMPL algorithm by 29.75%.
- **Adaptive Exploration:** Dynamic bias adjustment enables effective exploration in narrow, cluttered environments.
- **Transparency:** Reward-based mechanism avoids the overhead of data-driven learning or model training.

---

## Methodology

1. **Reward Function:**  
   Evaluates each new configuration based on distance to goal, collision status, and historical rewards.

2. **Kalman Filter Prediction:**  
   Predicts cumulative reward and reward increment to guide tree growth.

3. **Bidirectional Expansion:**  
   Expands the tree with lower predicted cumulative reward to accelerate convergence.

4. **Dynamic Sampling Bias:**  
   Adjusts the probability of selecting high-reward nodes versus random nodes to balance exploration and exploitation.

5. **Simulation & Real-World Testing:**  
   Conducted in four different scenarios:
   - Indoor service
   - Medical test
   - Indoor greenhouse picking
   - Industrial pipeline inspection

