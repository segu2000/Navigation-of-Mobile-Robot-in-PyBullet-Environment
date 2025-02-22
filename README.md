# 🚀 Navigation of Mobile Robot in the Pybullet Environment

## 📌 Overview

This project focuses on **simulating mobile robot navigation** in the **Pybullet** environment, with an emphasis on **path planning, obstacle avoidance, and dynamic trajectory adjustments**. Using **Lidar sensors** and **control algorithms**, the robot autonomously navigates from the start location to a defined destination while avoiding obstacles. 

This project demonstrates the efficiency of **sensor-driven real-time navigation algorithms**, which are applicable to **autonomous robots, self-driving cars, and industrial automation**.

## 🎯 Features

- **Autonomous Navigation**: The robot finds the optimal path to its destination.
- **Obstacle Avoidance**: Uses **Lidar sensor data** to detect and avoid obstacles in real time.
- **Dynamic Pathfinding**: Adapts to environmental changes.
- **Real-Time Control**: Adjusts **wheel velocity** and **steering angle** dynamically.
- **Continuous Simulation**: Runs in a loop for **real-time observation and analysis**.
- **Visualization**: Uses **Pybullet's physics engine** for realistic simulation.

## 📸 Screenshots

![Picture4](https://github.com/user-attachments/assets/45755325-280c-4122-a2c1-69904508a9b4)
![Picture3](https://github.com/user-attachments/assets/f01e3da7-512e-4bff-a84d-8720429a1f6a)
![Picture2](https://github.com/user-attachments/assets/ac9b3722-18f6-4abd-8f7f-760f0b81c35d)
![Picture1](https://github.com/user-attachments/assets/881f101b-20d1-42d9-ab47-587c6cd0f757)


## 🚀 Installation & Setup

### Prerequisites

Ensure you have the following installed:

- **Python** (Recommended: Python 3.8+)
- **Pybullet** (Physics engine for simulation)
- **NumPy** (For data processing)
- **PyCharm (or any Python IDE)**
- **Anaconda (Optional, for virtual environment management)**

### Steps to Run the Project

1. **Clone the Repository**
   ```sh
   git clone https://github.com/your-username/your-repo-name.git
   cd your-repo-name
   ```

2. **Create a Virtual Environment (Optional)**
   ```sh
   conda create --name robot_nav python=3.8
   conda activate robot_nav
   ```

3. **Install Dependencies**
   ```sh
   pip install pybullet numpy
   ```

4. **Run the Simulation**
   ```sh
   python main.py
   ```

## 📂 Project Structure

```
/Navigation_Robot
│── /Models         # 3D robot and obstacle URDF files
│── /Scripts        # Python scripts for navigation and control
│── /Sensors        # Lidar sensor processing
│── main.py         # Entry point for simulation
│── README.md       # Project documentation
└── requirements.txt # Dependencies
```

## 🛠️ Algorithm & Implementation

### 📌 Navigation Algorithm
1. **Obstacle Detection**: Lidar scans the environment for obstacles.
2. **Steering Control**:
   - Uses **proportional control** to adjust steering angle.
   - Determines the difference between **current and target orientation**.
   - Adjusts the **wheel velocity** dynamically.
3. **Path Planning**:
   - If an obstacle is detected, recalculates a **safe path**.
   - Uses **real-time trajectory adjustments** to avoid collisions.
4. **Simulation Loop**:
   - Runs continuously to update the **robot's movement and obstacle positions**.

### 📌 Control Function:
```python
for wheel in wheels:
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)

for steer in steering:
    p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)
```
- The **steering angle** is dynamically adjusted based on **Lidar sensor readings**.
- The **target velocity** remains constant throughout the movement.

## 📈 Results & Observations

- The robot successfully navigates from **start to destination**.
- **Avoids obstacles dynamically** based on sensor readings.
- **Real-time visualization** confirms smooth movement and collision-free navigation.
- **Pybullet simulation environment** accurately renders the robot’s movement and interactions.

## 🛠️ Technologies Used

- **Python** - Programming language
- **Pybullet** - Physics simulation
- **NumPy** - Data processing
- **URDF** - 3D modeling for robot and obstacles

## 📜 References

1. **Pybullet Documentation** - https://pybullet.org
2. **GitHub Repositories**
3. **StackOverflow Discussions**
