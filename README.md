# UR5_gripper_camera_gazebo：the built simulation environment and the control code under MoveIt

🌟This repository contains 📗[**Demonstration**](#demonstration) | [**Installation**](#installation) | [**Model Weights**](#model-weights) | [**Getting Started**](#getting-started) 

## Demonstration

🤖[Video](https://www.bilibili.com/video/BV1Jj2nY1EhR/?spm_id_from=333.999.0.0&vd_source=63cd8055657905c0ac8a9388d7a972ed)
🌐[Personal homepage](https://space.bilibili.com/485363351/video)

## Installation

The repository is based on ubuntu18.04.

Before you start, ensure that [ROS (Robot Operating System)](http://wiki.ros.org/) is installed on your system.
```bash
wget http://fishros.com/install -O fishros && . fishros
```

### Step 1: Clone the Repository

Open your terminal (**Python 2**) and run the following command to clone the repository:

```bash
mkdir ur_ws && cd ur_ws
git clone https://github.com/Geo-JTao/UR5_gripper_camera_gazebo.git
```

Install the necessary libraries under the current terminal .

```bash
sudo chmod +x install_ros_packages.sh
./install_ros_packages.sh
catkin_make
```

### Step 2: Create a new environment

Create a **Python 3** virtual environment using [conda](https://docs.conda.io/en/latest/). 
```bash
conda create -n your-env-name python=3.7
conda activate your-env-name
pip3 install xxxxx
```

## Model Weights

| Resource             | Description          |
|----------------------|----------------------|
| [Sim_model](https://github.com/nizhihao/Collaborative-Pushing-Grasping/tree/master/myur_ws/src/ur_robotiq/ur_robotiq_gazebo/meshes) | Place the downloaded simulation model under "/home/xxx/.gazebo/models". |

## Getting Started

### Usage Guidelines

When using ROS with MoveIt for control, please follow these guidelines:

- The terminal for controlling the system must run **Python 2**.
- The terminal for executing algorithm must run **Python 3**.

This setup is crucial for ensuring proper functionality and compatibility between the different components of the system.

### Step 1: Launch simulation and load MoveIt

Please turn on the simulation button in the lower left corner of gazebo, then you can control the robot through [MoveIt](https://moveit.ros.org/).

```bash
cd ur_ws
source ./devel/setup.bash
roslaunch gjt_ur_moveit_gazebo start_gjt_ur_moveit_gazebo.launch 
```

We provide many useful [unit test scripts](src/gjt_ur_moveit_gazebo/gazebo_scripts). Preloading the object model in gazebo helps with later execution speed. 

So it is recommended to run in sequence at terminal 2:

```bash
python src/gjt_ur_moveit_gazebo/gazebo_scripts/spawn_model.py
python src/gjt_ur_moveit_gazebo/gazebo_scripts/moveitServer.py
```

### Step 2: Executing algorithm

```bash
conda activate your-env-name
python src/your-algorithm/xxxxx.py
```

