# Ground4Act: Leveraging Visual-Language Model for Collaborative Pushing and Grasping in Clutter

🌟This repository contains the implementation of [Ground4Act](https://www.sciencedirect.com/science/article/pii/S0262885624003858), a two-stage approach for collaborative pushing and grasping in clutter using a visual-language model.📗[**Demonstration**](#demonstration) | [**Installation**](#installation) | [**Model Weights**](#model-weights) | [**Getting Started**](#getting-started) | [**Related Work**](#related-work) | [**BibTeX**](#bibtex)

## Demonstration

🤖[Video](https://www.bilibili.com/video/BV1Jj2nY1EhR/?spm_id_from=333.999.0.0&vd_source=63cd8055657905c0ac8a9388d7a972ed)
🌐[Personal homepage](https://space.bilibili.com/485363351/video)

## Installation

The repository is based on ubuntu18.04.

Before you start, ensure that [ROS (Robot Operating System)](http://wiki.ros.org/) is installed on your system.

### Step 1: Clone the Repository

Open your terminal (**Python 2**) and run the following command to clone the repository:

```bash
mkdir ur_ws && cd ur_ws
git clone https://github.com/HDU-VRLab/Ground4Act.git
```

Install the necessary libraries under the current terminal for [push network](https://github.com/nizhihao/Collaborative-Pushing-Grasping).

```bash
sudo chmod +x install_ros_packages.sh
./install_ros_packages.sh
catkin_make
pip install torch==1.0.0 scipy==1.2.3 torchvision==0.2.1
```

### Step 2: Create a new environment

Create a **Python 3** virtual environment using [conda](https://docs.conda.io/en/latest/). For information on Visual Grounding, please refer to [RefTR](https://github.com/ubc-vision/RefTR).

```bash
conda create -n Vlpg python=3.7
conda activate Vlpg
pip3 install torch torchvision torchaudio scikit-image
cd vl_grasp/RoboRefIt
pip3 install -r requirements.txt 
```

## Model Weights

| Resource             | Description          |
|----------------------|----------------------|
| [Sim_model](https://github.com/nizhihao/Collaborative-Pushing-Grasping/tree/master/myur_ws/src/ur_robotiq/ur_robotiq_gazebo/meshes) | Place the downloaded simulation model under "/home/xxx/.gazebo/models". |
| [Ground4Act](https://pan.baidu.com/s/1jalj3nmUaaE2AAztAjAgfw?pwd=1234) |Place the downloaded Push network weight in "src\gjt_ur_moveit_gazebo\env_info\push.pth".<br> The Visual Grounding weight is placed in "src\vl_grasp\logs". |

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
conda activate Vlpg
python src/vl_grasp/vl_push_grasp.py
```

## Related Work

Many thanks to previous researchers for sharing their excellent work:

```bibtex
@article{yang2021collaborative,
  title={Collaborative pushing and grasping of tightly stacked objects via deep reinforcement learning},
  author={Yang, Yuxiang and Ni, Zhihao and Gao, Mingyu and Zhang, Jing and Tao, Dacheng},
  journal={IEEE/CAA Journal of Automatica Sinica},
  volume={9},
  number={1},
  pages={135--145},
  year={2021},
  publisher={IEEE}
}

@inproceedings{lu2023vl,
  title={VL-Grasp: a 6-Dof Interactive Grasp Policy for Language-Oriented Objects in Cluttered Indoor Scenes},
  author={Lu, Yuhao and Fan, Yixuan and Deng, Beixing and Liu, Fangfu and Li, Yali and Wang, Shengjin},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={976--983},
  year={2023},
  organization={IEEE}
}

@inproceedings{muchen2021referring,
  title={Referring Transformer: A One-step Approach to Multi-task Visual Grounding},
  author={Muchen, Li and Leonid, Sigal},
  booktitle={Thirty-Fifth Conference on Neural Information Processing Systems},
  year={2021}
}
```

## BibTeX

If you find our code or models useful in your work, please cite [our paper](https://www.sciencedirect.com/science/article/pii/S0262885624003858).

```bibtex
@article{YANG2024105280,
  title = {Ground4Act: Leveraging visual-language model for collaborative pushing and grasping in clutter},
  author = {Yuxiang Yang and Jiangtao Guo and Zilong Li and Zhiwei He and Jing Zhang},
  journal = {Image and Vision Computing},
  volume = {151},
  pages = {105280},
  year = {2024},
  url = {https://www.sciencedirect.com/science/article/pii/S0262885624003858}
}
```
