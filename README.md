![整体环境](./imgage.png)
## 说明
    这个项目是一个基于Ubuntu 18.04操作系统和ROS Melodic框架的抓取环境搭建指南。
    它提供了一个完整的机器人操作系统平台，包含UR5机械臂、夹爪以及摄像头等关键组件。
    通过使用本项目，您可以轻松地搭建一个功能齐全且稳定的抓取环境，适用于各种需要进行物体抓取和操控的研究和实际应用场景。
## 特点
    精简工程，代码清晰。解决了夹爪散架问题、丰富了相机传感信息（RGB+Depth）
## 使用步骤

1.rviz下查看模型：  
```
    roslaunch gjt_ur_description view_ur5_robotiq85_gripper.launch
```
2.gazebo下仿真
```
    roslaunch gjt_ur_gazebo ur5.launch
```
3.查看相机内容（RGB,Depth）：
```
    rqt_image_view
```
## 参考
    https://github.com/Luchuanzhao/UR5-robotiq85_gripper-gazebo
## TODO
    配置Moveit，结合代码控制机械臂移动；
    加入视觉算法，自适应抓取
    接入Chatgpt，交互式对话，无需自己code达到任务级抓取效果
