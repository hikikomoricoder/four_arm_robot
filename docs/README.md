# Four Arm Robot 项目文档

四臂机器人（four_arm_robot）是一个组合式的跨地形机器人项目：四个四自由度机械臂串联，通过控制关节电机变形实现跨地形运动，末端分支操控物体；每个机械臂底座采用舵轮控制平面运动，搭载 6 相机（4 路全景拼接 + 2 路双目测距）。

技术栈：C++ / Python 混合编程、ROS 2 Jazzy、Gazebo 仿真、传统图像处理、3D 视觉、目标检测/分割/追踪、ONNX/TRT/RKNN/NCNN 模型转换部署。

## 文档导航

- [开发环境搭建](env_prepare.md) — 从零搭建开发环境的完整流程
- [仿真启动流程](launch_flow.md) — Gazebo 仿真启动入口与完整流程
- [部署说明](deploy.md) — 部署相关说明

## 第三方扩展

- [extend 说明](extend/readme.md)
- [YOLO11s 知识蒸馏方案](extend/ultralytics/distillation.md)

## 源码模块文档

- [四相机水平拼接实现流程](src/panorama_camera/panorama_camera/concat_process.md)
- [robot_commander 测试指令文档](src/robot_commander/commander.md)
- [robot_state_manager 状态管理](src/robot_state_manager/robot_state_manager/state_manager.md)

> 项目主页（README）位于仓库根目录 [README.md](../README.md)。
