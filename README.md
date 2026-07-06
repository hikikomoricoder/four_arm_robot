# four_arm_robot
项目内容：开发一个组合式的跨地形机器人
现有技术栈：
cpp+python混合编程 传统图像处理 3D视觉 目标检测分割追踪 onnx/trt/rknn/ncnn模型转换部署 ros2及其工具 立创eda 焊板
不会的东西：
solidwork 3D打印 cuda编程 正逆动力学 视觉slam nlp 硬件调试 isaac sim
技术栈不大完整，但目前AI的能力太强了，对于不熟悉的方向可以通过agent解决大部分问题。硬件方面没什么基础，且真机实践成本高昂，争取走的尽可能远吧，至少把模拟部分做完,作为gap期结束前的技术整合。
模拟开发环境：
i7-13700 rtx4070 ubuntu24.04 ros2-jazzy cuda-13.0
硬件构想：
四个四自由度机械臂串联，通过控制关节电机变形，跨地形运动，末端分支操控物体。前三节使用关节电机，第四节为利用同步带的从动轮，每个机械臂底座改为舵轮控制平面运动， 末端分支额外用一个舵机操控。
jetson orin nano主控 达妙电机做关节（已大致判断算力与外设资源够用，电机扭矩够用） 6相机 4个拼接全景 2个双目测距 不用激光雷达 。
后期设想：
四臂结构基本一致，若去掉最后一个机械臂上的测距用双目相机，四个全景拼接的用相机改为结构光相机，除第一个机械臂每个机械臂上也加上独立供电与主控，分拆成四个完全独立的可串联主体，达成自我更换功能，但是大概率存在多机同步问题。

# 计划第一阶段（模拟）
- [x] 1.开发环境搭建
- [x] 2.项目框架搭建
- [x] 3.写urdf模型
- [x] 4.gazebo场景搭建
- [x] 5.urdf中增加gazebo配置
- [x] 6.相机配置
- [x] 7.打通流程，写launch文件，跑起来
- [ ] 8.ros2moveit生成controler与问题修复
- [ ] 9.关节预置位运动测试
- [ ] 10.舵轮底盘运动测试
- [x] 11.4相机全景拼接
- [x] 12.yolo11s det和seg转onnx
- [x] 13.onnx模型推理前后处理实现
- [x] 14.全景画面目标检测
- [x] 15.场景适配 finetune or distillation
- [ ] 16.转trt，cuda前后处理与推理
- [ ] 15.接入双目相机检测与分割
- [ ] 16.像素坐标，图像坐标，相机坐标，里程计坐标，世界坐标的转换
- [ ] 17.基于检测/分割结果做距离估计（估计基于分割做才够准确）
- [ ] 18.tiny-whisper-v2集成
- [ ] 19.qwen3-0.5B集成
- [ ] 20.VITS-tiny集成
- [ ] 21.语音识别+大语言模型+语音合成联调
- [ ] 22.拿qwen3 0.5B做自己语料的peft（没条件做模型退化评估）
- [ ] 23.一些事件响应开发测试
- [ ] 24.slam可能先用nav2跑通

# urdf
< img width="1459" height="1231" alt="Screenshot from 2026-06-18 11-42-39" src="https://github.com/user-attachments/assets/df39dc09-f0bf-4447-ae6e-8cb4dc699c32" />

# gazebo
< img width="1564" height="919" alt="Screenshot from 2026-07-04 19-49-39" src="https://github.com/user-attachments/assets/31066c51-11b1-4ab2-9b61-6c26ef1a2784" />

# panorama
< img width="1803" height="468" alt="Screenshot from 2026-06-30 13-43-37" src="https://github.com/user-attachments/assets/c0e09cb3-f8c2-47aa-8edb-529d7f0ec025" />

# yolo11s detect after distillation
<img width="1440" height="481" alt="1019420845" src="https://github.com/user-attachments/assets/0ef9de0b-6935-46c8-a5ce-7873806884d5" />
