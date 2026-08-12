# Gazebo 仿真性能优化指南

## 概述

本项目的 Gazebo 仿真（通过 `my_robot_gazebo.launch.xml` 启动）包含 6 路摄像头渲染、大量世界模型、MoveIt 运动规划、RViz 可视化等多个重量级组件，默认全部启动时资源消耗较高。本文档记录已实施的优化措施及按需使用方式。

---

## 已实施的优化

### 1. 物理步长降低（1000Hz → 100Hz）

**文件**: `src/my_robot_bringup/worlds/world_room.sdf`

| 参数 | 优化前 | 优化后 | 影响 |
|------|--------|--------|------|
| `max_step_size` | 0.001 | 0.01 | 物理仿真频率从 1000Hz 降至 100Hz |
| `real_time_update_rate` | 1000 | 100 | 匹配步长调整 |
| `physics type` | `ignored` | `ode` | 启用 ODE 引擎的 solver 配置 |

> **说明**: 100Hz 物理步长对轮式机器人运动控制足够精确，同时对 CPU 占用有明显降低。对于需要极高接触精度的场景（如精细抓取），可调回 0.001。

### 2. 关闭阴影

**文件**: `src/my_robot_bringup/worlds/world_room.sdf`

| 参数 | 优化前 | 优化后 | 位置 |
|------|--------|--------|------|
| `scene/shadows` | `true` | `false` | 场景全局阴影 |
| `light/cast_shadows` | `true` | `false` | 定向光源阴影投射 |

> **对视觉算法的影响分析**:
> - **图像拼接 (panorama)**: 阴影会导致重叠区域特征响应不一致，降低 ORB 特征匹配质量。关闭阴影后光照更均匀，**有利于拼接质量**。
> - **双目测距 (stereo)**: 阴影区域纹理梯度降低，视差计算精度可能下降。关闭阴影使场景亮度一致，**有利于视差估计**。
>
> 结论：关闭阴影不仅节省 GPU 渲染开销，对上层 CV 算法也呈正向或中性影响。

### 3. 按需启动非核心组件

**文件**: `src/my_robot_bringup/launch/my_robot_gazebo.launch.xml`

新增 4 个 launch argument，默认均为 `false`：

| Argument | 控制组件 | 默认值 |
|----------|----------|--------|
| `use_rviz` | RViz2 可视化 | `false` |
| `use_moveit` | MoveIt2 move_group 运动规划 | `false` |
| `use_panorama` | 四路全景拼接 (display_four_camera) | `false` |
| `use_stereo` | 双目测距流水线 (corrector + disparity + TRT) | `false` |

---

## 使用方式

### 仅运动控制（最低资源占用）

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

默认仅启动：Gazebo 仿真 + 控制器 + ros_gz_bridge + robot_state_publisher + state_manager + robot_panel。

### 需要视觉感知

```bash
# 全景 + 双目都开启
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml \
    use_panorama:=true use_stereo:=true

# 仅全景
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml \
    use_panorama:=true

# 仅双目
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml \
    use_stereo:=true
```

### 需要运动规划 + 可视化

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml \
    use_moveit:=true use_rviz:=true
```

### 全功能（原行为）

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml \
    use_rviz:=true use_moveit:=true use_panorama:=true use_stereo:=true
```

---

## 未实施的优化（参考）

以下优化点暂未实施，供按需参考：

| 优化方向 | 方法 | 风险/代价 |
|----------|------|-----------|
| 摄像头分辨率 | 640×480 → 320×240 | 影响上层算法精度，**未采用** |
| 摄像头帧率 | 5Hz → 3Hz | 影响检测实时性 |
| controller_manager 更新率 | 100Hz → 50Hz | 控制器响应可能变慢，**暂未调整** |
| 世界模型精简 | 创建不含装饰模型的轻量版 world | 需维护两份世界文件，失去环境交互 |
| Gazebo headless | `-s` 参数禁用渲染窗口 | 摄像头也无法渲染，**不可用** |
| Ogre2 渲染引擎 | 已启用（`render_engine: ogre2`） | 比 ogre1 性能更好 |

---

## 性能监控

运行时可通过以下方式查看 RTF（Real-Time Factor）：

```bash
# Gazebo GUI 底部状态栏查看 RTF 值
# 或通过 topic
ros2 topic echo /clock --once
```

RTF < 1 表示仿真慢于实时，需要进一步优化；RTF ≈ 1 表示实时运行正常。

---

## 相关文件

| 文件 | 说明 |
|------|------|
| `src/my_robot_bringup/launch/my_robot_gazebo.launch.xml` | 主仿真启动文件 |
| `src/my_robot_bringup/worlds/world_room.sdf` | 仿真世界定义（物理、光照、模型） |
| `src/my_robot_bringup/config/ros2_controllers.yaml` | 控制器配置（update_rate 等） |
| `src/my_robot_bringup/config/gazebo_bridge.yaml` | Gazebo-ROS 桥接配置 |
| `src/my_robot_description/urdf/unit_camera.xml.xacro` | 全景摄像头 URDF（分辨率、帧率） |
| `src/my_robot_description/urdf/stereo_camera.xml.xacro` | 双目摄像头 URDF |
