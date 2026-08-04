# robot_commander 测试指令文档

本文档记录 `wheel_commander_test.cpp` 与 `veer_commander_test.cpp` 中各 `mode`
对应的关节控制目标（目标位置 / 运动速度）。

---

## 1. veer_commander_test.cpp（转向关节，位置控制）

- 控制对象：`arm_veer_joint_1 ~ arm_veer_joint_4`（绕 Z 轴旋转的转向 veer 连杆）
- 控制方式：位置控制，通过 `veer_controller` 的 `FollowJointTrajectory` action
  （话题 `/veer_controller/follow_joint_trajectory`）
- 关节限位：`[-pi, pi]`，URDF 速度限位 `1.0 rad/s`
- **控制器关节顺序**：`[arm_veer_joint_4, arm_veer_joint_3, arm_veer_joint_2, arm_veer_joint_1]`
- Home（初始）位置：所有关节 `0 rad`（URDF 零位）

运行命令：

```bash
ros2 run robot_commander veer_commander_test <mode> [duration]
#   mode      'home' | 'forward' | 'turn' | 'lift'
#   duration  运动时长（秒），默认 3.0
```

| mode | 调用函数 | 各关节目标位置（rad） | 说明 |
|------|----------|------------------------|------|
| `home` | `setHomeState(duration)` | j1=0, j2=0, j3=0, j4=0 | 所有 veer 关节回到 URDF 零位 |
| `forward` | `setForwardState(duration)` | j1=0, j2=`-pi/2`, j3=0, j4=`-pi/2` | 两步动作：① 先 setHomeState；② 相对 home 施加偏移 |
| `turn` | `setTurnState(duration)` | j1=`+pi/4`, j2=`+pi/4`, j3=`+pi/4`, j4=`+pi/4` | 所有关节相对 home 转 +45° |
| `lift` | `setLiftState(duration)` | j1=`-pi/4`, j2=`-pi/4`, j3=`-pi/4`, j4=`-pi/4` | 所有关节相对 home 转 -45° |

### 各 mode 细节

**home**
- 目标位置向量（控制器顺序 `[j4, j3, j2, j1]`）：`[0, 0, 0, 0]`
- 运动时长：`duration`（默认 3.0 s）

**forward**
- 第一步 setHomeState：所有关节到 `0 rad`。
  - setHomeState 时长自动计算：`max(最大位移 / 1.0, 1.0)` s（依据 URDF 速度限位 1.0 rad/s）。
- 第二步前进偏移（相对 home）：
  - `arm_veer_joint_1` 保持 `0 rad`
  - `arm_veer_joint_2` 转 `-pi/2`（-90°）→ 目标 `-pi/2`
  - `arm_veer_joint_3` 保持 `0 rad`
  - `arm_veer_joint_4` 转 `-pi/2`（-90°）→ 目标 `-pi/2`
  - 目标位置向量（控制器顺序 `[j4, j3, j2, j1]`）：`[-pi/2, 0, -pi/2, 0]`
- 前进步时长：`max(duration, 2.0)` s（保证 Gazebo 物理下可靠收敛，位置误差 < 0.15 rad）。
- 结果状态：j1&j2 的 TF 朝向相同，j3&j4 的 TF 朝向相同，两组相差 180°。

**turn**
- 目标位置向量（控制器顺序 `[j4, j3, j2, j1]`）：`[+pi/4, +pi/4, +pi/4, +pi/4]`
- 运动时长：`duration`（默认 3.0 s）

**lift**
- 目标位置向量（控制器顺序 `[j4, j3, j2, j1]`）：`[-pi/4, -pi/4, -pi/4, -pi/4]`
- 所有关节相对 home（0 rad）转 `-pi/4`（-45°）→ 目标 `-pi/4`
- 运动时长：`duration`（默认 3.0 s）

---

## 2. wheel_commander_test.cpp（驱动轮，速度控制）

- 控制对象：`wheel_joint_1 ~ wheel_joint_4`
- 控制方式：速度控制，向 `wheel_controller`
  （`velocity_controllers/JointGroupVelocityController`）的话题
  `/wheel_controller/commands` 发布 `Float64MultiArray`
- 轮半径：`WHEEL_RADIUS = 0.04 m`
- 线速度 → 角速度换算：`angular_vel = linear_speed / WHEEL_RADIUS`
- **控制器关节顺序**：`[wheel_joint_4, wheel_joint_3, wheel_joint_2, wheel_joint_1]`
- 到达 `duration` 后自动发布全 0 速度停止

运行命令：

```bash
ros2 run robot_commander wheel_command <mode> [speed] [duration]
#   mode      'forward' | 'turn'
#   speed     线速度（m/s），默认 0.1
#   duration  运动时长（秒），默认 1.0
```

设 `w = linear_speed / 0.04`（rad/s），例如 `linear_speed = 0.1` 时 `w = 2.5 rad/s`。

| mode | 调用函数 | 各关节角速度（rad/s） | 说明 |
|------|----------|------------------------|------|
| `forward` | `driveForward(speed, duration)` | j1=`+w`, j2=`+w`, j3=`-w`, j4=`-w` | 差速前进：1,2 正转，3,4 反转 |
| `turn` | `driveTurn(speed, duration)` | j1=`+w`, j2=`+w`, j3=`+w`, j4=`+w` | 所有轮同向同速转动 |

### 各 mode 细节

**forward**
- 发布速度向量（控制器顺序 `[j4, j3, j2, j1]`）：`[-w, -w, +w, +w]`
  - `wheel_joint_1` = `+w`（正转）
  - `wheel_joint_2` = `+w`（正转）
  - `wheel_joint_3` = `-w`（反转）
  - `wheel_joint_4` = `-w`（反转）
- 持续 `duration` 后停止（发布 `[0, 0, 0, 0]`）

**turn**
- 发布速度向量（控制器顺序 `[j4, j3, j2, j1]`）：`[+w, +w, +w, +w]`
  - 四个轮均为 `+w`（同向同速）
- 持续 `duration` 后停止（发布 `[0, 0, 0, 0]`）

---

## 3. arm_commander_test.cpp（机械臂组，位置控制）

整体情况：arm有4个，主体结构完全相同，arm_joint_7_x为arm_joint_5_x的mimic joint，同步带连接arm_joint_5_x转N度，arm_joint_7_x转-1/2N度，由于gazebo不支持mimic joint所以先以独立控制实现，但需要保证1/2的反向转动关系
要注意结构限制下的不同joint转动速度的倍率关系，确保各joint在同一时刻达到设定位置，且过程均速
下面_x指四臂都进行相同运动

mode 1
名称：low
调用函数：setLowState(duration)
arm_joint_3_x相对初始状态转动-pi/4
arm_joint_5_x相对初始状态转动+pi/2
arm_joint_7_x相对初始状态转动-pi/4

mode 2
名称：high
调用函数：setHighState(duration)
arm_joint_3_x相对初始状态转动+pi/8
arm_joint_5_x相对初始状态转动-pi/4
arm_joint_7_x相对初始状态转动+pi/8

mode 3
名称：rhombus_1
调用函数：setRhombus1State(duration)
arm_joint_1_1相对初始状态转动+pi/4
arm_joint_1_2相对初始状态转动-pi/4
arm_joint_1_3相对初始状态转动+pi/4
arm_joint_1_4相对初始状态转动-pi/4

mode 4
名称：rhombus_2
调用函数：setRhombus2State(duration)
arm_joint_1_1相对初始状态转动-pi/4
arm_joint_1_2相对初始状态转动+pi/4
arm_joint_1_3相对初始状态转动-pi/4
arm_joint_1_4相对初始状态转动+pi/4

mode 4
名称：home
调用函数：setHomeState(duration)
arm各关节均回到初始状态