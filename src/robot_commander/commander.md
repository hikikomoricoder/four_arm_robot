# robot_commander 测试指令文档

本文档记录 `wheel_commander_test.cpp`、`veer_commander_test.cpp`、
`arm_commander_test.cpp` 与 `compound_commander_test.cpp` 中各 `mode`
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

#### 状态管理控制闭环

每个 `mode` 在执行 `sendPositionGoal` 前后会向 `/group_state_manager` service 发送请求，形成控制闭环：

1. **执行前** — 调用 `get_group` 查询 `veer` 组的 `status`：
   - 若为 `"free"`，则调用 `set_group` 将 `status` 设为 `"occupy"`、`position` 设为对应的 mode 名（`home`/`forward`/`turn`/`lift`）。
   - 若不为 `"free"`，则拒绝执行并返回 `false`。
2. **执行后** — 调用 `set_group` 将 `status` 重置为 `"free"`（`position` 保持不变）。

> **`setForwardState` 特殊处理**：该 mode 内部包含两步运动（先 `setHomeState` 再 forward），锁定（reserve）在整个方法入口处执行一次，内部的 `setHomeState` 调用不会重复锁定；释放（release）在整个方法出口处执行一次。

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

#### 状态管理控制闭环

每个 `mode` 在发布速度命令（`driveWithVelocities`）前后会向 `/group_state_manager` service 发送请求，形成控制闭环：

1. **执行前** — 调用 `get_group` 查询 `wheel` 组的 `status`：
   - 若为 `"free"`，则调用 `set_group` 将 `status` 设为 `"occupy"`、`position` 设为对应的 mode 名（`forward`/`turn`）。
   - 若不为 `"free"`，则拒绝执行并返回 `false`（不会发布任何速度命令）。
2. **执行后** — 调用 `set_group` 将 `status` 重置为 `"free"`（`position` 保持不变）。

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

## 3. arm_commander_test.cpp（机械臂组，低层位置控制）

- 控制对象：4 个机械臂共 16 个关节（`arm_joint_1_x / 3_x / 5_x / 7_x`，x = 1~4，四臂主体结构完全相同）
- 控制方式：位置控制，通过 `all_arms_controller`
  （`joint_trajectory_controller/JointTrajectoryController`）的 `FollowJointTrajectory` action
  （话题 `/all_arms_controller/follow_joint_trajectory`）
- 关节限位：`arm_joint_1_x` 为 `[-3.14, 3.14]`，`arm_joint_3/5/7_x` 为 `[-2.35, 2.35]`，URDF 速度限位 `1.0 rad/s`
- **同步带 mimic 关系**：`arm_joint_7_x` 是 `arm_joint_5_x` 的 mimic joint（同步带连接，
  `arm_joint_5_x` 转 N 度 → `arm_joint_7_x` 转 `-N/2` 度）。由于 Gazebo 不支持 mimic joint，
  两关节以独立控制实现，但所有 preset 均保持 `-1/2` 的反向转动关系
- **控制器关节顺序**（每臂一行为 `[j1, j3, j5, j7]`）：
  ```
  [arm_joint_1_1, arm_joint_3_1, arm_joint_5_1, arm_joint_7_1,   // 臂 1
   arm_joint_1_2, arm_joint_3_2, arm_joint_5_2, arm_joint_7_2,   // 臂 2
   arm_joint_1_3, arm_joint_3_3, arm_joint_5_3, arm_joint_7_3,   // 臂 3
   arm_joint_1_4, arm_joint_3_4, arm_joint_5_4, arm_joint_7_4]   // 臂 4
  ```
- Home（初始）位置：所有关节 `0 rad`（URDF 零位）

> **注意**：arm 各命名 preset（`home`/`low`/`high`/`rhombus_1`/`rhombus_2`）已移至
> `CompoundCommander`（见第 4 节），`ArmCommander` 只保留低层轨迹发送接口。
> **弃用**：该测试已不再单独使用。所有预设姿态均通过 `compound_commander_test`
> 执行（见第 4 节），此文档仅保留以记录低层接口行为。

运行命令：

```bash
ros2 run robot_commander arm_commander_test [duration]
#   duration  运动时长（秒），默认 3.0
```

低层烟雾测试：通过 `sendPositionGoal(homePositions(), duration)` 将全部 16 个
arm 关节发送到 home（0 rad）。

### 低层接口说明

#### 执行前置流程

每次运行先阻塞等待两个就绪条件（超时均为 5 s）：

1. `waitForJointStates()` — 在 `/joint_states` 上收齐全部 16 个 arm 关节的当前角度。
2. `waitForActionServer()` — `all_arms_controller` 的 action server 可用。

就绪后调用 `sendPositionGoal`；发送前还会校验 16 维目标向量
是否超出 URDF 限位（越限仅打印 WARN，不阻止发送）。

#### 匀速同步到达机制（sendPositionGoal）

每次调用生成单点轨迹发送给控制器：

- 每关节速度按 `v_i = (目标_i - 当前_i) / duration` 写入 `point.velocities`，
  即各关节在 `duration` 内**匀速**运动并在**同一时刻**到达目标位置；
- 位移越大速度越快（速度与位移成正比）；
- `time_from_start = duration`，`goal_time_tolerance = duration + 3.0` s。

#### 非阻塞接口（供 CompoundCommander 使用）

- `asyncSendPositionGoal(positions, duration, goal_future)` — 校验并发送 goal 后立即返回，
  不阻塞等待；调用方自行 spin 节点并轮询返回的 future。
- `asyncGetResult(goal_handle)` — 非阻塞请求执行结果，返回可轮询的 future。
- `currentPositions()` — 返回 `/joint_states` 上最新的关节位置表（只读）。

> 与 veer / wheel 相同：arm 低层接口**不参与** `/group_state_manager` 的
> occupy/free 状态锁定闭环（状态锁定由 `CompoundCommander` 统一执行）。

---

## 4. compound_commander_test.cpp（机械臂 + 移动底盘复合预设）

- 控制对象：**arm 部分**同第 3 节（16 个 arm 关节，经 `all_arms_controller` action）；
  **mobile_base 部分**为 4 个驱动轮（经 `/wheel_controller/commands` 速度控制）
- 前置构型：veer 关节已处于 lift（-45°）构型（由 `veer_commander_test lift` 预先执行）
- 各 preset 的 arm 目标均定义为相对初始状态的偏移量，整个过程中四轮以 lift 模式
  半正弦速度剖面同步运行，实现机器人变形

运行命令：

```bash
ros2 run robot_commander compound_commander_test <mode> [duration]
#   mode      'home' | 'low' | 'high' | 'rhombus_1' | 'rhombus_2'
#   duration  运动时长（秒），默认 3.0
```

| mode | 调用函数 | arm 各关节目标位置（rad） | 说明 |
|------|----------|------------------------|------|
| `home` | `setHomeState(duration)` | 全部 16 关节 = 0 | 所有关节回到 URDF 零位（单步，无偏移） |
| `low` | `setLowState(duration)` | 每臂：j1=0, j3=`-pi/4`, j5=`+pi/2`, j7=`-pi/4` | 四臂同动作；j7 为 j5 的 `-1/2` mimic |
| `high` | `setHighState(duration)` | 每臂：j1=0, j3=`+pi/8`, j5=`-pi/4`, j7=`+pi/8` | 四臂同动作；j7 为 j5 的 `-1/2` mimic |
| `rhombus_1` | `setRhombus1State(duration)` | j1_1=`+pi/4`, j1_2=`-pi/4`, j1_3=`+pi/4`, j1_4=`-pi/4` | 仅 `arm_joint_1_x` 运动（对角两臂同向），其余 12 关节保持 0 |
| `rhombus_2` | `setRhombus2State(duration)` | j1_1=`-pi/4`, j1_2=`+pi/4`, j1_3=`-pi/4`, j1_4=`+pi/4` | 仅 `arm_joint_1_x` 运动，与 `rhombus_1` 反向的菱形 |

### 各 mode 细节

#### 状态管理控制闭环

每个 `mode` 在执行前后会向 `/group_state_manager` service 发送请求，形成控制闭环：

1. **执行前（前置检查）** — 调用 `get_all` 查询全部组状态，仅当以下条件**全部满足**才执行：
   - `arm` status 为 `"free"`
   - `veer` position 为 `"lift"`
   - `veer` status 为 `"free"`
   - `wheel` status 为 `"free"`
   任一不满足则拒绝执行并返回 `false`（不发送任何运动命令）。
2. **开始执行（锁定）** — 依次调用 `set_group`：
   - `arm`：position = 目标 mode 名（`home`/`low`/`high`/`rhombus_1`/`rhombus_2`），status = `"occupy"`
   - `veer`：status = `"occupy"`（position 保持 `"lift"` 不变）
   - `wheel`：position = `"lift"`，status = `"occupy"`
   任一步失败则回滚已锁定的组（恢复为 `"free"`）。
3. **执行后（释放）** — 调用 `set_group` 将 `arm`/`veer`/`wheel` 的 status 均重置为 `"free"`
   （position 保持不变）。

#### 复合运动机制（arm + wheel lift 并发，双线程）

arm 运动与 wheel lift 剖面**并发**执行，采用**双线程独立驱动**设计，两者同时开始、
各自独立运行，且**相位均以仿真时间（Gazebo `/clock`）为准**：

| 组件 | 线程 | 时钟 | 控制方式 |
|------|------|------|----------|
| arm | 主线程 | 仿真时间（Gazebo `/clock`） | `sendPositionGoal` 阻塞（内用 `spin_until_future_complete`） |
| wheel lift | 后台线程 | 相位：仿真时间（`node_->now()`，`use_sim_time=true`）；发布节奏：墙钟 10 Hz | 10 Hz 循环发布 `Float64MultiArray` 到 `/wheel_controller/commands` |

- **wheel 半正弦速度剖面**（`t ∈ [0, T]`，`t` 为仿真时间）：
  ```
  w(t) = -w_peak · sin(π·t / T)
  ```
  其中峰值角速度 `w_peak = 0.1 / 0.04 = 2.5 rad/s`（峰值线速度 0.1 m/s），
  负号表示轮的实际旋转方向与关节正方向相反（RViz 中实测修正）。
- **T 的取值**：`T = total_duration`（规划时长，与 arm 轨迹的 `time_from_start`
  完全一致，无需任何倍率）。原因是 Gazebo 的 real_time_factor ≈ 0.25
  （实测：arm 1.0 s → 4.1 s，5.0 s → 20.4 s；veer 2.0 s → 8.0 s），
  即仿真时间走得比墙钟慢约 4 倍。wheel 剖面的相位直接读自仿真时间
  （`node_->now()`，节点 `use_sim_time=true` 时自动订阅 `/clock`），
  仿真变慢时剖面自动被拉伸，与 arm 轨迹天然同时开始、同时结束，
  RTF 变化时无需修改任何参数（替代了原先硬编码的 `kSimDurationScale = 4.0`）。
- **发布周期**：`kPeriod = 0.1 s`（10 Hz，按墙钟计时），后台线程发布后
  不阻塞。
- 墙钟仅用于发布节奏与**看门狗**：若仿真时间长时间不推进（仿真暂停 /
  无 `/clock`），超过 `T × 10 + 30` 秒墙钟后发布全 0 并退出。
- 到达 `T`（仿真时间）后自动发布全 0 速度并退出线程；主线程通过 `join()`
  等待后台线程结束。

#### 迁移机制（两步 → 一步）

`low` / `high` / `rhombus_1` / `rhombus_2` 默认执行两步运动，保证偏移始终相对初始（home）状态：

1. **第一步回 home（自动时长）**：所有关节回到 0 rad。时长按当前位移与速度限位
   自动计算：`max(最大位移 / 1.0, 1.0)` s。
2. **第二步偏移运动**：目标位置 = home（0）+ 各 mode 偏移量，时长为 `max(duration, 2.0)` s
   （与 veer `forward` preset 相同的地板值，保证 Gazebo 物理下可靠收敛）。

**优化**：若所有关节已处于 home 位置（`max_home_delta < 0.01 rad`），则跳过第一步回 home，
直接发送单步偏移目标（此时 arm 仅被控制一次，避免 "arm controlled twice" 的冗余操作）。

**home**
- arm 目标位置向量（控制器顺序，16 维）：全 0
- 单步执行，运动时长：`duration`（默认 3.0 s），wheel lift 剖面时长
  `T = duration`（仿真时间）

**low**
- 每臂偏移 `[j1, j3, j5, j7]`（控制器顺序，四臂相同）：`[0, -pi/4, +pi/2, -pi/4]`
  - `arm_joint_3_x` 转 `-pi/4`（-45°）
  - `arm_joint_5_x` 转 `+pi/2`（+90°）
  - `arm_joint_7_x` 转 `-pi/4`（-45°）＝ `-1/2 × (+pi/2)`，满足 mimic 关系
  - `arm_joint_1_x` 保持 `0`
- 偏移步时长：`max(duration, 2.0)` s

**high**
- 每臂偏移 `[j1, j3, j5, j7]`（控制器顺序，四臂相同）：`[0, +pi/8, -pi/4, +pi/8]`
  - `arm_joint_3_x` 转 `+pi/8`（+22.5°）
  - `arm_joint_5_x` 转 `-pi/4`（-45°）
  - `arm_joint_7_x` 转 `+pi/8`（+22.5°）＝ `-1/2 × (-pi/4)`，满足 mimic 关系
  - `arm_joint_1_x` 保持 `0`
- 偏移步时长：`max(duration, 2.0)` s

**rhombus_1**
- 偏移向量（控制器顺序）：仅 `arm_joint_1_x` 非零，其余 12 关节为 0
  - `arm_joint_1_1` = `+pi/4`
  - `arm_joint_1_2` = `-pi/4`
  - `arm_joint_1_3` = `+pi/4`
  - `arm_joint_1_4` = `-pi/4`
- 对角两臂（1&3、2&4）同向转动，形成菱形
- 偏移步时长：`max(duration, 2.0)` s

**rhombus_2**
- 偏移向量（控制器顺序）：仅 `arm_joint_1_x` 非零，与 `rhombus_1` 方向相反的菱形
  - `arm_joint_1_1` = `-pi/4`
  - `arm_joint_1_2` = `+pi/4`
  - `arm_joint_1_3` = `-pi/4`
  - `arm_joint_1_4` = `+pi/4`
- 偏移步时长：`max(duration, 2.0)` s