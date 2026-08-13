# robot_panel

机器人状态显示与控制面板，tkinter 实现。已接入 ROS2，通过后台线程订阅全景图话题并在 `PanoramaBlock` 中显示。

## 界面规格

- 窗口：1070×750，无边框（`overrideredirect`），启动居中，`Alt+F4` 关闭
- 移动：按住 `Ctrl` + 左键拖动窗口（记录按下点与窗口左上角的偏移，拖动时按偏移更新位置）
- 外边距：四边各 10px；内容块间距：10px
- 配色：基底灰黑 `#2b2b2b`，文字白色，内容块白色 1px 边线

## 布局

| 区块 | 位置 (x, y) | 尺寸 (w×h) | 说明 |
| --- | --- | --- | --- |
| robot_state | (10, 10) | 320×480 | 模块开关、抽屉内容选择、显示设置 |
| control_commander | (340, 10) | 720×480 | 控制指令区（三区：basic_commander / robot_interact / semantic_commander） |
| panorama | (340, 500) | 720×240 | 全景图像（ROS2 订阅 `/panorama/annotated`，1/2 尺寸显示） |
| camera | (10, 500) | 320×240 | 相机图像（ROS2 订阅 `/camera_N/image_raw`，1/2 尺寸显示） |
| drawer | 右边缘 | 180×750 | 抽屉栏，覆盖于内容之上 |

尺寸核算：主区 320+10+720=1050（=1070−2×10），纵向 480+10+240=730（=750−2×10）。

### control_commander 内部布局

control_commander 区块（720×480）内部划分为三个子区：

| 子区 | 位置 (x, y) | 尺寸 (w×h) | 说明 |
| --- | --- | --- | --- |
| basic_commander | (0, 0) | 360×288 | 上 60% 左半：veer / wheel / compound 各 mode 按钮，底部 speed / duration 输入框 |
| robot_interact | (360, 0) | 360×288 | 上 60% 右半：panorama_info_broadcast / stereo_distance_broadcast 按钮 |
| semantic_commander | (0, 288) | 720×192 | 下 40% 整块（当前留空，待后续实现） |

**basic_commander mode 按钮**（均来自 [commander.md](../../robot_commander/commander.md) 各 commander 的 mode）：

- Veer（4 个）：`home`、`forward`、`turn`、`lift`
- Wheel（3 个）：`forward`、`turn_right`、`turn_left`
- Compound（5 个）：`home`、`low`、`high`、`rhom_1`、`rhom_2`

底部输入框：`speed`（默认 0.1 m/s）和 `duration`（默认 3.0 s），供各 mode 使用。当前所有按钮均为 UI 占位，未接入功能。

## 抽屉栏

- 收起：右边缘仅显示 24px 宽把手，点击 `<` 展开
- 展开：向左覆盖 180×750 区域，点击 `>` 收起（`drawer.lift()` 保证悬浮在最上层）
- 内容区当前为占位，`set_drawer_page()` 为预留接口，后续按运行状态切换不同页面

## ROS2 集成

- `panel.py` 启动时调用 `rclpy.init()`，关闭时调用 `rclpy.shutdown()`
- `PanoramaBlock` 在独立后台线程中创建 ROS2 节点 `panorama_display`，订阅 `/panorama/annotated`（`sensor_msgs/Image`），通过 `after(0, ...)` 回到主线程更新 UI
- `CameraBlock` 在独立后台线程中创建 ROS2 节点 `camera_display`，动态订阅 `/camera_N/image_raw`（`sensor_msgs/Image`），通过主线程定时器按配置刷新率更新 UI，图像缩放至 1/2 后显示
- `RobotStateBlock.is_panorama_visible()` 同时检查 **Show Panorama** 和 **panorama_concat** 两个条件，`PanoramaBlock` 仅在满足时显示图像
- 模块开关通过 `subprocess.run(['ros2', 'param', 'set', ...])` 同步到 `display_four_camera` 节点的 ROS2 参数
- `RobotStateBlock.set_camera_block()` 注册 `CameraBlock` 引用，**Show Camera** 勾选时调用 `start_display()` 启动显示循环，取消时调用 `stop_display()` 销毁订阅并清理画面

### 参数同步

| 面板开关 | ROS2 参数 | 默认值 | 依赖关系 |
| --- | --- | --- | --- |
| `panorama_concat` | `/display_four_camera/panorama_concat` | False | — |
| `panorama_detect` | `/display_four_camera/panorama_detect` | False | ON 时强制 `panorama_concat=ON` |

- `panorama_detect` 开启且 `panorama_concat` 未开启时，自动勾选 `panorama_concat` 并同步
- `panorama_concat` 关闭且 `panorama_detect` 开启时，自动取消 `panorama_detect` 并同步
- `panorama_concat` 关闭时 **Show Panorama** 无法勾选

## 代码结构

| 文件 | 类 | 说明 |
| --- | --- | --- |
| `panel.py` | `RobotPanel(tk.Tk)` | 主窗口，rclpy 生命周期管理，手动创建各区块并传入依赖 |
| `robot_state.py` | `RobotStateBlock(tk.Frame)` | 三区控制面板：模块开关（与 ROS2 参数同步）、抽屉内容、显示设置 |
| `control_commander.py` | `ControlCommanderBlock(tk.Frame)` | 控制指令区块：上 60% 左右对分（basic_commander / robot_interact），下 40% 整块（semantic_commander） |
| `panorama.py` | `PanoramaBlock(tk.Frame)` | 全景图像区块，后台 ROS2 线程订阅 `/panorama/annotated`，经 PIL 缩放 1/2 后显示 |
| `camera.py` | `CameraBlock(tk.Frame)` | 相机图像区块，后台 ROS2 线程订阅所选 `/camera_N/image_raw`，经 PIL 缩放 1/2 后按刷新率显示 |
| `drawer.py` | `Drawer(tk.Frame)` | 右侧抽屉栏，自管理展开/收起与内容切换 |

- `setup.py` console_scripts：`robot_panel = robot_panel.panel:main`

## 依赖

`package.xml` 中声明：

- `rclpy` — ROS2 Python 客户端库
- `sensor_msgs` — 图像消息类型
- `cv_bridge` — ROS Image ↔ OpenCV 转换
- `python3-numpy` — 图像数组操作
- `python3-pil` — PIL.ImageTk 显示
- `tkinter` — 界面框架

## 运行

```bash
colcon build --packages-select robot_panel
ros2 run robot_panel robot_panel
```

## 显示设置

### Show Camera（相机显示）

- **Show Camera** 勾选时，`CameraBlock` 启动显示循环，订阅所选相机话题 `/camera_N/image_raw`，图像缩放至 1/2 后显示在 320×240 区块中
- **Show Camera** 取消时，销毁 ROS2 订阅并清理画面，恢复为占位文字
- **Camera** 下拉框：选择 `camera1`～`camera6`（对应话题 `/camera_1/image_raw`～`/camera_6/image_raw`），切换时自动销毁旧订阅并创建新订阅
- **Refresh (Hz)** 下拉框：控制相机画面刷新率，可选 10 / 15 / 20 / 30 / 60 Hz，默认 10 Hz

## 后续计划

- robot_state：对接 ROS 服务控制抽屉内容切换、显示参数下发
- control_commander：调用 commander 接口（veer / wheel / compound 各 mode，panorama_info_broadcast / stereo_distance_broadcast）；semantic_commander 待实现
