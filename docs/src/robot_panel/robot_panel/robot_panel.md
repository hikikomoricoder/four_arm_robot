# robot_panel

机器人状态显示与控制面板，tkinter 实现。当前仅完成布局骨架，未接入 ROS 数据。

## 界面规格

- 窗口：1070×750，无边框（`overrideredirect`），启动居中，`Alt+F4` 关闭
- 移动：按住 `Ctrl` + 左键拖动窗口（记录按下点与窗口左上角的偏移，拖动时按偏移更新位置）
- 外边距：四边各 10px；内容块间距：10px
- 配色：基底灰黑 `#2b2b2b`，文字白色，内容块白色 1px 边线

## 布局

| 区块 | 位置 (x, y) | 尺寸 (w×h) | 说明 |
| --- | --- | --- | --- |
| robot_state | (10, 10) | 320×480 | 机器人状态显示 |
| control_commander | (340, 10) | 720×480 | 控制指令区 |
| panorama | (340, 500) | 720×240 | 全景图像 |
| camera | (10, 500) | 320×240 | 相机图像 |
| drawer | 右边缘 | 180×750 | 抽屉栏，覆盖于内容之上 |

尺寸核算：主区 320+10+720=1050（=1070−2×10），纵向 480+10+240=730（=750−2×10）。

## 抽屉栏

- 收起：右边缘仅显示 24px 宽把手，点击 `<` 展开
- 展开：向左覆盖 180×750 区域，点击 `>` 收起（`drawer.lift()` 保证悬浮在最上层）
- 内容区当前为占位，`set_drawer_page()` 为预留接口，后续按运行状态切换不同页面

## 代码结构

- `src/robot_panel/robot_panel/panel.py`
  - `RobotPanel(tk.Tk)`：主窗口，`BLOCK_LAYOUT` 常量描述四个内容块坐标
  - `toggle_drawer()`：抽屉展开/收起
  - `close()`：Alt+F4 / WM 关闭入口
- `setup.py` console_scripts：`robot_panel = robot_panel.panel:main`

## 运行

```bash
colcon build --packages-select robot_panel
ros2 run robot_panel robot_panel
```

## 后续计划

- 接入 rclpy：后台线程 spin，UI 更新经 `after()` 回到主线程
- robot_state：对接 robot_state_manager 状态服务；control_commander：调用 commander 接口
- panorama / camera：订阅图像话题，经 PIL.ImageTk 显示
