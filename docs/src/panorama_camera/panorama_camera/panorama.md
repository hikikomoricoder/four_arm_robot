# 四相机全景显示节点流程（display_four_camera.py）

## 1. 节点概览

`DisplayFourCamera` 订阅四路相机图像，定时拼接全景，叠加 10° 方位区间色带与目标检测标注，输出全景图供 robot_panel 显示。

| 类别 | 名称 | 说明 |
|------|------|------|
| 订阅 | `/camera_1~4/image_raw` | 四路 BGR 图像 |
| 发布 | `/panorama/annotated` | 标注后全景图（frame_id=`panorama`） |
| 服务 | `recompute_stitch`（Trigger） | 请求重算拼接几何 |
| 服务 | `get_azimuth_description`（Trigger） | 拉取最新方位描述（TTS 播报用） |
| 定时器 | 0.066 s（~15 Hz） | `display_cameras` 主循环 |
| 定时器 | 1.0 s | `check_camera_tf` 相机方位漂移监测 |
| 参数 | `panorama_concat`（默认 False） | 是否执行拼接 |
| 参数 | `panorama_detect`（默认 False） | 是否执行检测 + 方位标注 |
| 参数 | `if_imshow`（默认 False） | 是否本地 cv2.imshow |

启动时一次性加载：`FourCameraStitcher`（拼接几何算一次、后续复用缓存）与 TensorRT 检测器（避免运行中开启检测时的加载延迟；加载失败则 `_detector=None`）。

## 2. 执行分支

### 2.1 图像回调 `image_callback(msg, index)`

```
cv_bridge 转 BGR → images[index] 更新 → _updated_indices[index] = True
└─ all(_updated_indices)（四路到齐）?
     ├─ 是: _images_updated = True，并复位 _updated_indices
     └─ 否: 仅记录，等待其余相机
```

→ 每轮完整四帧只触发一次拼接（与相机刷新周期对齐，~1 Hz）。

### 2.2 主循环 `display_cameras`（~15 Hz 定时器）

```
每 tick 读取参数 panorama_concat / panorama_detect
│
├─ 分支 A: _images_updated 且 panorama_concat 开启
│    ├─ 复位 _images_updated
│    ├─ stitcher.stitch(images) 拼接
│    ├─ stitch 成功（非 None）:
│    │    ├─ 缓存 _last_panorama
│    │    ├─ 取 10° 区间边界表（stitcher 内部缓存，变化时打印一次）
│    │    ├─ resize 到 1440×480
│    │    ├─ 分支 A1: panorama_detect 开启
│    │    │    ├─ _detector 为空 → warn（10 s 节流），跳过检测
│    │    │    └─ 否则 detect_panorama → 方位角富化（区间表映射）
│    │    │         → 画框 + 标签（class score azimuth）
│    │    │         → if_azimuth_desc: 生成方位描述字符串，
│    │    │           更新 _latest_desc 并打印
│    │    ├─ 分支 A2: draw_intervals 且有区间表
│    │    │    ├─ 相邻主边界间叠加半透明色带（9 色循环，α=0.25）
│    │    │    ├─ 回绕复制带（双命中且 |dX| > 半幅宽）染橙色
│    │    │    └─ addWeighted 混合 + 相机区域起点角度标签
│    │    └─ 缓存 _last_annotated_pano
│    └─ stitch 失败（None）→ show_pano 保持 None
│
├─ 分支 B: 否则（无新帧 / concat 关闭 / stitch 失败）
│    └─ show_pano = _last_annotated_pano（复用缓存结果）
│
├─ _publish_panorama(show_pano): None → 直接返回；否则发布 /panorama/annotated
│
└─ 分支 C: if_imshow 开启
     ├─ show_pano 非空 → imshow 全景
     ├─ show_pano 为空 → 显示占位图 + stitcher.get_status()
     └─ 按键 'r' → stitcher.request_recompute()
```

要点：拼接、检测、区间叠加、日志均只在「四帧到齐且 concat 开启」的那一拍执行一次；其余 tick 直接复用缓存，高频定时器不产生额外计算。

### 2.3 TF 监测 `check_camera_tf`（1 Hz）

```
查 4 路 camera_optical_link_i 相对 base_footprint 的光轴方位角
├─ TF 异常 → warn（5 s 节流），返回
├─ 首次成功 → 记录 _recorded_azimuths，
│             计算 axis_angles（各路相对 cam0 的角度）
└─ 后续: 任一路漂移 > tf_change_thresh_deg（5°）
     → 更新记录值与 axis_angles → stitcher.request_recompute()
```

### 2.4 服务回调

| 回调 | 行为 |
|------|------|
| `handle_recompute_stitch` | 调用 `request_recompute()`，返回 success=True |
| `handle_get_azimuth_description` | 有 `_latest_desc` → success + 描述文本；否则 success=False |

## 3. 分支条件速查

| 条件 | 效果 |
|------|------|
| `all(_updated_indices)` | 一轮四帧到齐 → 置 `_images_updated` |
| `_images_updated` ∧ `panorama_concat` | 执行拼接 + 检测 + 叠加（每四帧周期仅一次） |
| `panorama_detect` ∧ 检测器就绪 | 检测 + 方位富化 + 标注 |
| `draw_intervals` ∧ `_last_intervals` 非空 | 叠加 10° 方位色带 |
| `if_imshow` | 本地窗口显示；'r' 键请求重算几何 |
| 方位漂移 > 5°（1 Hz 轮询） | 请求拼接几何重算 |
