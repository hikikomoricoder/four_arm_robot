# 四相机水平拼接实现流程

## 1. 总体架构

`FourCameraStitcher` 类负责将四幅水平排列的相机图像拼接为一幅全景图。拼接几何关系（单应性矩阵）**只在首次调用或显式请求重算时计算一次**，后续帧复用缓存的变换矩阵，保证实时性。

### 1.0 四路相机先验知识（基准配置）

四路相机安装在机器人四条机械臂的底座（`arm_base_link_1` ~ `arm_base_link_4`）上，四条臂底座依次通过 `group_joint`（翻转 180° 连接）和臂内 `arm_joint_1`（偏航 ±90°）排列，使得四条机械臂**互成 90° 向外辐射**。每个摄像头通过 `camera_mount_joint` 固定在各自 `arm_base_link` 上，偏航旋转统一为 135°（`3π/4`）。

**关键参数（来自 `unit_camera.xml.xacro`）：**

| 项目 | 值 | 说明 |
|------|-----|------|
| 相机数量 | 4 | cam1 ~ cam4（代码中编号为 cam0~cam3） |
| 水平 FOV | 2.0944 rad（120°） | 每个相机的水平视场角 |
| 图像分辨率 | 640 × 480 | 宽 × 高 |
| 像素格式 | R8G8B8 | 三通道彩色 |
| 更新频率 | 10 Hz | Gazebo 传感器更新率 |
| 相机间距（角向） | ~90° | 相邻相机光轴夹角 ≈ 90° |
| 相邻重叠角 | ~30° | 120° FOV - 90° 间距 = 30° 重叠 |
| 重叠占画幅比 | ~25% | 30° / 120° ≈ 160 px（640 的 1/4） |

**对拼接算法的影响：**
- 当前裁剪比例 `crop_ratio = 0.40`（40% 画幅 ≈ 256 px），略大于实际重叠区（~25% / 160 px），在抑制非重叠区误匹配的同时保留足够特征。
- 四台相机总覆盖角 ≈ 4 × 120° - 3 × 30° = 480° - 90° = 390°，存在冗余覆盖。
- **极线约束先验**：基准配置下相机近似水平排列，极线接近水平方向 → 匹配点的 y 坐标应相近。当前通过 `epipolar_thresh`（默认 5.0 px）过滤垂直视差过大的匹配对，减少明显错误的匹配。该先验在相机相对位姿变化后可能失效，需配合 `request_recompute()` 重新计算几何。

### 输入
- 4 幅 BGR 图像，按从左到右的顺序传入（cam0, cam1, cam2, cam3）
- 如果图像尺寸不一致，以第一幅的尺寸为基准做 resize 归一化

### 输出
- 一幅拼接后的全景 BGR 图像

### 关键参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `nfeatures` | 600 | ORB 特征点检测最大数量 |
| `match_ratio` | 0.95 | Lowe's ratio test 阈值（越小越严格） |
| `ransac_thresh` | 8.0 | RANSAC 重投影误差阈值（像素） |
| `min_matches` | 6 | 最小匹配点数，低于此值视为失败 |
| `crop_ratio` | 0.40 | 重叠区域裁剪比例（占画幅宽） |
| `epipolar_thresh` | 50.0 | 极线约束垂直视差阈值（像素） |
| `fast_threshold` | 20 | ORB FAST 角点检测阈值（越小越敏感，OpenCV 默认 20） |
| `grid_rows` | 6 | 网格化均匀分配的行数（≤1 则禁用网格，使用全图检测结果） |
| `grid_cols` | 6 | 网格化均匀分配的列数（≤1 则禁用网格，使用全图检测结果） |
| `_max_pair_scale` | 2.0 | 相邻单应性线性部分允许的最大缩放 |
| `_min_pair_scale` | 0.5 | 相邻单应性线性部分允许的最小缩放 |
| `_max_pair_perspective` | 0.1 | 透视项上限，防止消失点过近导致画布爆炸 |
| `_max_pair_width_ratio` | 1.5 | 单张图变换后宽度相对原始宽度的上限 |
| `_max_pair_height_ratio` | 1.2 | 单张图变换后高度相对原始高度的上限 |
| `_max_canvas_width_ratio` | 4.0 | 最终全景图宽度相对单张图宽度的上限 |
| `_max_canvas_height_ratio` | 1.5 | 最终全景图高度相对单张图高度的上限 |

### 调试开关
| 变量 | 默认值 | 说明 |
|------|--------|------|
| `debug_match` | True | 是否显示特征匹配可视化窗口 |
| `debug_concat` | False | 是否显示增量拼接中间结果窗口 |
| `debug_pair` | 2 | 显示哪一对的匹配结果：0=cam0↔cam1, 1=cam1↔cam2, 2=cam2↔cam3 |
| `blend_method` | 2 | 融合方法：0=加权平均, 1=多频带, 2=接缝, 3=三次幂加权, 4=最佳图像 |
| `force_center_alignment` | True | 是否在单应性矩阵估计后施加中心高度修正（消除垂直漂移） |

---

## 2. 核心流程

### 2.1 `stitch(images)` — 主入口

```
stitch(images)
  ├─ 输入校验: 4 幅图像均非空
  ├─ 如果 _recompute == True 或 _ready == False:
  │     └─ compute_stitch(images)  ← 计算拼接几何
  ├─ 图像尺寸归一化
  └─ 对每幅图像用缓存的 _warp_homographies 做透视变换 + 羽化融合
       └─ 返回全景图
```

---

### 2.2 `compute_stitch(images)` — 几何计算（仅执行一次或重算时）

```
compute_stitch(images)
  ├─ 输入校验 + 尺寸归一化
  ├─ Step A: 计算相邻单应性矩阵 adj[i] (共3对)
  │     for i in [0,1,2]:
  │       _match_pair(images[i], images[i+1]) → H_i
  │       若任何一对失败 → 返回 False, _ready=False
  ├─ Step B: 累积单应性矩阵（全部映射到 cam0 平面）
  │     cumulative[0] = I (单位矩阵, cam0 自身)
  │     cumulative[1] = I @ H_0        (cam1 → cam0)
  │     cumulative[2] = I @ H_0 @ H_1  (cam2 → cam0)
  │     cumulative[3] = I @ H_0 @ H_1 @ H_2 (cam3 → cam0)
  ├─ Step C: 计算全景画布范围
  │     将四幅图的四角用各自 cumulative 变换 → 所有角点
  │     → 取最小/最大 x,y → canvas_w, canvas_h
  ├─ Step D: 平移变换 T（使画布原点为 (0,0)）
  │     T 将 min_xy 平移到原点
  │     _warp_homographies[i] = T @ cumulative[i]
  ├─ Step E: 画布大小硬上限保护
  │     若 canvas_w > `_max_canvas_width_ratio` × 单图宽
  │        或 canvas_h > `_max_canvas_height_ratio` × 单图高
  │     则对所有 `_warp_homographies` 做统一缩放，使输出满足上限
  ├─ 保存状态: _ready=True, _recompute=False
  └─ 返回 True
```

---

### 2.3 `_match_pair(left_img, right_img, pair_idx)` — 单对应配

这是**核心匹配逻辑**，输入两幅水平相邻的图像，输出从右图到左图的单应性矩阵 H。

#### Step 1: 重叠区域裁剪

为了减少非重叠区域的误匹配，只对**两侧图像各取 `crop_ratio`（默认 40%）画幅**提取特征：

```
crop_w = int(w * crop_ratio)           # 640 × 0.40 = 256 px
left_crop  = left_img[:, :crop_w]      # 左图左侧 40%
right_crop = right_img[:, w - crop_w:] # 右图右侧 40%
```

> **坐标系修正**：在 right_crop 中检测到的关键点需要偏移 `+(w - crop_w)` 恢复到右图全图坐标；left_crop 从 x=0 开始，无需偏移。

#### Step 2: ORB 特征检测（全图检测 + 网格均匀分配）

对两个裁剪区域分别调用 `_detect()`：
- 若输入为 BGR 彩色图，先转为灰度图
- **检测策略（两阶段）**：
  1. **Phase 1 — 全图检测**：在完整的裁剪区域上使用 `cv2.ORB.detectAndCompute()` 提取特征点。这样 ORB 的尺度金字塔始终在足够大的图像上构建，检测质量不受网格参数影响。
  2. **Phase 2 — 网格均匀分配**：将裁剪区域划分为 `grid_rows × grid_cols`（默认 3×3）的均匀网格，把 Phase 1 检测到的特征点分配到各个格子中，每个格子按 `nfeatures / (grid_rows × grid_cols)` 的预算保留 `response` 分数最高的特征点。超出预算的纹理密集区域被裁剪，不足预算的区域如实接受。
- 该方案既避免了 ORB 响应值排序导致的局部过密聚集，又解决了原方案（逐格独立检测）在小格子上尺度金字塔过浅、特征检测不可靠的问题
- 当 `grid_rows ≤ 1` 且 `grid_cols ≤ 1` 时，网格失效，直接返回全图检测结果
- 网格设置有自动保护：格子边长不低于 30 px，防止网格过密时格子大小趋于零导致检测不可靠
- `fastThreshold`（默认 10，OpenCV 默认 20）：控制 FAST 角点检测灵敏度，值越小检测到的角点越多（包括较弱角点）
- 建议调试时搭配 `debug_match` 窗口观察绿色圆点（所有检测到的关键点）的分布是否均匀
- 若任一侧描述子为空（`None`），直接返回失败

#### Step 3: 特征匹配（双重策略）

使用 Hamming 距离的暴力匹配器 `cv2.BFMatcher(cv2.NORM_HAMMING)`。

##### 策略 A: Cross-Check（双向互最近邻匹配）— 优先使用

```
方向1: right → left  (右图每个描述子在左图中找 k=2 最近邻)
        → ratio test (distance < match_ratio * 次近距离)
        → 记录通过测试的: fwd[r_idx] = l_idx

方向2: left → right  (左图每个描述子在右图中找 k=2 最近邻)
        → ratio test
        → 记录通过测试的: bwd[l_idx] = r_idx

取交集: 保留同时满足 fwd[r_idx]=l_idx 且 bwd[l_idx]=r_idx 的匹配对
```

##### 策略 B: Unidirectional（单向 Ratio Test）— 降级回退

如果 Cross-Check 匹配数 `< min_matches`：
```
仅做 right → left 方向的 knnMatch(k=2) + ratio test
```

##### 匹配失败判定

如果两种策略的匹配数都 `< min_matches`，返回 `None`。

#### Step 3.5: 极线几何先验过滤

基于基准配置下相机近似**水平排列**的先验，极线接近水平方向 → 正确匹配点的 y 坐标应当相近：

```
对于每对匹配点 (src_pt, dst_pt):
    y_diff = |src_pt.y - dst_pt.y|
    保留 y_diff < epipolar_thresh (默认 5.0 px) 的匹配
```

- 过滤掉垂直视差过大的明显错误匹配，减少 RANSAC 的离群点
- 若过滤后匹配数 `< min_matches`，返回失败
- **注意**：该先验仅在相机保持近似水平排列时有效。当相机相对位姿改变时，需外部发出 `request_recompute()` 信号重新计算几何

#### Step 4: 单应性矩阵估计

```
H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, ransac_thresh)
```

- `src_pts`：右图关键点 → 源点
- `dst_pts`：左图关键点 → 目标点
- 使用 RANSAC 剔除离群点
- 若 H 为 None 或内点数 `< min_matches`，返回失败
- 成功后调用 `_pair_homography_is_reasonable(H, w, h)` 进行几何合理性检查：
  - 线性部分缩放需在 `[0.5, 2.0]`
  - 透视项需足够小（`_max_pair_perspective`）
  - 变换后图像宽度不超过原图的 `1.5` 倍，高度不超过 `1.2` 倍
- 若检查不通过，使用同一组匹配点拟合纯平移单应性矩阵 `_estimate_translation_homography`：
  - 以中位数位移作为 `tx, ty`
  - 按 `ransac_thresh` 统计内点
  - 内点数不足时返回失败
- 策略标记会追加 `+translation`，调试窗口只绘制最终内点

#### Step 4.5: 中心高度后修正（Centre-Height Post-Correction）

当 `force_center_alignment = True`（默认开启）时，在单应性矩阵 **成功估计并通过合理性检查之后**，施加一个后修正步骤。该步骤**不改变特征匹配或估计过程**，仅在最终返回的 H 上调整垂直平移分量。

**问题背景：**

四台相机在 Gazebo 中物理上处于同一水平面，理论上相邻图像的中心应在同一高度。但 `cv2.findHomography` 估计的完整 8 自由度单应性矩阵会捕获景深视差和旋转透视，引入微小的垂直漂移。该漂移在三对相邻拼接中累积放大，导致全景图上下出现黑边。

**修正方法：**

调用 `_constrain_center_height(H, w, h)`，重新计算 `H[1,2]`（垂直平移分量），使源图像中心 `(w/2, h/2)` 映射到目标平面的高度 `h/2`：

```
设源图像中心 (cx, cy) = (w/2, h/2)
denom = H[2,0]*cx + H[2,1]*cy + H[2,2]
new_H[1,2] = cy * denom - H[1,0]*cx - H[1,1]*cy
```

- **保留** H 的所有旋转、缩放、透视项不变 — 仅修正垂直平移
- **不干预**特征匹配、RANSAC 估计或离群点过滤 — 修正纯粹是后处理
- 对每对相邻图像分别施加，通过累积单应性矩阵传播到全局
- 若 `denom` 接近零（退化透视），跳过修正，返回原 H
- 修正后日志输出 `Applied centre-height correction (strategy)`

**设计要点：**

该方案曾尝试过用纯水平平移（dy=0）替代完整单应性估计，但对于 90° 向外辐射的相机阵列，旋转透视对重叠区对齐至关重要，纯平移模型会导致拼接失败。因此改为后修正方式：先让估计正常进行，再在结果上施加约束。

#### Step 5: 调试可视化（仅当 `debug_match=True` 且 `pair_idx == debug_pair`）

使用 `cv2.drawMatches` 绘制：
- **匹配上的关键点**：绿色圆圈 + 绿色连线
- **未匹配上的关键点**：绿色圆点（singlePointColor）
- 窗口标题显示配对编号和使用的匹配策略

---

### 2.4 图像融合（Blending）

在 `stitch()` 的每帧执行阶段，对每幅图像：

```
for img, H in images × _warp_homographies:
    warped = cv2.warpPerspective(img, H, canvas_size)     # 透视变换
    mask   = cv2.warpPerspective(ones, H, canvas_size)     # 变换后的有效区域掩码
    dist   = cv2.distanceTransform(mask, DIST_L2, 5)      # 距离变换（羽化权重）
    accumulator += warped * dist                           # 加权累加
    weights     += dist                                    # 权重累加

panorama = accumulator / weights                           # 归一化
```

- 使用 **distanceTransform** 计算每个像素到有效区域边界的距离
- 越靠近图像边缘权重越低，实现平滑过渡
- maskSize=5 意味着使用更精确的 L2 距离近似
- 最后 clip 到 [0, 255] 并转为 uint8
- `_debug_show_concat` 中间调试窗口同样受 `_max_canvas_width_ratio` / `_max_canvas_height_ratio` 画布上限约束

### 2.5 方位角 → 全景 X 坐标映射（10° 区间分割）

拼接后的全景图位于 cam0 成像平面上，**X 坐标与方位角不是线性关系**（正切/投影关系，且在距 cam0 光轴 ±90° 以外发散），因此不能对全景图 X 轴做均匀等分。但利用已知先验可以**逐相机精确解析**任意方位角在全景图中的 X 坐标：

**已知先验：**
- 每台相机光轴的方位角 `θ_i`（基准 0°/90°/180°/270°；机器人变形后由 TF 实时读取，仍为已知，且不要求均匀间隔）
- Gazebo 理想针孔相机（无畸变）：相对方位角 `rel`（逆时针为正）的光线在相机 i 中落在像素 `x_src = cx - f·tan(rel)`，其中 `f = (w/2)/tan(60°) ≈ 184.75 px。注意符号：标准光学坐标系（z 前、x 右、y 下）下图像 u 随逆时针方位角增大而**减小**

**映射方法（`angle_to_pano_x`）：**

```
对给定方位角 θ：
  1. 找出所有覆盖 θ 的相机（|wrap(θ - θ_i)| ≤ 60°）
  2. 计算该光线在相机 i 中的像素 x_src = 320 - f·tan(θ - θ_i)
  3. 用 stitch() 实际使用的 _warp_homographies[i] 将 (x_src, 240) 投影到全景画布 → X
```

- 由于使用与 `stitch()` **同一组最终单应性矩阵**，映射结果与实际全景图严格自洽（平移回退、中心高度修正、画布缩放全部自动包含在内）
- **首尾重复带**：被两台相机同时覆盖的方位角（如 300°~330° 同时被 cam3 和 cam0 覆盖）会返回**两个 X 值**——分别对应全景图右尾和左头的重复出现位置；返回列表按距光轴角距排序，最近相机在前
- 无相机覆盖的方位角返回 `None`

**10° 区间边界（`get_interval_boundaries`）：**

- 边界网格对齐到 cam0 光轴相对坐标系中 10° 的整数倍，覆盖整个相机环的方位角范围（基准配置下为 -60° ~ +330°，共 40 条边界、39 个区间）
- 每条目的**首个命中为主命中**：取展开（unwrapped）坐标系中距 θ 最近的相机，保证主 X 序列在首尾重叠带处仍然单调递增；重复带命中（若有）排在第二位
- **缓存与阈值**：结果缓存在 `_interval_cache`，仅当满足以下任一条件时重算：
  1. 拼接几何被重算（`_geom_version` 递增，即 `compute_stitch` 成功执行后）
  2. 任一相机光轴方位角与缓存值的偏差 **≥ 5°**（`change_thresh_deg`）
  - 小于阈值的漂移直接复用缓存表，与节点侧 TF 触发重算的阈值语义一致

**节点侧 TF 监控（`display_four_camera.py`）：**

- 通过 `tf2_ros` 以 1 Hz 查询各 `camera_optical_link_i` 相对 `base_footprint` 的变换
- 取光学坐标系 +Z 轴（光轴）在基座水平面的投影方位角
- 任一相机方位角相对记录值漂移 **> 5°** 时：更新记录值与 `axis_angles`，并调用 `request_recompute()` 触发下一帧几何重算
- 显示窗口中以 `cv2.addWeighted` 叠加半透明色带（`interval_band_alpha`，默认 0.25）：每个 10° 区间填充一个颜色（9 色调色板循环，即每 90° 循环一次，跨越相邻两条主边界 X；主 X 随 θ 可能递增或递减，绘制时取 min/max），首尾重复带在全景图另一端以橙色色带标出（仅当第二命中与主命中相距超过半幅画布时判定为重复带）
- 文字标注仅 4 个：各相机区域起点（相邻光轴中点，基准为 -45 / 45 / 135 / 225，由 `get_camera_region_starts` 计算），全景图中从右到左依次对应 camera1~camera4

---

## 3. 状态管理

| 属性 | 初始值 | 说明 |
|------|--------|------|
| `_ready` | False | 拼接几何是否已成功计算 |
| `_recompute` | True | 是否需要在下一帧重新计算几何 |
| `_adj_homographies` | None | 相邻单应性矩阵列表 [H_0, H_1, H_2] |
| `_warp_homographies` | None | 含平移的最终单应性矩阵列表 [T@cum[0], T@cum[1], T@cum[2], T@cum[3]] |
| `_canvas_size` | None | 全景画布尺寸 (w, h) |
| `_img_size` | None | 归一化输入图像尺寸 (h, w)，用于角度→X 映射的焦距/主点计算 |
| `_geom_version` | 0 | 几何版本号，`compute_stitch` 成功后递增，用于派生数据缓存失效判断 |
| `_interval_cache` | None | 10° 区间边界表缓存 (geom_version, axis_angles, step_deg, table) |

外部可通过 `request_recompute()` 方法设置 `_recompute=True`，触发下一帧重新计算几何。节点侧另有 TF 监控（1 Hz，5° 阈值）自动触发，见 §2.5。

---

## 4. 常见调试要点

### 4.1 匹配失败
- **症状**：日志显示 "Homography failed" 或 "matches insufficient"
- **排查方向**：
  1. 开启 `debug_match=True`，设置 `debug_pair` 到问题配对
  2. 观察可视化窗口中的绿色圆点数量：如果很少 → ORB 特征点检测不足（尝试增大 `nfeatures` 或检查图像质量）
  3. 如果绿色圆点多但连线少 → 匹配策略过严（适当放宽 `match_ratio` 或降低 `min_matches`）
  4. 检查重叠区域裁剪是否正确（左右半边选取逻辑）

### 4.2 拼接错位/鬼影
- **症状**：全景图中重叠区域有明显错位或重影
- **排查方向**：
  1. 检查 RANSAC 内点数是否偏低（放宽 `ransac_thresh`）
  2. 可能是相机之间的重叠区域不足，导致单应性估计不准确
  3. 检查相机是否在运动（若相机在动，需频繁 `request_recompute`）

### 4.3 融合边界生硬
- **症状**：重叠区域有明显接缝
- **排查方向**：
  1. 检查 `distanceTransform` 的 maskSize 参数（当前为 5，可尝试 3 获得更平滑的过渡）
  2. 可能是单应性矩阵不够精确，导致重叠区域未对齐

### 4.4 画布或 `_debug_show_concat` 窗口过大
- **症状**：全景图或中间拼接调试窗口尺寸巨大（远超 `4×` 单图宽 / `1.5×` 单图高）
- **原因**：`cv2.findHomography` 可能估计出退化的透视/缩放单应性矩阵，导致画布四角范围爆炸
- **处理**：当前实现已自动拒绝不合理的透视矩阵并回退到纯平移模型；若仍超限，则统一缩放整个画布。可检查日志中的 `Full homography unreasonable` 和 `Canvas ... exceeds limits` 信息。

### 4.5 ORB 特征点质量差
- **症状**：debug 窗口中绿色圆点稀疏、分布不均（某些区域密集、某些区域几乎没有），或明显角点被跳过
- **原因**：ORB 的 FAST 角点检测有三个关键控制参数：
  - `nfeatures`（默认 500）：ORB 最终保留的特征点上限。如果设置过低，ORB 会丢弃响应值较低的特征点，导致某些区域空白
  - `fastThreshold`（默认 10，OpenCV 默认 20）：FAST 检测器的灰度差阈值。值越低，越敏感，能检测到对比度较低的角点；值越高，只保留强角点
  - `grid_rows` / `grid_cols`（默认 3×3）：网格化均匀分配的行列数。ORB 本身按响应值排序保留前 N 个特征点，容易在纹理密集区过度聚集；网格化在检测后对特征点做均匀重分配，强制特征点均匀覆盖整个重叠区
- **调优方向**：
  1. **优先调低 `fastThreshold`**（如 5 ~ 8）：让 FAST 检测到更多候选角点，这是最有效的单一参数
  2. **增大 `nfeatures`**（如 1000 ~ 2000）：让 ORB 保留更多特征点，避免因数量上限丢弃有效角点
  3. **增大 `grid_rows` / `grid_cols`**（如 4×4、5×3）：更细粒度的网格可进一步抑制局部过密。由于检测在全图上进行，网格粒度不再影响检测质量，可以安全地将网格设置到 8×8 甚至更大而不会丢失特征点；但需要注意，网格越细每格预算越少，若预算过少（如每格仅分配 1~2 个特征点）可能削弱后续匹配的鲁棒性
  4. 注意：`fastThreshold` 过低可能导致大量低质量角点被检测，反而增加误匹配；建议结合 debug 窗口观察效果
  5. 如果图像纹理本身就弱（如白墙、均匀表面），任何检测器都难以提取足够特征，此时应考虑改善场景光照或增加纹理

### 4.6 全景图上下黑边（垂直漂移）
- **症状**：拼接后的全景图上下出现黑边，相邻图像中心高度不一致
- **原因**：完整 8 自由度单应性矩阵会捕获景深视差和旋转透视，引入微小垂直漂移；漂移在三对相邻拼接中累积放大
- **处理**：`force_center_alignment = True`（默认开启）会在每对单应性矩阵估计完成后施加中心高度后修正（见 §2.3 Step 4.5），将源图像中心映射到目标高度 `h/2`。检查日志中是否出现 `Applied centre-height correction` 确认修正已生效。若仍存在残余黑边，可检查 `_max_canvas_height_ratio`（默认 1.5）是否需要调小
