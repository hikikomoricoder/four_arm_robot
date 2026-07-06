# YOLO11s Detection 知识蒸馏方案

## 一、问题定义

在已基于海量数据训练好的 YOLO11s 检测模型基础上，使用少量新数据实现：

1. **新增类别**：扩展模型的检测类别（如从 COCO 80 类扩展到 85 类）
2. **原有类别新形态**：检测已有关别的新形态目标（如新角度、新尺度、新场景下的目标）

核心挑战：
- 新数据量少，直接微调会导致灾难性遗忘（catastrophic forgetting）
- 检测头需要扩展以支持新类别
- 必须保留原有模型对旧类别的检测能力

## 二、数据组织

### 数据集 YAML 格式

```yaml
# new_data.yaml
path: /path/to/dataset
train: images/train
val: images/val

# 类别名称：前 old_nc 个为原有类别，后 new_nc 个为新增类别
names:
  0: person        # 原有类别 0
  1: bicycle       # 原有类别 1
  ...
  79: toothbrush   # 原有类别 79 (old_nc=80)
  80: new_class_1  # 新增类别 0
  81: new_class_2  # 新增类别 1
  ...
nc: 85             # old_nc + new_nc
```

### 数据要求

- 每张图片可同时包含原有类别和新增类别的标注
- 标注格式为 YOLO 格式（class_id cx cy w h，归一化坐标）
- 建议新数据中保留部分原有类别的标注样本，帮助模型维持旧类别能力
- 如果新数据中不含某旧类别的样本，蒸馏损失将帮助保留该类别知识

## 三、模型架构

```
Teacher (yolo11s.pt, 冻结)
  ├── Backbone (冻结)      → 提供稳定的特征提取
  ├── Neck/FPN (冻结)      → 提供多尺度特征金字塔
  └── Detect Head (80类)   → 提供旧类别的 logit 监督信号

Student (训练中)
  ├── Backbone (可训练)    → 从 teacher 继承权重初始化
  ├── Neck/FPN (可训练)    → 从 teacher 继承权重初始化
  └── Detect Head (85类)   → 旧类别权重从 teacher 拷贝，新类别随机初始化
```

### Head 扩展策略

YOLO11 的 Detect Head 结构：
- `cv2[i]` — 边框回归分支，输出 `4 × reg_max` 通道（与类别数无关）
- `cv3[i]` — 分类分支，输出 `nc` 通道

扩展方式：
1. 创建新模型时设置 `nc = old_nc + new_nc`
2. 加载 teacher 权重：backbone + neck + cv2 完全匹配（`intersect_dicts`）
3. cv3 的最后一层 Conv2d（`cv3.i.2`）需要手动拷贝：
   - `weight[:old_nc, :, :, :]` 从 teacher 拷贝
   - `weight[old_nc:, :, :, :]` 随机初始化
   - `bias[:old_nc]` 从 teacher 拷贝
   - `bias[old_nc:]` 随机初始化（使用 `bias_init_with_prob`）

## 四、损失函数设计

总损失由三部分组成：

### 1. 检测损失（Detection Loss）

标准的 YOLO 检测损失，在包含新旧类别标签的数据上计算：

```
L_det = L_box + L_cls + L_dfl
```

使用 Ultralytics 内置的 `v8DetectionLoss`，学生模型的 `nc = old_nc + new_nc`，正常匹配 ground truth。

### 2. Logit 蒸馏损失（Logit Distillation Loss）

在三个检测尺度（P3/8, P4/16, P5/32）上，将学生旧类别的 logits 对齐到 teacher 的 logits：

```
L_kd = T² × KL( softmax(student_logits_old / T) || softmax(teacher_logits / T) )
```

- `T`：温度参数（默认 3.0），用于软化概率分布
- `student_logits_old`：学生分类输出中前 old_nc 个通道
- `teacher_logits`：教师分类输出（old_nc 通道）
- 使用 KL 散度（Kullback-Leibler divergence）

### 3. 特征蒸馏损失（Feature Distillation Loss）

在 FPN 的三个输出层（P3/P4/P5）匹配 teacher 和 student 的中间特征图：

```
L_feat = MSE(student_feat, teacher_feat)
```

- 匹配层索引：16 (P3), 19 (P4), 22 (P5)
- 这些是输入到 Detect Head 之前的 FPN 特征图

### 综合损失

```
L_total = L_det + λ_kd × L_kd + λ_feat × L_feat
```

默认权重：
- `λ_kd = 5.0`（logit 蒸馏权重）
- `λ_feat = 1.0`（特征蒸馏权重）

## 五、训练策略

### 阶段一：预热（Warmup）

epoch 1-3：仅训练检测头（冻结 backbone + neck）
- 让新增的 head 通道先适应新数据分布
- 此时仅使用检测损失（`λ_kd = 0, λ_feat = 0`）

### 阶段二：全模型蒸馏

epoch 4+：解冻 backbone + neck，启用全部损失
- 使用较小的学习率（lr0 = 0.001 ~ 0.005，通常为正常训练的 1/10）
- 同时使用检测损失 + 蒸馏损失

### 关键超参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| epochs | 30-100 | 取决于数据量 |
| lr0 | 0.002 | 初始学习率（正常训练的 1/5~1/10） |
| batch | 16 | 根据 GPU 显存调整 |
| imgsz | 640 | 与 teacher 模型保持一致 |
| warmup_epochs | 3 | 预热轮数 |
| temperature (T) | 3.0 | Logit 蒸馏温度 |
| kd_weight (λ_kd) | 5.0 | Logit 蒸馏损失权重 |
| feat_weight (λ_feat) | 1.0 | 特征蒸馏损失权重 |
| freeze | [10] | 冻结前 10 层（backbone），配合 warmup 使用 |

## 六、预期效果与监控

### 监控指标

训练过程中重点关注：
- `train/box_loss, train/cls_loss, train/dfl_loss` — 标准检测损失
- `train/kd_loss` — Logit 蒸馏损失（应逐渐降低并稳定）
- `train/feat_loss` — 特征蒸馏损失（应逐渐降低并稳定）
- `val/mAP50-95(all)` — 所有类别的 mAP
- `val/mAP50-95(old)` — 仅旧类别的 mAP（是否发生灾难性遗忘）
- `val/mAP50-95(new)` — 仅新类别的 mAP（新类别学习效果）

### 预期行为

1. kd_loss 从较高值开始，随着训练逐渐下降并趋于平稳
2. feat_loss 逐渐下降，表明学生特征与教师趋于一致
3. 旧类别 mAP 应保持接近 teacher 水平（不下降超过 5%）
4. 新类别 mAP 随着训练逐渐提升

### 调优建议

- **旧类别遗忘严重**：增大 `λ_kd` 和 `λ_feat`，或减小学习率
- **新类别学习不足**：增大学习率，或减小 `λ_kd`
- **训练不稳定**：降低学习率，增加 warmup epochs，检查数据标注质量
- **温度调整**：T 越大，teacher 的软标签越平滑，适用于类别间关系复杂的情况

## 七、使用方法

```python
from distillation import DistillationTrainer

trainer = DistillationTrainer(
    teacher_weights="yolo11s.pt",  # 预训练的教师模型
    old_nc=80,                      # 教师模型原有类别数（如 COCO=80）
    data="new_data.yaml",           # 新数据集配置（含新旧类别）
    epochs=50,
    batch=16,
    imgsz=640,
    lr0=0.002,
    warmup_epochs=3,
    temperature=3.0,
    kd_weight=5.0,
    feat_weight=1.0,
    name="distill_exp",
    device=0,
)

trainer.train()
```
