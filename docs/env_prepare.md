# 四臂机器人开发环境搭建文档

本文档描述从零开始搭建四臂机器人（four_arm_robot）开发环境的完整流程，涵盖：

1. Ubuntu 24.04 双系统安装（已有 Windows）
2. ROS 2 Jazzy 及 RViz / Gazebo / MoveIt / Nav2 / CycloneDDS 安装
3. TensorRT 安装与测试
4. Conda 环境安装（numpy / numba / cv2 / torch / onnxruntime）

> 适用系统：Ubuntu 24.04 LTS（Noble）
> ROS 发行版：ROS 2 Jazzy Jalisco
> GPU：NVIDIA（用于 TensorRT / CUDA 加速）

---

## 目录

- [一、Ubuntu 24.04 双系统安装（已有 Windows）](#一ubuntu-2404-双系统安装已有-windows)
- [二、系统基础配置](#二系统基础配置)
- [三、ROS 2 Jazzy 安装](#三ros-2-jazzy-安装)
- [四、RViz 安装](#四rviz-安装)
- [五、Gazebo 安装](#五gazebo-安装)
- [六、MoveIt 2 安装](#六moveit-2-安装)
- [七、Nav2 安装](#七nav2-安装)
- [八、CycloneDDS 安装与配置](#八cyclonedds-安装与配置)
- [九、TensorRT 安装与测试](#九tensorrt-安装与测试)
- [十、Conda 环境安装](#十conda-环境安装)
- [附录：常见问题排查](#附录常见问题排查)

---

## 一、Ubuntu 24.04 双系统安装（已有 Windows）

### 1.1 准备工作

- 一个 ≥ 8GB 的 U 盘（用于制作启动盘）
- Ubuntu 24.04 LTS 桌面版 ISO 镜像
  - 下载地址：<https://ubuntu.com/download/desktop>
- 建议磁盘剩余空间 ≥ 100GB（用于 Ubuntu 分区）
- 备份 Windows 重要数据（安装过程有风险，务必先备份）

### 1.2 在 Windows 中划分空闲分区

1. 右键「此电脑」→「管理」→「磁盘管理」。
2. 右键目标磁盘分区 →「压缩卷」，压缩出至少 100GB（102400 MB）空闲空间。
3. 压缩后保持「未分配」状态，**不要**在 Windows 中新建分区或格式化。

### 1.3 关闭 Windows 快速启动与 BitLocker

- 关闭快速启动（避免磁盘被锁定）：
  - 控制面板 → 电源选项 → 选择电源按钮的功能 → 更改当前不可用的设置 → 取消勾选「启用快速启动」。
- 关闭 BitLocker（若已开启）：
  - 设置 → 隐私和安全 → 设备加密 → 关闭。

### 1.4 制作 Ubuntu 启动盘

推荐使用 [Rufus](https://rufus.ie/)（Windows 下）：

1. 插入 U 盘，打开 Rufus。
2. 选择设备（U 盘）、引导类型选择下载好的 ISO。
3. 分区类型选择 **GPT**，目标系统选择 **UEFI**。
4. 点击「开始」写入。

### 1.5 BIOS/UEFI 设置

1. 重启电脑，按 `F2` / `Del` / `F12`（不同主板不同）进入 BIOS。
2. 关闭 **Secure Boot**（安全启动）。
3. 将启动模式设置为 **UEFI**（不要使用 Legacy/CSM）。
4. 调整启动顺序，将 U 盘设为第一启动项（或按 `F12` 临时选择启动项）。

### 1.6 安装 Ubuntu

1. 从 U 盘启动，选择「Try or Install Ubuntu」。
2. 进入安装界面后选择语言、键盘布局、时区。
3. 在「安装类型」步骤选择 **「与 Windows Boot Manager 共存安装」**（Install alongside Windows Boot Manager）。
   - 若未出现该选项，选择「其他选项」手动分区，在之前划分的空闲空间上创建：
     - `/`（根分区）：ext4，建议 ≥ 80GB
     - `swap`：建议等于内存大小（内存 ≥ 32GB 可设 16GB）
     - `/boot/efi`：若已存在 Windows 的 EFI 分区，**直接复用**（不要格式化），否则新建 512MB EFI 分区
4. 确认引导加载器安装位置（一般默认即可）。
5. 创建用户名与密码，开始安装。
6. 安装完成后重启，拔出 U 盘。

### 1.7 验证双系统

- 重启后应出现 GRUB 菜单，可选择进入 Ubuntu 或 Windows。
- 若直接进入 Windows 未显示 GRUB，进入 BIOS 将 `ubuntu` 启动项调整为第一优先级。

---

## 二、系统基础配置

### 2.1 更换软件源（可选，加速下载）

```bash
sudo cp /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak
sudo sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list.d/ubuntu.sources
sudo sed -i 's|http://security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list.d/ubuntu.sources
sudo apt update && sudo apt upgrade -y
```

### 2.2 安装基础工具

```bash
sudo apt update
sudo apt install -y build-essential cmake git curl wget vim \
    python3-pip python3-dev software-properties-common \
    net-tools htop unzip
```

### 2.3 安装 NVIDIA 显卡驱动

```bash
# 查看可用驱动
ubuntu-drivers devices

# 自动安装推荐驱动
sudo ubuntu-drivers autoinstall

# 重启
sudo reboot
```

重启后验证：

```bash
nvidia-smi
```

能正常显示 GPU 信息与驱动版本即安装成功。

---

## 三、ROS 2 Jazzy 安装

### 3.1 设置语言环境（locale）

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3.2 添加 ROS 2 apt 仓库

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 3.3 安装 ROS 2 Jazzy（desktop 完整版）

`desktop` 版本已包含 RViz、仿真工具等，推荐安装：

```bash
sudo apt install -y ros-jazzy-desktop
```

如需开发工具（编译器、调试器等）：

```bash
sudo apt install -y ros-dev-tools
```

### 3.4 配置环境变量

将以下内容加入 `~/.bashrc`：

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.5 验证安装

```bash
# 终端 1
ros2 run demo_nodes_cpp talker

# 终端 2
ros2 run demo_nodes_py listener
```

若 listener 能正常接收 talker 消息，则 ROS 2 安装成功。

---

## 四、RViz 安装

RViz2 已包含在 `ros-jazzy-desktop` 中，无需单独安装。

验证：

```bash
rviz2
```

若未安装，可单独安装：

```bash
sudo apt install -y ros-jazzy-rviz2
```

---

## 五、Gazebo 安装

ROS 2 Jazzy 推荐使用新版 **Gazebo（原 Ignition Gazebo，Harmonic 版本）**，同时也可安装经典 Gazebo。

### 5.1 安装 Gazebo Harmonic（推荐）

```bash
sudo apt install -y ros-jazzy-ros-gz
```

验证：

```bash
ign gazebo shapes.sdf
# 或
gz sim shapes.sdf
```

### 5.2 安装经典 Gazebo（可选，兼容旧项目）

```bash
sudo apt install -y gazebo libgazebo-dev
sudo apt install -y ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros2-control
```

验证：

```bash
gazebo
```

### 5.3 Gazebo 与 ROS 2 桥接

```bash
sudo apt install -y ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image
```

---

## 六、MoveIt 2 安装

### 6.1 二进制安装（推荐）

```bash
sudo apt install -y ros-jazzy-moveit
```

### 6.2 安装 MoveIt 相关工具

```bash
sudo apt install -y \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-setup-assistant \
    ros-jazzy-moveit-servo
```

### 6.3 验证安装

```bash
# 启动 MoveIt 2 演示（若已安装 moveit_resources）
sudo apt install -y ros-jazzy-moveit-resources-panda-moveit-config
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

能打开 RViz 并加载 Panda 机械臂规划界面即安装成功。

---

## 七、Nav2 安装

### 7.1 二进制安装

```bash
sudo apt install -y ros-jazzy-navigation2
sudo apt install -y ros-jazzy-nav2-bringup
```

### 7.2 验证安装

```bash
sudo apt install -y ros-jazzy-turtlebot3* 
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

能打开 Gazebo + RViz 的 TurtleBot3 仿真导航界面即安装成功。

---

## 八、CycloneDDS 安装与配置

CycloneDDS 是 ROS 2 支持的 DDS 中间件之一，常用于多机通信、降低延迟。

### 8.1 安装 CycloneDDS RMW 实现

```bash
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
```

### 8.2 安装 CycloneDDS 工具（可选）

```bash
sudo apt install -y cyclonedds-tools
```

### 8.3 配置为默认 DDS

将以下内容加入 `~/.bashrc`：

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 8.4 自定义 CycloneDDS 配置（可选）

创建配置文件 `~/cyclonedds.xml`：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

在 `~/.bashrc` 中指定：

```bash
echo "export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml" >> ~/.bashrc
source ~/.bashrc
```

### 8.5 验证

```bash
echo $RMW_IMPLEMENTATION
# 输出：rmw_cyclonedds_cpp

# 再次运行 talker/listener，确认通信正常
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

---

## 九、TensorRT 安装与测试

> 前提：已完成 NVIDIA 驱动安装（见 2.3）。TensorRT 依赖 CUDA 与 cuDNN。

### 9.1 安装 CUDA Toolkit

推荐使用 CUDA 12.x（与 TensorRT 10.x 匹配）：

```bash
# 方式一：使用 ubuntu 仓库（简化版）
sudo apt install -y nvidia-cuda-toolkit

# 查看版本
nvcc --version
```

如需指定版本（推荐从 NVIDIA 官网获取最新 deb 安装方式）：

```bash
# 以 CUDA 12.4 为例（请以官网为准）
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-4
```

配置环境变量（加入 `~/.bashrc`）：

```bash
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 9.2 安装 cuDNN

```bash
sudo apt install -y libcudnn8 libcudnn8-dev
# 或使用 cuda 仓库的 cudnn 包
sudo apt install -y libcudnn*-dev
```

### 9.3 安装 TensorRT

```bash
sudo apt update
sudo apt install -y tensorrt libnvinfer-dev libnvinfer-plugin-dev \
    python3-libnvinfer python3-libnvinfer-dev
```

### 9.4 验证 TensorRT 安装

```bash
# 查看 TensorRT 版本
dpkg -l | grep -i nvinfer

# Python 中验证
python3 -c "import tensorrt as trt; print('TensorRT version:', trt.__version__)"
```

### 9.5 TensorRT 功能测试

使用自带的 `trtexec` 工具测试推理性能：

```bash
# 查看 trtexec 位置
which trtexec
# 一般在 /usr/src/tensorrt/bin/trtexec

# 用一个简单的 ONNX 模型测试（替换为你的模型路径）
/usr/src/tensorrt/bin/trtexec \
    --onnx=/path/to/model.onnx \
    --saveEngine=/path/to/model.engine \
    --fp16

# 仅做性能基准测试（使用随机权重）
/usr/src/tensorrt/bin/trtexec --fp16 --shapes=input:1x3x640x640
```

Python 端简单测试脚本：

```python
import tensorrt as trt

print("TensorRT version:", trt.__version__)
logger = trt.Logger(trt.Logger.WARNING)
builder = trt.Builder(logger)
network = builder.create_network(
    1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
print("TensorRT 初始化成功，Builder/Network 创建正常")
```

---

## 十、Conda 环境安装

### 10.1 安装 Miniconda

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -r ~/miniconda3/miniconda.sh
rm ~/miniconda3/miniconda.sh

# 初始化
~/miniconda3/bin/conda init bash
source ~/.bashrc
```

> 也可使用清华镜像加速：
> ```bash
> conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
> conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free
> conda config --set show_channel_urls yes
> ```

### 10.2 创建项目环境

```bash
# 创建 Python 3.10 环境（与 torch/ROS 工具兼容性较好）
conda create -n four_arm python=3.10 -y
conda activate four_arm
```

### 10.3 安装 numpy / numba / opencv

```bash
conda install -n four_arm -y numpy numba
# opencv（cv2）
pip install opencv-python opencv-contrib-python
# 或使用 conda
# conda install -n four_arm -y -c conda-forge opencv
# PIL（Pillow，图像处理）
pip install pillow
```

### 10.4 安装 PyTorch

根据 CUDA 版本选择（以 CUDA 12.1 为例）：

```bash
# CUDA 12.1
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# 若使用 CPU 版本
# pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

验证 PyTorch 与 GPU：

```bash
python -c "import torch; print('torch:', torch.__version__); \
    print('cuda available:', torch.cuda.is_available()); \
    print('device count:', torch.cuda.device_count())"
```

### 10.5 安装 onnxruntime

```bash
# GPU 版本（依赖 CUDA）
pip install onnxruntime-gpu

# 或 CPU 版本
# pip install onnxruntime
```

验证：

```bash
python -c "import onnxruntime as ort; \
    print('onnxruntime:', ort.__version__); \
    print('providers:', ort.get_available_providers())"
```

### 10.6 环境验证汇总

```bash
conda activate four_arm
python - <<'EOF'
import numpy, numba, cv2, torch, onnxruntime
print("numpy       :", numpy.__version__)
print("numba       :", numba.__version__)
print("opencv(cv2) :", cv2.__version__)
print("torch       :", torch.__version__, "| cuda:", torch.cuda.is_available())
print("onnxruntime :", onnxruntime.__version__)
EOF
```

### 10.7 注意事项：Conda 与 ROS 2 的 PYTHONPATH 冲突

> ⚠️ 重要：在 ROS 2 环境中激活 conda 时，conda 会覆盖 `PYTHONPATH`，
> 可能导致 `rclpy` 等 ROS Python 包无法导入。

推荐做法：

- 运行 ROS 2 节点时，**不要**激活 conda 环境，使用系统 Python。
- 运行深度学习推理（torch / tensorrt / onnxruntime）时，激活 conda 环境。
- 若必须在 conda 中调用 ROS，可在激活后临时还原：

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
```

或在 conda 环境中安装与系统一致的 Python 版本，避免 ABI 冲突。

---

## 附录：常见问题排查

### A. `nvidia-smi` 报错 / 无 GPU

- 确认 BIOS 中显卡未被禁用。
- 重新安装驱动：`sudo ubuntu-drivers autoinstall && sudo reboot`。
- 检查 Secure Boot 是否关闭（开启会阻止第三方驱动加载）。

### B. `ros2` 命令找不到

- 确认已执行 `source /opt/ros/jazzy/setup.bash`，并写入 `~/.bashrc`。

### C. Gazebo 启动黑屏 / 崩溃

- 多显卡笔记本需确认使用独显运行：
  ```bash
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  ```

### D. TensorRT Python 导入失败

- 确认 `python3-libnvinfer` 已安装，且 Python 版本匹配。
- 在 conda 环境中需单独 `pip install tensorrt`（对应 CUDA 版本）。

### E. conda 环境内 `import rclpy` 失败

- 见 [10.7 注意事项](#107-注意事项conda-与-ros-2-的-pythonpath-冲突)，避免 conda 覆盖 ROS 的 `PYTHONPATH`。

### F. DDS 多机通信不通

- 确认所有机器 `ROS_DOMAIN_ID` 一致：
  ```bash
  export ROS_DOMAIN_ID=0
  ```
- 确认防火墙放行 UDP 多播，或正确配置 CycloneDDS 网卡接口。
