## 项目介绍

MOSS-TTS-Nano 是来自 [MOSI.AI](https://mosi.cn/#hero) 和 [OpenMOSS 团队](https://www.open-moss.com/) 的开源**多语言微型语音生成模型**。仅 **0.1B 参数**，专为**实时语音生成**设计，可直接在 **CPU 上运行（无需 GPU）**，部署栈足够简单，适用于本地演示、网络服务和轻量级产品集成。

- **主要特性**：48 kHz 立体声输出、20 种语言、纯自回归架构（Audio Tokenizer + LLM）、流式推理、4 核 CPU 即可运行
- **源码仓库**：https://github.com/OpenMOSS/MOSS-TTS-Nano
- **本地版本**：commit `cc7bdf1`（2026-07-26），基于 `moss-tts-nano` conda 环境（Python 3.12）


## 环境配置

```bash
conda create -n moss-tts-nano python=3.12 -y
conda activate moss-tts-nano

git clone https://github.com/OpenMOSS/MOSS-TTS-Nano.git
cd MOSS-TTS-Nano

pip install -r requirements.txt
pip install -e .
```

如果 `WeTextProcessing` / `pynini` 安装失败，先用 conda 装 pynini 再装 WeTextProcessing：

```bash
conda install -c conda-forge pynini=2.1.6.post1 -y
pip install git+https://github.com/WhizZest/WeTextProcessing.git
pip install -r requirements.txt
```

CUDA 推理需额外安装：

```bash
pip uninstall -y onnxruntime
pip install "onnxruntime-gpu>=1.20.0"
```


## 联网屏蔽

为确保离线可用（避免 huggingface_hub 每次向 hf-mirror.com 发 HEAD 请求导致超时约 5 分钟），已对以下文件做离线屏蔽：

| 文件 | 修改内容 |
| --- | --- |
| `infer.py` | 设置 `HF_HUB_OFFLINE=1`、`TRANSFORMERS_OFFLINE=1`；`from_pretrained` 加 `local_files_only=True` |
| `moss_tts_nano_runtime.py` | 同上，环境变量必须在 `import transformers` 之前设置 |
| `onnx_tts_runtime.py` | `ensure_browser_onnx_model_dir()` 缺 ONNX 资产时直接抛错，不再自动下载 |
| `finetuning/sft.py`、`verify.py`、`prepare_data.py` | `from_pretrained` 加 `local_files_only=True` |

本地缓存均已就绪（`~/.cache/huggingface/`），无需联网。

恢复联网时按 `NETWORK_OFFLINE_CHANGES.md` 中的「如何恢复联网」逐项还原即可。


## 模型准备

### PyTorch 模型（`infer.py` / `app.py` 使用）

首次运行时会自动从 Hugging Face 下载到 `~/.cache/huggingface/`，需联网。涉及两个模型：

| 模型 | Hugging Face 仓库 |
| --- | --- |
| TTS 主模型 | `OpenMOSS-Team/MOSS-TTS-Nano` |
| 音频分词器 | `OpenMOSS-Team/MOSS-Audio-Tokenizer-Nano` |

下载后本地缓存路径：
- `~/.cache/huggingface/hub/models--OpenMOSS-Team--MOSS-TTS-Nano/`
- `~/.cache/huggingface/hub/models--OpenMOSS-Team--MOSS-Audio-Tokenizer-Nano/`

### ONNX 模型（`infer_onnx.py` / `app_onnx.py` 使用）

ONNX 资产需从 Hugging Face 下载（需有梯子），放置到项目 `./models/` 目录：

| 模型 | Hugging Face 仓库 | 本地目录 |
| --- | --- | --- |
| TTS ONNX | `OpenMOSS-Team/MOSS-TTS-Nano-100M-ONNX` | `./models/MOSS-TTS-Nano-100M-ONNX/` |
| 音频分词器 ONNX | `OpenMOSS-Team/MOSS-Audio-Tokenizer-Nano-ONNX` | `./models/MOSS-Audio-Tokenizer-Nano-ONNX/` |

下载命令（有网 / 有梯子时执行）：

```bash
pip install -U huggingface_hub

huggingface-cli download OpenMOSS-Team/MOSS-TTS-Nano-100M-ONNX \
  --local-dir ./models/MOSS-TTS-Nano-100M-ONNX

huggingface-cli download OpenMOSS-Team/MOSS-Audio-Tokenizer-Nano-ONNX \
  --local-dir ./models/MOSS-Audio-Tokenizer-Nano-ONNX
```

> 当前环境的 ONNX 模型是之前有梯子的设备下载后直接拷贝过来的，离线模式下不会自动下载（见「联网屏蔽」）。


## 可用性验证流程

```bash
# PyTorch 后端
python infer.py \
  --prompt-audio-path assets/audio/zh_1.wav \
  --text "欢迎关注模思智能。"

# ONNX 后端（CPU）
python infer_onnx.py \
  --prompt-audio-path assets/audio/zh_1.wav \
  --text "欢迎使用 ONNX Runtime CPU 版本。"

# ONNX 后端（CUDA）
python infer_onnx.py \
  --execution-provider cuda \
  --prompt-audio-path assets/audio/zh_1.wav \
  --text "欢迎使用 ONNX Runtime CUDA 版本。"
```

输出默认写入 `generated_audio/infer_output.wav`。


## 语音合成服务启动

```bash
# PyTorch 后端（默认端口 18083）
python app.py --host 0.0.0.0 --port 18083

# ONNX CPU 后端
python app_onnx.py --host 0.0.0.0 --port 18083

# ONNX CUDA 后端
python app_onnx.py --host 0.0.0.0 --port 18083 --execution-provider cuda
```

浏览器打开 `http://127.0.0.1:18083`。


## 流式语音输出简单测试

`stream_output_test.py` 是一个独立的流式播放测试脚本，不依赖 `TTSClient`：

1. `POST /api/generate-stream/start` 提交文本 + 音色，获取 `stream_id` 和音频参数
2. 启动 `aplay` 子进程，通过 stdin 管道实时喂入 PCM 原始数据
3. `GET {audio_url}` 流式拉取音频块，逐块写入 `aplay` 的 stdin
4. 播放完毕后 `POST /api/generate-stream/{stream_id}/close` 释放服务端资源

```bash
# 内置中文音色
python stream_output_test.py --text "你好，这是测试。" --demo-id demo-1

# voice cloning（自定义参考音频）
python stream_output_test.py --text "你好" --prompt-audio-path assets/阿布二_... .wav
```


## 流式语音输出接口、实现、调用

### TTSClient 封装

`tts_request.py` 提供 `TTSClient` 类，封装 MOSS TTS Nano HTTP 接口的完整调用：

| 方法 | 说明 |
| --- | --- |
| `start_stream(text, demo_id, prompt_audio_path)` | 启动流式生成，返回 `{stream_id, sample_rate, channels, audio_url}` |
| `stream_audio(audio_url)` | 流式拉取 raw PCM 数据（生成器，逐块 yield） |
| `close_stream(stream_id)` | 关闭流，释放服务端资源 |
| `synthesize_stream(text, ...)` | 一键生成器：`start_stream` → `stream_audio` → 自动 `close_stream` |
| `synthesize_to_file(text, output_path, ...)` | 合成并保存为 WAV 文件（自动添加 RIFF 头） |
| `synthesize_and_play(text, ...)` | 合成并通过 `aplay` 实时播放 |
| `quick_speak(text, ...)` | 类方法，一行调用：`TTSClient.quick_speak("你好")` |

配置从 `tts_config.json` 读取（`server_url`、`chunk_size`、`request_timeout`、`default_attn`、`default_seed` 等）。

### 命令行使用

```bash
# 内置音色播放
python tts_request.py --text "你好世界"

# 自定义音色（voice cloning）
python tts_request.py --text "你好" --voice abubu

# 保存到 WAV 文件
python tts_request.py --text "你好" --voice abubu --output test.wav
```

### 代码调用示例

```python
from tts_request import TTSClient

client = TTSClient()

# 流式播放
client.synthesize_and_play("检测到前方3个目标", demo_id="demo-1")

# 保存到文件
client.synthesize_to_file("你好世界", "output.wav", prompt_audio_path="assets/abubu.wav")

# 一行快速播放
TTSClient.quick_speak("你好", demo_id="demo-1")
```


## 语音合成服务管理

TTS 服务由 `seperated_service` 包的 `service_manager` 节点统一管理，无需手动启停。

### 配置

在 `tts_config.json` 中通过 `seperated_service` 字段定义：

```json
"seperated_service": [
    {
        "name": "moss-tts-nano",
        "conda_env": "moss-tts-nano",
        "work_dir": "/home/dcx/MOSS-TTS-Nano",
        "cmd": "python",
        "args": ["app_onnx.py", "--host", "0.0.0.0", "--port", "18083", "--execution-provider", "cuda"],
        "check_port": 18083,
        "check_keyword": "app_onnx.py"
    }
]
```

### 启动方式

在 `my_robot_gazebo.launch.xml` 最顶部启动，早于所有依赖 TTS 的节点：

```xml
<node pkg="seperated_service" exec="service_manager" name="service_manager" output="screen">
    <param name="config_file" value="/home/dcx/four_arm_robot/extend/moss_tts_nano/tts_config.json" />
    <param name="check_interval" value="10.0" />
    <param name="startup_delay" value="2.0" />
</node>
```

### 运行行为

- 启动前检查 18083 端口 + `pgrep -f app_onnx.py`，残留进程直接 kill
- 在 `moss-tts-nano` conda 环境下启动 `app_onnx.py --execution-provider cuda`
- 每 10s 健康检查：进程退出则自动重启
- 输出日志：`~/.ros/seperated_service/moss-tts-nano.log`
- 节点关闭时自动 terminate 子进程

详见 [service_manager 文档](../seperated_service/service_manager.md)。
