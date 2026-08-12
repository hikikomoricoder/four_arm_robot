## 概述

`service_manager` 是一个 ROS 2 节点（`seperated_service` 包），管理独立于 ROS 2 的外部服务进程（如 TTS 推理后端）的完整生命周期。

**核心能力**：

- **启动时清理**：检查端口/关键字，kill 残留进程后重新启动
- **定期健康检查**：配置间隔轮询各进程状态，死掉自动重启
- **日志记录**：子进程 stdout/stderr 写入 `~/.ros/seperated_service/<name>.log`

## 配置

服务定义以 JSON 方式配置，key 为 `seperated_service`，每个条目包含：

| 字段 | 说明 |
| --- | --- |
| `name` | 服务名，用于日志标识 |
| `conda_env` | 激活的 conda 环境 |
| `work_dir` | 子进程工作目录 |
| `cmd` | 可执行文件 (`python`) |
| `args` | 命令行参数列表 |
| `check_port` | 用于检测残留进程的端口号 |
| `check_keyword` | pgrep 关键字，辅助 kill 残留进程 |

示例（moss-tts-nano，写入 `tts_config.json`）：

```json
{
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
}
```

## 启动

在 launch XML 中作为最顶层节点启动（确保先于其他依赖该服务的节点）：

```xml
<node pkg="seperated_service" exec="service_manager" name="service_manager" output="screen">
    <param name="config_file" value="/home/dcx/four_arm_robot/extend/moss_tts_nano/tts_config.json" />
    <param name="check_interval" value="10.0" />
    <param name="startup_delay" value="2.0" />
</node>
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `config_file` | 无（走硬编码后备） | JSON 配置文件路径 |
| `check_interval` | `10.0` | 健康检查间隔（秒） |
| `startup_delay` | `2.0` | 首次启动前等待（秒） |

## 运行机制

1. 节点初始化：加载配置，创建定时器
2. `startup_delay` 后触发一次性启动：对每个服务，先通过 `lsof` / `pgrep` kill 残留进程（SIGTERM → 仍存活则 SIGKILL），再通过 `bash -c "source conda.sh && conda activate <env> && exec <cmd>"` 启动
3. 每隔 `check_interval` 健康检查：`proc.poll()` 非 `None` 则重新执行步骤 2
4. 节点 shutdown 时 `terminate()` 所有子进程，关闭日志文件

## 日志

每个服务的输出写入 `~/.ros/seperated_service/<name>.log`（追加模式，带时间戳分隔线）。日志文件输出不会被 pipe 缓冲区阻塞，确保子进程能正常完成初始化。
