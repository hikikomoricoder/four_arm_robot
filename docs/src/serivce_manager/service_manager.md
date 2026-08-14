# service_manager

`seperated_service` 包节点（`service_manager.py`），两大职责：

1. **外部服务进程管理**：按 JSON 配置启动 / 健康检查 / 自动重启独立于 ROS2 的服务进程（当前为 MOSS-TTS-Nano，端口 18083）。详见 [extend 文档](../../extend/seperated_service/service_manager.md)
2. **系统 TTS 网关**：订阅 `/tts/say`，经 MOSS-TTS-Nano HTTP 接口合成语音并播放

## TTS 网关

- **接口**：订阅 `/tts/say`（`std_msgs/String`），fire-and-forget，任何节点发布文本即播报
- **队列**：`queue.Queue(maxsize=1)`，latest-wins —— 队列满时丢弃未播报的旧文本，过期播报不积压
- **worker 线程**：订阅回调只入队；秒级阻塞的 `synthesize_and_play()` 在独立 daemon 线程执行，不阻塞 executor 的健康检查定时器
- **client 定位**：`tts_request.py` 与 `config_file` 参数指向的 `tts_config.json` 同目录（`extend/moss_tts_nano/`），据此推导 `sys.path` 后导入 `TTSClient`；导入或加载失败仅禁用网关并记 error，不影响进程管理职责
- **停止**：`destroy_node` 置 stop event，worker 最多 0.5s 内退出

## 场景播报链路

```
display_four_camera            （缓存最新 desc：_latest_desc）
        ↑ Trigger 服务 get_azimuth_description（面板点击时拉取）
robot_panel                    [panorama_info_broadcast 按钮]
        ↓ /tts/say（std_msgs/String）
service_manager                （latest-wins 队列 → worker 线程）
        ↓ HTTP
MOSS-TTS-Nano → aplay 播放
```

- `display_four_camera` 每次检测产出非空方位描述时更新 `_latest_desc`，只存最新一条；`get_azimuth_description` 无描述时返回 `success=False`
- 面板按钮点击 → `call_async` 拉取 desc（不阻塞 UI）→ done 回调发布 `/tts/say`

## 注意

- 托管服务列表属性名为 `_svc_defs`，不可命名 `_services`（会遮蔽 rclpy `Node._services` 导致 AttributeError）
- TTS 服务启动较慢（`startup_delay` + 模型加载），就绪前的播报请求会在 worker 中失败并记 warn，健康检查保证服务后续可用
