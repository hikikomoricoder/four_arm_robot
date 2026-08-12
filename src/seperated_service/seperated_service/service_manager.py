"""External service manager for ROS 2.

Manages long-running processes that are independent of ROS 2 (e.g. TTS servers,
inference backends). On startup, kills any stale process on the configured port
and spawns a fresh instance. Periodically health-checks each service and
auto-restarts on failure.

The service definition is read from a JSON config file. The JSON must contain a
top-level ``"seperated_service"`` key whose value is an array of service objects,
each with: name, conda_env, work_dir, cmd, args, check_port, check_keyword.
"""

import json
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.node import Node


@dataclass
class ServiceDef:
    """Descriptor for a single managed external service."""

    name: str
    conda_env: str
    work_dir: str
    cmd: str
    args: List[str] = field(default_factory=list)
    check_port: Optional[int] = None
    check_keyword: Optional[str] = None
    process: Optional[subprocess.Popen] = None
    log_file: Optional[object] = None


class ServiceManager(Node):
    """ROS 2 node that manages the lifecycle of external services."""

    def __init__(self):
        super().__init__("service_manager")

        # --- parameters ---
        self.declare_parameter(
            "config_file",
            value="",
        )
        self.declare_parameter("check_interval", value=10.0)
        self.declare_parameter("startup_delay", value=2.0)

        cfg_path = self.get_parameter("config_file").value
        check_interval = self.get_parameter("check_interval").value
        startup_delay = self.get_parameter("startup_delay").value

        # --- load service definitions ---
        services: List[ServiceDef] = []
        if cfg_path and os.path.isfile(cfg_path):
            services = self._load_config(cfg_path)
        else:
            services = self._default_config()

        self._svc_defs = services
        self._lock = threading.Lock()

        self.get_logger().info(
            f"Loaded {len(services)} service(s) — check every {check_interval}s"
        )

        # --- start all services after a short delay (allow ROS 2 init to settle) ---
        self._startup_timer = self.create_timer(startup_delay, self._on_startup)

        # --- periodic health check ---
        self.create_timer(check_interval, self._health_check)

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------

    def _load_config(self, path: str) -> List[ServiceDef]:
        with open(path, "r") as f:
            data = json.load(f)
        services = []
        for entry in data.get("seperated_service", []):
            services.append(
                ServiceDef(
                    name=entry["name"],
                    conda_env=entry["conda_env"],
                    work_dir=entry.get("work_dir", ""),
                    cmd=entry["cmd"],
                    args=entry.get("args", []),
                    check_port=entry.get("check_port"),
                    check_keyword=entry.get("check_keyword"),
                )
            )
        return services

    def _default_config(self) -> List[ServiceDef]:
        return [
            ServiceDef(
                name="moss-tts-nano",
                conda_env="moss-tts-nano",
                work_dir="/home/dcx/MOSS-TTS-Nano",
                cmd="python",
                args=[
                    "app_onnx.py",
                    "--host",
                    "0.0.0.0",
                    "--port",
                    "18083",
                    "--execution-provider",
                    "cuda",
                ],
                check_port=18083,
                check_keyword="app_onnx.py",
            ),
        ]

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------

    def _on_startup(self):
        # oneshot: cancel the timer so it only fires once
        self._startup_timer.cancel()
        for svc in self._svc_defs:
            self._start_service(svc)

    # ------------------------------------------------------------------
    # Port / process helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _pids_on_port(port: int) -> List[int]:
        """Return PIDs of processes listening on *port* via ``lsof``."""
        try:
            res = subprocess.run(
                ["lsof", "-ti", f":{port}"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if res.returncode == 0 and res.stdout.strip():
                return [int(p) for p in res.stdout.strip().split()]
        except (subprocess.TimeoutExpired, FileNotFoundError, ValueError):
            pass
        return []

    @staticmethod
    def _kill_pids(pids: List[int], sig=signal.SIGTERM):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass

    @staticmethod
    def _find_pids_by_keyword(keyword: str) -> List[int]:
        """Return PIDs whose command line contains *keyword* (via ``pgrep``)."""
        try:
            res = subprocess.run(
                ["pgrep", "-f", keyword],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if res.returncode == 0 and res.stdout.strip():
                return [int(p) for p in res.stdout.strip().split()]
        except (subprocess.TimeoutExpired, FileNotFoundError, ValueError):
            pass
        return []

    # ------------------------------------------------------------------
    # Service lifecycle
    # ------------------------------------------------------------------

    def _start_service(self, svc: ServiceDef):
        with self._lock:
            self.get_logger().info(f"[{svc.name}] Starting ...")

            # 1. kill stale processes on the port
            if svc.check_port is not None:
                stale = self._pids_on_port(svc.check_port)
                if stale:
                    self.get_logger().warn(
                        f"[{svc.name}] Port {svc.check_port} busy by PID(s) "
                        f"{stale} — killing ..."
                    )
                    self._kill_pids(stale)
                    time.sleep(0.5)
                    # still alive → SIGKILL
                    still_alive = self._pids_on_port(svc.check_port)
                    if still_alive:
                        self._kill_pids(still_alive, signal.SIGKILL)

            # 2. also kill any leftover process matching the keyword
            if svc.check_keyword:
                leftovers = self._find_pids_by_keyword(svc.check_keyword)
                if leftovers:
                    self.get_logger().warn(
                        f"[{svc.name}] Stale process(es) {leftovers} "
                        f"matching '{svc.check_keyword}' — killing ..."
                    )
                    self._kill_pids(leftovers)
                    time.sleep(0.5)

            # 3. build conda-run command
            conda_sh = os.path.expanduser("~/miniconda3/etc/profile.d/conda.sh")
            shell_cmd = (
                f'source "{conda_sh}" && '
                f"conda activate {svc.conda_env} && "
                f"exec {svc.cmd} {' '.join(svc.args)}"
            )

            # 4. child stdout/stderr go to a log file, NOT a pipe. A pipe can
            #    fill up and block the child if it emits a burst of output,
            #    leaving the service alive but hung before it binds its port.
            log_path = self._service_log_path(svc.name)
            log_file = open(log_path, "a")
            log_file.write(f"\n===== start {time.strftime('%Y-%m-%d %H:%M:%S')} "
                           f"(cwd={svc.work_dir}) =====\n")
            log_file.flush()

            self.get_logger().info(
                f"[{svc.name}] Launching: bash -c \"{shell_cmd}\"  "
                f"(log: {log_path})"
            )

            try:
                proc = subprocess.Popen(
                    ["bash", "-c", shell_cmd],
                    cwd=svc.work_dir or None,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )
            except Exception as e:
                log_file.close()
                self.get_logger().error(f"[{svc.name}] Failed to start: {e}")
                return

            svc.process = proc
            svc.log_file = log_file
            self.get_logger().info(f"[{svc.name}] Started (PID {proc.pid})")

    @staticmethod
    def _service_log_path(name: str) -> str:
        """Path of the log file capturing a managed service's stdout."""
        log_dir = os.path.join(
            os.path.expanduser("~"), ".ros", "seperated_service"
        )
        os.makedirs(log_dir, exist_ok=True)
        return os.path.join(log_dir, f"{name}.log")

    def _health_check(self):
        for svc in self._svc_defs:
            with self._lock:
                alive = (
                    svc.process is not None
                    and svc.process.poll() is None
                )
            if not alive:
                self.get_logger().warn(
                    f"[{svc.name}] Process not running — restarting ..."
                )
                self._start_service(svc)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info("Shutting down — terminating managed services ...")
        for svc in self._svc_defs:
            proc = svc.process
            if proc is not None and proc.poll() is None:
                self.get_logger().info(f"  Terminating {svc.name} (PID {proc.pid})")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=3)
            if svc.log_file is not None:
                try:
                    svc.log_file.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServiceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()