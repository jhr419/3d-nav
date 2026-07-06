import datetime as _dt
import math
import os
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Type

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _now_stamp() -> str:
    return _dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def _line_stamp() -> str:
    return _dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def _find_workspace_root() -> Path:
    env_value = os.environ.get("NAV3D_WS", "")
    if env_value:
        candidate = Path(env_value).expanduser()
        if candidate.joinpath("src").is_dir() and candidate.joinpath("maps").is_dir():
            return candidate

    for candidate in [Path.cwd(), *Path.cwd().parents]:
        if candidate.joinpath("src").is_dir() and candidate.joinpath("maps").is_dir():
            return candidate
    return Path.cwd()


def _resolve_project_path(path: str) -> Path:
    raw = str(path or "").strip()
    if not raw:
        return _find_workspace_root()
    candidate = Path(raw).expanduser()
    if candidate.is_absolute():
        return candidate
    return (_find_workspace_root() / candidate).resolve()


def _status_rank(status: str) -> int:
    return {"OK": 0, "WARN": 1, "ERROR": 2}.get(status, 2)


def _worse(left: str, right: str) -> str:
    return left if _status_rank(left) >= _status_rank(right) else right


class FrequencyTracker:
    def __init__(self, window_sec: float) -> None:
        self.window_sec = max(1.0, float(window_sec))
        self.stamps: deque[float] = deque()
        self.last_age_sec: Optional[float] = None

    def record(self, msg: Any, node: Node) -> None:
        now_mono = time.monotonic()
        self.stamps.append(now_mono)
        self._trim(now_mono)
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0):
            msg_time = rclpy.time.Time.from_msg(stamp)
            self.last_age_sec = max(0.0, (node.get_clock().now() - msg_time).nanoseconds / 1.0e9)

    def _trim(self, now_mono: Optional[float] = None) -> None:
        now_mono = time.monotonic() if now_mono is None else now_mono
        while self.stamps and now_mono - self.stamps[0] > self.window_sec:
            self.stamps.popleft()

    def hz(self) -> float:
        self._trim()
        if len(self.stamps) < 2:
            return 0.0
        span = max(1.0e-6, self.stamps[-1] - self.stamps[0])
        return float(len(self.stamps) - 1) / span

    def seen(self) -> bool:
        self._trim()
        return bool(self.stamps)


class DiagnosticsLogger:
    def __init__(self, log_dir: str, prefix: str) -> None:
        directory = _resolve_project_path(log_dir)
        directory.mkdir(parents=True, exist_ok=True)
        safe_prefix = "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in prefix) or "nav3d_diagnostics"
        self.path = directory / f"{safe_prefix}_{_now_stamp()}.log"
        self.stream = self.path.open("a", encoding="utf-8")

    def write(self, level: str, event: str, message: str) -> None:
        line = f"[{_line_stamp()}] [{level}] [{event}] {message}"
        self.stream.write(line + "\n")
        if level == "ERROR":
            self.stream.flush()

    def info(self, event: str, message: str) -> None:
        self.write("INFO", event, message)

    def warn(self, event: str, message: str) -> None:
        self.write("WARN", event, message)

    def error(self, event: str, message: str) -> None:
        self.write("ERROR", event, message)

    def flush(self) -> None:
        self.stream.flush()

    def close(self) -> None:
        self.flush()
        self.stream.close()


class Nav3DSystemDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("nav3d_system_diagnostics_node")
        self.declare_parameter("config_file", "")

        self.config_file = str(self.get_parameter("config_file").value)
        self.config = self._load_config(self.config_file)
        self.params = self.config.get("nav3d_system_diagnostics_node", {}).get("ros__parameters", {})

        self.diagnostics_enabled = bool(self.params.get("diagnostics_enabled", True))
        self.check_duration_sec = float(self.params.get("check_duration_sec", 5.0))
        self.startup_check_delay_sec = float(self.params.get("startup_check_delay_sec", 2.0))
        self.monitor_frequency = max(0.2, float(self.params.get("monitor_frequency", 1.0)))
        self.warn_lidar_hz_below = float(self.params.get("warn_lidar_hz_below", 8.0))
        self.error_lidar_hz_below = float(self.params.get("error_lidar_hz_below", 5.0))

        self.logger = DiagnosticsLogger(
            str(self.params.get("log_dir", "debug/logs/diagnostics")),
            str(self.params.get("log_file_prefix", "nav3d_diagnostics")),
        )
        self.logger.info("STARTUP", f"diagnostics started config_file={self.config_file}")

        self.status_pub = self.create_publisher(String, "/nav3d/diagnostics/status", self._latched_qos())
        self.report_pub = self.create_publisher(String, "/nav3d/diagnostics/report", self._latched_qos())

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trackers: Dict[str, FrequencyTracker] = {}
        self.monitor_specs = self._collect_monitor_specs()
        self.monitor_subscriptions = []
        self._create_topic_monitors()

        self.startup_check_done = False
        self.monitor_timer = self.create_timer(1.0 / self.monitor_frequency, self._monitor_callback)
        self.startup_timer = self.create_timer(
            self.startup_check_delay_sec + self.check_duration_sec,
            self._startup_check_callback,
        )

        self.get_logger().info(
            f"nav3d system diagnostics ready. log={self.logger.path} monitors={len(self.monitor_specs)}"
        )

    def destroy_node(self) -> bool:
        self.logger.close()
        return super().destroy_node()

    def _load_config(self, config_file: str) -> Dict[str, Any]:
        if not config_file:
            return {}
        path = Path(config_file).expanduser()
        if not path.is_absolute():
            path = _resolve_project_path(config_file)
        if not path.is_file():
            self.get_logger().warning(f"diagnostics config file not found: {path}")
            return {}
        with path.open("r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream) or {}
        return loaded if isinstance(loaded, dict) else {}

    @staticmethod
    def _latched_qos() -> QoSProfile:
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def _collect_monitor_specs(self) -> Dict[str, Dict[str, Any]]:
        specs: Dict[str, Dict[str, Any]] = {}

        def add(group: str, name: str, data: Dict[str, Any], msg_type: Type[Any]) -> None:
            topic = str(data.get("topic", "")).strip()
            if not topic:
                return
            specs[name] = {
                "group": group,
                "name": name,
                "topic": topic,
                "min_ok_hz": float(data.get("min_ok_hz", 0.0)),
                "min_required_hz": float(data.get("min_required_hz", 0.0)),
                "expected_hz": float(data.get("expected_hz", 0.0)),
                "msg_type": msg_type,
            }

        for name, data in dict(self.params.get("sensor_topics", {})).items():
            add("sensor", str(name), dict(data or {}), PointCloud2)
        for name, data in dict(self.params.get("perception_topics", {})).items():
            add("perception", str(name), dict(data or {}), PointCloud2)
        for name, data in dict(self.params.get("planner_topics", {})).items():
            msg_type: Type[Any] = NavPath if "path" in str(name).lower() else Marker
            add("planner", str(name), dict(data or {}), msg_type)
        for name, data in dict(self.params.get("control_topics", {})).items():
            add("control", str(name), dict(data or {}), Twist)

        required_topics = [str(topic) for topic in self.params.get("required_topics", [])]
        for topic in required_topics:
            if topic not in [spec["topic"] for spec in specs.values()]:
                msg_type = String if "state" in topic else Twist if "cmd_vel" in topic else NavPath
                specs[f"required:{topic}"] = {
                    "group": "required",
                    "name": f"required:{topic}",
                    "topic": topic,
                    "min_ok_hz": 0.0,
                    "min_required_hz": 0.0,
                    "expected_hz": 0.0,
                    "msg_type": msg_type,
                }
        return specs

    def _create_topic_monitors(self) -> None:
        for name, spec in self.monitor_specs.items():
            tracker = FrequencyTracker(window_sec=self.check_duration_sec)
            self.trackers[name] = tracker
            msg_type = spec["msg_type"]
            qos = QoSProfile(depth=10)
            if msg_type is PointCloud2:
                qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            if msg_type in (NavPath, String):
                qos = QoSProfile(
                    depth=10,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                )
            callback = self._make_topic_callback(name)
            self.monitor_subscriptions.append(self.create_subscription(msg_type, spec["topic"], callback, qos))

    def _make_topic_callback(self, name: str):
        def callback(msg: Any) -> None:
            self.trackers[name].record(msg, self)

        return callback

    def _topic_names(self) -> Dict[str, List[str]]:
        return {name: types for name, types in self.get_topic_names_and_types()}

    def _topic_exists(self, topic: str) -> bool:
        return topic in self._topic_names()

    def _topic_status(self, name: str, spec: Dict[str, Any]) -> Tuple[str, str, float]:
        topic = spec["topic"]
        exists = self._topic_exists(topic)
        tracker = self.trackers[name]
        hz = tracker.hz()
        if not exists:
            return "ERROR", "topic_missing", hz
        if spec["min_required_hz"] > 0.0 and hz < spec["min_required_hz"]:
            return "ERROR", "frequency_below_required", hz
        if spec["min_ok_hz"] > 0.0 and hz < spec["min_ok_hz"]:
            return "WARN", "frequency_below_ok", hz
        return "OK", "ok", hz

    def _tf_status(self) -> Tuple[str, List[str]]:
        details: List[str] = []
        overall = "OK"
        for item in self.params.get("required_tfs", []):
            parent = str(item.get("parent", "")).strip()
            child = str(item.get("child", "")).strip()
            if not parent or not child:
                continue
            try:
                ok = self.tf_buffer.can_transform(parent, child, Time(), timeout=Duration(seconds=0.05))
            except Exception as exc:
                ok = False
                details.append(f"{parent}->{child}=ERROR({exc})")
            else:
                details.append(f"{parent}->{child}={'OK' if ok else 'ERROR'}")
            if not ok:
                overall = "ERROR"
        return overall, details

    def _build_report(self) -> Tuple[str, str]:
        overall = "OK"
        fields: List[str] = []
        possible_causes: List[str] = []

        for name, spec in self.monitor_specs.items():
            status, reason, hz = self._topic_status(name, spec)
            overall = _worse(overall, status)
            key = name.split(":", 1)[-1].replace("/", "_").strip("_") or name
            fields.append(f"{key}_hz={hz:.3f}")
            age = self.trackers[name].last_age_sec
            if age is not None:
                fields.append(f"{key}_age_sec={age:.3f}")
            fields.append(f"{key}_status={status}")
            if status != "OK":
                fields.append(f"{key}_reason={reason}")
                if reason == "topic_missing":
                    possible_causes.append(f"{spec['topic']} publisher not started or remap mismatch")
                elif reason == "frequency_below_required":
                    possible_causes.append(f"{spec['topic']} publisher too slow or blocked")

        lidar_spec = self.monitor_specs.get("lidar")
        if lidar_spec is not None:
            lidar_hz = self.trackers["lidar"].hz()
            if lidar_hz < self.error_lidar_hz_below:
                overall = "ERROR"
                fields.append("lidar_judgement=ERROR")
                possible_causes.append("lidar frequency below 5Hz: driver, network, CPU load, or QoS mismatch")
            elif lidar_hz < self.warn_lidar_hz_below:
                overall = _worse(overall, "WARN")
                fields.append("lidar_judgement=WARN")
                possible_causes.append("lidar frequency below 8Hz: sensor pipeline may be overloaded")
            else:
                fields.append("lidar_judgement=OK")

        tf_overall, tf_details = self._tf_status()
        overall = _worse(overall, tf_overall)
        fields.append("tf_status=" + ",".join(tf_details))
        fields.append(f"overall_status={overall}")
        if possible_causes:
            fields.append("possible_causes=" + "; ".join(possible_causes))
        return overall, " ".join(fields)

    def _monitor_callback(self) -> None:
        if not self.diagnostics_enabled:
            return
        overall, report = self._build_report()
        status_msg = String()
        status_msg.data = overall
        report_msg = String()
        report_msg.data = report
        self.status_pub.publish(status_msg)
        self.report_pub.publish(report_msg)

    def _startup_check_callback(self) -> None:
        if self.startup_check_done:
            return
        self.startup_check_done = True
        self.startup_timer.cancel()
        if not self.diagnostics_enabled:
            self.logger.warn("STARTUP_CHECK", "diagnostics disabled")
            self.logger.flush()
            return

        self.logger.info("STARTUP_CHECK", "self-check started")
        self.logger.info("CHECK_TOPICS", "required_topics=" + ",".join(self.params.get("required_topics", [])))

        topic_map = self._topic_names()
        for topic in self.params.get("required_topics", []):
            exists = topic in topic_map
            level = "INFO" if exists else "ERROR"
            self.logger.write(level, "TOPIC_EXISTENCE", f"topic={topic} exists={exists}")

        for name, spec in self.monitor_specs.items():
            status, reason, hz = self._topic_status(name, spec)
            level = "INFO" if status == "OK" else status
            age = self.trackers[name].last_age_sec
            self.logger.write(
                level,
                "TOPIC_FREQUENCY",
                f"name={name} topic={spec['topic']} hz={hz:.3f} status={status} reason={reason} age_sec={age if age is not None else -1:.3f}",
            )

        lidar = self.monitor_specs.get("lidar")
        if lidar is not None:
            lidar_hz = self.trackers["lidar"].hz()
            if lidar_hz >= self.warn_lidar_hz_below:
                self.logger.info("LIDAR_HZ", f"lidar_hz={lidar_hz:.3f} judgement=OK")
            elif lidar_hz >= self.error_lidar_hz_below:
                self.logger.warn("LIDAR_HZ", f"lidar_hz={lidar_hz:.3f} judgement=WARN")
            else:
                self.logger.error("LIDAR_HZ", f"lidar_hz={lidar_hz:.3f} judgement=ERROR")

        tf_overall, tf_details = self._tf_status()
        self.logger.write("INFO" if tf_overall == "OK" else "ERROR", "TF_CHECK", " ".join(tf_details))

        overall, report = self._build_report()
        self.logger.write("INFO" if overall == "OK" else overall, "OVERALL", report)
        if overall == "ERROR":
            self.logger.error(
                "POSSIBLE_CAUSES",
                "topic missing, lidar below 5Hz, TF disconnected, control gate not started, or local planner blocked",
            )
        self.logger.info("STARTUP_CHECK", "self-check finished")
        self.logger.flush()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav3DSystemDiagnostics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
