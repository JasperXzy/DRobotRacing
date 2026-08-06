#!/usr/bin/env python3
"""低开销竞赛运行记录器：结束后输出资源、节点与话题频率图。"""

import argparse
import csv
import datetime as dt
import math
import signal
import sys
import threading
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import psutil
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from tf2_msgs.msg import TFMessage


TOPICS = {
    "/scan": LaserScan,
    "/chassis/odom": Odometry,
    "/cmd_vel": Twist,
    "/tf": TFMessage,
    "/qr_code": String,
    "/sign4return": Int32,
}
SENSOR_QOS_TOPICS = {"/scan", "/chassis/odom", "/tf"}
SENSOR_MONITOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

PROCESS_GROUPS = {
    # 监控器自身必须单独统计，便于确认25Hz采样没有反过来造成性能瓶颈。
    "competition_monitor": ("competition_run_monitor.py",),
    "path_follower": ("visual_lidar_path_follower",),
    "lidar_location": ("lidar_location_node",),
    "lidar_driver": ("lidar_driver_node",),
    "chassis_driver": ("chassis_driver_node",),
    "perception": ("perception_detector_node", "line_detector"),
    "qr_detector": ("qr_detector_node",),
    "vlm_tts": ("vlm_tts_demo",),
}


class CompetitionRunMonitor(Node):
    def __init__(self, workspace: Path, interval: float) -> None:
        super().__init__("competition_run_monitor")
        self.workspace = workspace
        self.path_root = workspace / "src/origincar_system/path"
        self.started_wall = time.time()
        self.started_monotonic = time.monotonic()
        self.interval = max(0.04, interval)
        self.topic_counts = {topic: 0 for topic in TOPICS}
        self.previous_counts = dict(self.topic_counts)
        self.topic_count_lock = threading.Lock()
        self.rows = []
        self.cpu_count = max(1, psutil.cpu_count(logical=True) or 1)
        self.monitored_processes = {name: [] for name in PROCESS_GROUPS}
        self.last_process_discovery = 0.0
        self.previous_sample_monotonic = self.started_monotonic

        for topic, message_type in TOPICS.items():
            self.create_subscription(
                message_type, topic, lambda _msg, name=topic: self._count_topic(name),
                SENSOR_MONITOR_QOS if topic in SENSOR_QOS_TOPICS else 10)
        psutil.cpu_percent(None)
        self._refresh_process_cache()
        self.create_timer(self.interval, self.sample_once)
        self.get_logger().info(
            "竞赛监控已开始：每%.3fs采样系统、节点资源和%d个轻量话题。"
            % (self.interval, len(TOPICS)))

    def _count_topic(self, topic: str) -> None:
        with self.topic_count_lock:
            self.topic_counts[topic] += 1

    @staticmethod
    def _cpu_temperature() -> float:
        try:
            readings = psutil.sensors_temperatures(fahrenheit=False)
        except (AttributeError, OSError):
            return math.nan
        values = []
        for entries in readings.values():
            for entry in entries:
                if entry.current is not None and math.isfinite(entry.current):
                    values.append(float(entry.current))
        return max(values) if values else math.nan

    def _refresh_process_cache(self) -> None:
        """进程发现只做1Hz；25Hz采样阶段只读取已缓存的目标PID。"""
        discovered = {name: [] for name in PROCESS_GROUPS}
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                info = proc.info
                command = " ".join(info.get("cmdline") or [])
                if not command:
                    command = info.get("name") or ""
                matched = [
                    group for group, keywords in PROCESS_GROUPS.items()
                    if any(keyword in command for keyword in keywords)
                ]
                if not matched:
                    continue
                for group in matched:
                    discovered[group].append(proc)
                # 建立CPU增量的基线；后续25Hz采样不再遍历系统全部进程。
                proc.cpu_percent(None)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        self.monitored_processes = discovered
        self.last_process_discovery = time.monotonic()

    def _process_metrics(self):
        if time.monotonic() - self.last_process_discovery >= 1.0:
            self._refresh_process_cache()
        result = {
            name: {"cpu_percent": 0.0, "rss_mb": 0.0, "processes": 0}
            for name in PROCESS_GROUPS
        }
        for group, processes in self.monitored_processes.items():
            for proc in processes:
                try:
                    result[group]["cpu_percent"] += proc.cpu_percent(None) / self.cpu_count
                    result[group]["rss_mb"] += proc.memory_info().rss / (1024.0 * 1024.0)
                    result[group]["processes"] += 1
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    continue
        return result

    def sample_once(self) -> None:
        now_monotonic = time.monotonic()
        elapsed = max(1.0e-6, now_monotonic - self.previous_sample_monotonic)
        process_metrics = self._process_metrics()
        row = {
            "elapsed_s": now_monotonic - self.started_monotonic,
            "system_cpu_percent": psutil.cpu_percent(None),
            "memory_percent": psutil.virtual_memory().percent,
            "cpu_temperature_c": self._cpu_temperature(),
        }
        for group, metrics in process_metrics.items():
            row[f"{group}_cpu_percent"] = metrics["cpu_percent"]
            row[f"{group}_rss_mb"] = metrics["rss_mb"]
            row[f"{group}_processes"] = metrics["processes"]
        with self.topic_count_lock:
            topic_counts = dict(self.topic_counts)
        for topic, count in topic_counts.items():
            row[f"hz:{topic}"] = (count - self.previous_counts[topic]) / elapsed
        self.previous_counts = topic_counts
        self.previous_sample_monotonic = now_monotonic
        self.rows.append(row)

    def _output_directory(self) -> Path:
        candidates = [
            path for path in self.path_root.iterdir()
            if path.is_dir() and path.stat().st_mtime >= self.started_wall - 3.0
        ] if self.path_root.exists() else []
        if candidates:
            return max(candidates, key=lambda path: path.stat().st_mtime)
        fallback = self.path_root / ("monitor_" + dt.datetime.now().strftime("%Y.%m.%d_%H:%M:%S"))
        fallback.mkdir(parents=True, exist_ok=True)
        return fallback

    def _write_csv(self, output_dir: Path) -> Path:
        path = output_dir / "05_Runtime_monitor.csv"
        if not self.rows:
            path.write_text("elapsed_s\n", encoding="utf-8")
            return path
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        return path

    def _plot(self, output_dir: Path) -> Path:
        path = output_dir / "05_Runtime_monitor.png"
        if not self.rows:
            return path
        # CSV 始终保留完整25Hz原始数据；长时间比赛仅对PNG下采样，避免赛后
        # 绘图在上位机上占用过久。
        stride = max(1, math.ceil(len(self.rows) / 5000))
        plot_rows = self.rows[::stride]
        elapsed = [row["elapsed_s"] for row in plot_rows]
        figure, axes = plt.subplots(
            4, 1, figsize=(16, 14), sharex=True,
            gridspec_kw={"height_ratios": [1.0, 2.4, 1.0, 1.0]})
        figure.suptitle(
            "Competition runtime monitor — %.0f s sampled at %.1f Hz"
            % (elapsed[-1], 1.0 / self.interval), fontsize=14)

        axes[0].plot(elapsed, [row["system_cpu_percent"] for row in plot_rows], label="System CPU %")
        axes[0].plot(elapsed, [row["memory_percent"] for row in plot_rows], label="Memory %")
        temperature = [row["cpu_temperature_c"] for row in plot_rows]
        if any(math.isfinite(value) for value in temperature):
            axes[0].plot(elapsed, temperature, label="CPU temperature C")
        axes[0].set_ylabel("System")
        axes[0].set_ylim(bottom=0)
        axes[0].grid(alpha=0.3)
        axes[0].legend(ncol=3, loc="upper right")

        for group in PROCESS_GROUPS:
            axes[1].plot(elapsed, [row[f"{group}_cpu_percent"] for row in plot_rows], label=group)
        axes[1].set_title("Key node CPU usage (primary competition metric)")
        axes[1].set_ylabel("Node CPU %\n(all cores)")
        axes[1].set_ylim(bottom=0)
        axes[1].grid(alpha=0.3)
        axes[1].legend(ncol=4, fontsize=8, loc="upper right")

        for group in PROCESS_GROUPS:
            axes[2].plot(elapsed, [row[f"{group}_rss_mb"] for row in plot_rows], label=group)
        axes[2].set_ylabel("Node RSS MB")
        axes[2].set_ylim(bottom=0)
        axes[2].grid(alpha=0.3)
        axes[2].legend(ncol=4, fontsize=8, loc="upper right")

        for topic in TOPICS:
            axes[3].plot(elapsed, [row[f"hz:{topic}"] for row in plot_rows], label=topic)
        axes[3].set_ylabel("Topic Hz")
        axes[3].set_xlabel("Elapsed time (s)")
        axes[3].set_ylim(bottom=0)
        axes[3].grid(alpha=0.3)
        axes[3].legend(ncol=3, fontsize=8, loc="upper right")
        figure.tight_layout()
        figure.savefig(path, dpi=140)
        plt.close(figure)
        return path

    def finalize(self) -> None:
        self.sample_once()
        output_dir = self._output_directory()
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self._write_csv(output_dir)
        png_path = self._plot(output_dir)
        summary_path = output_dir / "05_Runtime_monitor_summary.txt"
        summary_lines = [
            "samples=%d" % len(self.rows),
            "duration_s=%.3f" % self.rows[-1]["elapsed_s"],
            "sample_interval_s=%.3f" % self.interval,
            "plot=%s" % png_path.name,
            "csv=%s" % csv_path.name,
            "",
            "node_cpu_percent_all_cores: average,peak; peak_rss_mb",
        ]
        for group in PROCESS_GROUPS:
            cpu_values = [row[f"{group}_cpu_percent"] for row in self.rows]
            rss_values = [row[f"{group}_rss_mb"] for row in self.rows]
            summary_lines.append(
                "%s=%.3f,%.3f; %.3f" % (
                    group, sum(cpu_values) / len(cpu_values), max(cpu_values), max(rss_values)))
        summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
        self.get_logger().info("竞赛监控报告已生成：%s" % png_path)


class HistoricalReport:
    """复用运行期绘图方法，为已写入的CSV补画报告。"""

    def __init__(self, rows, interval: float) -> None:
        self.rows = rows
        self.interval = interval

    _plot = CompetitionRunMonitor._plot


def render_existing_csv(csv_path: Path) -> int:
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        rows = []
        for row in csv.DictReader(handle):
            rows.append({key: float(value) if value else math.nan for key, value in row.items()})
    if not rows:
        raise RuntimeError("CSV中没有可绘制的样本。")
    deltas = [
        rows[index]["elapsed_s"] - rows[index - 1]["elapsed_s"]
        for index in range(1, len(rows))
    ]
    interval = sorted(deltas)[len(deltas) // 2] if deltas else 0.04
    report = HistoricalReport(rows, max(0.04, interval))
    png_path = report._plot(csv_path.parent)
    print("Rendered runtime monitor PNG: %s" % png_path)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Record competition runtime metrics and render a post-run PNG.")
    parser.add_argument("--workspace", type=Path)
    parser.add_argument("--interval", type=float, default=0.04)
    parser.add_argument("--render-csv", type=Path)
    args = parser.parse_args()
    if args.render_csv is not None:
        return render_existing_csv(args.render_csv.resolve())
    if args.workspace is None:
        parser.error("运行监控时必须提供 --workspace。")
    rclpy.init()
    monitor = CompetitionRunMonitor(args.workspace.resolve(), args.interval)
    finalizing = False

    def stop(_signum, _frame):
        if finalizing:
            return
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(monitor)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        finalizing = True
        # 脚本退出或用户连续按Ctrl+C时，第二个信号不应中断已经开始的PNG写入。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            monitor.finalize()
        finally:
            executor.shutdown()
            monitor.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
