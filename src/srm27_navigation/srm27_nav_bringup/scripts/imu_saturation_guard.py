#!/usr/bin/env python3
# Copyright 2026 SRM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""IMU 加速度饱和保护：给 ZLIO 前端补上 Point-LIO 早就有的那一层。

为什么需要它
============
Point-LIO 的配置里有 `check_satu: True` + `satu_acc: 30.0`（nav2_params_srm_fakeyaw.yaml:90/141），
也就是"加速度模长超过 30 m/s² 就不信这一帧"。**ZLIO 前端运行时没有任何等价的检查**
（`src/frontend` 全文搜 `satur`/`clip` 为空），它唯一的门是"一个面都没匹配到"
（`filter.cpp:112`）。

后果在撞墙这种场景上是实打实的：Gazebo 里 0.2 m/s 撞停在 1~2 个物理步内完成，
等效加速度比 3 g 高一个量级，这一脚冲量被原样积分成几十 m/s 的速度，而激光匹配在
"正对一面大墙"时是退化的（法向可观、切向不可观），救不回来 —— 表现就是
**航向还贴着真值、位置飞出去几千米**（2026-11-05 实跑，见
docs/仿真LIO前端替换Z-LIO实施记录(ai).md §5.4）。

本节点做的事只有一件：把 `|linear_acceleration|` 限到 `max_acceleration`（默认 30 m/s²，
与 Point-LIO 的 `satu_acc` 对齐），**方向不变、只缩模长**，并把被削的次数打出来。

不是它在管的事
==============
* 不做低通/去噪（ZLIO 的噪声模型假定输入就是原始 IMU）；
* 不动角速度（Point-LIO 的饱和参数也只管加速度）；
* 不做"撞击检测后重置定位"——那属于上层决策，本节点只保证"滤波器看不到物理上不可能的加速度"。

为什么是"削"而不是"丢"
======================
丢一帧会在 IMU 序列里制造一个空洞，而 ZLIO 的预积分要求相邻样本严格递增、不能有缺口
（`inertial.cpp` 会抛 `IMU interval endpoints must be strictly increasing`）。削模长保持了
时间连续性，代价是"撞击后的速度被少算"，需要靠随后的点云更新把速度拉回真值 ——
这正是设计意图：宁可让激光去纠，也不要让加速度计单方面把速度积分到 50 m/s。

参数
====
| 参数 | 默认 | 说明 |
|---|---|---|
| `input_topic` | `livox/imu` | 原始 IMU（相对名，跟随命名空间） |
| `output_topic` | `livox/imu_saturated` | 削顶后的 IMU，ZLIO 订这一个 |
| `max_acceleration` | `30.0` | 加速度模长上限（m/s²）；设成很大（如 1e6）等于关掉削顶 |
| `warn_throttle_sec` | `2.0` | 告警节流（秒） |
| `report_interval_sec` | `10.0` | 周期性汇报：共削了多少帧、累计时长占比 |

输出与输入**同 stamp、同 header**，其它字段（角速度、协方差、朝向）原样透传。
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuSaturationGuard(Node):
    """把 IMU 加速度模长削到上限以内，并把削顶事件显式报出来。"""

    def __init__(self):
        super().__init__("imu_saturation_guard")

        self.declare_parameter("input_topic", "livox/imu")
        self.declare_parameter("output_topic", "livox/imu_saturated")
        self.declare_parameter("max_acceleration", 30.0)
        self.declare_parameter("warn_throttle_sec", 2.0)
        self.declare_parameter("report_interval_sec", 10.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.warn_throttle_sec = float(self.get_parameter("warn_throttle_sec").value)
        report_interval = float(self.get_parameter("report_interval_sec").value)

        if not (self.max_acceleration > 0.0) or not math.isfinite(self.max_acceleration):
            raise ValueError("max_acceleration must be finite and positive")
        if not input_topic or not output_topic:
            raise ValueError("input_topic / output_topic must not be empty")

        self.samples = 0
        self.clipped = 0
        self.worst = 0.0
        self.last_warn = None
        self._report_samples = 0
        self._report_time = None

        # 订阅：源头（ros_gz_bridge）可能是 best-effort，这里按传感器 QoS 收；
        # 发布：只有 zlio 一个订阅者（best-effort keep_last(4000)），所以也用 best-effort，
        # 但把深度放大到 1000 —— 200 Hz 的 IMU 用深度 5 会丢帧，而 ZLIO 的预积分
        # 要求时间连续。
        sensor_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(Imu, output_topic, sensor_qos)
        # 深度放大到 1000：200 Hz 的 IMU 用默认 depth=5 只剩 25 ms 缓冲，
        # 而 ZLIO 的预积分对"样本间隔"敏感（`maximum_imu_interval`: 0.1 s）。
        self.subscription = self.create_subscription(
            Imu, input_topic, self._on_imu, sensor_qos
        )
        self.report_timer = self.create_timer(max(1.0, report_interval), self._report)

        self.get_logger().info(
            f"IMU 饱和保护: {input_topic} -> {output_topic}, "
            f"|a| 上限 {self.max_acceleration:.1f} m/s²"
        )

    def _on_imu(self, msg: Imu):
        self.samples += 1
        acceleration = msg.linear_acceleration
        norm = math.sqrt(
            acceleration.x * acceleration.x
            + acceleration.y * acceleration.y
            + acceleration.z * acceleration.z
        )
        if norm > self.max_acceleration:
            scale = self.max_acceleration / norm
            acceleration.x *= scale
            acceleration.y *= scale
            acceleration.z *= scale
            self.clipped += 1
            self.worst = max(self.worst, norm)
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.last_warn is None or now - self.last_warn >= self.warn_throttle_sec:
                self.last_warn = now
                self.get_logger().warn(
                    "IMU 加速度饱和: |a|=%.1f m/s² 超过上限 %.1f，已按方向削顶"
                    "（累计 %d/%d 帧，最大 %.1f）"
                    % (norm, self.max_acceleration, self.clipped, self.samples, self.worst)
                )
        self.publisher.publish(msg)

    def _report(self):
        """周期性汇报：IMU 实际频率 + 削顶比例。

        频率这一项很值得看：仿真 IMU 应该是 200 Hz，如果这里只有 60~100 Hz，
        说明链路上有丢帧（ros_gz_bridge 的 QoS/负载），而 ZLIO 的预积分会因此变差 ——
        在排查"里程计抖"的时候，这是第一个该看的数。
        """
        now = time.monotonic()
        elapsed = 0.0 if self._report_time is None else now - self._report_time
        delta = self.samples - self._report_samples
        rate = (delta / elapsed) if elapsed > 0.0 else float("nan")
        self._report_time = now
        self._report_samples = self.samples
        if self.samples == 0:
            self.get_logger().warn("IMU 饱和保护: 一条 IMU 都没收到（检查 input_topic / 命名空间）")
            return
        ratio = 100.0 * self.clipped / max(1, self.samples)
        message = "IMU 饱和保护统计: %d 帧, 实测 %.0f Hz, 削顶 %d 帧 (%.2f%%)" % (
            self.samples, rate, self.clipped, ratio)
        if self.clipped:
            message += ", 最大 |a|=%.1f m/s²" % self.worst
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImuSaturationGuard()
    except Exception as error:  # noqa: BLE001 - 参数错误要让启动直接失败
        print(f"[imu_saturation_guard] 初始化失败: {error}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
