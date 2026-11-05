#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# SRM LIO vs 真值 航向(yaw) 波形调试工具 (debug_tools #2)
#
# 用途：回答"LIO 算出来的车头朝向，和 Gazebo 真值差多少、什么时候开始差"。
#   一张图 4 个面板：
#     ① yaw 曲线        ── 真值(GT) 与 导航用的 /odometry，两条线叠在一起
#     ② yaw 误差        ── 差值(°)，已折到 ±180，0 就是完全对齐
#     ③ 位置轨迹 (x-y)  ── 真值 vs 导航；云台转时若出现小圆，这里一眼可见
#     ④ 速度符号        ── /odometry 的 twist.x 与 /cmd_vel_sim 的 linear.x
#                          （判断"前进变后退"是 LIO 认错方向，还是命令本身反了）
#
# -------------------------------------------------------------
# 数据从哪来（只读，不往任何 cmd_vel 写东西）
# -------------------------------------------------------------
#   真值:  <ns>/chassis_odometry_gt   (nav_msgs/Odometry, Gazebo 插件直报)
#   LIO :  <ns>/odometry              (nav_msgs/Odometry, loam_interface +
#                                       sensor_scan_generation 算出来的车体位姿)
#   命令:  <ns>/cmd_vel_sim           (geometry_msgs/Twist, mux 的唯一出口)
#   可选:  <ns>/lidar_odometry        (odometry 之前那一级，加 --with-raw 才画)
#
#   注意：真值话题的 yaw 是"车体在 Gazebo 世界系里的朝向"；/odometry 的 yaw 是
#   "车体在 odom 系里的朝向"。两者在 map→odom 为静态单位阵时可以直接比较；
#   若你改了 MAP2ODOM_YAW，这条曲线里的差值里就会包含那个常量（这是有意义的，
#   因为它正是导航"以为"的朝向误差）。
#
# -------------------------------------------------------------
# 用法
# -------------------------------------------------------------
#   # 1) 不带启动器跑（先看工具本身能不能出图）
#   ./debug_tools/plot_lio_gt_yaw.py --demo
#
#   # 2) 真实录制（默认一直录，Ctrl-C 结束时出图）
#   ./debug_tools/plot_lio_gt_yaw.py --headless --save /tmp/lio_yaw.png
#   ./debug_tools/plot_lio_gt_yaw.py --ros-args -r __ns:=/red_standard_robot1
#
#   # 3) 想固定时长也行
#   ./debug_tools/plot_lio_gt_yaw.py --duration 60 --headless --save yaw.png
#
#   # 4) 只想看统计数字（不画图）
#   ./debug_tools/plot_lio_gt_yaw.py --duration 30 --stats-only
#
#   典型流程（复现"前进变后退"）：
#     终端A: ./script/start_sim_nav.sh -w rmuc_2025 -m rmuc_2025 --yaw-mode spin --lio
#     终端B: ./debug_tools/plot_lio_gt_yaw.py --duration 60 --headless --save /tmp/lio_yaw.png
#     终端C: 发目标/发速度；看图上 ② 什么时候开始离开 0、④ 两条线是否同号
#
# 判读要点：
#   · ② 常年贴着 0（±2°）→ 坐标换算没问题，问题在下游（控制/执行）
#   · ② 在"开始动/云台转起来"之后才拉开 → LIO 在动态下配准不住（点云太稀）
#   · ④ 里 /odometry.twist.x 与 /cmd_vel_sim.linear.x 长期不同号 → 速度符号/坐标约定反了
#   · ③ 里导航轨迹画小圆而真值不动 → 云台离轴杠杆臂（本车约 13 cm）残差
# =============================================================

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

import signal

try:  # 只在真正录制时才需要 rclpy；--demo 模式不依赖 ROS
    from rclpy.executors import ExternalShutdownException
except Exception:  # noqa: BLE001 - 没有 source ROS 时用占位异常
    class ExternalShutdownException(Exception):
        pass



# ROS 专有参数：带值的要连带吃掉它的值，不带值的直接丢。
_ROS_FLAGS_WITH_VALUE = ("-r", "--remap", "-p", "--param", "-e", "--enclave",
                         "--log-level", "--log-config-file", "--params-file")
_ROS_FLAGS_BARE = ("--ros-args", "--", "-a")


def _filter_ros_args(argv):
    """把 ROS 专有参数从命令行里剔出来（返回 (自定义参数, ROS 参数)）。"""
    custom, ros = [], []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in _ROS_FLAGS_WITH_VALUE and i + 1 < len(argv):
            ros.extend([a, argv[i + 1]])
            i += 2
            continue
        if a in _ROS_FLAGS_BARE or a.startswith("__"):
            ros.append(a)
            i += 1
            continue
        custom.append(a)
        i += 1
    return custom, ros


def wrap_deg(angle_deg):
    """把角度折到 (-180, 180]。"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def quat_to_yaw(x, y, z, w):
    """四元数 -> 航向角（度）。"""
    return math.degrees(2.0 * math.atan2(z, w))


def quat_to_roll_pitch(x, y, z, w):
    """四元数 -> (roll, pitch)，单位度。

    为什么需要它：航向由陀螺决定，通常很准；**平移**靠加速度计 + 激光匹配，
    一旦 LIO 的姿态基准有几度误差，重力就会漏进水平加速度（g·sinθ），
    被积分成"车在匀速/加速漂走"——此时 yaw 完好、位置发散。
    """
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


class Recorder(object):
    """订阅两个里程计 + 速度命令，按时间戳存成序列。"""

    def __init__(self, args):
        import rclpy
        from rclpy.node import Node  # noqa: F401
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist

        self.args = args
        self.rows = []
        self.t0 = None

        rclpy.init(args=[])  # args=[] 而不是 None：否则自定义命令行开关会被当成 ROS 参数解析
        self.node = rclpy.create_node("plot_lio_gt_yaw")
        self.ns = (args.namespace or "").strip("/")
        if not self.ns:
            self.ns = self._autodetect_namespace(rclpy, args.gt_topic)

        def full(topic):
            if not self.ns or topic.startswith("/"):
                return topic
            return "/" + self.ns.strip("/") + "/" + topic.strip("/")

        gt_topic = full(args.gt_topic)
        lio_topic = full(args.nav_topic)
        cmd_topic = full(args.cmd_topic)
        raw_topic = full(args.raw_topic)

        self.gt = None
        self.lio = None
        self.cmd = None
        self.raw = None
        self.imu = None

        self.node.create_subscription(
            Odometry, gt_topic, self._on_gt, 20)
        self.node.create_subscription(
            Odometry, lio_topic, self._on_lio, 20)
        self.node.create_subscription(
            Twist, cmd_topic, self._on_cmd, 20)
        if args.with_raw:
            self.node.create_subscription(
                Odometry, raw_topic, self._on_raw, 20)
        from sensor_msgs.msg import Imu
        self.node.create_subscription(
            Imu, full(args.imu_topic), self._on_imu, 50)

        self.node.get_logger().info(
            "录制中: GT=%s  NAV=%s  CMD=%s  %s" %
            (gt_topic, lio_topic, cmd_topic,
             "时长=一直（Ctrl-C 结束）" if args.duration <= 0.0
             else "时长=%.0f s" % args.duration))

    def _autodetect_namespace(self, rclpy, topic_leaf):
        """没给 --namespace 时，自动找带命名空间的同名话题（如 /red_standard_robot1/xxx）。

        这样 `./debug_tools/plot_lio_gt_yaw.py` 直接就能用，不必每次敲 __ns:=...
        """
        for _ in range(20):          # 等图发现，最多约 2 s
            rclpy.spin_once(self.node, timeout_sec=0.1)
        candidates = [
            name.rsplit("/", 1)[0].strip("/")
            for name in dict(self.node.get_topic_names_and_types())
            if name.endswith("/" + topic_leaf.strip("/")) and name.count("/") > 1
        ]
        candidates = [c for c in candidates if c]
        if len(set(candidates)) == 1:
            ns = candidates[0]
            self.node.get_logger().info("自动识别命名空间: /%s（可用 --namespace 覆盖）" % ns)
            return ns
        if len(set(candidates)) > 1:
            self.node.get_logger().warn(
                "发现多个命名空间都有 %s: %s —— 请用 --namespace 指定"
                % (topic_leaf, sorted(set(candidates))))
            return ""
        self.node.get_logger().warn(
            "没发现 /<ns>/%s 这样的话题，按无命名空间处理；若话题确实带命名空间，请用 "
            "--namespace 或 --ros-args -r __ns:=/xxx 指定" % topic_leaf)
        return ""

    # ---- 回调：只存最新值，采样由定时器统一做（三条曲线天然同时间轴） ----
    def _on_gt(self, msg):
        self.gt = msg

    def _on_lio(self, msg):
        self.lio = msg

    def _on_cmd(self, msg):
        self.cmd = msg

    def _on_raw(self, msg):
        self.raw = msg

    def _on_imu(self, msg):
        self.imu = msg

    def _snapshot(self):
        if self.gt is None or self.lio is None:
            return
        import rclpy
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        g, o = self.gt.pose.pose.orientation, self.lio.pose.pose.orientation
        gp, op = self.gt.pose.pose.position, self.lio.pose.pose.position
        yaw_gt = quat_to_yaw(g.x, g.y, g.z, g.w)
        yaw_lio = quat_to_yaw(o.x, o.y, o.z, o.w)
        roll_gt, pitch_gt = quat_to_roll_pitch(g.x, g.y, g.z, g.w)
        roll_lio, pitch_lio = quat_to_roll_pitch(o.x, o.y, o.z, o.w)
        self.rows.append(
            dict(
                t=now - self.t0,
                yaw_gt=yaw_gt,
                yaw_lio=yaw_lio,
                err=wrap_deg(yaw_lio - yaw_gt),
                gt_x=gp.x, gt_y=gp.y,
                lio_x=op.x, lio_y=op.y,
                lio_vx=self.lio.twist.twist.linear.x,
                cmd_vx=(self.cmd.linear.x if self.cmd is not None else float("nan")),
                roll_gt=roll_gt, pitch_gt=pitch_gt,
                roll_lio=roll_lio, pitch_lio=pitch_lio,
                imu_ax=(self.imu.linear_acceleration.x if self.imu is not None else float("nan")),
                imu_ay=(self.imu.linear_acceleration.y if self.imu is not None else float("nan")),
                imu_az=(self.imu.linear_acceleration.z if self.imu is not None else float("nan")),
                raw_yaw=(quat_to_yaw(
                    self.raw.pose.pose.orientation.x, self.raw.pose.pose.orientation.y,
                    self.raw.pose.pose.orientation.z, self.raw.pose.pose.orientation.w)
                    if self.raw is not None else float("nan")),
            )
        )

    def run(self):
        """录制：--duration<=0 时一直录，直到 Ctrl-C。"""
        import rclpy
        period = 1.0 / self.args.rate
        start = time.time()
        last_report = start
        unlimited = self.args.duration <= 0.0
        print(
            "开始录制（%s）。%s"
            % (
                "一直录，按 Ctrl-C 结束并出图" if unlimited
                else "录 %.0f s" % self.args.duration,
                "" if unlimited else "想提前结束也可以随时 Ctrl-C（已录部分照样出图）",
            )
        )
        stop = {"flag": False}

        def _on_sigint(_signum, _frame):
            stop["flag"] = True

        old_handler = signal.signal(signal.SIGINT, _on_sigint)
        try:
            while (unlimited or (time.time() - start) < self.args.duration) and not stop["flag"]:
                try:
                    rclpy.spin_once(self.node, timeout_sec=period)
                except KeyboardInterrupt:
                    stop["flag"] = True
                    continue
                except ExternalShutdownException:
                    stop["flag"] = True
                    continue
                self._snapshot()
                now = time.time()
                if now - last_report >= 5.0:
                    last_report = now
                    print(
                        "  已录 %5.1f s, %4d 个采样点%s"
                        % (
                            now - start,
                            len(self.rows),
                            "" if self.rows else "  <-- 还没有数据，检查仿真/命名空间/话题名",
                        )
                    )
        finally:
            signal.signal(signal.SIGINT, old_handler)
            if stop["flag"]:
                print("")
                print("收到 Ctrl-C：停止录制，开始出图/统计")
            self.node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001 - 可能已 shutdown
                pass


def demo_rows(duration=20.0, rate=20.0):
    """合成数据：前 8 秒完全对齐，之后 LIO 逐渐漂 + 偶发跳变。"""
    rows = []
    t = 0.0
    while t < duration:
        yaw_gt = 15.0 * math.sin(0.25 * t)
        drift = 0.0 if t < 8.0 else 6.0 * (t - 8.0) / duration * 10.0
        jump = 40.0 if abs((t % 6.0) - 3.0) < 0.06 else 0.0
        yaw_lio = yaw_gt + drift + jump
        r = 0.133  # 云台离轴杠杆臂（m）
        rows.append(
            dict(
                t=t,
                yaw_gt=yaw_gt,
                yaw_lio=wrap_deg(yaw_lio),
                err=wrap_deg(yaw_lio - yaw_gt),
                gt_x=0.0, gt_y=0.0,
                lio_x=r * math.sin(2.0 * t), lio_y=r * (1 - math.cos(2.0 * t)),
                # demo 里造一个 3° 的 roll 偏差：等效重力泄漏 ≈ 0.51 m/s²
                roll_gt=0.0, pitch_gt=0.0,
                roll_lio=3.0, pitch_lio=0.5,
                imu_ax=float("nan"), imu_ay=float("nan"), imu_az=float("nan"),
                lio_vx=0.2 * math.cos(0.5 * t),
                cmd_vx=0.2,
                raw_yaw=float("nan"),
            )
        )
        t += 1.0 / rate
    return rows


def print_stats(rows):
    err = np.array([r["err"] for r in rows], dtype=float)
    jump = np.abs(np.diff(np.array([r["yaw_lio"] for r in rows], dtype=float)))
    jump = np.where(jump > 180.0, 360.0 - jump, jump)
    print("")
    print("================ 统计 ================")
    print("采样点数      : %d" % len(rows))
    print("平均误差      : %+7.2f°   (导航 - 真值)" % err.mean())
    print("误差标准差    : %7.2f°" % err.std())
    print("误差范围      : %+7.2f° ~ %+7.2f°" % (err.min(), err.max()))
    print("单帧跳变>5°   : %d 次, 最大 %.1f°" % (int((jump > 5.0).sum()), jump.max()))
    cmd = np.array([r["cmd_vx"] for r in rows], dtype=float)
    vx = np.array([r["lio_vx"] for r in rows], dtype=float)
    mask = np.isfinite(cmd) & (np.abs(cmd) > 0.05)
    if mask.sum() > 5:
        same_sign = float(np.mean(np.sign(vx[mask]) == np.sign(cmd[mask]))) * 100.0
        print("命令 vs LIO 速度同号比例: %.0f%%  (低 <70%% 就是速度方向反了)" % same_sign)
    else:
        print("命令 vs LIO 速度同号比例: 样本太少（录制期间没有明显速度命令）")
    rg = np.array([r.get("roll_gt", np.nan) for r in rows], dtype=float)
    rl = np.array([r.get("roll_lio", np.nan) for r in rows], dtype=float)
    pg = np.array([r.get("pitch_gt", np.nan) for r in rows], dtype=float)
    pl = np.array([r.get("pitch_lio", np.nan) for r in rows], dtype=float)
    if np.isfinite(rl).any():
        droll = float(np.nanmean(rl - rg))
        dpitch = float(np.nanmean(pl - pg))
        tilt = math.sqrt(droll * droll + dpitch * dpitch)
        print("姿态偏差      : roll %+.2f°, pitch %+.2f°, 合成 %.2f°"
              % (droll, dpitch, tilt))
        print("  -> 等效重力泄漏加速度 ≈ %.2f m/s²  "
              "(= 9.81*sin(%.2f°)，越大越会漂)" % (9.81 * math.sin(math.radians(tilt)), tilt))
    print("CSV 已写出    : %s" % os.path.abspath(CSV_PATH))
    print("=====================================")


CSV_PATH = "/tmp/lio_gt_yaw.csv"  # 运行时按 --save/--csv 覆盖
# 工程根 = debug_tools/ 的上一级；默认把测试产物写进 <工程根>/user_test_log/
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def save_csv(rows, path=None):
    global CSV_PATH
    if path:
        CSV_PATH = path
    with open(CSV_PATH, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _pyplot(headless):
    """延迟导入 pyplot：先决定后端，避免在有 DISPLAY 但连不上 X 时卡住。"""
    import matplotlib
    if headless or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    _CJK_CANDIDATES = (
        "Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei",
        "Droid Sans Fallback", "AR PL UMing CN", "Source Han Sans SC",
    )
    _installed = {f.name for f in font_manager.fontManager.ttflist}
    for _name in _CJK_CANDIDATES:
        if _name in _installed:
            plt.rcParams["font.sans-serif"] = [_name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def plot(rows, save_path, title_suffix="", headless=False):
    plt = _pyplot(headless)
    t = np.array([r["t"] for r in rows])
    yg = np.array([r["yaw_gt"] for r in rows])
    yl = np.array([r["yaw_lio"] for r in rows])
    er = np.array([r["err"] for r in rows])
    gx = np.array([r["gt_x"] for r in rows])
    gy = np.array([r["gt_y"] for r in rows])
    lx = np.array([r["lio_x"] for r in rows])
    ly = np.array([r["lio_y"] for r in rows])
    vx = np.array([r["lio_vx"] for r in rows])
    cvx = np.array([r["cmd_vx"] for r in rows])
    ry = np.array([r["raw_yaw"] for r in rows])

    rg = np.array([r.get("roll_gt", np.nan) for r in rows], dtype=float)
    pg = np.array([r.get("pitch_gt", np.nan) for r in rows], dtype=float)
    rl = np.array([r.get("roll_lio", np.nan) for r in rows], dtype=float)
    pl = np.array([r.get("pitch_lio", np.nan) for r in rows], dtype=float)

    fig, axes = plt.subplots(5, 1, figsize=(13, 13.5))
    # 上面板做 unwrap：否则 yaw 越过 ±180 会画成"跳一大截"，看不出真实趋势。
    yg_u = np.degrees(np.unwrap(np.radians(yg)))
    yl_u = np.degrees(np.unwrap(np.radians(yl)))
    ax = axes[0]
    ax.plot(t, yg_u, color="#1f77b4", lw=1.8, label="真值 GT (chassis_odometry_gt)")
    ax.plot(t, yl_u, color="#d62728", lw=1.4, ls="--", label="导航 /odometry (LIO)")
    if np.isfinite(ry).any():
        ry_mask = np.isfinite(ry)
        ry_u = np.degrees(np.unwrap(np.radians(ry[ry_mask])))
        ax.plot(t[ry_mask], ry_u, color="#2ca02c", lw=1.0, alpha=0.8,
                label="原始 lidar_odometry")
    ax.set_ylabel("yaw (°)")
    ax.set_title("① 航向曲线：LIO vs 真值" + title_suffix)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)

    ax = axes[1]
    ax.plot(t, er, color="#ff7f0e", lw=1.6)
    ax.axhline(0.0, color="k", lw=0.8)
    ax.axhspan(-2, 2, color="#2ca02c", alpha=0.15, label="±2° 正常带")
    ax.set_ylabel("误差 (°)")
    ax.set_title("② 航向误差 = 导航 − 真值（折到 ±180°）")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)

    ax = axes[2]
    ax.plot(gx, gy, color="#1f77b4", lw=1.8, label="真值轨迹")
    ax.plot(lx, ly, color="#d62728", lw=1.2, ls="--", label="导航轨迹")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("③ 位置轨迹：云台转时若这里的导航轨迹画小圆，就是离轴杠杆臂（本车 ≈13 cm）")
    ax.grid(alpha=0.3)
    ax.axis("equal")
    ax.legend(loc="best", fontsize=9)

    ax = axes[3]
    ax.plot(t, cvx, color="#1f77b4", lw=1.6, label="cmd_vel_sim.linear.x (命令)")
    ax.plot(t, vx, color="#d62728", lw=1.4, ls="--", label="/odometry.twist.x (LIO 认为的速度)")
    ax.axhline(0.0, color="k", lw=0.8)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("x 方向速度 (m/s)")
    ax.set_title("④ 速度符号：两条线长期不同号 => 速度方向/坐标约定反了")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)

    ax = axes[4]
    ax.plot(t, rg, color="#1f77b4", lw=1.6, label="真值 roll")
    ax.plot(t, rl, color="#1f77b4", lw=1.2, ls="--", label="LIO roll")
    ax.plot(t, pg, color="#ff7f0e", lw=1.6, label="真值 pitch")
    ax.plot(t, pl, color="#ff7f0e", lw=1.2, ls="--", label="LIO pitch")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("角度 (°)")
    ax.set_title(
        "⑤ 姿态 roll/pitch：LIO 与真值若长期差几度 => 重力会漏进水平加速度"
        "（1° ≈ 0.17 m/s²），位置就会被积分成漂移")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", ncol=2, fontsize=9)

    fig.tight_layout()
    fig.savefig(save_path, dpi=130, bbox_inches="tight")
    print("图已保存: %s" % save_path)
    if os.environ.get("DISPLAY") and not headless:
        plt.show()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="LIO vs 真值 的 yaw 波形（debug_tools #2）",
        add_help=False,
    )
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="录制时长（秒）。不写或写 0 = 一直录，按 Ctrl-C 结束并出图（默认）")
    parser.add_argument("--rate", type=float, default=20.0, help="采样频率（Hz）")
    parser.add_argument("--gt-topic", default="chassis_odometry_gt", help="真值话题")
    parser.add_argument("--nav-topic", default="odometry", help="导航用车体位姿话题")
    parser.add_argument("--cmd-topic", default="cmd_vel_sim", help="速度命令话题")
    parser.add_argument("--raw-topic", default="lidar_odometry", help="原始雷达里程计话题")
    parser.add_argument("--with-raw", action="store_true", help="同时画 lidar_odometry 的 yaw")
    parser.add_argument("--imu-topic", default="livox/imu", help="IMU 话题（用于看重力的泄漏）")
    parser.add_argument("--namespace", default="", help="命名空间，如 red_standard_robot1")
    parser.add_argument("--demo", action="store_true", help="用合成数据出图（不需要 ROS）")
    parser.add_argument("--headless", action="store_true", help="不弹窗，只存文件")
    parser.add_argument(
        "--save", default="",
        help="PNG 保存路径。不写则自动放到 <工程根>/user_test_log/ 下，"
             "文件名带时间戳（如 lio_yaw_20261105_0231.png）")
    parser.add_argument(
        "--csv", default="",
        help="CSV 保存路径（默认与 --save 同目录同名，只把后缀换成 .csv）")
    parser.add_argument("--stats-only", action="store_true", help="只打印统计，不画图")

    # 兼容 ros2 run 风格：把 ROS 专有参数剔除，其余照常解析。
    # 关键：不按 --ros-args 截断 —— 否则写在它后面的自定义开关（如 --duration）会被吞掉，
    # 表现为"明明给了时长却一直录"（踩过）。
    if argv is None:
        argv = sys.argv[1:]
    argv, ros_args = _filter_ros_args(argv)
    for i, a in enumerate(ros_args):
        if a.startswith("__ns:="):
            pass
    args = parser.parse_args(argv)
    for a in ros_args:
        if a.startswith("__ns:="):
            args.namespace = a.split(":=", 1)[1]
    for i, a in enumerate(ros_args):
        if a in ("-r", "--remap") and i + 1 < len(ros_args) \
                and ros_args[i + 1].startswith("__ns:="):
            args.namespace = ros_args[i + 1].split(":=", 1)[1]
    if args.help:
        print(__doc__)
        return 0


    if args.demo:
        print("使用合成数据（--demo）：前 8 秒对齐，之后漂移+跳变，并画一个 13 cm 小圆")
        rows = demo_rows(duration=(args.duration if args.duration > 0 else 20.0))
    else:
        rec = Recorder(args)
        rec.run()
        rows = rec.rows
        if not rows:
            print("[错误] 没有采到数据：确认仿真在跑、命名空间对、话题名对（--gt-topic/--nav-topic）。")
            return 2

    if args.save:
        png_path = os.path.abspath(args.save)
    else:
        out_dir = os.path.join(REPO_ROOT, "user_test_log")
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M")
        png_path = os.path.join(out_dir, "lio_yaw_%s.png" % stamp)
    if args.csv:
        csv_path = os.path.abspath(args.csv)
    else:
        csv_path = os.path.splitext(png_path)[0] + ".csv"
    save_csv(rows, csv_path)
    print("")
    print("保存位置（都是绝对路径）:")
    print("  图  : %s" % png_path)
    print("  数据: %s" % csv_path)
    print_stats(rows)
    if not args.stats_only:
        plot(rows, png_path, headless=args.headless)
    return 0


if __name__ == "__main__":
    sys.exit(main())
