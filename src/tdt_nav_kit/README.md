# tdt_nav_kit

把东北大学 T-DT 开源组件 [`tdt-nav-kit`](https://github.com/T-DT-Algorithm-2026/tdt-nav-kit)
的 **YAstar** 包装成 Nav2 `nav2_core::GlobalPlanner` 插件。

- 上游版本：commit `814d5ce0237acc7faa8c070e5c5e61e2ddc81996`（tree `3274cebb...`，2026-10-09）
- 许可：`vendor/**` 为 MIT（见 `vendor/LICENCE`）；本包其余文件为 Apache-2.0
- 来源与补丁：`THIRD_PARTY_NOTICES.md`、`PATCHES.md`

## 阶段一范围

| 做 | 不做 |
| --- | --- |
| 一个 `TdtGlobalPlanner` 插件、Costmap 快照转换层、3 个 gtest | Smoother 插件、MinimumSnap / OSQP / OsqpEigen、KinodynamicAstar |
| 输出 `nav_msgs/msg/Path`（`map` 帧） | 输出 `Twist` / `cmd_vel`、动 Costmap / 行为树 / 控制器 / 底盘 |
| 新增独立参数文件与启动封装 | 修改任何既有文件 |

上游 8 个源码全部入库，但 CMake **只编译 `vendor/src/YAstar/yastar.cpp`**：
`minimumSnap.hpp` 无条件 include `OsqpEigen.h`，阶段一借此完全避开 OSQP 依赖。

## 约束（本次移植特有的两条）

1. 改动只在分支 `feat/tdt-nav-kit-yastar`（基于 `b2d6fc2`）上，`main` 不提交。
2. **既有文件零改动**：所有产出都在 `src/tdt_nav_kit/` 内，连
   `srm27_nav_bringup/config/simulation/` 都不新增文件。

## 启动前置（重要）

`src/srm27_navigation/srm27_nav_bringup/package.xml` **没有**把本包声明为
`exec_depend`（约束 2 的代价）。pluginlib 靠 ament resource index 发现插件，
只要本包已 build 且 overlay 已 source 就能加载，但：

- `rosdep install --from-paths src` 不会自动装本包的系统依赖；
- `colcon build --packages-up-to srm27_nav_bringup` **不会**带上本包；
- 二进制打包（bloom / .deb）会漏装本包 → 生产部署需显式补上。

因此**每次都要显式单独构建并 source**：

```bash
cd <workspace>
colcon build --packages-select tdt_nav_kit --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --parallel-workers 2
source install/setup.bash            # Windows: install\setup.bat
ros2 pkg prefix tdt_nav_kit          # 应输出 install 路径
```

## 启动

```bash
# 新链路（封装内会 include 原 nav_srm_simulation_launch.py 并覆盖 params_file）
ros2 launch tdt_nav_kit tdt_nav_simulation.launch.py

# 回退原链路：不带任何参数，仍是 ThetaStar + SimpleSmoother
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py
```

**不要**传 `speed_limits_file`。原 launch 的默认值已是显式绝对路径
`config/simulation/speed_limits.yaml`，与本包的参数文件所在目录无关；
若改成 `auto`，`ReplaceString` 会让参数文件落到临时目录，
`srm_speed_limits.py` 会静默不合并限幅，退回插件默认高值（Omni 3.0 m/s）。

`config/nav2_params_srm_tdt.yaml` 以 `nav2_params_srm.yaml` 为底，**只改了
`planner_server` 段**；`controller_server.FollowPath.plugin` 保持
`srm27_omni_pid_controller::OmniPidPursuitController`，使 `speed_limits.yaml`
的 `@omni` 档位判定不变；文件里没有重复定义任何限幅键。

### 构建机接口已核对（2026-11-04）

按 `ros-navigation/navigation2` 的 `humble` 分支头文件核对并据此实现：

- `nav2_core::GlobalPlanner::createPlan(start, goal)` 是**两参**，没有较新版本的 `cancel_checker` 第三参数；
- `configure` 的 `name` 为按值 `std::string`，`parent` 为 `rclcpp_lifecycle::LifecycleNode::WeakPtr`；
- `Costmap2D::getMutex()` 返回 `std::recursive_mutex*`；`getCharMap()` 返回 `unsigned char*`；
- `cost_values.hpp` 的 `NO_INFORMATION / LETHAL_OBSTACLE / INSCRIBED_INFLATED_OBSTACLE / MAX_NON_OBSTACLE / FREE_SPACE` 名称一致；
- `PlannerException` 位于 `nav2_core/exceptions.hpp`。

### 真实构建验证（2026-11-04，ROS 2 Humble 容器）

在 Ubuntu 22.04 + ROS 2 Humble 容器中（脚本 `D:/ros2/run_tdt_build_in_docker.sh`）：

- `colcon build --packages-select tdt_nav_kit` **通过**（约 26 s）；
- `colcon test` **3 个测试可执行文件、13 个 gtest 用例全部通过**，`colcon test-result` 汇总 16 项、0 errors / 0 failures；
- pluginlib 加载测试证明 `tdt_nav_kit::TdtGlobalPlanner` 已正确注册到 `nav2_core::GlobalPlanner` 且能被实例化。

构建期修正：`ament_auto_add_library()` 内部通过 `ament_target_dependencies()` 使用
`target_link_libraries` 的 plain 签名，因此本包链接 Eigen3 时也**不能写 `PUBLIC`**，
否则 CMake 报 all-plain / all-keyword 混用。另有一个非致命警告：`ament_auto_package`
建议使用 `USE_SCOPED_HEADER_INSTALL_DIR`；本包头已是 `include/tdt_nav_kit/*.hpp`
布局，保持默认非 scoped 才能让下游 `#include "tdt_nav_kit/xxx.hpp"` 找到，故保留。

> 本机 WSL 发行版是 Ubuntu 24.04，而 ROS 2 Humble 只支持 22.04，因此验证放在
> Ubuntu 22.04 容器内进行，未改动 WSL 宿主机系统，也未往本仓库写构建产物。

## 验证

```bash
colcon test --packages-select tdt_nav_kit --event-handlers console_direct+
colcon test-result --all --verbose
ros2 param get /planner_server GridBased.plugin        # tdt_nav_kit::TdtGlobalPlanner
ros2 param get /controller_server FollowPath.plugin    # 仍为 OmniPidPursuitController
```

启动日志不应出现 `Failed to create planner` 或 `[srm_speed_limits] 警告`。
三个 gtest 分别覆盖：Costmap 语义/索引/快照、YAstar 链路与 SDF 一致性、
pluginlib 加载。

零改动判据（在仓库根执行）：

```bash
git diff --name-only b2d6fc2..HEAD | grep -v '^src/tdt_nav_kit/'   # 应无输出
git rev-parse main                                                    # 应仍是 b2d6fc2
```

## 阶段二待办

- 引入 OSQP v1.0.0 / osqp-eigen v0.11.0，编译 `minimumSnap.cpp` / `sfcSquare.cpp`；
- 第二个 `nav2_core::Smoother` 插件替换 / 并列 SimpleSmoother；
- 对齐 SFC 的相反占据语义（`0=障碍` vs YAstar `0=可行`）；
- 用**负原点** costmap 验证 `PATCHES.md` 的 P7（`sfcSquare.cpp` 原点符号，本阶段未编译）；
- 显式暴露后端参数（README 示例值 ≠ 库默认值）、补 OSQP 失败回退；
- 定位 2026 地图上 126.7 ms 的后端长尾；
- 与 `MincoMpcController`（`odom` 帧 / 50 Hz）统一帧与朝向语义。
