# tdt_nav_kit

把东北大学 T-DT 开源组件 [`tdt-nav-kit`](https://github.com/T-DT-Algorithm-2026/tdt-nav-kit)
包装成两个 Nav2 插件：

| 插件 | 基类 | 上游算法 |
| --- | --- | --- |
| `tdt_nav_kit::TdtGlobalPlanner` | `nav2_core::GlobalPlanner` | YAstar（分层 Mask 的 A*） |
| `tdt_nav_kit::TdtSmoother` | `nav2_core::Smoother` | MinimumSnap（OSQP 求解）+ SfcSquare 走廊 |

- 上游版本：commit `814d5ce0237acc7faa8c070e5c5e61e2ddc81996`（tree `3274cebb...`，2026-10-09）
- 许可：`vendor/**` 为 MIT（见 `vendor/LICENCE`）；本包其余文件为 Apache-2.0
- 来源与补丁：`THIRD_PARTY_NOTICES.md`、`PATCHES.md`

## 范围

| 做 | 不做 |
| --- | --- |
| 两个插件、Costmap 快照转换层、5 个 gtest | KinodynamicAstar（约 2 s）、动态障碍 / 多层 Mask 调优 |
| 输出 `nav_msgs/msg/Path`（`map` 帧），不改 `cmd_vel` | 动 Costmap / 行为树 / 控制器 / 底盘 |
| 新增独立参数文件与启动封装 | 修改任何既有文件 |

上游 8 个源码全部入库；CMake 编译 `yastar.cpp`、`minimumSnap.cpp`、`sfcSquare.cpp`，
**不编译 `kinodynamicAstar.cpp`**（上游自述全局搜索约 2 s，主链路不用）。

## 约束（本次移植特有的两条）

1. 改动只在分支 `hj_dev`（基于 `b2d6fc2`）上，`main` 不提交。
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

## 阶段二依赖（OSQP / OsqpEigen）

`TdtSmoother` 需要 OSQP 与 OsqpEigen，装法与阶段一无依赖的情况不同：

```bash
# 1) OSQP：ROS 包直接装（头文件在 /opt/ros/humble/include/osqp，库在 /opt/ros/humble/lib）
sudo apt install ros-humble-osqp-vendor

# 2) OsqpEigen：没有 rosdep key，源码装到 /usr/local
git clone --depth 1 -b v0.8.0 https://github.com/robotology/osqp-eigen.git
cmake -S osqp-eigen -B osqp-eigen/build -DCMAKE_BUILD_TYPE=Release \
      -DOSQP_EIGEN_BUILD_TESTS=OFF -DOSQP_EIGEN_BUILD_EXAMPLES=OFF
sudo cmake --build osqp-eigen/build -j2 && sudo cmake --install osqp-eigen/build

# 3) 关键一步：把 /usr/local/lib 加进动态库搜索路径
#    否则运行期报 libOsqpEigen.so.0.8.0: cannot open shared object file
#    （表现为 colcon test 全部 "1 test, 1 error"，pluginlib 实例化也失败）
echo /usr/local/lib | sudo tee /etc/ld.so.conf.d/usr-local.conf && sudo ldconfig
```

`package.xml` 里只声明了 `<depend>osqp_vendor</depend>`；OsqpEigen 没有 rosdep key，
无法在其中声明，CMake 用 `find_package(OsqpEigen REQUIRED)` 直接找。

## 启动

```bash
# 只换全局规划器（平滑器仍是 SimpleSmoother）
ros2 launch tdt_nav_kit tdt_nav_simulation.launch.py \
  params_file:=$(ros2 pkg prefix tdt_nav_kit)/share/tdt_nav_kit/config/nav2_params_srm_tdt.yaml

# 规划器 + 平滑器都换
ros2 launch tdt_nav_kit tdt_nav_simulation.launch.py \
  params_file:=$(ros2 pkg prefix tdt_nav_kit)/share/tdt_nav_kit/config/nav2_params_srm_tdt_smooth.yaml
```

# 回退原链路：不带任何参数，仍是 ThetaStar + SimpleSmoother
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py
```

**不要**传 `speed_limits_file`。原 launch 的默认值已是显式绝对路径
`config/simulation/speed_limits.yaml`，与本包的参数文件所在目录无关；
若改成 `auto`，`ReplaceString` 会让参数文件落到临时目录，
`srm_speed_limits.py` 会静默不合并限幅，退回插件默认高值（Omni 3.0 m/s）。

`config/nav2_params_srm_tdt.yaml` 以 `nav2_params_srm.yaml` 为底，**只改了
`planner_server` 段**；`nav2_params_srm_tdt_smooth.yaml` 在此基础上再**只改
`smoother_server` 段**。两份文件都保持 `controller_server.FollowPath.plugin` 为
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

- `colcon build --packages-select tdt_nav_kit` **通过**（阶段一约 26 s，阶段二约 18 s）；
- `colcon test` **5 个测试可执行文件、32 个 gtest 用例全部通过**，`colcon test-result` 汇总 32 项、0 errors / 0 failures；
- pluginlib 加载测试证明 `TdtGlobalPlanner` 与 `TdtSmoother` 均已注册到对应基类、可被实例化，且来自同一个共享库。

构建期修正：`ament_auto_add_library()` 内部通过 `ament_target_dependencies()` 使用
`target_link_libraries` 的 plain 签名，因此本包链接 Eigen3 / OsqpEigen 时也**不能写 `PUBLIC`**，
否则 CMake 报 all-plain / all-keyword 混用。`SolveInput::setTimeAllocated()` 取的是非 const
引用，必须传具名 `std::vector<double>` 左值。另有一个非致命警告：`ament_auto_package`
建议使用 `USE_SCOPED_HEADER_INSTALL_DIR`；本包头已是 `include/tdt_nav_kit/*.hpp`
布局，保持默认非 scoped 才能让下游 `#include "tdt_nav_kit/xxx.hpp"` 找到，故保留。

> 本机 WSL 发行版是 Ubuntu 24.04，而 ROS 2 Humble 只支持 22.04，因此验证放在
> Ubuntu 22.04 容器内进行，未改动 WSL 宿主机系统，也未往本仓库写构建产物。

## 验证

```bash
colcon test --packages-select tdt_nav_kit --event-handlers console_direct+
colcon test-result --all --verbose
ros2 param get /planner_server GridBased.plugin        # tdt_nav_kit::TdtGlobalPlanner
ros2 param get /smoother_server tdt_smoother.plugin    # 仅 smooth 变体：tdt_nav_kit::TdtSmoother
ros2 param get /controller_server FollowPath.plugin    # 仍为 OmniPidPursuitController
```

启动日志不应出现 `Failed to create planner` / `Failed to create smoother`
或 `[srm_speed_limits] 警告`。5 个 gtest 分别覆盖：Costmap 语义/索引/快照、
SFC 反向占据语义与边界补圈、P7 原点符号与两套坐标换算一致性、
YAstar 链路与 SDF 一致性、pluginlib 加载。

零改动判据（在仓库根执行）：

```bash
git diff --name-only b2d6fc2..HEAD | grep -v '^src/tdt_nav_kit/'   # 应无输出
git rev-parse main                                                    # 应仍是 b2d6fc2
```

## 阶段二进展与遗留

已完成：OSQP（`osqp_vendor`）+ OsqpEigen v0.8.0 接入；`minimumSnap.cpp` /
`sfcSquare.cpp` 纳入编译；`TdtSmoother` 插件；SFC 反向语义转换与边界补圈；
P7 原点补丁经单测验证；OSQP 失败 / 超时一律保留原路径并返回 `false`。

仍遗留（需真机或仿真数据才能定论）：

- **126.7 ms 长尾**：2026 地图上 Minimum Snap 的最大求解耗时，需用真实路径采样确认是否超平滑预算；
- **corridor 不裁剪**：上游 `_getBound()` 显式不判边界，纯自由图上走廊会外扩到 `max_corridor_range`，
  建议在参数里限制该值（默认 1e9 等价于不限）；
- **`obstacleTolerance = 2` 硬编码**：非严格模式下检测线可"擦过" 2 个障碍栅格，穿薄墙不算碰撞；
- **`setDt()` 实际不生效**：`lineDecoder` 每段固定 11 点；
- **与 `MincoMpcController`（`odom` 帧 / 50 Hz）叠加**：需统一帧与朝向语义后再联调；
- **未接动态 Mask 与多层 Mask**：`mask_num_layers` 已暴露但只用于静态图。
