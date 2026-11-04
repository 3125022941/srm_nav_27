// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "tdt_nav_kit/tdt_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <vector>

#include "minimumSnap.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace tdt_nav_kit
{
namespace
{

/// max_corridor_range 的安全上限。上游 _getBound 把 maxRange/mapping 转 int，
/// 必须保证结果落在 int 范围内（1e6 / 0.05 = 2e7）。
constexpr float kMaxCorridorRangeLimit = 1.0e6f;

MinimumSnap::Backend toBackend(const std::string & name)
{
  if (name == "path") {
    return MinimumSnap::Backend::OSQPPath;
  }
  if (name == "close") {
    return MinimumSnap::Backend::Close;
  }
  return MinimumSnap::Backend::OSQPCorridor;
}

/// 把栅格拷进 SfcSquare 的 Eigen::Map（行主序，rows=y, cols=x）。
/// 上游 Map 是 `Eigen::Matrix<unsigned char, Dynamic, Dynamic, RowMajor>`
/// （sfcSquare.hpp:34），线性索引同为 y * cols + x，顺序拷贝即可，不做 Y 翻转。
MinimumSnap::Map toEigenMap(const SfcGrid & grid)
{
  MinimumSnap::Map map(grid.size_y, grid.size_x);
  std::memcpy(
    map.data(), grid.data.data(),
    grid.data.size() * sizeof(unsigned char));
  return map;
}

/// 从 Path 取 xy。上游只处理平面点，z 与朝向在此丢弃（朝向在写回时重算）。
std::vector<Eigen::Vector2f> toPoints(const nav_msgs::msg::Path & path)
{
  std::vector<Eigen::Vector2f> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    points.emplace_back(
      static_cast<float>(pose.pose.position.x),
      static_cast<float>(pose.pose.position.y));
  }
  return points;
}

/// 用平滑后的点重写 path，保留原 header（frame_id / stamp）。
void writeBack(
  nav_msgs::msg::Path & path,
  const std::vector<Eigen::Vector2f> & points,
  const std::string & orientation_mode)
{
  const auto header = path.header;
  path.poses.clear();
  path.poses.reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = points[i].x();
    pose.pose.position.y = points[i].y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    if (orientation_mode == "tangent") {
      const std::size_t prev = (i > 0) ? i - 1 : i;
      const std::size_t next = (i + 1 < points.size()) ? i + 1 : i;
      const double dx = static_cast<double>(points[next].x()) - points[prev].x();
      const double dy = static_cast<double>(points[next].y()) - points[prev].y();
      if (std::hypot(dx, dy) > 1e-9) {
        const double half_yaw = 0.5 * std::atan2(dy, dx);
        pose.pose.orientation.w = std::cos(half_yaw);
        pose.pose.orientation.z = std::sin(half_yaw);
      }
    }
    path.poses.push_back(pose);
  }
}

}  // namespace

void TdtSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("TdtSmoother: unable to lock lifecycle node");
  }
  if (!costmap_sub) {
    throw std::runtime_error("TdtSmoother: costmap_sub is null");
  }

  node_ = parent;
  tf_ = tf;
  costmap_sub_ = costmap_sub;
  footprint_sub_ = footprint_sub;
  plugin_name_ = name;
  clock_ = node->get_clock();

  const std::string p = plugin_name_ + ".";
  declare_parameter_if_not_declared(
    node, p + "occupancy_threshold", rclcpp::ParameterValue(converter_cfg_.occupancy_threshold));
  declare_parameter_if_not_declared(
    node, p + "treat_unknown_as_obstacle",
    rclcpp::ParameterValue(converter_cfg_.treat_unknown_as_obstacle));
  declare_parameter_if_not_declared(node, p + "order", rclcpp::ParameterValue(order_));
  declare_parameter_if_not_declared(node, p + "max_dx", rclcpp::ParameterValue(max_dx_));
  declare_parameter_if_not_declared(
    node, p + "strict_collision", rclcpp::ParameterValue(strict_collision_));
  declare_parameter_if_not_declared(node, p + "backend", rclcpp::ParameterValue(backend_));
  declare_parameter_if_not_declared(
    node, p + "collision_iter", rclcpp::ParameterValue(collision_iter_));
  declare_parameter_if_not_declared(node, p + "norm_time", rclcpp::ParameterValue(norm_time_));
  declare_parameter_if_not_declared(
    node, p + "max_corridor_range", rclcpp::ParameterValue(max_corridor_range_));
  declare_parameter_if_not_declared(
    node, p + "corridor_shrink", rclcpp::ParameterValue(corridor_shrink_));
  declare_parameter_if_not_declared(node, p + "max_speed", rclcpp::ParameterValue(max_speed_));
  declare_parameter_if_not_declared(node, p + "max_acc", rclcpp::ParameterValue(max_acc_));
  declare_parameter_if_not_declared(node, p + "min_points", rclcpp::ParameterValue(min_points_));
  declare_parameter_if_not_declared(
    node, p + "orientation_mode", rclcpp::ParameterValue(orientation_mode_));

  converter_cfg_.occupancy_threshold =
    static_cast<int>(node->get_parameter(p + "occupancy_threshold").as_int());
  converter_cfg_.treat_unknown_as_obstacle =
    node->get_parameter(p + "treat_unknown_as_obstacle").as_bool();
  order_ = static_cast<int>(node->get_parameter(p + "order").as_int());
  max_dx_ = static_cast<int>(node->get_parameter(p + "max_dx").as_int());
  strict_collision_ = node->get_parameter(p + "strict_collision").as_bool();
  backend_ = node->get_parameter(p + "backend").as_string();
  collision_iter_ = static_cast<int>(node->get_parameter(p + "collision_iter").as_int());
  norm_time_ = node->get_parameter(p + "norm_time").as_bool();
  max_corridor_range_ =
    static_cast<float>(node->get_parameter(p + "max_corridor_range").as_double());
  corridor_shrink_ = static_cast<float>(node->get_parameter(p + "corridor_shrink").as_double());
  max_speed_ = static_cast<float>(node->get_parameter(p + "max_speed").as_double());
  max_acc_ = static_cast<float>(node->get_parameter(p + "max_acc").as_double());
  min_points_ = static_cast<int>(node->get_parameter(p + "min_points").as_int());
  orientation_mode_ = node->get_parameter(p + "orientation_mode").as_string();
}

void TdtSmoother::cleanup()
{
  costmap_sub_.reset();
  footprint_sub_.reset();
  tf_.reset();
  active_ = false;
}

void TdtSmoother::activate()
{
  active_ = true;
}

void TdtSmoother::deactivate()
{
  active_ = false;
}

bool TdtSmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  if (!active_) {
    return false;
  }
  if (static_cast<int>(path.poses.size()) < min_points_) {
    // 点数太少无需平滑：返回 true 表示「已完成」，保持原路径。
    return true;
  }

  auto node = node_.lock();
  if (!node) {
    return false;
  }

  // 1) 取栅格快照。CostmapSubscriber 返回的是共享指针，
  //    但内容仍可能被 costmap 线程更新，故先深拷贝再算。
  auto costmap = costmap_sub_ ? costmap_sub_->getCostmap() : nullptr;
  if (!costmap) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000,
      "TdtSmoother: costmap unavailable, keep original path");
    return false;
  }

  SfcGrid grid;
  try {
    grid = makeSfcGrid(makeSnapshotFromCostmap(costmap.get(), converter_cfg_));
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000,
      "TdtSmoother: costmap conversion failed (%s), keep original path", e.what());
    return false;
  }

  // 2) 上游对象。setMap 必须传 originx/originy：SfcSquare 用
  //    (x - originPos)/mapping 反算索引（P7 补丁已统一为减号）。
  MinimumSnap smoother;
  smoother.setMap(
    toEigenMap(grid), grid.resolution, grid.origin_x, grid.origin_y);
  smoother.setOrder(order_);
  smoother.setMaxDx(max_dx_);
  smoother.setStrictCollision(strict_collision_);

  const double limit_s = max_time.seconds();
  smoother.setTL(limit_s > 0.0 ? limit_s : 1.0);

  // 3) 填输入。上游默认 backend=Close、normTime=false、collisionIter=2，
  //    与 README 示例不同，因此这里全部显式设置，不依赖库默认。
  const std::vector<Eigen::Vector2f> points = toPoints(path);

  MinimumSnap::SolveInput input;
  input.setPath(points);
  input.setBackend(toBackend(backend_));
  input.setMaxSpeed(max_speed_);
  input.setMaxAcc(max_acc_);
  // 上游 setTimeAllocated 取的是非 const 引用，必须传具名左值。
  std::vector<double> time_allocated =
    MinimumSnap::trapezoidalTimeAllocation(points, max_speed_, max_acc_);
  input.setTimeAllocated(time_allocated);
  input.setNormTime(norm_time_);
  input.setCollisionCheckIter(collision_iter_);

  // 上游 SfcSquare::_getBound() 把 maxRange 直接 static_cast<int>(maxRange/mapping)。
  // 若参数给 1e9（"相当于不限"），在 0.05 m/格 的地图上得到 2e10，超出 int 范围，
  // 溢出后循环边界异常，_getBound 会越界读并直接把进程打崩。
  // 这里做一次硬性夹紧：1e6 / 0.05 = 2e7，仍在 int 范围内。
  input.setMaxCorridorRange(std::min(max_corridor_range_, kMaxCorridorRangeLimit));
  input.setCorridorShrink(corridor_shrink_);

  // 4) 求解。上游不收敛时返回空路径且 success=false（无闭式兜底），
  //    此时保留原路径并返回 false，由 smoother_server 决定后续行为。
  const auto t0 = std::chrono::steady_clock::now();
  MinimumSnap::SolveOutput result = smoother.solve(input);
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  if (!result.success || result.path.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000,
      "TdtSmoother: OSQP solve failed (backend=%s, iter=%d, %.1f ms), keep original path",
      backend_.c_str(), result.iter, result.time * 1000.0);
    return false;
  }

  // 超过时限也放弃结果：Nav2 的 max_time 是平滑预算，超时结果不应再改变路径。
  if (limit_s > 0.0 && elapsed > limit_s) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000,
      "TdtSmoother: exceeded max_time (%.1f ms > %.1f ms), keep original path",
      elapsed * 1000.0, limit_s * 1000.0);
    return false;
  }

  // 5) 写回。只有确认成功后才改 path，保证失败路径不被破坏。
  writeBack(path, result.path, orientation_mode_);
  path.header.stamp = node->now();

  RCLCPP_DEBUG(
    node->get_logger(),
    "TdtSmoother: %zu -> %zu points, iter=%d, solve %.1f ms, wall %.1f ms",
    points.size(), result.path.size(), result.iter, result.time * 1000.0, elapsed * 1000.0);
  return true;
}

}  // namespace tdt_nav_kit

PLUGINLIB_EXPORT_CLASS(tdt_nav_kit::TdtSmoother, nav2_core::Smoother)
