// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 上游 vendor/ 内的文件适用 MIT，见 vendor/LICENCE 与 THIRD_PARTY_NOTICES.md。

#ifndef TDT_NAV_KIT__TDT_SMOOTHER_HPP_
#define TDT_NAV_KIT__TDT_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tdt_nav_kit/costmap_converter.hpp"
#include "tf2_ros/buffer.h"

namespace tdt_nav_kit
{

/// 把 vendored MinimumSnap(SfcSquare + OSQP) 包装成 Nav2 平滑器。
///
/// 签名按 ros-navigation/navigation2 的 humble 分支头文件核对（2026-11-04）：
///   nav2_core/include/nav2_core/smoother.hpp
/// configure 与 GlobalPlanner 不同：第 4/5 参数是 CostmapSubscriber 与
/// FootprintSubscriber（不是 Costmap2DROS）；smooth 的第二参数是 rclcpp::Duration。
///
/// **失败时不得改动传入的 path**：上游 OSQP 不收敛会返回空路径且没有闭式兜底
/// （PATCHES.md「未打补丁的已知上游问题」），此时直接返回 false，
/// 让 smoother_server 保留原始路径继续下发。
class TdtSmoother : public nav2_core::Smoother
{
public:
  TdtSmoother() = default;
  ~TdtSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  bool active_{false};

  ConverterConfig converter_cfg_;

  /// SFC / MinimumSnap 参数，默认值刻意与上游 README 示例一致而非库默认。
  int order_{5};
  int max_dx_{3};
  bool strict_collision_{false};
  /// "close"（闭式，默认）| "path"（OSQPPath）| "corridor"（OSQPCorridor）
  ///
  /// 默认取 close：在 RMUC 2026 仿真地图上实测，corridor 后端生成的走廊约束
  /// 会让 OSQP 长期不收敛（状态 -4/-6），而 close 后端 0.2~19 ms 即可解出且
  /// 经迭代碰撞检查后无碰撞。corridor 仍保留可配，待走廊生成调好后可切回。
  std::string backend_{"close"};
  int collision_iter_{2};
  bool norm_time_{true};
  /// 走廊最大扩张范围（米）。不要给 1e9 之类的"不限"值：
  /// 上游 _getBound 会把它除以分辨率再转 int，0.05 m/格 时直接溢出。
  float max_corridor_range_{10.0f};
  float corridor_shrink_{0.0f};
  float max_speed_{2.0f};
  float max_acc_{2.0f};

  /// 少于该点数的路径不做平滑（直接返回 true）。
  int min_points_{3};
  /// "identity"（w=1）或 "tangent"（由相邻点求 yaw）。
  std::string orientation_mode_{"tangent"};
};

}  // namespace tdt_nav_kit

#endif  // TDT_NAV_KIT__TDT_SMOOTHER_HPP_
