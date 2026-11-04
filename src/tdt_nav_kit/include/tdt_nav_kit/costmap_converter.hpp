// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 上游 vendor/ 内的文件适用 MIT，见 vendor/LICENCE 与 THIRD_PARTY_NOTICES.md。

#ifndef TDT_NAV_KIT__COSTMAP_CONVERTER_HPP_
#define TDT_NAV_KIT__COSTMAP_CONVERTER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

// 前向声明：让本头文件不拉入 ROS 依赖，纯算法单测可以直接包含。
namespace nav2_costmap_2d
{
class Costmap2DROS;
}  // namespace nav2_costmap_2d

namespace tdt_nav_kit
{

struct ConverterConfig
{
  /// 代价 >= 该值视为障碍。默认 252 = nav2_costmap_2d::MAX_NON_OBSTACLE。
  /// Nav2 inflation 层产生 1..252，交给上游 YAstar 的势场处理，不在此判障碍。
  int occupancy_threshold{252};
  /// NO_INFORMATION(255) 的处理方式，默认按障碍。
  bool treat_unknown_as_obstacle{true};
};

struct CostmapSnapshot
{
  int size_x{0};
  int size_y{0};
  float resolution{0.0f};
  float origin_x{0.0f};
  float origin_y{0.0f};
  /// 行主序 y * size_x + x；0 = 可行，1 = 障碍。
  /// 该语义与 YAstar 的 6 参 setMap 重载一致（不要用 3 参 u_char* 重载，那个是 0 = 障碍）。
  std::vector<int8_t> data;
};

/// 纯转换逻辑，不碰 ROS，可无节点单测。
/// char_map 为 Costmap2D::getCharMap() 的原始缓冲，count = size_x * size_y。
std::vector<int8_t> convertCostmapCells(
  const unsigned char * char_map, std::size_t count, const ConverterConfig & cfg);

/// 在 costmap 互斥锁内一次性深拷贝出快照；之后规划只读快照，
/// 避免规划过程中读到正在被更新线程改写的 Costmap 内部缓冲。
/// 失败（空指针 / 零尺寸）抛 std::invalid_argument 或 std::runtime_error。
CostmapSnapshot makeSnapshot(
  nav2_costmap_2d::Costmap2DROS * costmap_ros, const ConverterConfig & cfg);

}  // namespace tdt_nav_kit

#endif  // TDT_NAV_KIT__COSTMAP_CONVERTER_HPP_
