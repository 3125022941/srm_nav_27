// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "tdt_nav_kit/costmap_converter.hpp"

#include <algorithm>
#include <mutex>
#include <stdexcept>

// 构建机核对点（本机无 ROS 环境）：
//  - cost_values.hpp 里 NO_INFORMATION / MAX_NON_OBSTACLE 等常量的头路径
//  - Costmap2D::getMutex() 的返回类型（Nav2 中为 std::recursive_mutex*）
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace tdt_nav_kit
{

std::vector<int8_t> convertCostmapCells(
  const unsigned char * char_map, std::size_t count, const ConverterConfig & cfg)
{
  if (char_map == nullptr && count > 0) {
    throw std::invalid_argument("convertCostmapCells: null char_map with non-zero count");
  }

  const auto threshold = static_cast<unsigned char>(
    std::clamp(cfg.occupancy_threshold, 0, 255));
  std::vector<int8_t> out(count, 1);

  for (std::size_t i = 0; i < count; ++i) {
    const unsigned char cost = char_map[i];
    bool occupied;
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      occupied = cfg.treat_unknown_as_obstacle;
    } else {
      occupied = cost > threshold;
    }
    out[i] = occupied ? 1 : 0;
  }
  return out;
}

CostmapSnapshot makeSnapshot(
  nav2_costmap_2d::Costmap2DROS * costmap_ros, const ConverterConfig & cfg)
{
  if (costmap_ros == nullptr) {
    throw std::invalid_argument("makeSnapshot: costmap_ros is null");
  }
  auto * costmap = costmap_ros->getCostmap();
  if (costmap == nullptr) {
    throw std::runtime_error("makeSnapshot: Costmap2DROS::getCostmap() returned null");
  }

  CostmapSnapshot snap;
  snap.size_x = static_cast<int>(costmap->getSizeInCellsX());
  snap.size_y = static_cast<int>(costmap->getSizeInCellsY());
  snap.resolution = static_cast<float>(costmap->getResolution());
  snap.origin_x = static_cast<float>(costmap->getOriginX());
  snap.origin_y = static_cast<float>(costmap->getOriginY());

  if (snap.size_x <= 0 || snap.size_y <= 0 || snap.resolution <= 0.0f) {
    throw std::runtime_error("makeSnapshot: invalid costmap geometry");
  }

  const auto count = static_cast<std::size_t>(snap.size_x) *
    static_cast<std::size_t>(snap.size_y);

  // getCharMap() 是行主序 y * size_x + x；YAstar::TMatrix 也是 RowMajor，
  // 线性索引同为 y * cols + x，因此这里顺序拷贝即可，**不做 Y 翻转**。
  // 上游 main.cpp 的翻转只因读 PNG（图像左上原点），与本链路无关。
  {
    auto * mutex = costmap->getMutex();
    std::lock_guard<std::recursive_mutex> lock(*mutex);
    snap.data = convertCostmapCells(costmap->getCharMap(), count, cfg);
  }

  return snap;
}

}  // namespace tdt_nav_kit
