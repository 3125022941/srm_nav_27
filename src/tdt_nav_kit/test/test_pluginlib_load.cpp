// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 验证插件确实通过 pluginlib 导出并可被 Nav2 加载。
// 只加载类、不调用 configure()（那需要真实的 TF / costmap）。
// 范式参照 srm27_minco_controller/test/test_pluginlib_loading.cpp。

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_loader.hpp>
#include <string>
#include <vector>

namespace
{
constexpr const char * kPackage = "nav2_core";
constexpr const char * kBaseClass = "nav2_core::GlobalPlanner";
constexpr const char * kPlugin = "tdt_nav_kit::TdtGlobalPlanner";
}  // namespace

TEST(TdtPlannerPluginTest, DeclaredAsNav2GlobalPlanner)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(kPackage, kBaseClass);
  const std::vector<std::string> classes = loader.getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(), std::string(kPlugin)), classes.end())
    << "插件未在 nav2_core 描述文件中声明；已声明的类共 " << classes.size() << " 个";
}

TEST(TdtPlannerPluginTest, CanBeInstantiated)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(kPackage, kBaseClass);
  std::shared_ptr<nav2_core::GlobalPlanner> instance;
  ASSERT_NO_THROW(instance = loader.createSharedInstance(kPlugin));
  ASSERT_TRUE(instance != nullptr);
  // 能创建即说明共享库与 PLUGINLIB_EXPORT_CLASS 导出符号正确；未 configure 前不可直接使用。
}

TEST(TdtPlannerPluginTest, PluginXmlIsInstalled)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(kPackage, kBaseClass);
  const std::string manifest = loader.getPluginManifestPath(kPlugin);
  EXPECT_FALSE(manifest.empty());
  EXPECT_NE(manifest.find("tdt_planner_plugin.xml"), std::string::npos)
    << "插件描述文件路径异常: " << manifest;
}
