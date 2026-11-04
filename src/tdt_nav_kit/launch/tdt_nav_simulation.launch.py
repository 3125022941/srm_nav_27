# Copyright 2026 SRM27 Navigation Team
# Licensed under the Apache License, Version 2.0.
#
# 阶段一的启动封装：不改任何既有 launch 文件，只 include 原仿真入口并覆盖 params_file。
# 用 get_package_share_directory 取 install 后的路径，避免硬编码 F:/... 源码绝对路径
# （仓库既有用法参见 nav_srm_simulation_launch.py:202 / :221）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_launch = os.path.join(
        get_package_share_directory("srm27_nav_bringup"),
        "launch",
        "nav_srm_simulation_launch.py",
    )
    tdt_params = os.path.join(
        get_package_share_directory("tdt_nav_kit"),
        "config",
        "nav2_params_srm_tdt.yaml",
    )

    # 不传 speed_limits_file：原 launch 的默认值是显式绝对路径
    # config/simulation/speed_limits.yaml，与 params_file 所在目录无关。
    # 若显式改成 auto，参数文件经 ReplaceString 落到临时目录后 basename 变化，
    # srm_speed_limits.py 会静默不合并限幅，退回插件默认高值。
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={"params_file": tdt_params}.items(),
        ),
    ])
