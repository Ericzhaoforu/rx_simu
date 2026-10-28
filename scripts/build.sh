#!/usr/bin/env bash
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$USV_NAV_ROOT"
cmake -S external/ardupilot_gazebo -B external/ardupilot_gazebo/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=TRUE \
  -DCMAKE_CXX_FLAGS="-I$USV_NAV_ROOT/external/deps/usr/include"
cmake --build external/ardupilot_gazebo/build --target ArduPilotPlugin -j8
colcon build --symlink-install --base-paths src --packages-select usv_nav2 usv_sim
cmake -S tools/tf_audit -B build/tf_audit
cmake --build build/tf_audit -j4
