# Orin 实机部署：启动顺序、命令与测试流程

更新：2026-11-04。对象为 Orin + 真实 MID-360 + Pixhawk / 双推进器船，目标飞控固定 **ArduRover V4.4.0，`d0356316eb0fa34ed1c9f39f7b059b9ae7ba4664`**。本文提供现场执行流程；仓库仿真通过不代表实船通过。首次水试先用普通 Navfn + DWB 导航及独立原生 LOITER，Task1 自动任务仍有动态占据清除缺口，不能直接当作已验收的比赛实船程序。

## 1. 本次核查的 Orin 环境

| 项目 | SSH 只读核查结果 |
|---|---|
| 登录 / 主机 | `robotx@100.91.153.16` / `t-wave-orin` |
| 平台 | Ubuntu 22.04.5、aarch64、L4T R36.4.3 |
| ROS | `/opt/ros/humble`，系统 Python；已有 colcon、tmux |
| 既有工作空间 | `/home/robotx/robotx_ws`，本次不改动其源码、安装和配置 |
| MAVROS | 加载既有 workspace 后为 **2.14.0**；仿真服务器为 2.16.0，不能把服务器版本写成 Orin 已安装版本 |
| Livox | 既有 workspace 中 `livox_ros_driver2` 包版本 1.0.0，Git `21445540f0d100dc86a7e6df312dd70bbdb4afdf`；已有本地修改，保留 |
| 点云格式 | 既有驱动源码包含 `timestamp: FLOAT64`，逐点绝对纳秒时间；运行时仍须检查实际消息和时钟 |
| Nav2 | 既有 ROS 环境中未发现 `nav2_bringup` / `nav2_navfn_planner`，需补齐依赖后构建 |
| 点云投影 / 地理数据 | pointcloud_to_laserscan 2.0.1；`/usr/share/GeographicLib/geoids/egm96-5.pgm` 已存在 |
| 雷达网络 | 现有配置为雷达 `192.168.1.188` → Orin `192.168.1.5`（`enP8p1s0`）；这是配置核查，未据此宣称实时收包通过 |
| 飞控接口 | 已枚举 Pixhawk6C 的两个 USB 接口；实际 MAVLink 端口、波特率和 SYSID 尚需核实 |
| 本仓库目录 | `/home/robotx/rx_simu`，分支 `codex/orin-deployment` |

`100.91.153.16` 是管理 SSH 地址，**不是 MID-360 数据接收网卡地址，也不是 FCU URL**。账号密码不写入仓库。MAVROS / 雷达 / 导航使用独立 ROS domain；仍须检查串口和雷达 UDP 端口是否被旧进程占用。

## 2. 获取源码、补依赖和构建

在电脑或服务器登录；长期前台节点可分别放入 tmux 窗口：

```bash
ssh robotx@100.91.153.16
# 首次获取；已有目录时不要再次 clone 或覆盖。
git clone --branch codex/orin-deployment --single-branch \
  https://github.com/Ericzhaoforu/rx_simu.git "$HOME/rx_simu"
cd "$HOME/rx_simu"
git status --short --branch
git rev-parse HEAD
```

后续更新先停止本仓库节点、检查本地改动，再在该分支 `git pull --ff-only`；有分叉或改动时先保存和检查，不用 `reset --hard` 清掉现场文件。

**不要运行仿真 `scripts/bootstrap.py`、`scripts/build.sh`、`scripts/env.sh`、`sim_session.py` 或任何 `validate_*.py` 控船脚本。** 它们面向服务器上的 SITL / Garden / WAM-V，有 amd64 二进制、仿真时钟、假 GPS 或场景假设。实船不需要 Gazebo、VRX、SITL 或 `livox_gz_sim`。

以下是准备部署时执行的安装命令，本次拉取源码不等于已执行安装：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan ros-humble-rosbag2 \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-yaml

source /opt/ros/humble/setup.bash
source "$HOME/robotx_ws/install/setup.bash"
ros2 pkg prefix mavros
ros2 pkg prefix livox_ros_driver2
# 首次机器尚无 rosdep 数据库时先 sudo rosdep init；已初始化则不重复。
rosdep update
cd "$HOME/rx_simu"
rosdep install --from-paths src/robotx_task1_interfaces src/usv_nav2 src/usv_sim \
  --ignore-src -r -y
colcon build --symlink-install --base-paths src \
  --packages-select robotx_task1_interfaces usv_nav2 usv_sim
source install/setup.bash
```

本流程复用 Orin 既有 MAVROS / Livox，不覆盖其 vendor 源码。记录实际版本并重新做接口验收；若必须对齐服务器 MAVROS 2.16.0，应在独立 ARM64 工作空间构建并复测，不能拷贝服务器 amd64 的 `external/deps`。没有既有 MAVROS 的新机器可安装 `ros-humble-mavros` 和 `ros-humble-mavros-extras`；缺 geoid 数据时再运行该包的 GeographicLib 安装脚本。本 Orin 已有 geoid 文件。

本阶段只构建普通实船导航所需三包。Task1 规则插件、任务许可、交通预测与真实观测接入是后续阶段，不能只启动 `mission` 就认为比赛链已就绪。

## 3. 填写实测配置和地图

```bash
cd "$HOME/rx_simu"
# 首次创建；-n 保留已有现场设置。
cp -n config/real/boat.env.example .env.boat
nano .env.boat
```

`.env.boat` 和 `runs/` 已被 Git 忽略。必须填的值：

| 变量 | 填写依据 |
|---|---|
| `USV_FCU_URL`、`USV_FCU_SYSID` | 实际 MAVLink 连接和飞控 SYSID。例如串口 URL 形如 `/dev/serial/by-id/<确认的接口>:<确认的波特率>`；不能直接套 SITL 的 localhost UDP |
| `USV_LIVOX_SETUP` / `USV_LIVOX_CONFIG` | 既有驱动的 setup 与本仓库独立配置副本 |
| `USV_LIDAR_FRAME` 和 6 个安装量 | 实际点云 frame、相对 `base_link` 的 x/y/z（m）和 roll/pitch/yaw（rad） |
| `USV_FOOTPRINT` | `base_link` 中的实测船体多边形，包含推进器和突出物；不能套 WAM-V，也不能仅凭“1.2×0.7 m”假定参考点位于中心 |
| `USV_MARK_MIN_Z/MAX_Z` | `base_stabilized` 中的障碍高度窗口；先看真实点云，确认不会过滤掉浮标和低矮障碍 |
| `USV_MAP_LAT/LON`、`USV_MAP_YAML` | 实测地图原点经纬度和地图文件；地图 x 向东、y 向北，单位 m |

`base_link` 应对应 FCU 状态所使用的机体参考点。GNSS 杆臂、罗盘方向、雷达外参和 footprint 都要相对此点定义。Livox SDK 的外参保持零、由 TF 应用一次安装变换，避免 SDK 和 TF 重复转换。

先复制雷达配置，再核对网卡、目标 IP 和 UDP 端口；不改动旧工作空间的配置：

```bash
source .env.boat
mkdir -p runs/hardware_config
cp -n "$HOME/robotx_ws/src/vendor/livox_ros_driver2/config/MID360_config.json" \
  "$USV_LIVOX_CONFIG"
ip -brief address
ip route get 192.168.1.188
ss -lunp
ls -l /dev/serial/by-id/
```

使用实测的可航行边界和禁航多边形，按 [demo_area.json 的结构](../src/usv_nav2/maps/demo_area.json) 编写 `runs/hardware_config/area.json`，再生成地图。示例地图不代表现场水域。

```bash
source /opt/ros/humble/setup.bash
source "$USV_LIVOX_SETUP"
source install/setup.bash
ros2 run usv_nav2 make_water_map \
  runs/hardware_config/area.json runs/hardware_config/water
```

`map_origin` 只把飞控报告的真实 EKF 原点平移到地图原点，**不设置飞控原点，不旋转地图，不提高 GNSS 精度**。若地图不是 ENU 朝向，先完成地图坐标转换；不能用改 frame 名或固定零变换假装对齐。飞控原点变化时停止任务并重新核验配准。

填好 `.env.boat` 后生成本船参数（默认拒绝覆盖已有文件；修改参数时先备份旧版）：

```bash
source .env.boat
python3 - <<'PY'
import json, math, os
from pathlib import Path
import yaml
root = Path(os.environ['USV_WORKSPACE'])
def number(name):
    value = float(os.environ[name])
    assert math.isfinite(value), name
    return value
hull = json.loads(os.environ['USV_FOOTPRINT'])
assert len(hull) >= 3 and all(len(p) == 2 and all(math.isfinite(float(x)) for x in p) for p in hull)
low, high = number('USV_MARK_MIN_Z'), number('USV_MARK_MAX_Z')
assert low < high
speed, yaw_rate = number('USV_MAX_SPEED'), number('USV_MAX_YAW_RATE')
assert 0 < speed <= 0.4 and 0 < yaw_rate <= 0.4
cfg = yaml.safe_load((root/'src/usv_nav2/config/nav2_params.yaml').read_text())
cfg['odom_tf']['ros__parameters']['expected_parent_frame'] = 'odom'
bridge = cfg['rover_bridge']['ros__parameters']
bridge.update(expected_parent_frame='odom', enable_control=False,
              require_task_permission=False, enable_stationkeeping=False,
              max_speed=speed, max_yaw_rate=yaw_rate)
dwb = cfg['controller_server']['ros__parameters']['FollowPath']
dwb.update(max_vel_x=speed, max_speed_xy=speed, max_vel_theta=yaw_rate)
for name in ('local_costmap', 'global_costmap'):
    cost = cfg[name][name]['ros__parameters']
    cost['footprint'] = json.dumps(hull)
    cost['footprint_padding'] = 0.2
    cost['inflation_layer']['inflation_radius'] = 1.0
cfg['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = 'odom'
lidar = {'livox_filter': {'ros__parameters': dict(
    use_sim_time=False, footprint=json.dumps(hull), body_frame='base_link',
    stabilized_frame='base_stabilized', fixed_frame='odom', input_topic='/livox/lidar',
    marking_min_height=low, marking_max_height=high, input_timeout=0.5)}}
out = root/'runs/hardware_config'
out.mkdir(parents=True, exist_ok=True)
for filename, data in [('nav2.yaml', cfg), ('livox_filter.yaml', lidar)]:
    with (out/filename).open('x') as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
print('Generated', out)
PY
```

初始速度上限 **0.2 m/s / 0.2 rad/s**、padding **0.2 m**、膨胀 **1.0 m** 是低速调试初值，尚未由本船停车/转弯试验验证。全局 Navfn、局部 DWB；目标容差保留 0.5 m / 0.25 rad，超时保留 command/odom/scan 0.5 s、state 2 s。不得通过放宽数据超时或缩小 footprint 让异常消失。

## 4. 每次启动顺序：先只读与预览

顺序为 **现场飞控检查 → MAVROS → 地图配准/安装 TF → 真实雷达 → 导航预览 → 点云过滤 → 扫描投影 → 数据检查与记录 → 现场启用控制**。所有长驻命令各占一个终端或 tmux 窗口。每个窗口先执行：

```bash
cd "$HOME/rx_simu"
source .env.boat
source /opt/ros/humble/setup.bash
source "${USV_LIVOX_SETUP:?填写驱动环境}"
source install/setup.bash
```

该配置使用 `ROS_DOMAIN_ID=62`、`ROS_LOCALHOST_ONLY=1`，ROS 节点都在 Orin 上；Tailscale 只负责 SSH。若要电脑直接运行 RViz，需要另外配置双方 DDS 网络，不把改成 `ROS_LOCALHOST_ONLY=0` 当作已经打通发现。现场先通过遥控器确认 MANUAL / HOLD 的接管和推进器停止能力；岸上阶段保持 disarm，测试推进器时由现场人员单独安排。

**A. MAVROS，只建立真实 FCU 数据链。** 确认旧 MAVROS、MAVProxy 或 router 没有独占同一串口；不启动第二个控制器。

```bash
ros2 run mavros mavros_node --ros-args \
  --params-file config/real/mavros.yaml \
  -p fcu_url:="${USV_FCU_URL:?填写真实FCU连接}" \
  -p target_system_id:="${USV_FCU_SYSID:?填写真实SYSID}"
```

另一个终端检查：

```bash
ros2 topic echo /mavros/state --once --qos-reliability best_effort
ros2 service call /mavros/vehicle_info_get mavros_msgs/srv/VehicleInfoGet '{get_all: true}'
ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull '{force_pull: false}'
ros2 param dump /mavros/param > runs/hardware_config/fcu_before.yaml
ros2 topic echo /mavros/local_position/odom --once --qos-reliability best_effort
```

确认 `connected=true`、初始 `armed=false`、真实 SYSID 和 **4.4.0 / d0356316**；`flight_custom_version` 或地面站固件信息用于核对提交。里程计必须是 `odom → base_link`，并有正确的 ENU / FLU 方向和新鲜时间。检查实际船舶类别、左右输出、正反转、RC 与 GCS 失联行为、GNSS/EKF 状态；不整包导入 SITL 参数，尤其不用仿真的 `GPS_TYPE=14` 和 GPS_INPUT 注入。

**B. 地理配准；C. 雷达安装 TF，分别运行。**

```bash
# B
ros2 run usv_sim map_origin --ros-args -p use_sim_time:=false \
  -p origin_latitude:="${USV_MAP_LAT:?填写地图纬度}" \
  -p origin_longitude:="${USV_MAP_LON:?填写地图经度}"
```

```bash
# C：已有 URDF 提供同一变换时省略，确保只有一个发布者。
ros2 launch usv_nav2 sensor_tf.launch.py \
  lidar_frame:="${USV_LIDAR_FRAME:?}" \
  x:="${USV_LIDAR_X:?}" y:="${USV_LIDAR_Y:?}" z:="${USV_LIDAR_Z:?}" \
  roll:="${USV_LIDAR_ROLL:?}" pitch:="${USV_LIDAR_PITCH:?}" yaw:="${USV_LIDAR_YAW:?}"
```

**D. 真实 MID-360 驱动。** 本命令显式选择带逐点时间的 PointCloud2，不启动驱动附带 RViz；只运行一个雷达驱动实例。

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_lidar_publisher -p use_sim_time:=false \
  -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 \
  -p frame_id:="${USV_LIDAR_FRAME:?}" \
  -p user_config_path:="${USV_LIVOX_CONFIG:?}"
```

`xfer_format=0` 和参数名已对照 [Livox 官方 ROS 2 启动文件](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/launch_ROS2/rviz_MID360_launch.py)；时间字段对照 [官方发布代码](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp) 及 Orin 当前源码。CustomMsg 不能仅靠重映射变成 PointCloud2。保证 MID 与 Orin / MAVROS 处在可比较的时间基准，检查实际 PTP/GNSS/驱动时钟处理；仅 Orin 系统 NTP 正常不能证明雷达已同步。不能用“当前时间”覆盖旧消息。

**E. 导航预览。** 此步骤建立 odom TF 和 Nav2；控制默认关闭，即使输入齐全也没有向 FCU 的 setpoint 发布者。

```bash
ros2 launch usv_nav2 navigation.launch.py \
  map:="${USV_MAP_YAML:?}" params_file:="$USV_WORKSPACE/runs/hardware_config/nav2.yaml" \
  use_sim_time:=false enable_control:=false rviz:=false
```

TF 链为 `map → odom → base_stabilized → base_link → <lidar_frame>`：第一段由 map_origin、船体两段由 odom_tf、最后一段由安装 TF/URDF 发布。MAVROS 的位置 TF 已关闭，不能再让其他定位器重复发布相同边。

**F. 船体过滤与 FCU TF 去畸变；G. pointcloud_to_laserscan，分别运行。**

```bash
# F：虽然位于 usv_sim 包，这个适配器读取真实驱动消息，不生成真值。
ros2 run usv_sim livox_filter --ros-args \
  --params-file "$USV_WORKSPACE/runs/hardware_config/livox_filter.yaml"
```

```bash
# G：必须输出到 /usv/scan_raw，之后由过滤器保留源时间并发布 /scan。
ros2 launch usv_nav2 cloud_to_scan.launch.py \
  cloud_topic:=/usv/lidar/filtered scan_topic:=/usv/scan_raw \
  target_frame:=base_stabilized use_sim_time:=false \
  min_height:="${USV_MARK_MIN_Z:?}" max_height:="${USV_MARK_MAX_Z:?}" \
  range_min:=0.3 range_max:=20.0
```

完整数据流：真实 MID → `/livox/lidar` → livox_filter（实测船体、飞控 TF、回波支持过滤）→ `/usv/lidar/filtered` → pointcloud_to_laserscan → `/usv/scan_raw` → `/scan` → Nav2 costmap。飞控 → MAVROS EKF odom → odom_tf → `/usv/odom` / TF。控制开启后才有 Nav2 → `/usv/cmd_vel` → rover_bridge → MAVROS → ArduRover → 推进器。

此处普通 Nav2 使用 LaserScan 障碍层，**没有启用 Task1 的 Voxel / 交通预测层**。过滤器还发布 marking/clearing 云，但不能将这些发布话题误当成普通参数已订阅。无返回保持未知，不能把无回波水面解释为已证明可通行；本轮仿真确认过的动态残留问题仍需实测。

## 5. 数据检查、规划预览与记录

```bash
ros2 node list
ros2 param get /mavros use_sim_time
ros2 param get /rover_bridge use_sim_time
ros2 param get /livox_filter use_sim_time
ros2 topic info /livox/lidar -v
ros2 topic echo /livox/lidar --once --field fields --qos-reliability best_effort
ros2 topic info /scan -v
ros2 topic echo /scan --once --field header --qos-reliability best_effort
ros2 topic echo /usv/control_status --once
ros2 topic echo /usv/map_alignment --once --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_link
```

`tf2_echo` / `topic hz` 等持续输出命令用 Ctrl-C 结束。另测 `/livox/lidar`、`/scan` 和 `/usv/odom` 的频率；10 Hz 只是目标值，还要检查采集源龄、最大间隔和 TF 查找失败。`timestamp` 必须是 FLOAT64 绝对 ns，位于帧 header 至其后约 0.12 s 范围；过滤器会拒绝不兼容格式。所有实机节点 `use_sim_time=false`，不启动 `/clock`。预览状态在 disarm / 非 GUIDED 时为零属于门控正常，规划输出和路径可单独检查。

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
# 有本地桌面或已配置显示时才运行 RViz；Fixed Frame 设 map。
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix usv_nav2)/share/usv_nav2/rviz/navigation.rviz"
```

四个 lifecycle 节点应为 active。在 RViz 对比船体朝向、实测标志物、雷达障碍与地图；推动/转动船时，位置和方向变化必须一致。全 NaN 扫描、明显时钟漂移或地图错位不能进入控制阶段。

选取实测地图中的安全目标，将坐标与航向填入下列变量，生成通用目标文件。不是把当前位置硬编码为原点，也不默认“前进 5 m 就等于 map x=5”。

```bash
export USV_GOAL_X='' USV_GOAL_Y='' USV_GOAL_YAW=''
# 填好上面三项后执行；yaw 为 ENU 弧度，0 朝东、pi/2 朝北。
python3 - <<'PY'
import json, math, os
from pathlib import Path
x,y,a = [float(os.environ['USV_GOAL_'+k]) for k in ('X','Y','YAW')]
assert all(map(math.isfinite, (x,y,a)))
pose = dict(header=dict(frame_id='map'), pose=dict(
    position=dict(x=x,y=y,z=0.0), orientation=dict(x=0.0,y=0.0,z=math.sin(a/2),w=math.cos(a/2))))
root = Path(os.environ['USV_WORKSPACE'])/'runs/hardware_config'
(root/'goal.json').write_text(json.dumps(dict(pose=pose)))
(root/'plan.json').write_text(json.dumps(dict(goal=pose,planner_id='GridBased',use_start=False)))
PY
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "$(cat runs/hardware_config/plan.json)" --feedback
```

规划 action 只请求路径，不执行运动。检查禁航区、完整 footprint、膨胀、局部障碍和目标朝向。移动测试用实体模拟障碍后再移开，确认原位置确实清除；`observation_persistence=0` 不会使障碍格自动到期，禁止通过反复清图掩盖失败。

记录终端先启动，再开始任何水试。每次使用独立目录，不覆盖既有记录：

```bash
USV_RUN="$USV_WORKSPACE/runs/hardware_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$USV_RUN"
git rev-parse HEAD > "$USV_RUN/revision.txt"
cp -r runs/hardware_config "$USV_RUN/config"
ros2 bag record -o "$USV_RUN/bag" \
  /mavros/state /mavros/local_position/odom /mavros/global_position/global \
  /mavros/imu/data /mavros/rc/out /mavros/setpoint_raw/local \
  /mavros/setpoint_raw/target_global /usv/odom /usv/cmd_vel \
  /usv/setpoint_preview /usv/control_status /usv/map_alignment \
  /livox/lidar /usv/lidar/filtered /scan /tf /tf_static \
  /map /plan /local_costmap/costmap /global_costmap/costmap
```

目标反馈话题未由 FCU 输出时不能用空 bag 证明目标不变，应检查 MAVLink 消息流。记录时同步保存现场视频、独立位置测量、风流情况、操作时刻和急停方式；MAVROS / EKF 自身位置不是独立真值。RC 输出或 ESC 指令也不等于实测牛顿推力。

## 6. 水试：分阶段启用，不跳级

| 阶段 | 操作 | 通过条件 / 记录 |
|---|---|---|
| 岸上接线与静态感知 | disarm；看版本、状态、点云、TF、地图 | 无重复控制器/TF；时间正确；实体方位和尺寸匹配；船体回波被正确过滤 |
| 现场人工操船 | 先 MANUAL / HOLD，单侧/双侧、正反转、归零 | 左右和 yaw 符号正确；测停车时间/距离、转弯半径；确认 RC 接管和断链行为 |
| 独立原生 LOITER | 开阔水域，Nav2 未执行任务；现场进入 LOITER | 三次、每次至少 120 s；记录固定目标、实际轨迹、最大/RMS 偏差和推力；不每秒重设定点 |
| 普通 Nav2 单目标 | 低速，先 3–5 m 合适目标，再延长 | 至少三次；位置 ≤0.5 m、航向 ≤0.25 rad 是待验目标；独立测量，记录到点后至少 10 s 漂移 |
| 多途经点、取消、不可达 | 新鲜目标；主动取消与封闭目标 | 正确退出；分别记录零指令、推进器响应、实际停止，不能互相替代 |
| 静态障碍 | 已测停车余量基础上绕行 | 无接触，记录最小净空、低障碍漏检、地图清除；失败先复现排查 |
| 动态障碍与输入故障 | 前述通过后，现场有人接管；逐项中断 | 测 scan/odom/command 中断发零 ≤0.6 s、state ≤2.1 s；进程退出另测飞控行为，不假定同一时延 |
| Task1 / 风浪 / GNSS 组合 | 后续单独验收 | 当前不把开放水面动态清除、真实灯态/UAV、原生保持与任务的实机结合写成已通过 |

### 独立 LOITER

优先由现场遥控器切换；现场按正常 arming checks 解锁后才会主动定点。需从 MAVROS 进行模式请求时，先取消全部导航动作，再请求并检查 `/mavros/state` 实际变为 LOITER：

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
ros2 service call /navigate_through_poses/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'LOITER'}"
ros2 topic echo /mavros/state --once --qos-reliability best_effort
```

无活动 Nav2 节点时省略前两条。`mode_sent=true` 只说明请求已发送，实际状态确认才算模式切换。实船 `LOIT_RADIUS/LOIT_TYPE/LOIT_SPEED_GAIN` 先记录现值并依据水试调整，不能默认复制 WAM-V 的 0.3 m 半径；原生 LOITER 会改变船头方向，其转向/纠偏空间必须足够。

HOLD 不补偿风流，GUIDED 零速度也不是定点。当前桥没有通用 RTK/EKF 质量门控；Fixed 不能排除缓慢一致性偏差。测试人员必须监视真实定位质量和漂移，不能将桥的“数据新鲜”当作“位置正确”。GPS 失锁后的 HOLD 与重新定点是不同操作，不默认自动恢复。

### 普通 Nav2 控制

现场确认可控、记录运行、输入和路径检查通过后：取消旧动作，切 HOLD 并按现场程序 disarm，Ctrl-C 停止 **E 导航预览**，确认旧 rover_bridge 已退出，再以相同参数启动控制实例；不同时保留两个桥。

```bash
ros2 launch usv_nav2 navigation.launch.py \
  map:="$USV_MAP_YAML" params_file:="$USV_WORKSPACE/runs/hardware_config/nav2.yaml" \
  use_sim_time:=false enable_control:=true rviz:=false
```

`enable_control` 在节点创建时决定是否建立输出发布者；当前实现没有动态重配置回调，**不能靠 `ros2 param set` 就认为已切换控制**。重新启动后再次确认生命周期、TF、扫描与状态。

以下两条由现场操作者在水试条件满足时执行；本次 SSH 拉取代码不执行解锁或模式切换：

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
ros2 topic echo /mavros/state --once --qos-reliability best_effort
# 确认 armed=true、GUIDED 且现场允许后，发送刚重新核验的目标。
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "$(cat runs/hardware_config/goal.json)" --feedback
```

不使用 force-arm，不关闭 arming checks。恢复 GUIDED 时取消旧导航并重新生成目标/路径；自动 Task1 的 `require_task_permission`、`enable_stationkeeping`、252 号专用心跳属于另一套完整模式合同，**本实机普通导航配置均关闭**。仅手工打开 `enable_stationkeeping` 会缺少新鲜许可和飞控失联配置，不能照搬仿真 `--task1-loiter`。

普通模式下 MAVROS 还活着时，其 GCS 心跳可能掩盖导航桥退出；实船必须分别测桥退出、MAVROS 退出和整机断链。仿真可选模式的 252 号心跳 / FS 参数不能整包覆盖现有遥控与地面站保护。

## 7. 取消、停止和关机

先结束动作并由现场确认安全模式/推进器状态，再停数据链。紧急情况优先用现场 RC / 硬件接管，不能只依赖 SSH：

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
ros2 service call /navigate_through_poses/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'HOLD'}"
ros2 topic echo /mavros/state --once --qos-reliability best_effort
# 在现场确认可 disarm 时执行，随后检查实际反馈。
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: false}'
```

若有风流需要继续定点，现场明确选择 LOITER 并保持必要定位/数据链，不能把上述 HOLD 流程理解为保持位置。常规关闭顺序：导航/桥 → 点云投影/过滤 → 雷达驱动 → 安装 TF/配准 → MAVROS；记录器最后 Ctrl-C 正常结束，检查 `ros2 bag info <bag目录>` 和日志后再关机。不使用 `killall` 清理整台 Orin 的其他任务。

## 8. 本次交付与尚未执行项

本次工作包括 SSH 环境核查、独立仓库获取、部署文档和配置模板，以及不连接设备的源码基础检查。**没有将源码拉取、单元测试或离线参数解析写成 ARM64 全栈构建/实机闭环通过。** Nav2 依赖安装、配置中的硬件测量值、MAVROS 2.14 实时接口、驱动时钟/数据、真正的模式切换、解锁和水试均按上文逐项验收。

每次现场试验保存：仓库 SHA、实际驱动/MAVROS 版本、飞控完整参数、地图和外参、bag、视频、独立位置证据、退出状态、失败原因及修复后的新运行目录。现有仿真证据及其限制见 [MID-360 / LOITER 验证报告](experiments/20261104_simulation_verification.md)。
