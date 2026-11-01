# Task 1 复现流程

工作目录 `/home/zjy/usv_nav`。先读 `implementation.md` 的规则边界和 `validation.md` 的实际通过/失败记录。首次环境安装、固定固件及原始导航流程沿用仓库主 README。

## 构建与检查

```bash
source scripts/env.sh
colcon build --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
cmake -S tools/task1_gazebo -B build/task1_gazebo -DCMAKE_BUILD_TYPE=Release
cmake --build build/task1_gazebo -j4
(cd src/usv_nav2 && python3 -m unittest discover -s test -v)
python3 -m unittest discover -s src/robotx_task1/test -v
ROS_DOMAIN_ID=43 python3 -m unittest discover -s src/robotx_task1/test_ros -v
build/robotx_task1_nav/test_rules
git diff --check
```

运行过程中不要覆盖已加载的共享库。运行文件由 `sim_session.py` 创建在忽略目录 runs 中；每次保存本仓库源码快照、固件/插件哈希、生成世界与参数，不覆盖既有工程。

随时用 `python3 scripts/task1_status.py` 查看当前进程是否存活、最近的物理帧及记录年龄、任务阶段和已结束的批测结果。`managed_session: false` 表示当前没有本仓库管理的仿真；旧的 COMPLETE 记录不会被解释为仍在运行。

## 场景及任务

只看模型，控制保持关闭：

```bash
source scripts/env.sh
python3 scripts/sim_session.py start --task1 --navigation --name task1_preview
```

另一个已 source 的终端运行 `python3 scripts/check_task1_model.py` 检查真实相机五种灯态、1 s 亮/1 s 暗、OFF 实体和侧视遮光；`python3 scripts/check_task1_lidar.py` 记录原始点云、过滤点云和扫描回波。模型检查要求 SITL 未解锁，会改变测试浮标的灯态。结束用 `python3 scripts/sim_session.py stop`。

完整任务由以下命令执行正常解锁、任务开始、证据记录、10 s 末端漂移记录和清理：

```bash
python3 scripts/run_task1_suite.py --seed 1 --changes 2 --record-maps
```

`--group all --record-maps` 共 18 项：3 种固定布局 × 0/1/2 次语义变化，以及各布局的横穿、临时阻挡、横穿加两次灯变。seed 3 含一个 OFF 实体。`--group dynamic --seed 1 --changes 0 --dynamic cross` 可单独选例。无匹配时脚本报错，失败运行保留，结果按独立裁判判定，不以进程退出码代替任务成功。

人工观察时，可先 `sim_session.py start --task1 --navigation --control ...`，再运行 `validate_task1.py`；控制开启只面向脚本核验过的本次固定版本 SITL。不要同时运行其他速度控制器。

异常场景分别冷启动：

```bash
python3 scripts/run_task1_suite.py --fault observation_gap
python3 scripts/run_task1_suite.py --fault reordered_observation
python3 scripts/run_task1_suite.py --fault mission_exit
python3 scripts/run_task1_suite.py --fault cancel
python3 scripts/run_task1_suite.py --fault role_change
python3 scripts/run_task1_suite.py --fault no_path
python3 scripts/run_task1_suite.py --fault late_change
```

异常测试的“通过”表示按预期停车/拒绝任务，和正常 Task1 完成分开存储。重放观测保留原时间戳。任务进程退出时，飞控、MAVROS 和物理仿真继续运行。

## 数据和图像

每次目录包含 task1_mission.jsonl（事件）、task1_judge.jsonl（独立真值、场景、接触）、task1.jsonl（指令、EKF、推力反馈、真值）、task1_result.json（独立判定）、task1_execution.json（动作执行）、两个实际相机 MP4。推进器力由 Gazebo Thruster 反馈角速度和固定配置反算，详见 thrust_observation.json，不是实体测力计或轴动力学模型。

启动时选择 `--record-maps` 才能生成地图回放：

```bash
python3 scripts/plot_task1.py runs/实际运行目录 --animate
```

visualization 中提供参考路径/真值/EKF 航迹、期望末端与误差、实际静态/全局/局部地图、MP4 和 GIF。参考是空间路径，不是带时序的速度轨迹；GIF 是全视频节选。所有浮标实体持续绘制；空心灯标为闪烁暗相位，深灰为语义 OFF。

`--animate` 同时将真实 Gazebo 总览相机转为便于播放的 H.264 `task1_vrx.mp4`；`task1_replay.mp4` 是按记录数据绘制的 2.5 倍速地图回放。真实相机按收到的帧顺序编码为 5 FPS，接收帧率可能低于配置值，因此不能用视频播放时长测量停车或灯变延迟；以画面内仿真时标、较新运行的 `task1_video.json` 源时标和原始日志为准。图像与视频的来源及校验值写入 `task1_manifest.json`。

`python3 scripts/analyze_task1_maps.py runs/实际运行目录 --buoys` 检查每张全局地图持续保留全部已知浮标；不加 `--buoys` 检查动态障碍的标记和离开后清除。已知浮标占据层来自本轮允许的已知观测，不能代替 `check_task1_lidar.py` 的真实传感器回波检查。

## 已知观察接口与规则规划

`/task1/observations` 为 BuoyArray：map 坐标、源时间戳、完整十个 ID、碰撞半径、语义状态、revision、survey_complete。已知 UAV 替身从 Gazebo 场景取得这些信息；没有读取船体真值定位。将来接视觉时保持此契约和来源时间，不把旧观测改成新时间戳。

任务机先检查全部调查记录，再依次接近入口、顺时针完整绕圈、合法穿越、逆时针完整绕出口。全局规划含位置、航向和红/绿通过标志；使用权重 1.4 的 A*、0.4 m 状态格、36 航向、0.6 m 前向运动边、逐 0.1 m 碰撞/规则检查、3 s 搜索预算。穿越段仅生成可前向跟踪的曲线，不生成原地转向段；错误侧正横边界留 1 m 纵向余量，判断更近的遮挡浮标时留 0.3 m 余量。独立裁判仍按原始正横规则计量。局部 DWB 增加硬性规则 critic 和可恢复的末端转向 critic。动态障碍由物理 LiDAR 感知，未向规划器发布障碍物真值轨迹。

语义变化时版本作废并取消动作；运动许可为 0.5 s 租约。EKF 平动低于 0.04 m/s、角速低于 0.04 rad/s，持续 1.5 s 后才重新规划。该余量来自实测 EKF 与真值速度的短时差异。独立裁判另外用真值低于 0.05 m/s / 0.05 rad/s、持续 1 s 检查停车；发零、推进器反馈、停车距离和仿真帧推进分别报告。闪烁暗相位不增加版本，也不触发重规划。

入口/出口身份中途重分配按不支持场景中止；无合法路径保持停车重试，整场 1200 仿真秒超时。没有新增 RTK/EKF 质量策略或持续定点保持。新模型用固定系泊和保守碰撞外廓，适合本轮船端规则测试，尚不代表波浪中的浮标运动或真实相机识别精度。

## 本轮实际参数与诊断

每次运行的 `generated/nav2_wamv.yaml`、`generated/task1_scenario.json` 和 `rover.parm` 保存该次参数。Nav2 启动文件将 `use_sim_time` 和 `enable_control` 作为显式启动覆盖项；因此不能只根据生成参数模板中的默认 `false` 判断运行状态。可用 `ros2 param get /controller_server use_sim_time` 或只读的 `python3 scripts/audit_graph.py` 检查实际值，后者同时记录 QoS、源时间戳和 TF 边。

| 项目 | 本轮数值 |
|---|---|
| DWB 频率、预测窗口 | 10 Hz、3 s |
| 前进、转向上限 | 0.3 m/s、0.4 rad/s；不规划侧向速度 |
| 平动、角加速度及减速度绝对值 | 0.2 m/s²、0.4 rad/s² |
| DWB 速度采样 | 前向 15、侧向 1、角速度 25 |
| 导航到点窗口 | 0.4 m、0.2 rad；独立真值验收 0.5 m、0.25 rad |
| WAM-V 碰撞外廓 | 约 5.371 m × 2.537 m，使用包含推进器的实际凸包；另加 0.5 m padding |
| 膨胀半径、衰减系数 | 4.5 m、1.0 |
| 全局地图 | 100 m × 100 m 水域，0.2 m 栅格，2 Hz 更新 |
| 局部地图 | 30 m × 30 m 滚动窗口，0.05 m 栅格，5 Hz 更新 |
| LaserScan 投影 | `pointcloud_to_laserscan`，`base_stabilized`，高度 0.25–3 m，角分辨率 0.5°，距离 0.3–20 m |
| 障碍标记 / 射线清除 | 15 m / 20 m；已知浮标层持续保留 OFF 实体 |
| 桥输入超时 | 指令、odom、scan 各 0.5 s，飞控 state 2 s；桥发布 20 Hz |

任务机的停车、绕圈和搜索设置见上一节。`config/task1/defaults.json` 同时记录模型依据和当前任务默认值；现阶段仅边界与圈半径由场景生成器读取，任务机其他值以其 ROS 参数声明为准，搜索离散参数在 `rules.hpp` 中。修改这些值时同时检查场景、任务机和独立裁判的一致性，不能只改记录文件便认为运行配置已更新。

规划耗时按每次事件的单调墙钟记录在 `results.json` 的 `execution_metrics`，灯变到重新行驶的暂停则按仿真时钟记录。搜索超过 3 s 可以安全停车后重试；最终到达并不表示每次搜索都在预算内成功。
