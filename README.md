# 无人船导航验证

在 Ubuntu 上使用 **VRX WAM-V + ArduRover SITL + ROS 2 Humble Nav2** 验证导航、避障和异常响应。目标固件固定为 **ArduRover V4.4.0 / `d0356316eb0fa34ed1c9f39f7b059b9ae7ba4664`**。

已实现独立的 VRX 场景、固定版本 SITL、MAVROS 控制链、飞控 EKF 定位和 LiDAR 避障输入。WAM-V 配置按实际碰撞几何生成；原包 1.2 m × 0.7 m 参数保留。验证结果、已知限制与失败记录见 [验证报告](docs/validation_report.md)。

## 启动

本服务器的 Ubuntu 22.04、ROS 2 Humble、Garden 7.9 与既有 VRX 安装是环境基础。新增依赖安装在仓库的 `external/`，不要求 sudo，不覆盖既有工程。详细依赖和排障见 [运行说明](docs/runbook.md)。

```bash
cd /home/zjy/usv_nav
python3 scripts/bootstrap.py
source scripts/env.sh
# 前台保持运行；默认只检查传感器和规划，控制关闭。
python3 scripts/sim_session.py start --name inspection --navigation
```

另一个终端：

```bash
cd /home/zjy/usv_nav
source scripts/env.sh
python3 scripts/validate.py plan --label preview
# 启动器校验固定 SITL 源码和本机端口；确认当前会话后启用控制。
python3 scripts/sim_session.py navigation --replace --control
python3 scripts/validate.py goal --goals '[[5,0,0]]' --label goal
python3 scripts/sim_session.py stop
```

每次运行的场景、参数、节点日志、飞控日志和验证 JSONL 保存在 `runs/<时间>_<名称>/`。`runs/latest.txt` 指向当前运行目录。仿真与 Nav2 使用 Gazebo 时钟，控制超时使用单调墙钟。

## 项目文件

| 路径 | 用途 |
|---|---|
| [AGENTS.md](AGENTS.md) | 工作约定，包括遇到问题先自主 debug、修复后回归验证 |
| [docs/validation_plan.md](docs/validation_plan.md) | 当前 VRX、ArduRover 和 Nav2 验证计划 |
| [docs/environment_baseline.md](docs/environment_baseline.md) | 服务器调查结果、版本和待补依赖 |
| [docs/navigation_handoff.md](docs/navigation_handoff.md) | 原始交接附件，原文保存 |
| [docs/import_manifest.json](docs/import_manifest.json) | 附件及 24 个导入文件的 SHA256 |
| [src/usv_nav2](src/usv_nav2) | ROS 2 导航包、参数、地图与现有测试 |

交接附件中的部分 GPS 参数来自较新固件；本项目以固定提交源码和现行验证计划为准。现阶段先用 WAM-V 检验软件链路，模型与最终实船的尺寸、水动力和推进器参数需要后续单独适配。

## 基础测试

在仓库根目录运行，无需启动 ROS 或仿真：

```bash
(cd src/usv_nav2 && python3 -m unittest discover -s test -v)
git diff --check
```

首次导入的 20 项测试、12 个 Python 文件语法检查及附件字节校验均通过。后续新增检查与真实闭环结果分别记录，不以单元测试替代仿真验收。

包内 [README](src/usv_nav2/README.md) 介绍导航接口。WAM-V 的独立参数由 `scripts/prepare_navigation.py` 生成，不使用包内实船 footprint 验收。

## 代码管理

本目录是独立 Git 仓库。导航源码、仿真适配、参数、测试和报告均在此管理；第三方依赖使用独立目录并记录准确提交，构建产物及大型仿真记录不进入 Git。

```bash
git status --short --branch
git log --oneline -5
git diff
```

当前没有配置远端。首次导入保留附件字节内容和包内 MIT 许可证，后续修改通过提交追踪。
