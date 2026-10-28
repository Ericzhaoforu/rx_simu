# 无人船导航验证

在 Ubuntu 上使用 **VRX WAM-V + ArduRover SITL + ROS 2 Humble Nav2** 验证导航、避障和异常响应。目标固件固定为 **ArduRover V4.4.0 / `d0356316eb0fa34ed1c9f39f7b059b9ae7ba4664`**。

当前已导入附件中的 `usv_nav2` 源码并建立 Git 管理。VRX 与 ArduRover 的连接、WAM-V 参数配置及闭环场景仍待按计划实现；原包参数对应 1.2 m × 0.7 m 船体，不能直接作为 WAM-V 的验收配置。

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

2026-10-29 在本服务器运行：20 项现有单元测试通过，12 个 Python 文件通过 Python 3.10 语法解析；24 个源码文件及交接文档的 SHA256 与附件一致。测试中的 ROS 消息使用替身，尚未验证真实 DDS、节点启动或仿真闭环。

ROS 构建、依赖安装、传感器和控制联调按 [验证计划](docs/validation_plan.md) 分阶段执行。包内 [README](src/usv_nav2/README.md) 介绍已有导航接口；其中示例地图和实船初始参数尚未适配 VRX。

## 代码管理

本目录是独立 Git 仓库。导航源码、仿真适配、参数、测试和报告均在此管理；第三方依赖使用独立目录并记录准确提交，构建产物及大型仿真记录不进入 Git。

```bash
git status --short --branch
git log --oneline -5
git diff
```

当前没有配置远端。首次导入保留附件字节内容和包内 MIT 许可证，后续修改通过提交追踪。
