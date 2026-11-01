# Task 1 实际验证记录

由 `scripts/report_task1.py` 根据保留的独立裁判结果生成。完整历史（包括调试失败）见 [results.json](results.json)。
正常任务与故障测试分开判定；程序报告 COMPLETE 不等于独立验收通过。尚未运行的组合明确列出。

| 用例 | 判定 | 位置 / 航向误差 | 最小净空 | 证据 |
|---|---|---|---|---|
| s1_c0_none | 通过 | 0.366 m / 0.050 rad | 2.855 m | [20261030_030121_task1_s1_c0_none](../../runs/20261030_030121_task1_s1_c0_none/task1_result.json) |
| s1_c1_none | 未验证 | — | — | — |
| s1_c2_none | 失败 | 0.373 m / 0.046 rad | 1.204 m | [20261030_033126_task1_s1_c2_none](../../runs/20261030_033126_task1_s1_c2_none/task1_result.json) |
| s1_c0_cross | 未验证 | — | — | — |
| s1_c0_block | 未验证 | — | — | — |
| s1_c2_cross | 失败 | 0.397 m / 0.125 rad | 0.673 m | [20261030_163410_task1_s1_c2_cross](../../runs/20261030_163410_task1_s1_c2_cross/task1_result.json) |
| s2_c0_none | 未验证 | — | — | — |
| s2_c1_none | 未验证 | — | — | — |
| s2_c2_none | 未验证 | — | — | — |
| s2_c0_cross | 未验证 | — | — | — |
| s2_c0_block | 未验证 | — | — | — |
| s2_c2_cross | 未验证 | — | — | — |
| s3_c0_none | 未验证 | — | — | — |
| s3_c1_none | 未验证 | — | — | — |
| s3_c2_none | 未验证 | — | — | — |
| s3_c0_cross | 未验证 | — | — | — |
| s3_c0_block | 未验证 | — | — | — |
| s3_c2_cross | 未验证 | — | — | — |
| fault_observation_gap | 未验证 | — | — | — |
| fault_reordered_observation | 未验证 | — | — | — |
| fault_mission_exit | 未验证 | — | — | — |
| fault_cancel | 未验证 | — | — | — |
| fault_role_change | 未验证 | — | — | — |
| fault_no_path | 未验证 | — | — | — |

模型和感知检查：

- 相机灯态/实体：通过；[最近证据](../../runs/20261030_024116_task1_model_fixed/model_validation.json)。

- LiDAR 三层回波：通过；[最近证据](../../runs/20261030_024116_task1_model_fixed/lidar_validation.json)。

固定固件、规则及范围见 [implementation.md](implementation.md)，启动命令见 [runbook.md](runbook.md)，根因与修复见 [debug_log.md](debug_log.md)。
WAM-V、固定系泊模型和本地报告不能代表正式 UAV 协同、RoboCommand 接入或实船比赛得分。
