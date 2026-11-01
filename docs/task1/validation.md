# Task 1 实际验证记录

由 `scripts/report_task1.py` 根据保留的独立裁判结果生成。完整历史（包括调试失败）见 [results.json](results.json)。
每项展示最近一次已完成的尝试，可能来自较早提交；运行目录的 provenance.json 与源码快照标识当时版本。新修改不能继承旧运行的通过结论。
正常任务与故障测试分开判定；程序报告 COMPLETE 不等于独立验收通过。尚未运行的组合明确列出。

| 用例 | 判定 | 位置 / 航向误差 | 最小净空 | 证据 |
|---|---|---|---|---|
| s1_c0_none | 通过 | 0.391 m / 0.039 rad | 1.055 m | [20261101_180105_task1_s1_c0_none](../../runs/20261101_180105_task1_s1_c0_none/task1_result.json) |
| s1_c1_none | 通过 | 0.376 m / 0.053 rad | 1.104 m | [20261101_181046_task1_s1_c1_none](../../runs/20261101_181046_task1_s1_c1_none/task1_result.json) |
| s1_c2_none | 通过 | 0.378 m / 0.049 rad | 1.021 m | [20261101_182043_task1_s1_c2_none](../../runs/20261101_182043_task1_s1_c2_none/task1_result.json) |
| s1_c0_cross | 未验证 | — | — | — |
| s1_c0_block | 未验证 | — | — | — |
| s1_c2_cross | 失败 | 0.397 m / 0.125 rad | 0.673 m | [20261030_163410_task1_s1_c2_cross](../../runs/20261030_163410_task1_s1_c2_cross/task1_result.json) |
| s2_c0_none | 通过 | 0.374 m / 0.071 rad | 1.188 m | [20261101_183035_task1_s2_c0_none](../../runs/20261101_183035_task1_s2_c0_none/task1_result.json) |
| s2_c1_none | 通过 | 0.366 m / 0.054 rad | 1.198 m | [20261101_184021_task1_s2_c1_none](../../runs/20261101_184021_task1_s2_c1_none/task1_result.json) |
| s2_c2_none | 通过 | 0.373 m / 0.051 rad | 1.216 m | [20261101_185028_task1_s2_c2_none](../../runs/20261101_185028_task1_s2_c2_none/task1_result.json) |
| s2_c0_cross | 未验证 | — | — | — |
| s2_c0_block | 未验证 | — | — | — |
| s2_c2_cross | 未验证 | — | — | — |
| s3_c0_none | 通过 | 0.382 m / 0.049 rad | 1.256 m | [20261101_190020_task1_s3_c0_none](../../runs/20261101_190020_task1_s3_c0_none/task1_result.json) |
| s3_c1_none | 通过 | 0.375 m / 0.060 rad | 1.388 m | [20261101_191009_task1_s3_c1_none](../../runs/20261101_191009_task1_s3_c1_none/task1_result.json) |
| s3_c2_none | 通过 | 0.378 m / 0.041 rad | 1.105 m | [20261101_192014_task1_s3_c2_none](../../runs/20261101_192014_task1_s3_c2_none/task1_result.json) |
| s3_c0_cross | 未验证 | — | — | — |
| s3_c0_block | 未验证 | — | — | — |
| s3_c2_cross | 未验证 | — | — | — |
| fault_observation_gap | 未验证 | — | — | — |
| fault_reordered_observation | 未验证 | — | — | — |
| fault_mission_exit | 未验证 | — | — | — |
| fault_cancel | 未验证 | — | — | — |
| fault_role_change | 未验证 | — | — | — |
| fault_no_path | 未验证 | — | — | — |
| fault_late_change | 未验证 | — | — | — |

模型和感知检查：

- 相机灯态/实体：通过；[最近证据](../../runs/20261030_024116_task1_model_fixed/model_validation.json)。

- LiDAR 三层回波：通过；[最近证据](../../runs/20261101_190020_task1_s3_c0_none/lidar_validation.json)。

固定固件、规则及范围见 [implementation.md](implementation.md)，启动命令见 [runbook.md](runbook.md)，根因与修复见 [debug_log.md](debug_log.md)。
WAM-V、固定系泊模型和本地报告不能代表正式 UAV 协同、RoboCommand 接入或实船比赛得分。
