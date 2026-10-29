# Task 1 调试记录

## 2026-10-30：模型和 ROS 启动

- 原始资料：已保存 2026-10-23 手册和两份官方制作说明；SHA256 见 sources.json。
- `20261030_023841_task1_model_check`：场景语义状态变化正常，但摄像头彩色像素为零；网格上的黑色遮光帽也显示为白色。渲染 Visual 没有直接 Material，需要先创建并绑定材质再更新；初版 DAE 的导入材质覆盖了 SDF 外观。改用无内置材质的 STL，并在渲染线程绑定独立材质。
- `20261030_024116_task1_model_fixed`：`scripts/check_task1_model.py` 的实际相机检查五种灯态全部通过。各状态有效画面 19/17/12/18/15 帧；红绿闪烁、闪蓝均同时捕获亮灭状态。原始截图和 MP4 在运行目录。本检查尚不代表频率精度、侧面可见性或完整任务验收。
- ROS 节点初次启动失败：任务程序用了 Node 的保留名称 `handle`、`context`。更名为 `follow_handle`、`publish_context`；重新启动后 Nav2 生命周期、Task1 规划插件和 DWB 规则插件均已激活。
- Task1 生成地图时报已有输出：默认生成器先创建通用地图，Task1 适配器在该独立运行目录用显式 overwrite 替换为 Task1 边界；原包地图和原场景不变。

这些调试运行保留失败日志；最终验收使用独立冷启动和完整运行源代码快照。

## 首轮闭环：出口接近点停滞

- `20261030_024116_task1_model_fixed` 完成入口顺时针绕圈并穿越红绿浮标，无碰撞，独立裁判最小净空 3.241 m。但在出口接近点反复报告 Failed to make progress，主动取消以保留复现；本轮整体失败。
- Nav2 1.1.20 的 [RotateToGoal 源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp) 将位置窗口永久锁存；船体在最终转向时漂出 0.4 m 窗口，非锁存 goal checker 仍要求位置，而 critic 继续拒绝平移。新增 Task1 专用子类，在漂出窗口时重置锁存，保留上游转向与减速逻辑及原到点阈值。
- SE(2) 搜索的末端连接还允许 0.65 m 范围内的侧移/倒退连接。修复为该距离内必须前向且方向一致，否则搜索至半个位置栅格以内才允许末端离散误差。增加反向末端连接回归。
- 30 秒传感器抽查：entry、buoy_01、buoy_02 在原始点云、船体过滤点云、guarded LaserScan 三层均有真实回波；完整逐帧结果见该运行目录 lidar_validation.json。扫描最大距离 20 m，远处模型不保证进入扫描；已知浮标图层与物理回波分开验证。
