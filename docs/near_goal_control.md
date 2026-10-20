# 0.5 m 倒车、0.2 m 提前对齐与保持朝向平移

2026-10-20 修改。适用于 `useFinalGoalPose=true`、`/goal_pose` 最终位姿导航。
本次已经编译，尚未真机复测；不是实机精度验收。

## 行为

- 距离使用原始 `/goal_pose` 的 map XY 目标与机器人当前位置。
- `twoWayDrive=true` 且距离 ≤0.50 m 时允许倒车，距离 >0.50 m 时禁止负 linear.x。
- 距离 ≤0.20 m 进入 `align`，先停止平移，调整到最终目标朝向。
- yaw 误差 ≤10° 后进入 `translate`，保持最终目标朝向，按机体系目标误差进行前进、
  后退或侧移，持续用角速度反馈修正朝向。不是把机头重新朝向 waypoint。
- 平移期间 yaw 误差 >20° 重新对齐；10°/20° 是状态迟滞，不改变最终验收标准。
- 进入近目标控制后，位置轻微漂出 0.20 m 不立即退回路径跟踪；
  超过 0.30 m 连续三个新定位帧才退出，等待新局部路径。没有旧的 0.10～0.15 m 停车带。
- 最终成功仍必须同时满足：位置 ≤0.10 m、yaw 误差 ≤20°、原有停稳及连续三帧确认。
  如成功后、下一目标发出前再次超差，会撤销 reached 并继续修正。

近目标平移速度上限 0.08 m/s，侧向上限 0.05 m/s，角速度上限 0.15 rad/s。
速度随位置误差缩小，保留现有安全门与 SDK 端限速。

## 路径与避障

localPlanner 新增原始目标订阅，在 0.50 m 内用原始目标进行最后接近。
最终位姿导航不再执行身后 waypoint 距离清零、heading freeze；0.50 m 内 follower 可倒车，
0.50 m 外保留可跟踪路径供 follower 转向接近，仍禁止负 linear.x。
旧 `/goal_point` 位置目标模式保留原分支。
0.30 m 内生成到原始目标的直线路径，用原有地形/激光、边界和附加障碍点检查扫掠范围。
扫掠半径为机器人 footprint 的外接圆半径加 0.03 m，兼顾旋转占用空间。
该检查较保守，狭窄位置可能拒绝平移；不能为通过测试关闭它。

新增 `/local_planner/final_path`（nav_msgs/Path，map 坐标）：
两个端点表示已检查的当前位置到原始目标的通道，末端四元数为原始目标 yaw；
空路径表示未授权。follower 检查目标一致、唯一发布者、消息新鲜、路径生成后位移不超过
余量，才在 align/translate 输出动作。普通多点曲线路径不能冒充这个许可。
没有新鲜且可用的通道时 reason=`final_corridor_blocked_or_stale`，速度归零；
通道恢复后自动继续，不越障或绕过人工锁停。

FAR 本身未修改，0.4 m 收敛仍存在。此次改善的是 0.50 m 内的原始目标接管，
不是完整重写远距离导航；FAR 在距原始目标超过 0.50 m 就结束时，仍可能缺少可用的最终接近交接。
原始目标真实不可达时仍会停下等待，巡检脚本按现有 180 秒超时终止，不会跳点或报成功。
没有新增未知空间、悬崖/落差或机器人足步级碰撞模型。

## 启动与检查

先结束旧导航会话，再按现有地图、定位和低速启动流程启动。
无需增加 `reverseMaxGoalDistance` 或 `inspection_mode` 参数，当前仍没有这两个启动参数。
0.50/0.20 为 planner 与 follower 共用源码常量 `src/finalGoalPolicy.h`。
`goalAlignmentExitDistance` 默认已同步为 0.30，旧的显式 `:=0.15` 必须移除或改成 `:=0.30`。

```bash
ros2 param get /pathFollower goalAlignmentExitDistance
ros2 param get /pathFollower goalPositionTolerance
ros2 param get /pathFollower goalYawTolerance
ros2 param get /localPlanner useFinalGoalPose
ros2 param get /pathFollower twoWayDrive
ros2 topic echo /fastlio_far/goal_status --field data
```

预期参数 0.30、0.10、约 0.349066、true、true。
goal_status 新增 `reverse_limit=0.5`、`align_enter_distance=0.2`，近目标应依次看到
`align/aligning_yaw`、`translate/translating_with_goal_yaw`、`reached/arrival_latched`。
如进入范围时位姿已经满足最终标准，可以直接确认 reached。
录包请在原有列表增加 `/local_planner/final_path`；保留 raw/safe cmd_vel、
motion_status、local_planner/status、state_estimation、Odometry、terrain_map 与 SDK 请求/响应。

先分别测试前方、后方、侧方目标，以及正负 yaw 调整；确认单点通过再启动原单轮巡检脚本。
发布单点与巡检的命令没有改变。

## 离线验证

实际编译的 localPlanner/pathFollower、合成输入、隔离 ROS Domain 87/88、
ROS_LOCALHOST_ONLY=1、realRobot=false，没有 SDK 或实机运动：

- `tools/test_near_goal_control.py`：8 项，两层节点联测与简单运动学闭环。
- `tools/test_reverse_limit.py`：8 项，0.49999/0.50000/0.50001 m 边界、离区撤销、
  XY 距离、无效目标/过期定位、twoWayDrive=false、manual override 等。
- `tools/test_inspection_route.py`：8 项原巡检逻辑回归。

开发工作区原始日志与修改前备份：`/home/lolo/G1/run_logs/near_goal_20261020/`。
本发布副本测试输出位于仓库自身的 `run_logs/near_goal_20261020/`，不提交生成日志。
独立巡检保存方式与路径适配见 [版本保存说明](save_near_goal_20261020.md)。
软件测试不能证明 G1 真实低速侧移、传感器参考点绕转动中心运动及定位噪声下的到点精度。
