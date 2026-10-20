# 近目标控制修改与离线验证

工作区：`/home/lolo/G1/G1-navigation`。2026-10-20。

## 修改完成

- planner 与 follower 共用 `finalGoalPolicy.h`：原始目标 XY 距离 ≤0.50 m 允许倒车，
  ≤0.20 m 提前对齐；最终完成判据保持 ≤0.10 m 且 yaw 误差 ≤20°。
- 新增 `translate` 状态：对齐到 10° 内后，保持目标 yaw 前进/后退/侧移；
  yaw 偏离超过 20° 再对齐。最终位姿已满足时直接按原停稳/三帧条件确认，不额外要求 10°。
- 近目标状态退出半径由 0.15 m 改为 0.30 m，保留三帧退出确认；
  不再存在旧 position_hysteresis_band 的停车分支。成功后位姿再超差会撤销 reached。
- localPlanner 在 0.50 m 内接管原始目标，防止追逐 FAR 偏移后的末端 waypoint。
  最终位姿导航不再清零身后 waypoint；0.50 m 外 follower 转向接近而不倒车。
  旧位置目标模式未改成新状态机。
- 新增 map 坐标 `/local_planner/final_path`：基于地形/激光、边界、附加障碍，
  检查外接圆 footprint 加 0.03 m 的扫掠通道，允许原地旋转及直线平移。
  follower 检查时效、原始目标/朝向一致、当前位置偏离和发布者唯一性。
  通道被阻挡/消息过期会停止，恢复后继续；没有绕过安全门或直接向 SDK 发速度。
- 近目标速度限额：平移模长 ≤0.08 m/s、侧向 ≤0.05 m/s、角速度 ≤0.15 rad/s。
- 现有 FAR、地图、SDK 桥、巡检点与独立巡检脚本没有修改。

## 检查结果

1. `cmake --build build/local_planner --parallel 2` 成功，localPlanner/pathFollower
   安装路径均使用构建产物；两层 launch 的退出半径默认值同步为 0.30 m。
2. `tools/test_near_goal_control.py`：8 项全部通过，最新完整测试 22.273 秒。
   使用实际编译的 planner/follower 两个进程：
   - 0.20000/0.20001 m 边界、正负 yaw 对齐；
   - 保持朝向的侧移与倒车；
   - 原始目标接管偏移/身后 waypoint；
   - 0.74 m 最终目标、0.63 m 身后 waypoint：多点路径、允许转向、禁止倒车；
   - 障碍出现后撤销动作，清障后恢复；
   - 点云/位姿过期、人工锁停；
   - 位置/朝向/停稳联合确认，恰好 0.10 m/20° 的完成边界，成功后超差撤销；
   - yaw 与距离迟滞、运动学闭环 align→translate→reached。
3. `tools/test_reverse_limit.py`：8 项全部通过，0.50 m 精确边界及离区撤销、
   XY 距离、无效目标/过期位姿、twoWayDrive=false、manual override 限制。
   此文件由旧 test_reverse_limit_010.py 更新并更名。
4. 原巡检 `tools/test_inspection_route.py`：8 项全部通过。
5. 两份 launch Python 语法解析与 `git diff --check` 通过。

共 24 项测试。ROS Domain 87/88，ROS_LOCALHOST_ONLY=1，realRobot=false，
没有启动 SDK、运动桥或真机。沙箱限制使日志中有 UDP 接口权限警告，
本机共享内存通信与消息断言完成，测试退出码均为 0。
简单运动学模型不包含 G1 步态、执行延迟或定位误差，不能当作实机验收。

## 文件

- `before/`：本轮修改前的两个 C++ 和两个 launch 文件。
- `build.log`：最新构建日志。
- `integration_tests.log`、`integration/`：两节点测试及状态。
- `reverse_tests.log`、`reverse/`：倒车限制回归。
- `patrol_regression.log`：巡检回归。
- 用户操作说明：`/home/lolo/G1/G1-navigation/docs/near_goal_control.md`。

## 下一次真机

停止旧导航后按原启动流程重启。不要使用不存在的 reverseMaxGoalDistance、
inspection_mode 参数。如手工指定过 goalAlignmentExitDistance:=0.15，
需移除或改为 :=0.30；新启动默认已为 0.30。
新增录包 `/local_planner/final_path`，监测 align→translate→reached。
先单点测试正前、后方、侧方及左右 yaw 调整，再运行原单轮巡检。

没有修改 FAR 的自由空间重投影或远距离收敛机制；若 FAR 在距离原始目标仍超过
0.50 m 时结束，仍可能缺少交接。真实障碍挡住原始目标时本版不会强行穿越，
巡检超时策略仍按原脚本执行。真实低速侧移能力、定位精度、sensor/base 转动中心
和 footprint 适配仍需现场验证。

本轮未提交或推送 Git。
