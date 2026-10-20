# 当前导航代码保存：2026-10-20

分支：`feature/near-goal-control-20261020`。
保存目标：`YiRAN7770/G1-navigation-2609`。
基于发布副本的 `11074f9fd239d2bbc7ee834d6fccc88e32525121` 创建新分支；
保留 `restore/pre-final-pose-20260929` 与 `main`。
开发工作区 `/home/lolo/G1/G1-navigation` 的分支、未提交内容不因本次保存而改变。

## 保存范围

- localPlanner.cpp、pathFollower.cpp、finalGoalPolicy.h、local_planner.launch.py：
  与开发工作区当前文件字节一致。
- 总导航 launch：保留发布版包路径、外部依赖配置和无 TARE 的适配，
  同步 goalAlignmentExitDistance 默认值为 0.30 m。
- 独立单轮巡检脚本、P1～P4 坐标和 START 占位、四份测试脚本。
- 控制说明、原开发工作区验证报告和本次同步文件 SHA256 清单。
- 地图、FAR、安全门与 SDK 桥沿用发布副本现有内容；不加入录包、build/install、虚拟环境。

算法行为：0.50 m 内允许倒车，0.20 m 内提前对齐，再保持目标朝向平移；
最终位置 ≤0.10 m、yaw 误差 ≤20°，且满足停稳/连续帧确认才成功。
巡检仍是 START→P1→P2→P3→P4→START 单轮，结束后锁停，不自动循环。
具体限制见 [近目标控制](near_goal_control.md)。

## 发布版路径适配

控制源码没有二次修改。Python 包装路径与开发目录有以下区别：

- ROS 工作区为仓库内 `ros2_ws`，测试执行此工作区的已构建程序。
- 配置 `prior_pcd` 使用相对于配置文件的 `../maps/site_04_nav_zup_ccw90/prior.pcd`；
  脚本支持相对路径和绝对路径，运行时仍严格检查定位节点所加载的实际地图。
- 测试与巡检日志默认写入副本自身 `run_logs/`，被 Git 忽略。
- 目标顺序、点位数值、到达/停稳检查和停止语义没有改变。

开发工作区文件路径到发布路径、两边 SHA256 与适配说明见
`near_goal_20261020.manifest.json`。

## 构建与检查

本次发布副本重新构建通过，8 项近目标联测、8 项倒车测试、8 项巡检单元测试、
4 项巡检模拟 ROS 测试共 28 项通过；PCD/VGH 校验通过。机器可读结果见
`near_goal_20261020.verification.json`。没有启动真机。

新克隆先按根 README 与 dependencies 文档准备依赖并运行 `scripts/build.sh`。
已有配置完成的本副本可重建局部规划器：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
cmake --build ros2_ws/build/local_planner --parallel 2
python3 -m unittest discover -s tools -p test_inspection_route.py -v
sha256sum -c routes/nv_zup_ccw90/map.sha256
```

隔离测试需明确使用本机非零 Domain，不启动 SDK：

```bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
python3 -m unittest discover -s tools -p test_near_goal_control.py -v
python3 -m unittest discover -s tools -p test_reverse_limit.py -v
python3 -m unittest discover -s tools -p test_inspection_route_ros.py -v
```

不要将上述测试环境直接用于真机。真实启动使用原现场的 Domain 与网络配置、
`scripts/run_fastlio_far.sh` 及已配置的 Unitree 环境。
本次没有启动机器人，测试通过不等于实机精度验收。

## 独立巡检入口

在发布副本根目录执行。先按现场流程完成导航启动、导图、initialpose、锁停与定位核验；
不同时启动旧 `tools.inspection_mission` 执行器。

```bash
python3 tools/inspection_route.py record START
python3 tools/inspection_route.py show
```

实际起点停稳、现场确认后解除既有软件锁停，确认 motion_status=ready。
以下命令会发布导航目标，可能驱动已启用的机器人：

```bash
python3 tools/inspection_route.py run --timeout 180 --dwell 3
```

录包增加 `/local_planner/final_path`。观察 `align → translate → reached`；
被障碍阻挡时不强行穿越，不扩大最终到达阈值。

## 获取该版本

```bash
git fetch origin
git switch --track origin/feature/near-goal-control-20261020
```

本地已经有此分支时使用 `git switch feature/near-goal-control-20261020`。
切换代码后需重新构建，再决定是否按现场流程启动。
