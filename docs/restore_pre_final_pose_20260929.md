# 9月29日双向/最终位姿修复前导航基线

恢复日期：2026-10-13。分支：`restore/pre-final-pose-20260929`。
基于发布仓库 `main` 的 `d7c21165e7268266e905b179cf83528f5182b9a3` 创建；
没有覆盖 main，也没有回退地图或路线。

## 恢复来源与范围

来源是原工作区 `run_logs/final_pose_contract_20260929/before/src/`，
即9月29日统一倒车行为和修复最终位姿交接之前的备份。
这不是10月8日添加20cm倒车限制之前的版本，也不是整个工作区的9月29日快照。

恢复12个源码/配置文件：

- local_planner：`localPlanner.cpp`、`pathFollower.cpp`、`local_planner.launch.py`、
  `CMakeLists.txt`、`package.xml`（5个）。
- far_planner：`far_planner.cpp`、`graph_planner.cpp`、对应两个头文件、
  `CMakeLists.txt`、`package.xml`（6个）。
- vehicle_simulator：`system_real_robot_with_fastlio_far.launch.py`（1个）。

在本发布仓库中它们位于 `ros2_ws/src/`。
`restore_pre_final_pose_20260929.manifest.json` 给出逐文件来源与SHA256。
算法源码保持备份原文；发布版保留两项已有的打包适配：总launch使用包路径和环境变量、
不引用发布仓库未包含的TARE；far_planner包清单不增加原发布版已去掉的opencv4条目。
OpenCV的CMake查找仍保留。其余10个文件与备份字节一致。

原 `/home/lolo/G1/G1-navigation` 工作区同时在同名本地分支恢复了这12个文件的原文，
该工作区的原分支 `test/pre-stage2-20260824` 仍保留提交 `7d5e407`。
发布仓库与旧仓库的历史不同，应从本发布仓库推送，不要把旧仓库历史强推到这里。

## 此分支的真实行为

- localPlanner 默认 twoWayDrive=true；pathFollower 的launch默认 twoWayDrive=false。
- 近距离身后waypoint仍执行 `relativeGoalDis=0; joyDir=0;`，没有双向模式豁免。
- 已有最终位置0.10m、yaw20°判定以及局部有效停车距离0.05m的早期修复。
- FAR仍按0.4m收敛并清除目标；不包含后来保留最终目标和local_handoff机制。
- 不包含后来独立最终位置接近、双向一致性、20cm倒车范围限制等改动。

因此，这个分支用于保留和对比旧逻辑，原有的“单点路径→零速度→永久approach”问题
可能再次出现。恢复完成不能当成导航故障修复或实机验收通过。

## 保留但不是旧版验收范围的内容

FAST-LIO、先验定位、安全门、SDK桥、地图、路线及巡检Python代码沿用main。
历史备份没有覆盖这些文件，不能声称它们全部回到了9月29日。
特别是后来的巡检执行器要求同目标标识和FAR状态接口，旧导航未完整提供这些字段；
不要直接将保留的P1→P4→HOME执行器视为与此导航基线兼容或已验证。
本次不启动机器人、不解除锁停，也不从历史备份复制已编译二进制。

## 本次验证

- 原工作区12个文件与备份SHA256完全一致；发布版10个一致，2个适配差异已核对。
- Python启动文件和XML解析通过；旧身后清零、无20cm倒车限制等逻辑特征核对通过。
- 发布工作区local_planner清理后重编译、far_planner重编译均成功，install指向新生成程序。
- 总launch的 `--show-args` 检查通过，没有启动节点。
- 地图PCD和VGH的既有SHA256校验通过。
- colcon首次停留在vehicle_simulator安装阶段，结束该进程后改用直接CMake完成编译；
  原日志保留在原工作区 `run_logs/restore_pre_final_pose_20261013/`。
- 编译存在旧版未使用参数、括号和异常按值捕获等警告；diff检查有4处历史行尾空格，
  为保持备份原文没有修改。没有进行实机或整轮巡检验收。

机器可读结果见 `restore_pre_final_pose_20260929.verification.json`。

## 构建与推送

首次克隆按根README安装依赖并构建。已有完整发布工作区可只重建受影响包：

```bash
cd /home/lolo/G1/G1-navigation-2609-release
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon \
  --log-base ros2_ws/log build --base-paths ros2_ws/src \
  --build-base ros2_ws/build --install-base ros2_ws/install \
  --packages-select local_planner far_planner vehicle_simulator \
  --executor sequential --symlink-install
```

推送的是新分支中12个文件的历史内容及恢复说明/校验清单；未变文件从main继承。
不提交build/install、旧可执行程序或录包，不需要重新单独上传地图。
提交完成后的推送命令：

```bash
git push -u origin restore/pre-final-pose-20260929
```

其他机器取出此分支：

```bash
git fetch origin
git switch --track origin/restore/pre-final-pose-20260929
```

切回发布主线可用 `git switch main`。切换源代码分支不会同步切换已编译程序，
每次切换后需重新构建相应包，再决定是否按现场流程启动。
