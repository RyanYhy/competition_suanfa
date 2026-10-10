# 比赛导航栈使用说明

首次安装见 [仓库 README](../README.md)。以下示例工作区采用 `~/community_ws`，仓库位于其 `src/smart-community/`；另开终端时先激活 `~/community_ws/.venv/bin/activate` 再加载 `devel/setup.bash`。

默认 `localization:=lidar planner:=custom profile:=balanced`：用户 jie_ware 激光匹配和自定义局部规划器的适配版。可切换 AMCL、DWA / TEB；最新实际对照与自动验收命令见 [导航验证与算法对比](../doc/导航验证与算法对比.md)。Cartographer 仅保留可选接口，本机当前缺少运行依赖。

## 1. 先用场地参考地图调试导航

关闭旧的 Gazebo、建图、导航和键盘节点。在桌面终端执行：

```bash
cd ~/community_ws
catkin_make
source devel/setup.bash
roslaunch smart_community_sim navigation.launch gui:=true rviz:=true
```

这会一起启动比赛场地、麦克纳姆机器人、地图服务、激光定位、move_base 和 RViz。默认地图 `maps/field_reference.yaml` 是由场地碰撞几何在雷达高度切片生成的参考地图，**不是机器人实际建图结果**。

在 RViz 中先确认激光点与地图围墙、信号灯立柱重合，再用 **2D Nav Goal** 在道路中心点选目标并拖动箭头设置朝向。先试同一条道路上 0.3–0.5 m 的短目标，再试转弯。如果定位偏移，用 **2D Pose Estimate** 在机器人实际位置重设朝向。

首轮建议先完成 `roslaunch smart_community_sim drive_test.launch verify:=true` 的运动测试，关闭后再启动导航。800 次 ODE 迭代、4 ms 步长下完整底盘检查已通过。小角速度仍有停滞/突发响应，导航规划器采用最低转向速度配合到点容差，不能视为任意低速驱动已通过。

如果已经用本版 `robot.launch` 启动机器人，可以用 `navigation.launch start_sim:=false`，避免重复生成机器人；机器人应在默认起点，或通过 `x/y/yaw` 参数设置定位节点的实际初始位置。默认起点 `(1.65, 1.75, π)`。

**导航时退出键盘节点。** 键盘空闲也会发送零速度，目前没有速度仲裁节点，不能同时让键盘与 move_base 控制 `/cmd_vel`。

## 2. 实际运行 SLAM 并保存地图

关闭导航和旧仿真，终端一：

```bash
source ~/community_ws/devel/setup.bash
roslaunch smart_community_sim mapping.launch
```

终端二：

```bash
source ~/community_ws/devel/setup.bash
rosrun smart_community_sim keyboard_teleop.py
```

沿外围道路和中间支路缓慢走一遍，回到起点闭环。键位 `w/x` 前后、`j/l` 横移、`a/d` 转向、空格停车。确认 RViz 地图完整后，在终端三保存：

```bash
source ~/community_ws/devel/setup.bash
rosrun map_server map_saver -f $(rospack find smart_community_sim)/maps/community_slam
python3 $(rospack find smart_community_sim)/scripts/generate_navigation_maps.py \
  --align-to-map $(rospack find smart_community_sim)/maps/community_slam.yaml \
  --keepout-output $(rospack find smart_community_sim)/maps/community_slam_keepout.yaml
```

第二条命令使禁行地图与 SLAM 地图使用相同分辨率、原点和栅格尺寸，防止两层 StaticLayer 相互改变全局代价地图尺寸。它只对齐栅格，不能纠正 SLAM 漂移或坐标系旋转。

关闭建图、键盘和仿真，再启动：

```bash
roslaunch smart_community_sim navigation.launch \
  map:=$(rospack find smart_community_sim)/maps/community_slam.yaml \
  keepout_map:=$(rospack find smart_community_sim)/maps/community_slam_keepout.yaml
```

AMCL 与正常建图 gmapping 都发布 `map → odom`，**不要同时运行**。本版插件仅在初始化时把轮式里程计设为已知生成位置，之后仍从轮关节角度增量积分；未使用 Gazebo 真实位置持续修正里程计。更新插件后须重新编译并重启机器人。

`mapping_observer.launch` 是单独的被动采图诊断工具：关闭 gmapping 的 TF 输出，地图另发到 `/mapping_observer/map`，可在 AMCL 导航时采集真实雷达/里程计建图结果。此图不参与当次导航。保存时使用 `rosrun map_server map_saver -f /tmp/observed_map map:=/mapping_observer/map`。每次重启 Gazebo 都应重启观察器，避免混用重置前后的仿真时间与地图状态。

禁行图和巡航点使用场地中心坐标。换用其他建图方式、旧版零起点里程计、旋转地图或真实场地地图时，需重新对齐禁行区域并录制巡航点。可用 `use_keepout:=false` 单独诊断定位与导航，此时地面区域线不再提供规划约束。不要用失配地图直接启动预设巡航。

## 3. 多点巡航与红绿灯等待

关闭普通导航和仿真后启动（默认自动出发；调试用以下参数关闭自动出发）：

```bash
roslaunch smart_community_sim patrol.launch autostart:=false
```

另开终端：

```bash
source ~/community_ws/devel/setup.bash
rosservice call /patrol/start "{}"
rostopic echo /patrol/status
```

停止巡航：

```bash
rosservice call /patrol/stop "{}"
```

`patrol.launch` 同样接受 `map`、`keepout_map`、`use_keepout`、`start_sim`、`x/y/yaw` 参数；`autostart:=true` 可直接开始。重复调用 start 会从路线开头重新执行，定位应先确认。巡航失败或等待超时会取消导航目标并发送停车指令。

路线在 `config/navigation/patrol_route.yaml`：19 个点，严格按新增示意图的一轮顺序安排。西路在中间横支路向东，不再走左下外围；北、南灯各通过一次；东侧依次观察 1/2/3 号车牌后返回起点。观察时原地转向，底盘仍在车道内。

默认灯色来源：

```text
/vision/traffic_lights/north_signal/state
/vision/traffic_lights/south_signal/state
```

消息为 `std_msgs/String`：`red` / `yellow` / `green` / `unknown`。视觉上下文由巡检节点指定当前观察的信号灯，灯色来自相机像素。缓存绿灯和过晚绿灯不放行，需要近期已确认的红/黄阶段后出现绿灯；未知灯色即时撤销许可。

巡检节点将 move_base 的 `/navigation/cmd_vel` 过滤后发送到 `/cmd_vel`。使用 map 定位、底盘轮廓、里程计速度、制动距离及延迟余量约束停止线；定位、里程计或导航指令失联时发送零速度。已越过停止线的车辆允许清空路口。该策略的实际停车距离、打滑和定位误差仍需动态验证。

`traffic_prefix:=/simulation/traffic_lights` 可用于对比诊断，明确属于仿真真值来源，不能用于声称完成视觉红绿灯检测。普通 `navigation.launch` 的 RViz 点选不经过巡检规则；不应以该模式录制自主任务完成演示。

相机识别节点同步发布带框图像 `/vision/annotated` 和 `/vision/detections`，街区统计在 `/vision/district_counts`。普通运行在包内 `validation/` 只保存 `yolo_images/`、`识别总览.jpg`、`终端日志.log`；日志包含各识别点结果/置信度，末尾有任务汇总。详细事件截图、JSONL及轨迹需显式开启 `diagnostics:=true`，另放 `doc/validation/识别调试结果/`。终端以黄色显示观察结果和完成提示，完整检测JSON不刷屏。RViz已加入Recognition图像显示。

当前视觉使用用户 YOLO ONNX 检测与通用 PP-OCRv4 车牌 OCR，阈值由实际 Gazebo 图像实验选择。人物按定位 TF、相机内参及立牌高度投影到地图位置去重，实例编号不是官方身份。默认 A/B 各 9 人、共 16 社区+2 非社区人员、三张不同车牌才报告完成；每个目标至少三张新帧确认。100° 相机视野与4秒观察时长，人物按同朝向排紧；A南/B北各合并为一个观察点。固定目录内的图片、任务表和导航诊断在新巡航开始时重置覆盖。未识别对象不从场景清单或 Gazebo 真值补填。过程与验收见 [视觉识别学习记录](../doc/视觉识别学习记录.md) 和 [验证说明](../doc/视觉识别验证说明.md)。

## 4. 参数与话题

| 功能 | 配置/行为 |
|---|---|
| SLAM | Gmapping 或可选 Cartographer，0.02 m 栅格 |
| 定位 | 默认 jie_ware 扫描匹配适配版；AMCL `omni-corrected` 对照；可选 Cartographer 接口 |
| 全局规划 | GlobalPlanner，禁止穿越未知栅格 |
| 局部规划 | 默认用户自定义规划器适配版；DWA / TEB 作对照，均支持横移和原地转向 |
| 速度 | 最大平移 0.18 m/s、横移 0.15 m/s、转向 0.5 rad/s |
| 碰撞轮廓 | 0.38 × 0.34 m 矩形，每边再加 0.01 m |
| 局部地图 | 2.4 × 2.4 m，5 Hz；控制 10 Hz |
| 地图层顺序 | 静态地图 → 激光障碍 → 禁行地图 → 膨胀 |
| 失败恢复 | 禁用自动原地旋转和盲目清图；停止后检查原因 |
| TF | `map → odom → base_footprint → base_link → laser_link` |

配置集中在 `config/navigation/`；launch 实际加载 `profiles/baseline.yaml`、`profiles/balanced.yaml` 或 `profiles/fast.yaml`，以及 `amcl_baseline.yaml` / `amcl_dense.yaml`，TEB 叠加 `teb.yaml`，自定义规划器叠加 `custom.yaml`。自定义规划器当前验收限速 0.18 m/s / 0.5 rad/s，不因 profile 切换提高速度。修改根目录保留的 `move_base.yaml` / `amcl.yaml` 不会改变这些 launch 配置。地面标线没有激光碰撞，因此另加禁行层，且放在激光清除层之后。`maps/navigation_preview.png` 中红色为禁行区域，黑色为雷达高度的实体障碍。

修改场地后同步重新生成参考地图和禁行图：

```bash
python3 $(rospack find smart_community_sim)/scripts/generate_navigation_maps.py
```

若使用实际 SLAM 地图，再执行第 2 节的禁行图对齐命令。

常用诊断：

```bash
rostopic hz /scan
rostopic hz /odom
rosrun tf tf_echo map base_footprint
rostopic echo /move_base/status
rostopic echo /cmd_vel
rostopic echo /patrol/motion_guard
```

有路径但车不动时，先检查仿真是否暂停、键盘节点是否仍发送零速度、激光是否持续更新；里程计变化但 Gazebo 车体不走或偏转时，应先调驱动接触与摩擦，再调导航。

## 5. 当前验证边界

2026-10-09 当前环境已能实际启动 ROS/Gazebo。800 次 ODE 迭代、4 ms 步长下，六方向底盘运动、释放停车、10 秒静止和接地检查通过。已执行 AMCL/DWA/TEB 在线对照，AMCL 与 jie_ware 配合自定义规划器均通过 21 点整圈；Gmapping 实际地图未验收通过，默认仍使用参考图。当前结果及失败实验见 [导航修复学习记录](../doc/导航修复学习记录.md)，原始数据在 `doc/validation/20261009_navigation_live/`。此前“无法在线验证”的说明是历史状态。

离线测试命令（无需 ROS master）：

```bash
source /opt/ros/noetic/setup.bash
cd ~/community_ws/src/smart-community
python3 -m unittest discover -s smart_community_sim/tests -v
```
