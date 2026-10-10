# 使用说明

[返回首页](../README.md) · [安装指南](INSTALL.md) · [开发与发布](DEVELOPMENT.md)

以下命令采用 `~/community_ws/src/smart-community/`。更换工作区或仓库名时，使用自己的路径。先完成安装；每次只启动一套 Gazebo/ROS 系统，切换模式前在原启动终端按 `Ctrl-C` 关闭。

## 1. 完整自主巡检

在桌面终端执行：

```bash
cd ~/community_ws/src/smart-community
bash tools/run.sh
```

默认启动比赛场地、机器人、激光定位、自定义局部规划器、相机识别和巡检，打开 Gazebo 与 RViz 后自动出发。默认定位 `lidar`、规划器 `custom`、配置 `balanced`。

观察三处：Gazebo 中车体真实运动；RViz 中地图/激光重合与 Recognition 图像；终端中黄色的航点完成和识别点报告。终端显示人物实例/类别及 YOLO 分数、车牌文字及 YOLO/OCR 分数、灯色及灯箱 YOLO 分数；失败原因以红色提示，末尾生成任务汇总。

巡检按 19 个航点执行北侧路口、A 区三个方向人物、B 区两个方向人物、南侧路口、三处车牌和返回起点。人物按相机内参和定位投影关联位置并去重；实例编号用于本轮跟踪，不是官方人物身份编号。车牌先 YOLO 裁剪，再 OCR 读字符，连续三张新帧一致后接受；不会查预设车牌文本补结果。

## 2. 启停与新终端环境

需要先检查再出发：

```bash
bash tools/run.sh autostart:=false
```

每个新终端先执行：

```bash
source ~/community_ws/.venv/bin/activate
source ~/community_ws/devel/setup.bash
cd ~/community_ws/src/smart-community
```

开始：

```bash
rosservice call /patrol/start '{}'
```

停止：

```bash
rosservice call /patrol/stop '{}'
```

停止取消任务并停车，保留已取得的图片和未完成汇总。**新的 start 会重置上一轮结果。** 重跑前确认上一轮已结束/停止，机器人位姿和定位符合路线初始条件；最容易复现的方法是关闭并重新启动整套仿真，让车从默认起点生成。

`Ctrl-C` 退出启动终端会关闭本次节点。查看巡航状态可以运行 `rostopic echo /patrol/status`。

## 3. 其他运行模式

### 关闭显示窗口

```bash
bash tools/run.sh --headless
```

Gazebo 服务器和相机仍运行，需要可用的 DISPLAY/OpenGL。主机图形驱动有问题时可尝试：

```bash
LIBGL_ALWAYS_SOFTWARE=1 bash tools/run.sh --headless
```

软件渲染增加 CPU 开销。本次换路径测试使用这一组合和主机 DISPLAY；未验收没有显示环境的纯服务器部署。

### 手动驾驶

终端一：

```bash
bash tools/run.sh --manual
```

终端二加载工作区环境后执行，键盘焦点保持在该终端：

```bash
rosrun smart_community_sim keyboard_teleop.py
```

| 按键 | 动作 |
|---|---|
| `w / x` | 前进 / 后退 |
| `j / l` | 左 / 右横移 |
| `a / d` | 原地左 / 右转 |
| `q / e` | 左前 / 右前斜移 |
| `s / 空格` | 停车 |
| `r / f` | 增加 / 降低线速度 |
| `t / g` | 增加 / 降低角速度 |
| `p / Ctrl-C` | 停车并退出 |

默认 0.12 m/s、0.5 rad/s，松键后自动发零速度，机器人仍有真实制动过程。**进入自主导航前退出键盘节点**：当前没有速度仲裁，键盘空闲的零速度也会干扰导航。

### RViz 点选导航

```bash
bash tools/run.sh --navigation
```

先确认激光与地图匹配，再用 RViz 的 **2D Nav Goal** 在道路中心点选短目标并设置朝向。此模式没有巡检节点、相机识别任务和巡检红绿灯过滤，不用于声称完成自主比赛任务。定位偏移时可用 **2D Pose Estimate** 重设实际位置。

## 4. 验证与查看结果

启动后在已加载环境的另一终端运行：

```bash
python3 tools/smoke_test.py
```

检查仿真时钟、地图、雷达、里程计、相机图像/内参、巡航状态、定位 TF 和视觉报告服务，成功输出 `"ok": true`。这是启动检查，不是完整导航精度或识别正确率验收。

完整巡检检查终端最后的任务汇总：

- 导航航点 19/19，返回起点；两处路口均到达出口。
- A/B 各 9 个独立人物，合计 16 名社区人员、2 名非社区人员。
- 三张不同车牌完成 OCR 确认。
- 本轮状态与视觉验收均报告通过，未检测到的对象不补填。

结果固定放在 `smart_community_sim/validation/`：

| 路径 | 内容 |
|---|---|
| `yolo_images/` | 各观察点最佳带框图，编号/类别/置信度可视化 |
| `识别总览.jpg` | 实际已取得图片与本轮任务状态，失败时也生成 |
| `终端日志.log` | 识别点报告、WARN/ERROR，最后为任务汇总，无 ANSI 颜色字符 |

查看日志末尾：

```bash
tail -35 smart_community_sim/validation/终端日志.log
```

下一轮自动覆盖旧结果，需保留时先自行复制到另一个目录。专门调试：

```bash
bash tools/run.sh diagnostics:=true
```

原图、车牌裁剪、逐帧记录和导航轨迹另存在 `doc/validation/识别调试结果/`，展示目录仍只有三项。诊断中的仿真真值仅用于评价，不反馈到定位、驱动或识别。

**当前已知误判**：2026-10-10 换路径整圈完成 19/19 航点和三张车牌，但 A 区非社区人员被误判为社区人员，完整视觉验收未通过。导航完成与全部识别通过应分别判断。[完整验证状态](../README.md#验证状态与已知问题)

## 5. 算法切换与地图

关闭上一套仿真后，对照实验可以执行其中一条：

```bash
bash tools/run.sh localization:=amcl planner:=custom
bash tools/run.sh localization:=amcl planner:=dwa
bash tools/run.sh localization:=lidar planner:=teb
```

定位选项为 `lidar / amcl / cartographer`，规划器为 `custom / dwa / teb`。Cartographer 不包含在默认安装中，需另装依赖、建图状态和坐标锚定；仅保留实验接口，不能当作已通过的定位版本。

默认地图为 `maps/field_reference.yaml`，来自场地碰撞几何，不是机器人实际 SLAM 建图结果。地面白线不是激光障碍，另用 `competition_keepout.yaml` 提供禁行区域。改用自己的地图时必须同步对齐禁行图和路线坐标，不能只换 YAML 后沿用默认航点。

Gmapping 的实际建图、地图保存和禁行层对齐见 [导航使用说明](../smart_community_sim/NAVIGATION.md)。本轮实际 SLAM 尚未验收。正常建图与定位都可能发布 `map → odom`，不要同时运行两个定位/建图后端。

## 6. 配置、话题与诊断

| 文件 | 修改内容 |
|---|---|
| `config/navigation/patrol_route.yaml` | 航点、观察方向与停留时间 |
| `config/vision.yaml` | YOLO 阈值、OCR 阈值、处理频率、要求人数 |
| `config/navigation/custom.yaml` | 自定义局部规划器参数 |
| `config/navigation/profiles/` | 导航速度及控制配置 |
| `config/navigation/amcl_baseline.yaml`、`amcl_dense.yaml` | 实际 launch 使用的 AMCL 配置 |
| `maps/` | 参考地图和禁行地图 |

这些路径相对 `smart_community_sim/`。修改后重启相关节点；C++ 插件修改还需重新编译。

| 话题 | 含义 |
|---|---|
| `/scan`、`/odom` | 雷达与轮式里程计 |
| `/front_camera/image_raw`、`/front_camera/camera_info` | 实际相机图像与内参 |
| `/vision/annotated`、`/vision/detections` | 带框图像与识别结果 |
| `/vision/district_counts` | 本轮去重人数、类别和车牌统计 |
| `/vision/traffic_lights/north_signal/state`、`south_signal/state` | 相机灯色判断，未知为 unknown |
| `/patrol/status`、`/patrol/motion_guard` | 巡航状态与运动保护原因 |
| `/navigation/cmd_vel`、`/cmd_vel` | 规划输出与巡检过滤后的底盘指令 |

常用排查（按需分别执行）：

```bash
rostopic hz /scan
rostopic hz /odom
rosrun tf tf_echo map base_footprint
rostopic echo /patrol/motion_guard
```

有路径但车不动，先确认仿真未暂停、键盘已退出、scan/odom 持续更新。RViz 模型旋转不等于 Gazebo 车体运动，驱动问题可在关闭旧仿真后执行：

```bash
roslaunch smart_community_sim drive_test.launch verify:=true
```

测试真实位移/转向、停稳漂移及滚子接地，报告写到 `/tmp/community_motion_report.json` 和同名 CSV。详细原理与历史失败实验见 `doc/导航修复学习记录.md`；原始历史日志不随 Git 仓库分发。
