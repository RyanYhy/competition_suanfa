# 智慧社区场地与接触驱动机器人

依据 `doc/复赛资料` 及 `doc/9.24智慧社区详细规则.pdf` 构建，ROS Noetic / Gazebo Classic 11。

首次获取 GitHub 源码请先按 [仓库安装说明](../README.md) 执行 `tools/setup.sh`，再用 `tools/run.sh` 启动。下面命令采用示例工作区 `~/community_ws`，可替换为自己的路径；历史报告的原始数据不随源码分发。

2026-10-08 已完成本轮场地静态修订。先用 `roslaunch smart_community_sim field.launch` 查看场地；[立体预览](preview/field_3d.png)、[车牌安装效果](preview/plates.png)、[人物立牌](preview/people.png) 均由实际 world 离线渲染，非 Gazebo 截图。[修订与验收记录](../doc/9.24场地修订与查看.md)。2026-10-09 已能实际启动 ROS/Gazebo，AMCL 与用户规划器/激光匹配适配版已完成整圈导航诊断；问题、实验和当前结果持续记录在 [导航修复学习记录](../doc/导航修复学习记录.md)。用户 ONNX 模型已接入，已完成当前 Gazebo 场景的 YOLO/OCR 自主巡航验收，过程及结果见 [视觉识别学习记录](../doc/视觉识别学习记录.md)。

## 建图与自主导航

已加入 Gmapping、全向 AMCL、用户 jie_ware 激光匹配适配版、自定义局部规划器、DWA / TEB、可选 Cartographer、独立禁行地图及多点巡航。默认关闭显示窗口；手动点选目标使用 `roslaunch smart_community_sim navigation.launch gui:=true rviz:=true`。导航时退出键盘节点。

默认 `localization:=lidar planner:=custom`，可用 `localization:=amcl` 对照定位、`planner:=dwa` / `planner:=teb` 对照规划器。最新真实整圈结果及复现命令见 [导航验证与算法对比](../doc/导航验证与算法对比.md)。

完整的实际建图、地图保存、巡航启停和红绿灯接入步骤见 [导航使用说明](NAVIGATION.md)。默认使用场地生成的参考地图；最新动态验证结果以学习记录及其链接报告为准。

## 启动场地和键盘控制

关闭之前的 Gazebo/roslaunch，在终端一执行：

```bash
cd ~/community_ws
catkin_make
source devel/setup.bash
roslaunch smart_community_sim robot.launch
```

终端二执行（键盘焦点留在此终端）：

```bash
source ~/community_ws/devel/setup.bash
rosrun smart_community_sim keyboard_teleop.py
```

| 按键 | 动作 |
|---|---|
| w / x | 前进 / 后退 |
| j / l | 左横移 / 右横移 |
| a / d | 原地左转 / 右转 |
| q / e | 左前斜移 / 右前斜移 |
| s / 空格 | 停车 |
| r / f | 增加 / 降低线速度 |
| t / g | 增加 / 降低角速度 |
| p / Ctrl-C | 停车并退出 |

默认 0.12 m/s、0.5 rad/s。松键约 0.3 秒后节点发零速度；驱动器另有 0.5 秒指令超时。机器人有真实惯性，停车需制动过程。键盘逻辑复用自 `~/raicom_ws/src/robot/scripts/keyboard_teleop.py`，源码已纳入本包，运行时不依赖旧工作区。

只看场地：`roslaunch smart_community_sim field.launch`。无 GUI 参数为 `gui:=false`。

## 本轮场地修订（9.24 详细规则）

- 场地 **4.2 × 4.2 m** 已有正式依据；三处标注的道路宽度为 **0.60 m**。采用地图的黑底白线，内部边界为平面标线，禁行地图另计标线外沿。
- 当前复赛场景仅含 A/B 人偶、两处红绿灯和三辆车辆立牌；已移除旧场景中的楼宇、站房、垃圾桶、电动车实体。总决赛的扩展任务保留在方案讨论范围。
- 单轮路线为：起点向西 → 西路向南 → 中间支路向东 → 中路向南 → 底路向东 → 东路向北 → 返回起点。A/B 人群分别安排多个朝向观察点；默认 19 个航点，北、南信号各通过一次。
- 围墙内侧位于 ±2.1 m，采用 PDF 第 15 页建议的 **0.50 m 高、0.005 m 厚**；模型含碰撞体，供雷达观测。
- 红绿灯总宽 **0.64 m**、总高 **0.48 m**、厚 **0.05 m**；灯箱 **0.59 × 0.14 × 0.05 m**，两侧立柱贯通 **0.48 m** 全高、截面 **0.025 × 0.025 m**。净高 **0.34 m**、净宽 **0.59 m**。六张官方亮/暗纹理由原插件切换，world 明确配置红 **10 秒**、绿 **15 秒**、黄 **5 秒**。
- 相机中心 **0.28 m**，向下俯视约 10.3°，输出 **1280 × 960**，水平视野 **100°**。相机及支架低于灯箱净空。
- 三辆车背景立牌宽 **0.345 m**、高 **0.25 m**、厚 **0.0005 m**；车牌宽 **0.095 m**、高 **0.03 m**、厚 **0.0001 m**，各为独立静态模型并有对应碰撞体。1、2 号分别使用 `doc/随机车牌1`、`doc/随机车牌2` 的原始图片；3 号保留官方第三张示例牌。
- 场上放置全部 **16 名社区人员 + 2 名非社区人员**，A/B 区各 8 名社区人员、1 名非社区人员。每人独立建模，背板高 **0.15 m**、宽 **0.05 m**、厚 **0.005 m**；原图等比例居中，避免人物拉伸。具体身份与区内坐标为仿真配置。
- 物料朝向对应示意箭头：A 西/南/北，B 北/东，车牌西向；北灯向东、南灯向北。

场地尺寸、物料身份、随机牌纹理路径和仿真假设见 `config/scene_manifest.json`；旧识别器的模板库仍为 `config/recognition_assets.json`，未将随机牌加入查表识别。原始文件没有改动，运行所需纹理已独立包含在包中。

![布局预览（非 Gazebo 截图）](field_layout.png)

## 相机识别与自主巡检

已接入 `model/` 下三个用户 YOLO ONNX 模型：人物两类、车牌框、灯箱框。车牌先由 YOLO 裁剪，再用 PP-OCRv4 读取字符；连续三张新帧字符一致才接受，不查已知牌号。相机水平视野默认 100°，人物保留原布置，观察航点朝向紧凑人物组中心并停留 4 秒，A南/B北各只观察一次。

使用工作区虚拟环境复现视觉依赖，保留 apt 的 ROS OpenCV/cv_bridge：

```bash
cd ~/community_ws/src/smart-community
bash tools/setup.sh
bash tools/run.sh
```

需要已有 `cv_bridge`、numpy、cv2、Pillow、Noto CJK 字体。默认自动巡检，CPU 按观察上下文运行一个 YOLO 模型，上限 5 帧/秒；模型阈值见 `config/vision.yaml`。灯箱内用 HSV 亮像素判断灯色，两帧一致后发布 `/vision/traffic_lights/*/state`。

人物按图像框、实际相机内参和定位 TF 投影到 0.075 m 立牌中心高度，跨观察点按位置去重，并用置信度加权角色投票；至少三张新帧才确认。模型只训练了社区/非社区两类，输出的实例编号不代表官方人物身份编号。默认要求 A/B 各 9 个独立人偶，合计 16 社区人员+2 非社区人员，以及三张不同车牌。没有识别到的对象不补填。

`/vision/annotated`、`/vision/detections` 使用实际图像时间戳。输出目录 `smart_community_sim/validation/` 只保存 `yolo_images/` 每个观察方向的最佳带框图、`识别总览.jpg` 和 `终端日志.log`。每个识别点结束时，黄色终端提示列出人物实例/类别/对应YOLO峰值，车牌文字/YOLO分数/OCR平均分数，以及灯色/灯箱YOLO分数。日志保留同样文本与运行中的WARN/ERROR，末尾汇总各任务；停止或失败也生成实际已拍图片的总览。下一轮自动覆盖旧结果。专用调试加 `diagnostics:=true`，原图、裁剪、逐帧/事件证据与导航轨迹另放 `doc/validation/识别调试结果/`。原模板模块仅保留为历史实验，默认节点不调用它。

自动整圈验收可用 `vision_benchmark.launch`，结束后自动关闭仿真并保存真实导航轨迹。详细命令与当前结果见 [视觉识别验证说明](../doc/视觉识别验证说明.md)。

巡检中 move_base 输出到 `/navigation/cmd_vel`，巡检节点依据定位、实测里程计、车身轮廓及制动余量过滤后输出 `/cmd_vel`。灯色未知、失联或红/黄灯时限制越线，已合法进入路口则继续清空路口。当前制动参数仍需物理实测，普通 RViz 点选导航不包含巡检交通规则。

测试与限制详见 [资料核对](../doc/复赛资料核对与修订.md) 和 [导航说明](NAVIGATION.md)。

## 车轮接触与控制

`urdf/community_mecanum.xacro` 使用自建底盘、雷达、降低后的相机与物理轮组：

- 机器人外廓约 0.37 × 0.33 m；四个轮毂受电机力矩驱动，每个轮毂有 16 个斜置自由滚子，滚子关节有质量、惯量、碰撞与摩擦。
- 中间轮毂半径 0.028 m，不接地。滚子回转外包络约 0.054 m；使用三个球形碰撞体近似一个鼓形滚子，滚子与地面接触后带动车身。
- `mecanum_drive_plugin.cpp` 只用 `SetForce` 给轮轴施加受限力矩。已删除车身 `SetLinearVel` / `SetAngularVel` 和轮轴强制 `SetVelocity`。
- `/odom` 累计四个轮轴实际角度增量，速度取发布周期内增量的平均；它是轮式里程计，仍会受打滑影响。
- `/joint_states` 包含轮毂与滚子；`robot_state_publisher` 发布各关节 TF。其他话题：`/scan`、`/front_camera/image_raw`、`/front_camera/camera_info`、`/ground_contacts`。

电机参数默认：轮速上限 8 rad/s、力矩上限 0.45 N·m、比例增益 0.04、积分增益 0.08、编码器速度滤波时间常数 0.01 s、目标轮速变化率 30 rad/s²。积分采用抗饱和限制，用于减小恒定负载下的速度偏差及零速漂移。此版是真实碰撞接触的近似模型，滚子外形、摩擦及电机参数仍需在桌面 Gazebo 和实车上调试，不是厂商标定模型。

## 使用键盘节点自动验证运动

若出现 **RViz 车体转向而 Gazebo 车体不转**，先关闭巡检，在空地只做左右转向测试：

```bash
cd ~/community_ws
catkin_make -j2
source devel/setup.bash
roslaunch smart_community_sim drive_test.launch verify:=true rotation_only:=true
```

终端逐项打印真实转角、编码器转角及误差；结果为 `/tmp/community_motion_report.json` 与同名 CSV。现在通过条件同时要求真实运动成立、四轮有接地记录、相对位移误差不超过 5 cm、转角误差不超过 0.15 rad。这是工程诊断阈值，不能把 RViz 模型转动当作底盘运动通过。

此外增加 **10 秒静止检查**：车体漂移须小于 1 cm / 0.03 rad，轮式里程计漂移须小于 1.5 cm / 0.05 rad。2026-10-07 用户短转向测试虽通过，后续巡检仍记录到停车时里程计持续偏转；本次修订针对该问题，不能沿用旧的短测试通过结果当作新版验收。

普通 `patrol.launch` 默认关闭轨迹记录。专用调试加 `diagnostics:=true`，每轮结束约3秒后保存到独立 `doc/validation/识别调试结果/navigation/`。驱动插件的 `drive_check` 为DEBUG级别，正常终端不输出；持续速度不一致仍会输出 `DRIVE_ODOMETRY_MISMATCH`。这些真值只用于诊断，不修正定位或驱动。详见 [本次转向问题检查](../doc/转向与里程计问题检查.md)。

先在原来的 roslaunch 终端按 Ctrl-C 关闭场地仿真，然后用一条命令启动测试地面并自动验证：

```bash
source ~/community_ws/devel/setup.bash
roslaunch smart_community_sim drive_test.launch verify:=true
```

该命令会等待 Gazebo、机器人生成与仿真时钟就绪，再自动启动验证脚本，无需再运行 rosrun。

也可以保持分两步使用：先运行不带 `verify:=true` 的 `drive_test.launch`，再在终端二执行：

```bash
source ~/community_ws/devel/setup.bash
rosrun smart_community_sim verify_keyboard_drive.py
```

脚本创建伪终端并启动同一个键盘节点，输入 w/x/j/l/a/d，检查 `/gazebo/get_model_state` 的真实位移、转角、翻倾、停稳漂移及 `/ground_contacts` 中四轮滚子接地信息。它不以“发布了 cmd_vel”作为通过标准，也不用编码器里程计代替实际位移。最终报告写入 `/tmp/community_motion_report.json`，失败时返回非零。自动测试时无需再开另一个手动键盘节点。

如果提示“当前 ROS 会话不是独立测试场地”，说明启动的是普通 `robot.launch` / `field.launch`，或测试场地尚未启动；这不是机器人运动测试失败。按上面的一条命令重新启动即可。

2026-10-09 已实际启动 ROS/Gazebo，真实接触驱动与导航整圈已测试；最新结果及低速转向限制见 [导航修复学习记录](../doc/导航修复学习记录.md)。历史静态验证记录不代表当前动态验收。

## 修改场地

```bash
cd ~/community_ws/src/smart-community
python3 smart_community_sim/scripts/generate_field.py
```

生成脚本有原始比赛素材和随机车牌时优先复制原文件；GitHub 源码没有原始 doc 素材时，复用 `materials/textures/` 中已打包的原始纹理。不要先删除这个目录。生成好的 world、纹理、DAE 已包含，运行仿真无需重新生成。场地坐标以中心为原点，图右为 +x、图上为 +y，默认起点 `(1.65, 1.75)`，机器人朝西。执行 `python3 smart_community_sim/scripts/preview_field.py` 可重新生成静态立体预览，需要 NumPy、Pillow 和 Noto CJK 字体；该预览不是 Gazebo 渲染结果。
