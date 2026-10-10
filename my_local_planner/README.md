# yhy 局部规划器：智慧社区适配版

来源：`~/raicom_ws/src/my_local_planner`。原包保留，当前包在 `~/community_ws/src/smart-community` 单独适配。

保留“前视目标 + 误差反馈”思路，作了较大控制逻辑调整，测试结论针对适配版。

- 使用 move_base 提供的 TF buffer；全部成员状态按插件实例保存。
- 根据实际麦克纳姆底盘，输出二维平移速度，直路保持当前车身朝向。
- 位置与朝向都满足容差且转速稳定后才报告成功，终点控制不会再被后续指令覆盖。
- 预测一秒内完整矩形轮廓，检查障碍/禁行栅格；无合法轨迹返回失败。
- 纯旋转采用可配置最低转速避开已实测的低速停滞区，最终效果由在线实验评价。
- 移除 OpenCV 依赖、图像绘制、窗口及键盘事件循环。

运行（同时关闭 Gazebo 客户端、RViz）：

```bash
source ~/community_ws/devel/setup.bash
roslaunch smart_community_sim navigation.launch planner:=custom gui:=false rviz:=false
```

参数：`smart_community_sim/config/navigation/custom.yaml`。
状态与证据：`doc/导航修复学习记录.md`。AMCL 与 jie_ware 配合适配版均已有 21/21 点整圈通过报告。
