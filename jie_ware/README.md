# 阿杰的 ROS 工具箱

## 视频介绍

Bilibili: [《一种简单易用的激光雷达定位方法》](https://www.bilibili.com/video/BV1fB29YzEgP/)  
Youtube: [《一种简单易用的激光雷达定位方法》](https://www.youtube.com/watch?v=0JqGX8lKRu0)  
Bilibili: [《代价地图清除》](https://www.bilibili.com/video/BV1kwzqYyEe7/)  
Youtube: [《代价地图清除》](https://www.youtube.com/watch?v=giHf_PY4EmY)  
Bilibili: [《去除ROS导航中的激光雷达噪点》](https://www.bilibili.com/video/BV1LFjBzREQu)  
Youtube: [《去除ROS导航中的激光雷达噪点》](https://www.youtube.com/watch?v=98GF6_zN_IA)  
基础：[《ROS 快速入门教材》](https://www.bilibili.com/video/BV1BP4y1o7pw/)  
扩展：[《ROS 导航，除了 DWA 和 TEB 还有没有其他选择？》](https://www.bilibili.com/video/BV1nQR4YsESM/)

## 下载源代码

1. 获取源码:
```
cd ~/catkin_ws/src/
git clone https://github.com/6-robot/jie_ware.git
```
国内镜像：
```
cd ~/catkin_ws/src/
git clone https://gitee.com/s-robot/jie_ware.git
```
2. 编译
```
cd ~/catkin_ws
catkin_make
```
## 激光定位
1. 修改导航Launch文件，用如下内容替换AMCL节点：
```
<node pkg="jie_ware" type="lidar_loc" name="lidar_loc" >
    <param name="base_frame" value="base_footprint" />
    <param name="odom_frame" value="odom" />
    <param name="laser_frame" value="laser" />
    <param name="laser_topic" value="scan" />
</node>
```
2. 运行修改后的Launch文件
```
roslaunch jie_ware lidar_loc_test.launch 
```
## 代价地图清除
1. 修改导航Launch文件，添加如下内容：
```
<node pkg="jie_ware" type="costmap_cleaner" name="costmap_cleaner" />
```
2. 运行修改后的Launch文件，正常设置机器人的估计位置即可。
## 激光雷达滤波
1. 修改导航Launch文件，添加如下内容：
```
<node pkg="jie_ware" type="lidar_filter_node" name="lidar_filter_node" />
```
2. 修改代价地图参数文件 costmap_common_params.yaml ，将激光雷达数据话题 topic 从 scan 修改为 scan_filtered 。
```
  observation_sources: scan
  
  scan:
    data_type: LaserScan
    topic: scan_filtered
```
3. 运行修改后的Launch文件，正常进行导航。

## 智慧社区工作区适配说明（2026-10-09）

原工作区 `~/raicom_ws/src/jie_ware` 保留。当前 `lidar_loc.cpp` 保留邻域地图匹配思路，进行了以下修改；效果结论仅针对适配版：

- 用标准 ROS 笛卡尔坐标与扫描消息的雷达 frame 进行 TF 外参变换，避免图片翻转坐标混入雷达外参。
- 用障碍距离场实现平滑梯度评分；粗搜索 2 cm/1°，细搜索 5 mm/0.25°，有迭代上限。
- 轮式 odom 只提供每次扫描的初始预测；用同一扫描时间的 odom 计算 map→odom，机器人移动由 odom 插值，不对旧扫描车身角度低通而造成运动滞后。
- 从 launch 提供已知起点，支持 map frame 的 `/initialpose`；地图未到或 TF 未到时不计算。
- 匹配分数过低拒绝更新，扫描超过 0.5 s 不继续发布 TF；不会启动自动清除比赛禁行层的节点。
- 发布 `/lidar_pose`、`/lidar_match_score` 供诊断，没有可视化窗口，没有仿真真值订阅。

在当前场地使用：

```bash
source ~/community_ws/devel/setup.bash
roslaunch smart_community_sim navigation.launch localization:=lidar planner:=custom gui:=false rviz:=false
```

请勿直接使用本包原来的 `lidar_loc_test.launch` 启动智慧社区，它是上游其他机器人场景。仅启动一个定位后端，避免与 AMCL 同时发布 map→odom。当前验证状态见 `doc/导航修复学习记录.md`。
