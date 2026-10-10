# 代码、模型与素材来源

本仓库包含多个来源的代码与数据，不新增覆盖全部文件的统一许可证。

| 范围 | 来源及当前声明 |
|---|---|
| `jie_ware/` | 上游 [6-robot/jie_ware](https://github.com/6-robot/jie_ware)，原作者 Zhang Wanjie；保留原 README、作者信息和 [GPL v2 全文](jie_ware/LICENSE)。扫描匹配、TF、起点初始化等适配修改见包内 README。 |
| `my_local_planner/` | 用户提供的局部规划器及本工程适配。原 `package.xml` 的许可证字段仍为 `TODO`，尚未指定通用开源许可证。 |
| `smart_community_sim/` | 本工程场景、底盘插件、导航配置和视觉任务代码；`package.xml` 原有声明为 MIT。当前工程没有提供独立 MIT 许可文件，此次打包保留原声明。 |
| `smart_community_sim/model/*.onnx` | 用户提供的三个 YOLO 检测权重，按原文件分发；文件校验和及类别见 [模型说明](smart_community_sim/model/README.md)。权重未附独立许可文件。 |
| `smart_community_sim/materials/textures/` | 比赛方提供的人物、灯箱、车辆背景及示例车牌；用户提供的两张随机车牌；少量由场景脚本生成的纹理。源码包包含实际运行所需的纹理，不包含原始规则 PDF、技术报告模板和历史验收数据。素材未附独立许可文件。 |
| Python/ROS 系统依赖 | 由安装脚本从软件仓库安装，不复制本机环境。RapidOCR 1.3.24 使用其 Python 包附带的 PP-OCR 模型；无需用户另行提供 OCR 权重。依赖自身许可由其发行包保留。 |

这里记录已有声明与来源，不把第三方代码、训练权重或比赛素材重新声明为 MIT。GitHub 仓库可以先用于源码交流；后续若要为自写代码选择统一开源许可，应另外明确范围。
