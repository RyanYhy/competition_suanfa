# 安装指南

[返回首页](../README.md) · [运行与验证](USAGE.md) · [开发与发布](DEVELOPMENT.md)

## 1. 环境与准备

| 项目 | 当前支持/使用版本 |
|---|---|
| 操作系统 | Ubuntu 20.04 x86_64（实测 20.04.6） |
| ROS / 仿真 | ROS Noetic / Gazebo Classic 11 |
| Python | 系统 Python 3.8，工作区独立 `.venv` |
| 图形 | 桌面 DISPLAY 和可用 OpenGL；相机需要渲染环境 |
| 推理 | CPU ONNX Runtime，无需 CUDA |
| 安装条件 | 网络、sudo 权限；工作区和仓库路径可写 |

安装脚本会检查操作系统，其他发行版会明确停止。Windows/macOS 可以准备 Ubuntu 20.04 虚拟机；虚拟机图形加速和性能不在当前实测范围。`--headless` 只关闭客户端和 RViz，不会取消相机的图形依赖；纯服务器 Xvfb/Mesa 组合尚未验收。

ROS Noetic 已结束官方维护，工程固定此环境用于现有 ROS 1 实验。[官方说明](https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160)

## 2. 获取全部源码

若尚未安装 Git：

```bash
sudo apt-get update
sudo apt-get install -y git
```

创建一个独立 catkin 工作区并克隆：

```bash
mkdir -p ~/community_ws/src
git clone https://github.com/RyanYhy/competition_suanfa.git ~/community_ws/src/smart-community
cd ~/community_ws/src/smart-community
```

SSH 用户可将 URL 换成 `git@github.com:RyanYhy/competition_suanfa.git`。私有仓库需要相应访问权限。目标目录已经存在时，请使用已有副本或换一个新路径，不要再次往非空目录克隆。

仓库名 `competition_suanfa` 与本地目录名不必相同。上述步骤刻意使用 `smart-community`，便于与指南、打包脚本一致；默认 Git 克隆成 `competition_suanfa` 也可以运行，只要仍在工作区 `src/` 下。

```text
community_ws/
└── src/
    └── smart-community/
        ├── smart_community_sim/
        ├── jie_ware/
        ├── my_local_planner/
        ├── tools/
        ├── docs/
        └── README.md
```

三个包和三个 ONNX 权重都在此仓库，不需要再克隆上游 `jie_ware` 或安装旧工作区。仓库没有采用 Git LFS，正常克隆应得到每个约 12 MB 的真实 `.onnx` 文件。

如果下载 GitHub 网页的源码 ZIP，应把内部根目录（通常为 `competition_suanfa-main`）放到 `~/community_ws/src/smart-community/`；不要多套一层同名包。仓库自带打包器生成的 ZIP 已以 `smart-community/` 为根目录，可以直接解压到工作区 `src/`。

## 3. 安装依赖并编译

在仓库根目录执行：

```bash
bash tools/setup.sh
```

此步骤会：

1. 安装编译工具、ROS Noetic 基础、Gazebo 11、RViz、导航、AMCL、Gmapping、TEB、cv_bridge、OpenCV 和中文字体。
2. 沿用已有 ROS 软件源；没有源时添加官方签名 key 和独立的 signed-by 源文件。
3. 创建 `~/community_ws/.venv/`，安装固定版本的视觉依赖。
4. 使用该 venv 的 Python 编译三个 ROS 包，生成工作区 `build/`、`devel/`。
5. 检查包路径、字体、地图/纹理，实际加载三个 ONNX 模型并执行 CPU 推理和示例车牌 OCR。

默认使用两个编译任务；内存充足时可以运行：

```bash
bash tools/setup.sh --jobs 4
```

已有完整 ROS、编译及系统依赖时跳过 apt：

```bash
bash tools/setup.sh --skip-apt
```

`--skip-apt` 仍会安装固定 Python 依赖、编译和检查；它不表示可以缺少 ROS/OpenCV/字体。失败后处理具体错误再重跑，不需要复制别人的编译产物。

### Python 依赖设计

使用 `.venv --system-site-packages` 继承 apt 安装的 `cv2`、`cv_bridge` 和 ROS 模块，pip 安装的视觉库留在工作区内。脚本用 `python -m pip`，不要求额外全局 `pip` 可执行文件。

主要版本：NumPy 1.24.4、ONNX Runtime 1.16.3、RapidOCR 1.3.24、Pillow 9.5.0。完整依赖见 [requirements-vision.txt](../smart_community_sim/requirements-vision.txt)。Pillow 版本保留当前代码使用的 `ImageFont.getsize` 接口，NumPy 固定以兼容此版 ORT。

安装使用 `--no-deps`，所有需要的 pip 依赖已显式列出，避免 RapidOCR 自动再装一份 pip OpenCV。`pip check` 可能报告缺少 `opencv-python` 分发元数据，因为实际模块由 apt 的 `python3-opencv` 提供；验证以真实导入和模型推理检查为准。OCR 权重包含在 RapidOCR Python 包中，三个 YOLO 权重包含在仓库中，无需另设隐藏模型缓存。

## 4. 确认安装成功

安装脚本成功时，结尾会出现模型类别表、示例车牌 OCR 结果、`Installation checks passed` 和 `Ready. Run: ...`。示例车牌应读为 `苏APL12A`；OCR 分数可能随环境略变。

也可在新终端独立复查：

```bash
source ~/community_ws/.venv/bin/activate
source ~/community_ws/devel/setup.bash
cd ~/community_ws/src/smart-community
python3 tools/check_environment.py --inference
```

检查应解析到当前仓库的三个 ROS 包，而不是旧工作区；`numpy`、`onnxruntime`、`rapidocr_onnxruntime` 应来自工作区 `.venv`，`cv2` 和 `cv_bridge` 来自系统/ROS 安装。

然后在桌面终端启动：

```bash
bash tools/run.sh
```

下一步按照 [使用说明](USAGE.md) 验证传感器、导航和本轮识别任务。

## 5. 安装故障排查

| 现象 | 排查与处理 |
|---|---|
| 操作系统检查失败 | 使用 Ubuntu 20.04；本工程的 ROS 1/Noetic 不能直接当作 ROS 2 工程编译。 |
| apt 无法连接或找不到 ROS 包 | 检查网络和 Ubuntu focal/ROS 1 软件源；ROS 2 的软件源不能提供 Noetic 包。 |
| ROS key 过期或 signed-by 冲突 | 按 [ROS 官方签名迁移说明](https://discourse.ros.org/t/ros-signing-key-migration-guide/43937) 修复已有源；脚本不自动删除用户原有源。 |
| sudo 需要密码 | 在自己的终端输入本机管理员密码；没有 sudo 权限时请先准备系统依赖，再用 `--skip-apt`。 |
| 下载 Python 包失败 | 检查网络或 pip 源后重跑；固定版本不应随意替换。 |
| NumPy、ORT 或 Pillow 导入/接口错误 | 激活该工作区 `.venv`，重新执行 setup；避免 Conda/旧工作区或另一个 venv 混入。 |
| 找到的 ROS 包在旧工作区 | 新开终端运行 `tools/run.sh`，或先激活 venv 再 source 新 `devel/setup.bash`。 |
| 中文字体缺失 | `sudo apt-get install fonts-noto-cjk` 后重试。 |
| 编译中进程被杀 | 保持默认 `--jobs 2`，检查内存和磁盘空间；按首次失败的编译错误处理。 |
| 搬迁工作区后脚本指向旧路径 | 在新路径重新执行 setup/编译；venv 和 devel 不作为跨电脑发布文件。 |

本次实测覆盖现有 Ubuntu 主机的 `--skip-apt`、全新 Python venv、独立编译和在线运行；全新操作系统的 apt 安装分支尚未实际执行。[验证范围](../README.md#验证状态与已知问题)
