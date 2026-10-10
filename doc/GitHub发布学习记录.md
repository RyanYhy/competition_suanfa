# GitHub 发布学习记录

## 问题与通过条件

2026-10-10：用户希望把当前智慧社区仿真工程交给别人，在另一台电脑运行，并获得可以上传到 GitHub 的版本。

通过条件：提供包含三个 ROS 包、运行素材及已有 YOLO 模型的源码压缩包；补齐依赖安装、首次编译、启动和结果查看说明；排除本机编译产物、历史日志、验证图片、嵌套 Git 仓库和工作区绝对路径。至少在新的路径下进行独立编译、依赖检查和实际仿真启动验证，并说明验证范围。此轮准备本地发布文件，不执行远程上传。

## 证据

- 实测：工作区包含 `smart_community_sim`、`jie_ware`、`my_local_planner`，根目录尚无 Git 仓库；`jie_ware/.git` 是已有嵌套仓库。
- 实测：工作区顶层 `CMakeLists.txt` 是指向 `/opt/ros/noetic` 的生成链接，不能作为可移植源码照搬。
- 实测：三个 ONNX 模型已位于 `smart_community_sim/model/`；当前识别输出和大量历史实验数据不属于首次运行所必需的源码。
- 待验证：换路径后的 catkin 编译、Python 视觉依赖、资源路径及启动流程。

## 假设与原理

catkin 可以扫描 `工作区/src/仓库名/` 下的多个 ROS 包。发布时保留包内源码和运行资源，由新工作区重新生成 `build/`、`devel/`；这样不会携带本机二进制及生成文件中的绝对路径。Python 依赖需要按 ROS Noetic 的 Python 版本验证，不能只复制当前电脑已经安装的软件。

## 实验与修改

当前方案：检查运行依赖和资源来源 → 增加安装、检查、运行及打包脚本 → 建立明确的发布文件清单 → 在临时工作区解包并独立编译、启动 → 生成压缩包与复现说明。原工作区的模型、历史数据和嵌套 Git 历史保留。

### 依赖与路径修订

- 实测：主机是 Ubuntu 20.04.6 / Python 3.8.10。此前四行 `requirements-vision.txt` 依赖当前电脑预装的库，不能完整复现 ONNX Runtime 的 coloredlogs、flatbuffers、protobuf、sympy 等依赖。
- 实测：`recognition_overlay.py` 使用 `ImageFont.getsize`。因此固定 Pillow 9.5.0，而不是安装已移除此接口的 Pillow 10；NumPy 固定 1.24.4，避免 NumPy 2 与旧 ONNX Runtime 的兼容问题。
- 修改：列出全部视觉 Python 运行依赖；通过工作区 `.venv` 继承 apt 的 OpenCV/cv_bridge，使用 `--no-deps` 防止 RapidOCR 再安装一份 pip OpenCV。此方式按功能检查，不把缺少 pip `opencv-python` 分发元数据误判成缺少实际 `cv2` 模块。
- 修改：场地生成器优先复制原始纹理，原文件未分发时复用包内纹理；增加 21 张识别素材的 SHA256 校验，移植后的测试仍检查纹理完整性，本机有原文件时额外比较原始文件。
- 修改：结果目录保护改为拒绝当前包及其所有上级目录，去除固定的 `suanfa_ws` 名称，保留对非专用目录的保护。
- 修改：增加 `tools/setup.sh`、`run.sh`、`check_environment.py`、`smoke_test.py`、`package_release.py`，发布采用明确文件范围，排除嵌套 `.git`、原始 PDF/素材目录、编译产物、备份源码及运行结果。
- 失败实验：主机直接运行 `python3 -m venv` 提示缺少 `ensurepip`（未安装 python3-venv），证明不能假设当前环境已有虚拟环境工具。安装脚本已明确包含此系统依赖，下一步补齐验证环境后再试。

### 换路径编译与真实模型检查

- 实测：主机 sudo 需要密码，本轮没有执行系统 apt 安装。为利用现有 ROS 依赖继续验证，脚本改为 `venv --system-site-packages --without-pip`，用 apt 的 `python -m pip` 安装到 venv 的 `sys.prefix`；`--ignore-installed` 确保所列依赖实际安装到新 venv，而不是依赖恰好存在的全局 pip 库。未绕过权限或修改系统 Python。
- 实测：从发布 ZIP 解出全新的临时 catkin 工作区，执行 `tools/setup.sh --skip-apt` 成功。三个包均重新编译，ROS 包解析全部指向新工作区，三个模型成功执行 CPU 推理；示例车牌 OCR 输出 `苏APL12A`，分数约 0.9812。
- 实测：缺少原始 `doc/复赛资料` 的发布树可以重新生成场地，两个 world、材料脚本和场景清单的 SHA256 与生成前完全相同。没有把缺失原素材的检查简单跳过。
- 实测：压缩包静态审计通过，212 个源码/资源文件、3 个完整 ONNX 权重，最大文件 12,266,095 字节；没有 `.git`、validation、缓存、build/devel、PDF、废弃备份文件或符号链接。
- 修改：默认启动脚本只打印环境检查汇总和错误，完整依赖路径留在首次安装/显式检查命令，保持巡检终端简洁。打包脚本发现目标目录包含 `.git` 时拒绝覆盖，防止下次打包丢失用户自己建立的提交历史。
- 待验证：新路径的 Gazebo 在线传感器、定位 TF、视觉服务与自主巡检；系统 apt 安装分支尚未在全新操作系统执行。

编译和推理原始记录：[setup_and_build.log](validation/20261010_github_release/setup_and_build.log)。发布审计：[archive_audit.json](validation/20261010_github_release/archive_audit.json)。

### 离线回归与在线启动

- 实测：新路径下全部 55 项单元回归通过，包括素材 SHA、目录保护和巡检报告逻辑：[relocated_unit_tests.log](validation/20261010_github_release/relocated_unit_tests.log)。
- 实测：`LIBGL_ALWAYS_SOFTWARE=1 bash tools/run.sh --headless` 实际启动 Gazebo、默认激光定位、自定义规划器和视觉；启动检查收到 `/clock`、`/map`、`/scan`、`/odom`、相机图像/内参和巡检状态，`map → base_footprint` TF 及视觉报告服务可用，相机 1280×960：[启动报告](validation/20261010_github_release/relocated_startup_report.json)。
- 实测：临时初始化发布目录 Git 并执行 `git add --dry-run .`，213 个文件会进入普通提交，包含完整三个模型，不含输出目录、缓存或导出标记；未创建提交或 remote，测试用空 `.git` 随后移除，原 `jie_ware/.git` 不变：[Git 审计](validation/20261010_github_release/git_staging_audit.json)。
- 观察：启动阶段出现两次 scan 更新约 0.31～0.32 秒、超过配置 0.30 秒的 WARN，原样保留在日志；后续巡检已完成 12/19 点，尚未确认整圈与最后识别汇总。

在线原始日志：[relocated_live_launch.log](validation/20261010_github_release/relocated_live_launch.log)。关闭 Gazebo 客户端与 RViz 不等于相机无需图形环境；本轮仍使用主机 DISPLAY 和软件 OpenGL。

### 完整巡检结果与打包目录修复

- 实测：19/19 航点完成，两处路口均到达出口，三张车牌识别为 `闽D5070Z`、`苏D7070A`、`苏APL12A`。A/B 均确认 9 个独立人物，但 A 区统计为社区9/非社区0，B 区社区8/非社区1；A 区非社区人员被误判，完整视觉验收**未通过**。这是实际失败结果，未改阈值、人物类别或验收标准来补成通过。
- 实测：10 个识别点报告、14 张 YOLO 图片；公开目录恰好三项，终端日志无 ANSI 控制字符且以任务汇总结束。原工作区上次的展示结果没有被此次临时工作区测试覆盖。
- 数据：[验证摘要](validation/20261010_github_release/verification_summary.json)、[本轮终端日志](validation/20261010_github_release/relocated_terminal.log)、[本轮总览](validation/20261010_github_release/relocated_overview.jpg)、[YOLO 图片](validation/20261010_github_release/relocated_yolo_images)、[详细巡检报告](validation/20261010_github_release/relocated_patrol_report.json)。
- 失败实验：在工作区 `src/release/` 放入三个源码副本后，`catkin_pkg.packages.find_packages('.')` 报三个同名包重复。`.gitignore` 只影响 Git，不会让 catkin 忽略目录，因此不能靠忽略提交解决构建扫描问题。
- 修改与实测：打包器在 `release/` 父目录生成 `CATKIN_IGNORE`，标记不进入源码 ZIP。原工作区重新扫描恰好仍为三个原包；解压到新工作区的仓库不含该标记，因此可以正常编译：[扫描审计](validation/20261010_github_release/catkin_scan_audit.json)。
- 发布说明保留本轮分类误判与当前验证范围；没有把 19 点路线完成写成所有识别任务通过，也没有把当前参考地图写成实际 SLAM 验收地图。

复现命令（在 Ubuntu 20.04 / ROS Noetic 的已有系统依赖环境）：

```bash
mkdir -p /tmp/community_release_repeat/src
unzip release/smart-community-github.zip -d /tmp/community_release_repeat/src
cd /tmp/community_release_repeat/src/smart-community
bash tools/setup.sh --skip-apt
source /tmp/community_release_repeat/.venv/bin/activate
source /tmp/community_release_repeat/devel/setup.bash
python3 -m unittest discover -s smart_community_sim/tests -v
LIBGL_ALWAYS_SOFTWARE=1 bash tools/run.sh --headless
# 另一终端加载同一环境后：python3 tools/smoke_test.py
# 等整圈结束，检查 smart_community_sim/validation/终端日志.log 末尾汇总。
```

## 结论与下一步

发布源码和脚本已完成，独立编译、实际模型推理、55 项回归、在线启动、19 点导航及三张车牌识别通过；本轮人物角色分类未全部通过。GitHub 上传说明已提供，本轮仅准备本地文件，没有远程发布。系统安装分支没有 sudo 权限执行，其他电脑/全新操作系统、Windows/macOS 原生运行、实际 SLAM 建图和实车均未验收。后续优先研究人物类别随视角变化的稳定性；原始失败证据已保留。

本地发布产物：`release/smart-community/` 干净源码目录、`release/smart-community-github.zip`（约 49.1 MiB）和同名 `.sha256`。提交源码目录的文件，ZIP 用于下载分享或 GitHub Release；每次打包会刷新文件清单与校验和。验证完成后已关闭此次测试 ROS/Gazebo。

## 2026-10-10：提交到用户指定 GitHub 仓库

### 问题与通过条件

用户提供空仓库 `git@github.com:RyanYhy/competition_suanfa.git`，明确要求提交并上传，同时编写规范使用说明与安装指南。此次授权包含提交和推送，不再停留在本地压缩包。

通过条件：README 使用真实仓库链接，安装与日常使用分开说明；提交三个包、模型和必需资源，排除本机生成数据与嵌套 Git 历史；验证文档命令与实际代码一致，建立可长期维护的独立 Git 工作副本；推送后核对远端 main 的提交和文件。

### 证据

- 实测：Git 版本为 2.25.1，已有用户姓名/邮箱配置，首次提交沿用现有配置。
- 实测：上一轮源码发布和换路径编译/启动测试已完成；当前 README 的克隆地址仍为占位符，安装、使用和发布混在一篇说明里，需要整理。
- 待验证：SSH 仓库读写权限、远端是否为空、实际推送与远端回读。

### 假设与原理

将干净发布树同步到独立 Git 副本，原 ROS 工作区与 `jie_ware/.git` 保留。模型属于运行必需资源，使用普通 Git 提交；生成结果只保留在本地。推送成功后以远端提交哈希和再次克隆检查为证据。

### 实验与修改

1. 实测：通过 SSH 执行 `git ls-remote git@github.com:RyanYhy/competition_suanfa.git` 成功，返回无 refs，确认远端为空；无需新的 SSH 配置或用户再次授权。
2. 修改：README 使用真实仓库地址与可直接复制的快速开始命令，列出模块、输出目录和已知限制；新增 `docs/INSTALL.md`（安装指南）、`docs/USAGE.md`（使用说明）、`docs/DEVELOPMENT.md`（更新与发布），旧安装文档改为索引，避免多份相互冲突的操作说明。
3. 修改：源码打包清单包含 `docs/`，新导出包含 215 个源码/资源文件，三个完整模型随仓库提交。默认未启用 LFS，不提交源码 ZIP 或原始验证大文件。
4. 进行中：文档链接、包扫描、文件校验和及 Git 暂存检查，完成后建立独立 Git 工作副本并推送。

### 结论与下一步

本轮上传工作已开始，尚未推送；上一轮人物分类误判与实际 SLAM 未验收的范围保持如实记录。
