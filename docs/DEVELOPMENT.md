# 开发、更新与发布

[返回首页](../README.md) · [安装指南](INSTALL.md) · [使用说明](USAGE.md)

## 更新与编译

在 Git 工作副本内确认修改后再拉取，使用快进更新避免意外合并：

```bash
git status --short
git pull --ff-only
```

有未提交修改时先处理自己的修改。依赖没有变动时，在已安装的工作区重新编译：

```bash
source ~/community_ws/.venv/bin/activate
source ~/community_ws/devel/setup.bash
catkin_make -C ~/community_ws -j2 -l2 -DCMAKE_BUILD_TYPE=Release
```

首次安装或 Python 依赖变动使用 `bash tools/setup.sh`；系统依赖齐全时加 `--skip-apt`。C++ 插件改变后重新编译并关闭旧 ROS/Gazebo 再启动，旧进程不会自动加载新的库。

## 检查

从仓库根目录，在已激活 venv 和加载 devel 环境的终端执行：

```bash
python3 tools/check_environment.py --inference
python3 -m unittest discover -s smart_community_sim/tests -v
```

安装/单元检查之后，涉及运行行为的改动按 [使用说明](USAGE.md) 做在线验证。离线通过、在线启动、整圈导航、完整识别、实际 SLAM 与实车验收分别记录，不能互相代替。

工程学习记录放 `doc/<主题>学习记录.md`，按照“问题与通过条件 → 证据 → 假设与原理 → 实验与修改 → 结论与下一步”维护。保留失败实验及撤回原因，标注实测/推断/未验证；原始数据放不提交的 `doc/validation/`。

## 修改场地

生成好的 world、纹理、模型和地图已经包含。改变几何布置后执行：

```bash
python3 smart_community_sim/scripts/generate_field.py
python3 smart_community_sim/scripts/generate_navigation_maps.py
```

场地生成器优先复制本机原始比赛素材；原文件不在 Git 仓库时复用 `materials/textures/` 中打包的纹理，不能先删除该目录。识别素材有 SHA256 记录，修改纹理时同步生成配置并验证。

禁行图、参考地图和巡检路线需保持相同坐标。使用实际 SLAM 图时需要另外对齐禁行区域，不能靠生成脚本消除 SLAM 漂移。

## 打包源码

```bash
python3 tools/package_release.py
```

输出：

```text
release/
├── CATKIN_IGNORE                  # 阻止 catkin 扫描导出的重复包
├── smart-community/               # 干净源码树，不含 .git
│   └── release_manifest.json      # 逐文件大小和 SHA256
├── smart-community-github.zip
└── smart-community-github.zip.sha256
```

校验压缩包：

```bash
cd release
sha256sum -c smart-community-github.zip.sha256
```

源码包包含三个包、三个真实 ONNX 模型、运行纹理/地图、工具、指南和学习笔记。排除 build/devel/install、venv、缓存、日志/识别结果、原始 PDF/资料目录、嵌套 Git 仓库和未编译备份源码。`release/CATKIN_IGNORE` 不进入 ZIP，解压到新工作区后可正常扫描三个包。

再次打包会覆盖脚本自己生成的源码树和 ZIP；目标中发现 `.git` 会拒绝覆盖。不要在可再生成的导出目录中维护唯一的 Git 历史。

## 提交到本仓库

在长期维护的 Git 工作副本中提交源码：

```bash
git status --short
git add README.md docs tools smart_community_sim jie_ware my_local_planner doc
# 按实际修改选择文件，检查 diff 后提交
git diff --cached --stat
git commit -m "Describe the concrete change"
git push origin main
```

`.gitignore` 排除了编译产物和结果，但仍需检查实际暂存范围。三个约 12 MB 的权重使用普通 Git；更换训练模型会增加仓库历史，更新时保留大小/校验信息及实际识别测试结果。源码 ZIP 可以作为 GitHub Release 附件，仓库提交解压后的文件，不提交 ZIP 本体。

SSH remote 为 `git@github.com:RyanYhy/competition_suanfa.git`，普通下载可用 HTTPS 地址。首次配置 Git 时使用自己的姓名和邮箱，不将密码、令牌或私钥写入源码或 URL。[GitHub 官方说明](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github)

## 原开发工作区与发布副本

作者原 `suanfa_ws/src/` 包含不提交的比赛资料和研究证据，`jie_ware` 还保留上游嵌套 Git 历史。首次上传使用干净导出树建立独立 Git 副本，避免将上游包误提交为缺源码的 gitlink。

后续在原工作区开发时，先验证并运行打包器，再将需要更新的源码同步到 Git 副本，保留该副本自己的 `.git`，检查差异后提交。也可以直接在按安装指南克隆的 Git 工作副本中继续开发；不要让两个副本同时成为互相冲突的唯一源码。

历史笔记中的原始数据链接只在作者工作区存在，未随仓库发布；它们用于保留研究过程，当前安装和操作以 `docs/` 为准。来源、许可与未指定许可范围见 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)。
