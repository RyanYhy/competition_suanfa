# YOLO 模型

三个权重已包含在源码包中，运行使用 CPU ONNX Runtime，无需安装 CUDA、PyTorch 或 Ultralytics。OCR 使用安装的 `rapidocr_onnxruntime==1.3.24` 包内 PP-OCR 模型。

| 文件 | 用途 | 字节数 | SHA256 |
|---|---|---:|---|
| `doll_best.onnx` | 人物社区/非社区两类检测 | 12266095 | `62740e303aa7317872c190c9531b40986e104076f1b9e8e26823028e75ee4145` |
| `plate_best.onnx` | 车牌框检测；裁剪后交给 OCR 读字符 | 12265292 | `0edfbe19fb7305f6faa05fbea0d822c2cf985f20caf92a4f1ebdfbceb67f1c33` |
| `traffic_light_best.onnx` | 灯箱框检测；框内亮像素判断灯色 | 12265308 | `820321ba1d18fc67d3516cfcde817ddea65ab83c65941cc4f1d0506fe49b654c` |

运行 `python3 tools/check_environment.py --inference` 可以加载全部模型、打印模型内的类别表并执行真实 CPU 推理。需先按根目录 README 编译并激活工作区环境。

当前单个权重约 12 MB，可以随普通 Git 提交；默认没有使用 Git LFS。若以后改用 LFS，接收者还需下载真实权重，只有指针文件不能运行。阈值在 `smart_community_sim/config/vision.yaml`，识别统计来自相机，不从场景清单补填。
