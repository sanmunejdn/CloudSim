# PointNet++ 训练工程

## 1. 概述

本工程用于训练 PointNet++ 模型，支持点云**分类**和**语义分割**两个任务。训练在软件外部完成，导出 ONNX 模型后部署到 CloudSim 的 PointNetPlugin 插件中。

## 2. 环境准备

### 2.1 Python 依赖

```bash
pip install -r requirements.txt
```

主要依赖：
- PyTorch >= 2.0
- ONNX Runtime >= 1.16
- numpy, plyfile, pyyaml, tqdm

### 2.2 GPU 要求

- 训练：推荐 NVIDIA GPU，显存 >= 4GB
- 推理：CPU 或 GPU 均可

## 3. 数据准备

### 3.1 使用合成数据（快速开始）

```bash
# 生成分类数据（每类 100 个样本，共 500）
python scripts/gen_synthetic_data.py --output datasets/classification --task cls --num-per-class 100

# 生成分割数据（50 个样本）
python scripts/gen_synthetic_data.py --output datasets/segmentation --task seg --num-samples 50
```

### 3.2 自定义数据集格式

**分类数据集** (`datasets/classification/`):
```
datasets/classification/
├── dataset.jsonl        # 标注文件
└── data/                # 点云文件
    ├── box_0001.ply
    ├── cylinder_0001.ply
    └── ...
```

`dataset.jsonl` 每行格式：
```json
{
  "instruction": "识别点云类型",
  "input": "box_0001.ply",
  "output": "{\"class_id\": 0, \"class_name\": \"box\"}"
}
```

**分割数据集** (`datasets/segmentation/`):
```
datasets/segmentation/
├── dataset.jsonl
└── data/
    ├── sample_0001.ply
    ├── sample_0001_labels.npy
    └── ...
```

`dataset.jsonl` 每行格式：
```json
{
  "instruction": "分割网格部件",
  "input": "sample_0001.ply",
  "output": "{\"label_file\": \"sample_0001_labels.npy\", \"num_classes\": 4}"
}
```

### 3.3 验证数据集

```bash
python scripts/build_dataset.py classification
python scripts/build_dataset.py segmentation
```

### 3.3 CloudSim 交互标注导出

在 CloudSim 中安装 **LabelingPlugin**（`com.cloudsim.labeling`）后：

1. 侧栏 **分割标注** Tab：点选/刷选/套索打标签，导出到 `datasets/segmentation/`（或自定义目录）。
2. 侧栏 **模型训练** Tab：启动 `run_seg_training_job.py`，查看 `metrics.jsonl`，一键部署 ONNX 到 `PointNetPlugin`。

宿主版本要求 **1.16.0+**（`IPluginLabelingHost`）。

## 4. 训练

### 4.1 分类训练

```bash
python scripts/train_cls.py --config configs/cls_config.yaml
```

### 4.2 分割训练

```bash
python scripts/train_seg.py --config configs/seg_config.yaml
```

**CloudSim UI**（推荐）：

```bash
python scripts/run_seg_training_job.py \
  --config configs/seg_config.generated.yaml \
  --metrics-file output/seg/metrics.jsonl \
  --summary-file output/seg/training_summary.json
```

`run_seg_training_job.py` 会在无 `split` 字段时自动 80/20 划分 train/val，并写入结构化指标供训练 Tab 解析。

### 4.3 从断点恢复

```bash
python scripts/train_cls.py --config configs/cls_config.yaml --resume output/cls/checkpoint_10.pth
```

## 5. 导出 ONNX

### 5.1 标准导出

```bash
# 分类模型
python scripts/export_onnx.py --task cls --config configs/cls_config.yaml \
    --checkpoint output/cls/best.pth --output models/pointnet_cls.onnx

# 分割模型
python scripts/export_onnx.py --task seg --config configs/seg_config.yaml \
    --checkpoint output/seg/best.pth --output models/pointnet_seg.onnx
```

### 5.2 ONNX 兼容导出（推荐）

标准 PointNet++ 的 FPS（最远点采样）使用动态操作，可能无法直接导出 ONNX。
训练工程提供了 ONNX 兼容版本 `models/pointnet2/pointnet2_cls_onnx.py`，用均匀采样替代 FPS：

```python
import sys
sys.path.insert(0, '.')
from models.pointnet2.pointnet2_cls_onnx import export_to_onnx

export_to_onnx('output/cls/best.pth', 'models/pointnet_cls.onnx', num_classes=5, num_points=1024)
```

### 5.3 依赖问题处理

如果 `pip install onnx` 因路径过长失败，可安装到短路径：

```bash
pip install onnx --no-deps --target "d:\py_packages"
# 然后在脚本中添加：sys.path.insert(0, 'd:/py_packages')
```

## 6. 部署

将导出的 ONNX 文件复制到插件目录：

```bash
# 复制到插件 models 目录
cp models/pointnet_cls.onnx bin/x64d/plugins/com.cloudsim.pointnet/models/
cp models/pointnet_seg.onnx bin/x64d/plugins/com.cloudsim.pointnet/models/
```

修改 `pointnet_config.json` 配置文件（位于插件目录或 exe 同目录）：

```json
{
  "models": {
    "classify": {
      "path": "models/pointnet_cls.onnx",
      "num_points": 1024,
      "classes": ["box", "cylinder", "sphere", "cone", "complex"]
    },
    "segment": {
      "path": "models/pointnet_seg.onnx",
      "num_points": 2048,
      "num_classes": 6
    }
  },
  "inference": {
    "provider": "cpu"
  }
}
```

## 7. 模型架构

### 7.1 PointNet2Cls（分类）

```
输入: [B, N, 3]
  → SA1(512, r=0.2, k=32) → [B, 512, 128]
  → SA2(128, r=0.4, k=64) → [B, 128, 256]
  → SA3(global)            → [B, 1024]
  → FC(512) → FC(256) → FC(num_classes)
输出: [B, num_classes]
```

**ONNX 兼容版本**（`pointnet2_cls_onnx.py`）：
- 用均匀采样替代 FPS（最远点采样），避免动态操作
- 用 `torch.cdist` + `topk` 替代 ball query，保持 ONNX 兼容
- 训练时使用标准版本，导出时使用 ONNX 版本

### 7.2 PointNet2Seg（分割）

```
输入: [B, N, 3]
  编码器:
    SA1(512, r=0.2, k=32) → [B, 512, 128]
    SA2(128, r=0.4, k=64) → [B, 128, 256]
    SA3(global)            → [B, 1024]
  解码器:
    FP3 → [B, 128, 256]
    FP2 → [B, 512, 128]
    FP1 → [B, N, 128]
  分割头:
    Conv1d → Conv1d → [B, N, num_classes]
输出: [B, N, num_classes]
```

## 8. 测试

```bash
python scripts/test_model.py
```

## 9. 目录结构

```
tools/pointnet-training/
├── README.md                      # 本文档
├── requirements.txt               # Python 依赖
├── configs/
│   ├── cls_config.yaml            # 分类训练配置
│   └── seg_config.yaml            # 分割训练配置
├── datasets/
│   ├── classification/
│   │   ├── dataset.jsonl
│   │   └── data/
│   └── segmentation/
│       ├── dataset.jsonl
│       └── data/
├── scripts/
│   ├── train_cls.py               # 分类训练
│   ├── train_seg.py               # 分割训练
│   ├── export_onnx.py             # ONNX 导出
│   ├── build_dataset.py           # 数据集验证
│   ├── gen_synthetic_data.py      # 合成数据生成
│   └── test_model.py              # 模型测试
└── models/
    └── pointnet2/
        ├── __init__.py
        ├── pointnet2_utils.py     # 核心模块（FPS, Ball Query, SA, FP）
        ├── pointnet2_cls.py       # 分类网络（训练用）
        ├── pointnet2_cls_onnx.py  # 分类网络（ONNX 导出用，均匀采样替代 FPS）
        └── pointnet2_seg.py       # 分割网络
```

## 10. 与 CloudSim 集成

### 10.1 AI 域

插件注册了两个 AI 域：
- `pointnet.classify`：点云/网格分类
- `pointnet.segment`：点云/网格语义分割

### 10.2 使用方式

1. 在 AI 助手面板选择 "PointNet++ 分类" 或 "PointNet++ 分割" 域
2. 选中场景中的点云或网格对象
3. 输入自然语言指令（如 "识别类型" 或 "分割部件"）
4. 插件自动提取点云数据、运行推理、返回结果

### 10.3 数据流

```
用户输入 → AI Dock → PointNetPlugin
  → 导出选中对象为临时 PLY
  → 读取 xyz 坐标
  → 预处理（采样/归一化）
  → ONNX Runtime 推理
  → 返回分类/分割结果
```
