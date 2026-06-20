"""将训练好的 PyTorch 模型导出为 ONNX 格式"""

import argparse
import os
import sys

import torch
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from python_io import configure_stdio_utf8
from models.pointnet2.pointnet2_cls_onnx import PointNet2ClsOnnx
from models.pointnet2.pointnet2_seg_onnx import PointNet2SegOnnx

configure_stdio_utf8()


def _export_onnx(model, dummy, output_path, input_names, output_names):
    torch.onnx.export(
        model,
        dummy,
        output_path,
        input_names=input_names,
        output_names=output_names,
        opset_version=13,
        dynamic_axes={
            input_names[0]: {0: 'batch'},
            output_names[0]: {0: 'batch'},
        },
        dynamo=False,
    )


def _validate_onnx_runtime(onnx_path: str, num_points: int) -> None:
    """用 ONNX Runtime 试加载并跑一次前向，与 CloudSim 加载行为一致。"""
    import numpy as np
    import onnxruntime as ort

    sess = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
    dummy = np.random.randn(1, num_points, 3).astype(np.float32)
    outputs = sess.run(None, {'points': dummy})
    logits = outputs[0]
    if logits.ndim != 3:
        raise RuntimeError(f'unexpected output rank: {logits.shape}')
    print(f'ONNX Runtime 推理通过: output shape={logits.shape}')


def export_cls(config_path: str, checkpoint: str, output_path: str):
    """导出分类模型"""
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    num_classes = len(cfg['classes'])
    num_points = cfg['num_points']

    model = PointNet2ClsOnnx(num_classes=num_classes, num_points=num_points, dropout=0.0)
    model.load_state_dict(torch.load(checkpoint, map_location='cpu'), strict=False)
    model.eval()

    dummy = torch.randn(1, num_points, 3)
    input_names = ['points']
    output_names = ['logits']

    _export_onnx(model, dummy, output_path, input_names, output_names)
    print(f"分类模型已导出: {output_path}")


def export_seg(config_path: str, checkpoint: str, output_path: str):
    """导出分割模型（ONNX 兼容结构，权重 strict=False 加载）"""
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    num_classes = cfg['num_classes']
    num_points = cfg['num_points']

    model = PointNet2SegOnnx(num_classes=num_classes, num_points=num_points, dropout=0.0)
    model.load_state_dict(torch.load(checkpoint, map_location='cpu'), strict=False)
    model.eval()

    dummy = torch.randn(1, num_points, 3)
    input_names = ['points']
    output_names = ['logits']

    _export_onnx(model, dummy, output_path, input_names, output_names)
    print(f"分割模型已导出: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Export PointNet++ to ONNX')
    parser.add_argument('--task', type=str, required=True, choices=['cls', 'seg'],
                        help='Task type: cls or seg')
    parser.add_argument('--config', type=str, required=True, help='Config YAML path')
    parser.add_argument('--checkpoint', type=str, required=True, help='PyTorch checkpoint path')
    parser.add_argument('--output', type=str, required=True, help='Output ONNX path')
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)

    if args.task == 'cls':
        export_cls(args.config, args.checkpoint, args.output)
        with open(args.config, 'r', encoding='utf-8') as f:
            num_points = yaml.safe_load(f)['num_points']
    else:
        export_seg(args.config, args.checkpoint, args.output)
        with open(args.config, 'r', encoding='utf-8') as f:
            num_points = yaml.safe_load(f)['num_points']

    try:
        import onnx
        model = onnx.load(args.output)
        onnx.checker.check_model(model)
        print("ONNX 结构验证通过")
    except Exception as e:
        print(f"ONNX 结构验证警告: {e}")
        raise SystemExit(1) from e

    try:
        _validate_onnx_runtime(args.output, num_points)
    except Exception as e:
        print(f"ONNX Runtime 验证失败: {e}")
        raise SystemExit(1) from e


if __name__ == '__main__':
    main()
