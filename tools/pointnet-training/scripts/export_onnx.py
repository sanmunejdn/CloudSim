"""将训练好的 PyTorch 模型导出为 ONNX 格式"""

import argparse
import os
import sys

import torch
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from models.pointnet2 import PointNet2Cls, PointNet2Seg


def export_cls(config_path: str, checkpoint: str, output_path: str):
    """导出分类模型"""
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    num_classes = len(cfg['classes'])
    num_points = cfg['num_points']

    model = PointNet2Cls(num_classes=num_classes, num_points=num_points, dropout=0.0)
    model.load_state_dict(torch.load(checkpoint, map_location='cpu'))
    model.eval()

    dummy = torch.randn(1, num_points, 3)
    input_names = ['points']
    output_names = ['logits']

    torch.onnx.export(
        model, dummy, output_path,
        input_names=input_names,
        output_names=output_names,
        opset_version=13,
        dynamic_axes={
            'points': {0: 'batch'},
            'logits': {0: 'batch'}
        }
    )
    print(f"分类模型已导出: {output_path}")


def export_seg(config_path: str, checkpoint: str, output_path: str):
    """导出分割模型"""
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    num_classes = cfg['num_classes']
    num_points = cfg['num_points']

    model = PointNet2Seg(num_classes=num_classes, num_points=num_points, dropout=0.0)
    model.load_state_dict(torch.load(checkpoint, map_location='cpu'))
    model.eval()

    dummy = torch.randn(1, num_points, 3)
    input_names = ['points']
    output_names = ['logits']

    torch.onnx.export(
        model, dummy, output_path,
        input_names=input_names,
        output_names=output_names,
        opset_version=13,
        dynamic_axes={
            'points': {0: 'batch'},
            'logits': {0: 'batch'}
        }
    )
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
    else:
        export_seg(args.config, args.checkpoint, args.output)

    # 验证
    try:
        import onnx
        model = onnx.load(args.output)
        onnx.checker.check_model(model)
        print("ONNX 模型验证通过")
    except Exception as e:
        print(f"ONNX 验证警告: {e}")


if __name__ == '__main__':
    main()
