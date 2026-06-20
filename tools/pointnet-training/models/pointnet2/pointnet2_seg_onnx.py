"""PointNet++ 分割模型（ONNX 导出版本）

Set Abstraction 用均匀采样替代 FPS，权重与训练模型结构对齐，strict=False 加载。
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

from .pointnet2_cls_onnx import SetAbstractionOnnx
from .pointnet2_utils import FeaturePropagation


class PointNet2SegOnnx(nn.Module):
    """PointNet++ 分割网络（ONNX 兼容）"""

    def __init__(self, num_classes: int, num_points: int = 2048, dropout: float = 0.0):
        super().__init__()
        self.num_classes = num_classes
        self.num_points = num_points

        self.sa1 = SetAbstractionOnnx(
            npoint=512, radius=0.2, nsample=32,
            in_channel=3, mlp_channels=[64, 64, 128],
        )
        self.sa2 = SetAbstractionOnnx(
            npoint=128, radius=0.4, nsample=64,
            in_channel=128 + 3, mlp_channels=[128, 128, 256],
        )
        self.sa3 = SetAbstractionOnnx(
            npoint=None, radius=None, nsample=None,
            in_channel=256 + 3, mlp_channels=[256, 512, 1024],
            group_all=True,
        )

        self.fp3 = FeaturePropagation(in_channel=1024 + 256, mlp_channels=[256, 256])
        self.fp2 = FeaturePropagation(in_channel=256 + 128, mlp_channels=[256, 128])
        self.fp1 = FeaturePropagation(in_channel=128 + 3, mlp_channels=[128, 128, 128])

        self.conv1 = nn.Conv1d(128, 128, 1)
        self.bn1 = nn.BatchNorm1d(128)
        self.drop1 = nn.Dropout(dropout)
        self.conv2 = nn.Conv1d(128, num_classes, 1)

    def forward(self, xyz: torch.Tensor) -> torch.Tensor:
        l1_xyz, l1_features = self.sa1(xyz, None)
        l2_xyz, l2_features = self.sa2(l1_xyz, l1_features)
        l3_xyz, l3_features = self.sa3(l2_xyz, l2_features)

        l2_features = self.fp3(l2_xyz, l3_xyz, l2_features, l3_features)
        l1_features = self.fp2(l1_xyz, l2_xyz, l1_features, l2_features)
        l0_features = self.fp1(xyz, l1_xyz, xyz, l1_features)

        x = l0_features.permute(0, 2, 1)
        x = self.drop1(F.relu(self.bn1(self.conv1(x))))
        x = self.conv2(x)
        return x.permute(0, 2, 1)


def export_to_onnx(
    checkpoint_path: str,
    output_path: str,
    num_classes: int,
    num_points: int = 2048,
) -> str:
    model = PointNet2SegOnnx(num_classes=num_classes, num_points=num_points, dropout=0.0)
    model.load_state_dict(torch.load(checkpoint_path, map_location='cpu'), strict=False)
    model.eval()

    dummy = torch.randn(1, num_points, 3)
    torch.onnx.export(
        model,
        dummy,
        output_path,
        input_names=['points'],
        output_names=['logits'],
        opset_version=13,
        dynamic_axes={'points': {0: 'batch'}, 'logits': {0: 'batch'}},
        dynamo=False,
    )
    return output_path
