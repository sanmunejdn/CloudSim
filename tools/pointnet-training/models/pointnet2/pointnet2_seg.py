"""PointNet++ 分割模型"""

import torch
import torch.nn as nn
import torch.nn.functional as F

from .pointnet2_utils import SetAbstraction, FeaturePropagation


class PointNet2Seg(nn.Module):
    """PointNet++ 分割网络（SSG 变体）

    输入: [B, N, 3] 点云坐标
    输出: [B, N, num_classes] 每点分类 logits
    """

    def __init__(self, num_classes: int, num_points: int = 2048,
                 dropout: float = 0.5):
        super().__init__()
        self.num_classes = num_classes
        self.num_points = num_points

        # Encoder (Set Abstraction)
        self.sa1 = SetAbstraction(
            npoint=512, radius=0.2, nsample=32,
            in_channel=3, mlp_channels=[64, 64, 128]
        )
        self.sa2 = SetAbstraction(
            npoint=128, radius=0.4, nsample=64,
            in_channel=128 + 3, mlp_channels=[128, 128, 256]
        )
        self.sa3 = SetAbstraction(
            npoint=None, radius=None, nsample=None,
            in_channel=256 + 3, mlp_channels=[256, 512, 1024],
            group_all=True
        )

        # Decoder (Feature Propagation)
        self.fp3 = FeaturePropagation(in_channel=1024 + 256, mlp_channels=[256, 256])
        self.fp2 = FeaturePropagation(in_channel=256 + 128, mlp_channels=[256, 128])
        self.fp1 = FeaturePropagation(in_channel=128 + 3, mlp_channels=[128, 128, 128])

        # 分割头
        self.conv1 = nn.Conv1d(128, 128, 1)
        self.bn1 = nn.BatchNorm1d(128)
        self.drop1 = nn.Dropout(dropout)
        self.conv2 = nn.Conv1d(128, num_classes, 1)

    def forward(self, xyz: torch.Tensor) -> torch.Tensor:
        """
        Args:
            xyz: [B, N, 3] 输入点云
        Returns:
            logits: [B, N, num_classes]
        """
        B, N, _ = xyz.shape

        # Encoder
        l1_xyz, l1_features = self.sa1(xyz, None)
        l2_xyz, l2_features = self.sa2(l1_xyz, l1_features)
        l3_xyz, l3_features = self.sa3(l2_xyz, l2_features)

        # Decoder
        l2_features = self.fp3(l2_xyz, l3_xyz, l2_features, l3_features)
        l1_features = self.fp2(l1_xyz, l2_xyz, l1_features, l2_features)
        l0_features = self.fp1(xyz, l1_xyz, xyz, l1_features)

        # 分割头
        x = l0_features.permute(0, 2, 1)  # [B, 128, N]
        x = self.drop1(F.relu(self.bn1(self.conv1(x))))
        x = self.conv2(x)  # [B, num_classes, N]
        x = x.permute(0, 2, 1)  # [B, N, num_classes]

        return x

    def predict(self, xyz: torch.Tensor) -> torch.Tensor:
        """返回每点预测类别"""
        logits = self.forward(xyz)
        return torch.argmax(logits, dim=-1)
