"""PointNet++ 分类模型"""

import torch
import torch.nn as nn
import torch.nn.functional as F

from .pointnet2_utils import SetAbstraction


class PointNet2Cls(nn.Module):
    """PointNet++ 分类网络（SSG 变体）

    输入: [B, N, 3] 点云坐标
    输出: [B, num_classes] 分类 logits
    """

    def __init__(self, num_classes: int, num_points: int = 1024,
                 dropout: float = 0.5):
        super().__init__()
        self.num_classes = num_classes
        self.num_points = num_points

        # Set Abstraction 层
        # SA1: 512 点, 半径 0.2, 32 邻域
        self.sa1 = SetAbstraction(
            npoint=512, radius=0.2, nsample=32,
            in_channel=3 + 0, mlp_channels=[64, 64, 128]
        )
        # SA2: 128 点, 半径 0.4, 64 邻域
        self.sa2 = SetAbstraction(
            npoint=128, radius=0.4, nsample=64,
            in_channel=128 + 3, mlp_channels=[128, 128, 256]
        )
        # SA3: 全局聚合
        self.sa3 = SetAbstraction(
            npoint=None, radius=None, nsample=None,
            in_channel=256 + 3, mlp_channels=[256, 512, 1024],
            group_all=True
        )

        # 分类头
        self.fc1 = nn.Linear(1024, 512)
        self.bn1 = nn.BatchNorm1d(512)
        self.drop1 = nn.Dropout(dropout)
        self.fc2 = nn.Linear(512, 256)
        self.bn2 = nn.BatchNorm1d(256)
        self.drop2 = nn.Dropout(dropout)
        self.fc3 = nn.Linear(256, num_classes)

    def forward(self, xyz: torch.Tensor) -> torch.Tensor:
        """
        Args:
            xyz: [B, N, 3] 输入点云
        Returns:
            logits: [B, num_classes]
        """
        B, N, _ = xyz.shape

        # SA1
        l1_xyz, l1_features = self.sa1(xyz, None)  # [B, 512, 3], [B, 512, 128]

        # SA2
        l2_xyz, l2_features = self.sa2(l1_xyz, l1_features)  # [B, 128, 3], [B, 128, 256]

        # SA3 (全局)
        _, l3_features = self.sa3(l2_xyz, l2_features)  # [B, 1, 1024]
        x = l3_features.squeeze(1)  # [B, 1024]

        # FC
        x = self.drop1(F.relu(self.bn1(self.fc1(x))))
        x = self.drop2(F.relu(self.bn2(self.fc2(x))))
        x = self.fc3(x)

        return x

    def predict(self, xyz: torch.Tensor) -> torch.Tensor:
        """返回预测类别"""
        logits = self.forward(xyz)
        return torch.argmax(logits, dim=-1)
