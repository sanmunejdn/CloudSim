"""PointNet++ 分类模型（ONNX 导出版本）

简化版：用均匀采样替代 FPS，使 ONNX 导出兼容
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


class SetAbstractionOnnx(nn.Module):
    """Set Abstraction 层（ONNX 兼容版，用均匀采样替代 FPS）"""

    def __init__(self, npoint: int, radius: float, nsample: int,
                 in_channel: int, mlp_channels: list,
                 group_all: bool = False):
        super().__init__()
        self.npoint = npoint
        self.radius = radius
        self.nsample = nsample
        self.group_all = group_all

        self.mlp_convs = nn.ModuleList()
        self.mlp_bns = nn.ModuleList()
        last_channel = in_channel
        for out_channel in mlp_channels:
            self.mlp_convs.append(nn.Conv2d(last_channel, out_channel, 1))
            self.mlp_bns.append(nn.BatchNorm2d(out_channel))
            last_channel = out_channel

    @staticmethod
    def _gather_by_index(src: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
        """沿 dim=1 取点；idx 必须为 int64，避免 ONNX GatherElements 索引变成 float。"""
        idx = idx.to(dtype=torch.int64)
        idx_expand = idx.unsqueeze(-1).repeat(1, 1, src.shape[-1])
        return torch.gather(src, 1, idx_expand)

    @staticmethod
    def _gather_neighbors(src: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
        """src [B,N,C], idx [B,S,K] int64 -> [B,S,K,C]"""
        idx = idx.to(dtype=torch.int64)
        npoint = idx.shape[1]
        src_exp = src.unsqueeze(1).repeat(1, npoint, 1, 1)
        idx_exp = idx.unsqueeze(-1).repeat(1, 1, 1, src.shape[-1])
        return torch.gather(src_exp, 2, idx_exp)

    def forward(self, xyz: torch.Tensor, features: torch.Tensor = None):
        if self.group_all:
            new_xyz = xyz.mean(dim=1, keepdim=True)
            grouped_xyz = xyz.unsqueeze(1)
            if features is not None:
                grouped_features = torch.cat([grouped_xyz, features.unsqueeze(1)], dim=-1)
            else:
                grouped_features = grouped_xyz
        else:
            _, n_points, _ = xyz.shape
            # index_select 导出为 Gather，比 gather+expand 更稳定
            sample_idx = torch.linspace(
                0, n_points - 1, self.npoint, dtype=torch.int64, device=xyz.device)
            new_xyz = torch.index_select(xyz, 1, sample_idx)

            dist = torch.cdist(new_xyz, xyz)
            _, idx = dist.topk(self.nsample, dim=-1, largest=False)
            grouped_xyz = self._gather_neighbors(xyz, idx)
            grouped_xyz = grouped_xyz - new_xyz.unsqueeze(2)

            if features is not None:
                grouped_features = self._gather_neighbors(features, idx)
                grouped_features = torch.cat([grouped_xyz, grouped_features], dim=-1)
            else:
                grouped_features = grouped_xyz

        grouped_features = grouped_features.permute(0, 3, 1, 2)
        for i, conv in enumerate(self.mlp_convs):
            bn = self.mlp_bns[i]
            grouped_features = F.relu(bn(conv(grouped_features)))

        new_features = torch.max(grouped_features, -1)[0]
        new_features = new_features.permute(0, 2, 1)
        return new_xyz, new_features


class PointNet2ClsOnnx(nn.Module):
    """PointNet++ 分类网络（ONNX 导出版本）"""

    def __init__(self, num_classes: int, num_points: int = 1024, dropout: float = 0.0):
        super().__init__()
        self.num_classes = num_classes
        self.num_points = num_points

        self.sa1 = SetAbstractionOnnx(npoint=512, radius=0.2, nsample=32,
                                       in_channel=3, mlp_channels=[64, 64, 128])
        self.sa2 = SetAbstractionOnnx(npoint=128, radius=0.4, nsample=64,
                                       in_channel=128 + 3, mlp_channels=[128, 128, 256])
        self.sa3 = SetAbstractionOnnx(npoint=None, radius=None, nsample=None,
                                       in_channel=256 + 3, mlp_channels=[256, 512, 1024],
                                       group_all=True)

        self.fc1 = nn.Linear(1024, 512)
        self.bn1 = nn.BatchNorm1d(512)
        self.drop1 = nn.Dropout(dropout)
        self.fc2 = nn.Linear(512, 256)
        self.bn2 = nn.BatchNorm1d(256)
        self.drop2 = nn.Dropout(dropout)
        self.fc3 = nn.Linear(256, num_classes)

    def forward(self, xyz: torch.Tensor) -> torch.Tensor:
        B, N, _ = xyz.shape
        l1_xyz, l1_features = self.sa1(xyz, None)
        l2_xyz, l2_features = self.sa2(l1_xyz, l1_features)
        _, l3_features = self.sa3(l2_xyz, l2_features)
        x = l3_features.squeeze(1)

        x = self.drop1(F.relu(self.bn1(self.fc1(x))))
        x = self.drop2(F.relu(self.bn2(self.fc2(x))))
        x = self.fc3(x)
        return x


def export_to_onnx(checkpoint_path: str, output_path: str, num_classes: int, num_points: int = 1024):
    """从训练检查点导出 ONNX 模型"""
    model = PointNet2ClsOnnx(num_classes=num_classes, num_points=num_points, dropout=0.0)
    model.load_state_dict(torch.load(checkpoint_path, map_location='cpu'), strict=False)
    model.eval()

    dummy = torch.randn(1, num_points, 3)
    torch.onnx.export(
        model, dummy, output_path,
        input_names=['points'],
        output_names=['logits'],
        opset_version=13,
        dynamic_axes={'points': {0: 'batch'}, 'logits': {0: 'batch'}}
    )
    return output_path
