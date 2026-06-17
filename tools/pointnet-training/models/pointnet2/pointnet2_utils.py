"""PointNet++ 核心模块：Set Abstraction 和 Feature Propagation"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Optional, Tuple


def square_distance(src: torch.Tensor, dst: torch.Tensor) -> torch.Tensor:
    """计算两点集间的欧氏距离平方

    Args:
        src: [B, N, C]
        dst: [B, M, C]
    Returns:
        dist: [B, N, M]
    """
    B, N, _ = src.shape
    _, M, _ = dst.shape
    dist = -2.0 * torch.matmul(src, dst.permute(0, 2, 1))
    dist += torch.sum(src ** 2, -1).view(B, N, 1)
    dist += torch.sum(dst ** 2, -1).view(B, 1, M)
    return dist


def index_points(points: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
    """根据索引提取点

    Args:
        points: [B, N, C]
        idx: [B, S] 或 [B, S, K]
    Returns:
        new_points: [B, S, C] 或 [B, S, K, C]
    """
    device = points.device
    B = points.shape[0]
    view_shape = list(idx.shape)
    view_shape[1:] = [1] * (len(view_shape) - 1)
    repeat_shape = list(idx.shape)
    repeat_shape[0] = 1
    batch_indices = torch.arange(B, dtype=torch.long).to(device).view(view_shape).repeat(repeat_shape)
    new_points = points[batch_indices, idx, :]
    return new_points


def farthest_point_sample(xyz: torch.Tensor, npoint: int) -> torch.Tensor:
    """最远点采样

    Args:
        xyz: [B, N, 3]
        npoint: 采样点数
    Returns:
        centroids: [B, npoint] 采样点索引
    """
    device = xyz.device
    B, N, C = xyz.shape
    centroids = torch.zeros(B, npoint, dtype=torch.long).to(device)
    distance = torch.ones(B, N).to(device) * 1e10
    farthest = torch.randint(0, N, (B,), dtype=torch.long).to(device)
    batch_indices = torch.arange(B, dtype=torch.long).to(device)

    for i in range(npoint):
        centroids[:, i] = farthest
        centroid = xyz[batch_indices, farthest, :].view(B, 1, 3)
        dist = torch.sum((xyz - centroid) ** 2, -1)
        mask = dist < distance
        distance[mask] = dist[mask]
        farthest = torch.max(distance, -1)[1]

    return centroids


def query_ball_point(radius: float, nsample: int, xyz: torch.Tensor,
                     new_xyz: torch.Tensor) -> torch.Tensor:
    """球查询：在半径内找最近的 nsample 个点

    Args:
        radius: 搜索半径
        nsample: 每个球内最大点数
        xyz: [B, N, 3]
        new_xyz: [B, S, 3]
    Returns:
        group_idx: [B, S, nsample]
    """
    B, N, _ = xyz.shape
    _, S, _ = new_xyz.shape
    sqrdists = square_distance(new_xyz, xyz)
    sqrdists[sqrdists > radius ** 2] = 1e10
    _, group_idx = torch.sort(sqrdists, dim=-1)
    group_idx = group_idx[:, :, :nsample]
    return group_idx


class SetAbstraction(nn.Module):
    """Set Abstraction 层"""

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

    def forward(self, xyz: torch.Tensor,
                features: Optional[torch.Tensor]) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Args:
            xyz: [B, N, 3]
            features: [B, N, C] 或 None
        Returns:
            new_xyz: [B, npoint, 3]
            new_features: [B, npoint, mlp_channels[-1]]
        """
        if self.group_all:
            new_xyz = xyz.mean(dim=1, keepdim=True)
            grouped_xyz = xyz.unsqueeze(1)  # [B, 1, N, 3]
            if features is not None:
                grouped_features = torch.cat([grouped_xyz,
                    features.unsqueeze(1)], dim=-1)  # [B, 1, N, 3+C]
            else:
                grouped_features = grouped_xyz
        else:
            # FPS
            fps_idx = farthest_point_sample(xyz, self.npoint)
            new_xyz = index_points(xyz, fps_idx)  # [B, npoint, 3]

            # Ball query
            idx = query_ball_point(self.radius, self.nsample, xyz, new_xyz)
            grouped_xyz = index_points(xyz, idx)  # [B, npoint, nsample, 3]
            grouped_xyz -= new_xyz.unsqueeze(2)

            if features is not None:
                grouped_features = index_points(features, idx)
                grouped_features = torch.cat([grouped_xyz, grouped_features], dim=-1)
            else:
                grouped_features = grouped_xyz

        # MLP
        grouped_features = grouped_features.permute(0, 3, 1, 2)  # [B, C, npoint, nsample]
        for i, conv in enumerate(self.mlp_convs):
            bn = self.mlp_bns[i]
            grouped_features = F.relu(bn(conv(grouped_features)))

        new_features = torch.max(grouped_features, -1)[0]  # [B, mlp[-1], npoint]
        new_features = new_features.permute(0, 2, 1)  # [B, npoint, mlp[-1]]
        return new_xyz, new_features


class FeaturePropagation(nn.Module):
    """Feature Propagation 层（用于分割）"""

    def __init__(self, in_channel: int, mlp_channels: list):
        super().__init__()
        self.mlp_convs = nn.ModuleList()
        self.mlp_bns = nn.ModuleList()
        last_channel = in_channel
        for out_channel in mlp_channels:
            self.mlp_convs.append(nn.Conv1d(last_channel, out_channel, 1))
            self.mlp_bns.append(nn.BatchNorm1d(out_channel))
            last_channel = out_channel

    def forward(self, xyz1: torch.Tensor, xyz2: torch.Tensor,
                features1: Optional[torch.Tensor],
                features2: torch.Tensor) -> torch.Tensor:
        """
        Args:
            xyz1: [B, N, 3] 细粒度点
            xyz2: [B, S, 3] 粗粒度点
            features1: [B, N, C1] 或 None
            features2: [B, S, C2]
        Returns:
            new_features: [B, N, mlp[-1]]
        """
        B, N, C = xyz1.shape
        _, S, _ = xyz2.shape

        if S == 1:
            new_features = features2.repeat(1, N, 1)
            if features1 is not None:
                new_features = torch.cat([new_features, features1], dim=-1)
        else:
            dists = square_distance(xyz1, xyz2)  # [B, N, S]
            dists, idx = dists.sort(dim=-1)
            dists, idx = dists[:, :, :3], idx[:, :, :3]  # 3-NN

            dist_recip = 1.0 / (dists + 1e-8)
            norm = torch.sum(dist_recip, dim=2, keepdim=True)
            weight = dist_recip / norm  # [B, N, 3]

            idx_expanded = idx.unsqueeze(-1).expand(-1, -1, -1, features2.shape[-1])
            features2_expanded = features2.unsqueeze(1).expand(-1, N, -1, -1)
            interpolated = torch.gather(features2_expanded, 2, idx_expanded)
            interpolated = torch.sum(interpolated * weight.unsqueeze(-1), dim=2)

            if features1 is not None:
                new_features = torch.cat([interpolated, features1], dim=-1)
            else:
                new_features = interpolated

        new_features = new_features.permute(0, 2, 1)  # [B, C, N]
        for i, conv in enumerate(self.mlp_convs):
            bn = self.mlp_bns[i]
            new_features = F.relu(bn(conv(new_features)))

        return new_features.permute(0, 2, 1)  # [B, N, mlp[-1]]
