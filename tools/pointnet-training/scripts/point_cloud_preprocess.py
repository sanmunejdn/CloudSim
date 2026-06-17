"""点云训练前处理（与 PointNet++ 固定 ball radius 对齐）"""

import numpy as np


def normalize_point_cloud(points: np.ndarray) -> np.ndarray:
    """去中心化并缩放到单位球"""
    points = np.asarray(points, dtype=np.float32)
    centroid = points.mean(axis=0)
    points = points - centroid
    scale = np.linalg.norm(points, axis=1).max()
    if scale > 1e-8:
        points = points / scale
    return points
