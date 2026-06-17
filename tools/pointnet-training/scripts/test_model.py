"""PointNet++ 模型单元测试

用法:
    python scripts/test_model.py
"""

import os
import sys
import unittest

import torch
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from models.pointnet2 import PointNet2Cls, PointNet2Seg


class TestPointNet2Cls(unittest.TestCase):
    """分类模型测试"""

    def test_forward_shape(self):
        """验证前向传播输出形状"""
        model = PointNet2Cls(num_classes=5, num_points=1024)
        x = torch.randn(2, 1024, 3)
        out = model(x)
        self.assertEqual(out.shape, (2, 5))

    def test_predict(self):
        """验证 predict 返回类别索引"""
        model = PointNet2Cls(num_classes=5, num_points=1024)
        x = torch.randn(4, 1024, 3)
        pred = model.predict(x)
        self.assertEqual(pred.shape, (4,))
        self.assertTrue((pred >= 0).all() and (pred < 5).all())

    def test_gradient_flow(self):
        """验证梯度能正常反传"""
        model = PointNet2Cls(num_classes=3, num_points=512)
        x = torch.randn(2, 512, 3, requires_grad=True)
        out = model(x)
        loss = out.sum()
        loss.backward()
        self.assertIsNotNone(x.grad)

    def test_different_point_counts(self):
        """验证不同点数输入也能工作"""
        model = PointNet2Cls(num_classes=4, num_points=1024)
        for n in [256, 512, 1024, 2048]:
            x = torch.randn(1, n, 3)
            out = model(x)
            self.assertEqual(out.shape, (1, 4))


class TestPointNet2Seg(unittest.TestCase):
    """分割模型测试"""

    def test_forward_shape(self):
        """验证前向传播输出形状"""
        model = PointNet2Seg(num_classes=6, num_points=2048)
        x = torch.randn(2, 2048, 3)
        out = model(x)
        self.assertEqual(out.shape, (2, 2048, 6))

    def test_predict(self):
        """验证 predict 返回每点类别"""
        model = PointNet2Seg(num_classes=4, num_points=1024)
        x = torch.randn(3, 1024, 3)
        pred = model.predict(x)
        self.assertEqual(pred.shape, (3, 1024))
        self.assertTrue((pred >= 0).all() and (pred < 4).all())

    def test_gradient_flow(self):
        """验证梯度能正常反传"""
        model = PointNet2Seg(num_classes=3, num_points=512)
        x = torch.randn(2, 512, 3, requires_grad=True)
        out = model(x)
        loss = out.sum()
        loss.backward()
        self.assertIsNotNone(x.grad)


class TestUtils(unittest.TestCase):
    """工具函数测试"""

    def test_square_distance(self):
        from models.pointnet2.pointnet2_utils import square_distance
        src = torch.tensor([[[0, 0, 0], [1, 0, 0]]]).float()
        dst = torch.tensor([[[0, 0, 0], [0, 1, 0]]]).float()
        dist = square_distance(src, dst)
        self.assertEqual(dist.shape, (1, 2, 2))
        self.assertAlmostEqual(dist[0, 0, 0].item(), 0.0, places=5)
        self.assertAlmostEqual(dist[0, 0, 1].item(), 1.0, places=5)
        self.assertAlmostEqual(dist[0, 1, 0].item(), 1.0, places=5)

    def test_index_points(self):
        from models.pointnet2.pointnet2_utils import index_points
        points = torch.tensor([[[1, 2, 3], [4, 5, 6], [7, 8, 9]]]).float()
        idx = torch.tensor([[0, 2]])
        result = index_points(points, idx)
        self.assertEqual(result.shape, (1, 2, 3))
        self.assertAlmostEqual(result[0, 0, 0].item(), 1.0)
        self.assertAlmostEqual(result[0, 1, 0].item(), 7.0)

    def test_farthest_point_sample(self):
        from models.pointnet2.pointnet2_utils import farthest_point_sample
        xyz = torch.randn(2, 100, 3)
        idx = farthest_point_sample(xyz, 10)
        self.assertEqual(idx.shape, (2, 10))
        # 索引应在有效范围内
        self.assertTrue((idx >= 0).all() and (idx < 100).all())

    def test_query_ball_point_large_coords(self):
        from models.pointnet2.pointnet2_utils import query_ball_point
        xyz = torch.tensor([[[1e6, 0.0, 0.0], [1e6, 0.1, 0.0]]]).float()
        new_xyz = xyz[:, :1, :]
        idx = query_ball_point(0.2, 2, xyz, new_xyz)
        self.assertEqual(idx.shape, (1, 1, 2))
        self.assertTrue((idx >= 0).all() and (idx < 2).all())


if __name__ == '__main__':
    unittest.main()
