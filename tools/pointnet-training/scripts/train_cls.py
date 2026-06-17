"""PointNet++ 分类训练脚本"""

import argparse
import json
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import yaml
from tqdm import tqdm

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from models.pointnet2 import PointNet2Cls
from point_cloud_preprocess import normalize_point_cloud


class PointCloudClsDataset(Dataset):
    """点云分类数据集

    数据目录结构:
    root/
      dataset.jsonl   -- 标注文件
      data/           -- 点云文件 (.ply/.npy)
    """

    def __init__(self, root: str, split: str, num_points: int,
                 classes: list, augment: bool = True,
                 augment_cfg: dict = None):
        self.root = root
        self.num_points = num_points
        self.classes = classes
        self.class_to_idx = {c: i for i, c in enumerate(classes)}
        self.augment = augment
        self.augment_cfg = augment_cfg or {}

        # 读取 dataset.jsonl
        self.samples = []
        jsonl_path = os.path.join(root, 'dataset.jsonl')
        if os.path.exists(jsonl_path):
            with open(jsonl_path, 'r', encoding='utf-8') as f:
                for line in f:
                    entry = json.loads(line.strip())
                    # output 字段含 class_id 或 class_name
                    output = json.loads(entry['output']) if isinstance(entry['output'], str) else entry['output']
                    cls_name = output.get('class_name', '')
                    cls_id = output.get('class_id', self.class_to_idx.get(cls_name, -1))
                    if cls_id >= 0:
                        input_path = entry.get('input', '')
                        if input_path:
                            self.samples.append({
                                'path': os.path.join(root, 'data', input_path),
                                'class_id': cls_id
                            })

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        sample = self.samples[idx]
        points = self._load_points(sample['path'])
        label = sample['class_id']

        # 采样到固定点数
        points = self._sample_points(points, self.num_points)
        points = normalize_point_cloud(points)

        # 数据增强
        if self.augment:
            points = self._augment(points)

        return torch.from_numpy(points).float(), torch.tensor(label).long()

    def _load_points(self, path: str) -> np.ndarray:
        """加载点云文件，返回 [N, 3]"""
        if path.endswith('.npy'):
            data = np.load(path)
            if data.ndim == 1:
                data = data.reshape(-1, 3)
            return data[:, :3]
        elif path.endswith('.ply'):
            return self._load_ply(path)
        else:
            raise ValueError(f"Unsupported file format: {path}")

    def _load_ply(self, path: str) -> np.ndarray:
        """简易 PLY 读取"""
        from plyfile import PlyData
        ply = PlyData.read(path)
        vertex = ply['vertex']
        x = np.array(vertex['x'])
        y = np.array(vertex['y'])
        z = np.array(vertex['z'])
        return np.stack([x, y, z], axis=-1)

    def _sample_points(self, points: np.ndarray, n: int) -> np.ndarray:
        """采样或填充到固定点数"""
        N = points.shape[0]
        if N >= n:
            indices = np.random.choice(N, n, replace=False)
        else:
            indices = np.random.choice(N, n, replace=True)
        return points[indices]

    def _augment(self, points: np.ndarray) -> np.ndarray:
        """数据增强"""
        cfg = self.augment_cfg

        # Z 轴随机旋转
        if cfg.get('random_rotate_z', False):
            theta = np.random.uniform(0, 2 * np.pi)
            rot = np.array([
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta),  np.cos(theta), 0],
                [0, 0, 1]
            ])
            points = points @ rot.T

        # 随机缩放
        if cfg.get('random_scale', False):
            lo, hi = cfg.get('scale_range', [0.8, 1.2])
            scale = np.random.uniform(lo, hi)
            points *= scale

        # 随机平移
        if cfg.get('random_translate', False):
            t = cfg.get('translate_range', 0.1)
            shift = np.random.uniform(-t, t, size=(1, 3))
            points += shift

        # 高斯抖动
        if cfg.get('jitter', False):
            std = cfg.get('jitter_std', 0.01)
            jitter = np.random.normal(0, std, points.shape)
            points += jitter

        return points


def train_one_epoch(model, loader, criterion, optimizer, device):
    model.train()
    total_loss = 0.0
    correct = 0
    total = 0

    for points, labels in tqdm(loader, desc='Train', leave=False):
        points = points.to(device)
        labels = labels.to(device)

        optimizer.zero_grad()
        logits = model(points)
        loss = criterion(logits, labels)
        loss.backward()
        optimizer.step()

        total_loss += loss.item() * points.size(0)
        preds = logits.argmax(dim=-1)
        correct += (preds == labels).sum().item()
        total += points.size(0)

    return total_loss / total, correct / total


def evaluate(model, loader, criterion, device):
    model.eval()
    total_loss = 0.0
    correct = 0
    total = 0

    with torch.no_grad():
        for points, labels in tqdm(loader, desc='Eval', leave=False):
            points = points.to(device)
            labels = labels.to(device)

            logits = model(points)
            loss = criterion(logits, labels)

            total_loss += loss.item() * points.size(0)
            preds = logits.argmax(dim=-1)
            correct += (preds == labels).sum().item()
            total += points.size(0)

    return total_loss / total, correct / total


def main():
    parser = argparse.ArgumentParser(description='PointNet++ Classification Training')
    parser.add_argument('--config', type=str, required=True, help='Config YAML path')
    parser.add_argument('--resume', type=str, default=None, help='Resume from checkpoint')
    args = parser.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    # 将训练工程根目录作为路径解析基准（配置文件在 configs/ 子目录）
    config_dir = os.path.dirname(os.path.dirname(os.path.abspath(args.config)))

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Device: {device}")

    # 数据集路径（相对于配置文件目录）
    ds_root = cfg['dataset']['root']
    if not os.path.isabs(ds_root):
        ds_root = os.path.join(config_dir, ds_root)
    classes = cfg['classes']
    num_points = cfg['num_points']
    aug_cfg = cfg.get('augmentation', {})

    train_ds = PointCloudClsDataset(ds_root, cfg['dataset']['train_split'],
                                     num_points, classes, augment=True, augment_cfg=aug_cfg)
    val_ds = PointCloudClsDataset(ds_root, cfg['dataset']['val_split'],
                                   num_points, classes, augment=False)

    train_loader = DataLoader(train_ds, batch_size=cfg['batch_size'], shuffle=True,
                              num_workers=cfg.get('num_workers', 4), drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=cfg['batch_size'], shuffle=False,
                            num_workers=cfg.get('num_workers', 4))

    print(f"Train: {len(train_ds)} samples, Val: {len(val_ds)} samples")

    # 模型
    model = PointNet2Cls(
        num_classes=len(classes),
        num_points=num_points,
        dropout=cfg['model'].get('dropout', 0.5)
    ).to(device)

    if args.resume:
        model.load_state_dict(torch.load(args.resume, map_location=device))
        print(f"Resumed from {args.resume}")

    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=cfg['learning_rate'],
                           weight_decay=cfg.get('weight_decay', 0.0001))

    scheduler_type = cfg.get('lr_scheduler', 'cosine')
    if scheduler_type == 'cosine':
        scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=cfg['epochs'])
    else:
        scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=30, gamma=0.1)

    # 输出目录
    output_dir = cfg.get('output_dir', 'output/cls')
    if not os.path.isabs(output_dir):
        output_dir = os.path.join(config_dir, output_dir)
    os.makedirs(output_dir, exist_ok=True)

    best_acc = 0.0
    for epoch in range(cfg['epochs']):
        t0 = time.time()
        train_loss, train_acc = train_one_epoch(model, train_loader, criterion, optimizer, device)
        val_loss, val_acc = evaluate(model, val_loader, criterion, device)
        scheduler.step()

        elapsed = time.time() - t0
        lr = optimizer.param_groups[0]['lr']
        print(f"Epoch {epoch+1}/{cfg['epochs']} ({elapsed:.1f}s) "
              f"lr={lr:.6f} "
              f"train_loss={train_loss:.4f} acc={train_acc:.4f} "
              f"val_loss={val_loss:.4f} acc={val_acc:.4f}")

        # 保存最优
        if val_acc > best_acc:
            best_acc = val_acc
            torch.save(model.state_dict(), os.path.join(output_dir, 'best.pth'))
            print(f"  -> Best model saved (acc={best_acc:.4f})")

        # 定期保存
        if (epoch + 1) % cfg.get('checkpoint_every', 10) == 0:
            torch.save(model.state_dict(),
                       os.path.join(output_dir, f'checkpoint_{epoch+1}.pth'))

    print(f"\nTraining complete. Best val accuracy: {best_acc:.4f}")
    print(f"Model saved to: {os.path.join(output_dir, 'best.pth')}")


if __name__ == '__main__':
    main()
