"""PointNet++ 分割训练脚本"""

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
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from python_io import configure_stdio_utf8

configure_stdio_utf8()

from models.pointnet2 import PointNet2Seg
from point_cloud_preprocess import normalize_point_cloud


def ensure_val_split(root: str, val_ratio: float = 0.2, seed: int = 42):
    """若 jsonl 无 split 字段，随机写入 train/val 标记。"""
    jsonl_path = os.path.join(root, 'dataset.jsonl')
    if not os.path.exists(jsonl_path):
        return
    lines = []
    needs_split = False
    with open(jsonl_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            if 'split' not in entry:
                needs_split = True
            lines.append(entry)
    if not needs_split or len(lines) < 2:
        return
    import random
    rng = random.Random(seed)
    indices = list(range(len(lines)))
    rng.shuffle(indices)
    val_n = max(1, int(len(lines) * val_ratio))
    val_set = set(indices[:val_n])
    for i, entry in enumerate(lines):
        entry['split'] = 'val' if i in val_set else 'train'
    with open(jsonl_path, 'w', encoding='utf-8') as f:
        for entry in lines:
            f.write(json.dumps(entry, ensure_ascii=False) + '\n')


def _load_samples(root: str, split: str):
    samples = []
    jsonl_path = os.path.join(root, 'dataset.jsonl')
    if not os.path.exists(jsonl_path):
        return samples
    with open(jsonl_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            entry_split = entry.get('split', 'train')
            if split and entry_split != split:
                continue
            output = json.loads(entry['output']) if isinstance(entry['output'], str) else entry['output']
            input_path = entry.get('input', '')
            if not input_path:
                continue
            label_file = output.get('label_file', '')
            labels_inline = output.get('labels', [])
            samples.append({
                'point_path': os.path.join(root, 'data', input_path),
                'label_path': os.path.join(root, 'data', label_file) if label_file else None,
                'labels_inline': labels_inline
            })
    return samples


class PointCloudSegDataset(Dataset):
    """点云分割数据集

    数据目录结构:
    root/
      dataset.jsonl   -- 标注文件
      data/           -- 点云文件 (.ply/.npy)

    每条标注的 output 格式:
    {
      "labels": [0, 0, 1, 1, ...],  // 每点标签，与点云点数一致
      "num_classes": 3
    }
    或使用 label_file:
    {
      "label_file": "labels/xxx.npy",
      "num_classes": 3
    }
    """

    def __init__(self, root: str, split: str, num_points: int,
                 num_classes: int, augment: bool = True,
                 augment_cfg: dict = None):
        self.root = root
        self.num_points = num_points
        self.num_classes = num_classes
        self.augment = augment
        self.augment_cfg = augment_cfg or {}

        self.samples = _load_samples(root, split)

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        sample = self.samples[idx]
        points = self._load_points(sample['point_path'])
        labels = self._load_labels(sample, points.shape[0])

        # 采样到固定点数
        points, labels = self._sample_with_labels(points, labels, self.num_points)
        points = normalize_point_cloud(points)

        # 数据增强
        if self.augment:
            points = self._augment(points)

        return torch.from_numpy(points).float(), torch.from_numpy(labels).long()

    def _load_points(self, path: str) -> np.ndarray:
        if path.endswith('.npy'):
            data = np.load(path)
            if data.ndim == 1:
                data = data.reshape(-1, 3)
            return data[:, :3]
        elif path.endswith('.ply'):
            return self._load_ply(path)
        else:
            raise ValueError(f"Unsupported: {path}")

    def _load_ply(self, path: str) -> np.ndarray:
        from plyfile import PlyData
        ply = PlyData.read(path)
        vertex = ply['vertex']
        x = np.array(vertex['x'])
        y = np.array(vertex['y'])
        z = np.array(vertex['z'])
        return np.stack([x, y, z], axis=-1)

    def _load_labels(self, sample: dict, num_points: int) -> np.ndarray:
        if sample['label_path'] and os.path.exists(sample['label_path']):
            return np.load(sample['label_path']).astype(np.int64)
        elif sample['labels_inline']:
            return np.array(sample['labels_inline'], dtype=np.int64)
        else:
            return np.zeros(num_points, dtype=np.int64)

    def _sample_with_labels(self, points, labels, n):
        N = points.shape[0]
        if N >= n:
            indices = np.random.choice(N, n, replace=False)
        else:
            indices = np.random.choice(N, n, replace=True)
        return points[indices], labels[indices]

    def _augment(self, points):
        cfg = self.augment_cfg
        if cfg.get('random_rotate_z', False):
            theta = np.random.uniform(0, 2 * np.pi)
            rot = np.array([
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta),  np.cos(theta), 0],
                [0, 0, 1]
            ])
            points = points @ rot.T
        if cfg.get('random_scale', False):
            lo, hi = cfg.get('scale_range', [0.8, 1.2])
            points *= np.random.uniform(lo, hi)
        if cfg.get('random_translate', False):
            t = cfg.get('translate_range', 0.1)
            points += np.random.uniform(-t, t, size=(1, 3))
        if cfg.get('jitter', False):
            std = cfg.get('jitter_std', 0.01)
            points += np.random.normal(0, std, points.shape)
        return points


def train_one_epoch(model, loader, criterion, optimizer, device):
    model.train()
    if len(loader) == 0:
        return 0.0, 0.0
    total_loss = 0.0
    correct = 0
    total = 0

    for points, labels in tqdm(loader, desc='Train', leave=False):
        points = points.to(device)
        labels = labels.to(device)

        optimizer.zero_grad()
        logits = model(points)  # [B, N, C]
        loss = criterion(logits.reshape(-1, logits.size(-1)), labels.reshape(-1))
        loss.backward()
        optimizer.step()

        total_loss += loss.item() * points.size(0)
        preds = logits.argmax(dim=-1)
        correct += (preds == labels).sum().item()
        total += labels.numel()

    return total_loss / len(loader), correct / total


def evaluate(model, loader, criterion, device):
    model.eval()
    if len(loader) == 0:
        return 0.0, 0.0
    total_loss = 0.0
    correct = 0
    total = 0

    with torch.no_grad():
        for points, labels in tqdm(loader, desc='Eval', leave=False):
            points = points.to(device)
            labels = labels.to(device)

            logits = model(points)
            loss = criterion(logits.reshape(-1, logits.size(-1)), labels.reshape(-1))

            total_loss += loss.item() * points.size(0)
            preds = logits.argmax(dim=-1)
            correct += (preds == labels).sum().item()
            total += labels.numel()

    return total_loss / len(loader), correct / total


def build_dataloaders(ds_root, cfg, num_points, num_classes, aug_cfg):
    train_ds = PointCloudSegDataset(ds_root, cfg['dataset']['train_split'],
                                     num_points, num_classes, augment=True, augment_cfg=aug_cfg)
    val_ds = PointCloudSegDataset(ds_root, cfg['dataset']['val_split'],
                                   num_points, num_classes, augment=False)
    if len(val_ds) == 0 and len(train_ds) > 1:
        val_ds = PointCloudSegDataset(ds_root, '', num_points, num_classes, augment=False)
        val_ds.samples = train_ds.samples[:max(1, len(train_ds.samples) // 5)]

    train_count = len(train_ds)
    batch_size = int(cfg['batch_size'])
    if train_count > 0:
        batch_size = min(batch_size, train_count)
    drop_last = train_count > batch_size
    num_workers = int(cfg.get('num_workers', 4))
    if sys.platform == 'win32' or train_count < 4:
        num_workers = 0

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True,
                              num_workers=num_workers, drop_last=drop_last)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False,
                            num_workers=num_workers)
    return train_loader, val_loader, len(train_ds), len(val_ds)


def main():
    parser = argparse.ArgumentParser(description='PointNet++ Segmentation Training')
    parser.add_argument('--config', type=str, required=True, help='Config YAML path')
    parser.add_argument('--resume', type=str, default=None, help='Resume from checkpoint')
    args = parser.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    # 将训练工程根目录作为路径解析基准（配置文件在 configs/ 子目录）
    config_dir = os.path.dirname(os.path.dirname(os.path.abspath(args.config)))

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Device: {device}")

    ds_root = cfg['dataset']['root']
    if not os.path.isabs(ds_root):
        ds_root = os.path.join(config_dir, ds_root)
    num_points = cfg['num_points']
    num_classes = cfg['num_classes']
    aug_cfg = cfg.get('augmentation', {})

    ensure_val_split(ds_root)

    train_loader, val_loader, train_count, val_count = build_dataloaders(
        ds_root, cfg, num_points, num_classes, aug_cfg)

    print(f"Train: {train_count} samples, Val: {val_count} samples")

    model = PointNet2Seg(
        num_classes=num_classes,
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

    output_dir = cfg.get('output_dir', 'output/seg')
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

        if val_acc > best_acc:
            best_acc = val_acc
            torch.save(model.state_dict(), os.path.join(output_dir, 'best.pth'))
            print(f"  -> Best model saved (acc={best_acc:.4f})")

        if (epoch + 1) % cfg.get('checkpoint_every', 10) == 0:
            torch.save(model.state_dict(),
                       os.path.join(output_dir, f'checkpoint_{epoch+1}.pth'))

    print(f"\nTraining complete. Best val accuracy: {best_acc:.4f}")


if __name__ == '__main__':
    main()
