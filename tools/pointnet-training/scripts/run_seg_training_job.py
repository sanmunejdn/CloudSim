"""PointNet++ 分割训练入口（供 CloudSim LabelingPlugin UI 调用）

写入 metrics.jsonl 与 training_summary.json，供训练 Tab 解析。
"""

import argparse
import json
import os
import sys
import time

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from python_io import configure_stdio_utf8

configure_stdio_utf8()

from scripts.train_seg import (
    PointCloudSegDataset,
    build_dataloaders,
    evaluate,
    train_one_epoch,
    ensure_val_split,
)
from models.pointnet2 import PointNet2Seg

import torch
import torch.nn as nn
import torch.optim as optim


def append_metrics(path: str, record: dict):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'a', encoding='utf-8') as f:
        f.write(json.dumps(record, ensure_ascii=False) + '\n')


def write_summary(path: str, summary: dict):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)


def run_job(config_path: str, metrics_file: str, summary_file: str, resume: str = None):
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    config_dir = os.path.dirname(os.path.dirname(os.path.abspath(config_path)))
    ds_root = cfg['dataset']['root']
    if not os.path.isabs(ds_root):
        ds_root = os.path.join(config_dir, ds_root)

    ensure_val_split(ds_root, val_ratio=0.2)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Device: {device}")

    num_points = cfg['num_points']
    num_classes = cfg['num_classes']
    aug_cfg = cfg.get('augmentation', {})

    train_loader, val_loader, train_count, val_count = build_dataloaders(
        ds_root, cfg, num_points, num_classes, aug_cfg)

    print(f"Train: {train_count} samples, Val: {val_count} samples")

    model = PointNet2Seg(
        num_classes=num_classes,
        num_points=num_points,
        dropout=cfg['model'].get('dropout', 0.5)
    ).to(device)

    if resume:
        model.load_state_dict(torch.load(resume, map_location=device))
        print(f"Resumed from {resume}")

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

    if metrics_file and os.path.exists(metrics_file):
        os.remove(metrics_file)

    best_acc = 0.0
    best_ckpt = os.path.join(output_dir, 'best.pth')
    total_epochs = int(cfg['epochs'])

    for epoch in range(total_epochs):
        t0 = time.time()
        train_loss, train_acc = train_one_epoch(model, train_loader, criterion, optimizer, device)
        val_loss, val_acc = evaluate(model, val_loader, criterion, device)
        scheduler.step()
        elapsed = time.time() - t0
        lr = optimizer.param_groups[0]['lr']
        is_best = val_acc > best_acc
        if is_best:
            best_acc = val_acc
            torch.save(model.state_dict(), best_ckpt)
            print(f"  -> Best model saved (acc={best_acc:.4f})")

        print(f"Epoch {epoch+1}/{total_epochs} ({elapsed:.1f}s) "
              f"lr={lr:.6f} "
              f"train_loss={train_loss:.4f} acc={train_acc:.4f} "
              f"val_loss={val_loss:.4f} acc={val_acc:.4f}")

        if metrics_file:
            append_metrics(metrics_file, {
                'epoch': epoch + 1,
                'total_epochs': total_epochs,
                'train_loss': train_loss,
                'train_acc': train_acc,
                'val_loss': val_loss,
                'val_acc': val_acc,
                'lr': lr,
                'elapsed_s': elapsed,
                'best': is_best,
            })

        if (epoch + 1) % cfg.get('checkpoint_every', 10) == 0:
            torch.save(model.state_dict(),
                       os.path.join(output_dir, f'checkpoint_{epoch+1}.pth'))

    print(f"\nTraining complete. Best val accuracy: {best_acc:.4f}")

    if summary_file:
        write_summary(summary_file, {
            'status': 'completed',
            'best_val_acc': best_acc,
            'best_checkpoint': best_ckpt,
            'device': str(device),
            'output_dir': output_dir,
        })


def main():
    parser = argparse.ArgumentParser(description='Run segmentation training job for CloudSim UI')
    parser.add_argument('--config', type=str, required=True)
    parser.add_argument('--metrics-file', type=str, default='output/seg/metrics.jsonl')
    parser.add_argument('--summary-file', type=str, default='output/seg/training_summary.json')
    parser.add_argument('--resume', type=str, default=None)
    args = parser.parse_args()

    config_dir = os.path.dirname(os.path.dirname(os.path.abspath(args.config)))
    metrics = args.metrics_file
    summary = args.summary_file
    if not os.path.isabs(metrics):
        metrics = os.path.join(config_dir, metrics)
    if not os.path.isabs(summary):
        summary = os.path.join(config_dir, summary)

    run_job(args.config, metrics, summary, args.resume)


if __name__ == '__main__':
    main()
