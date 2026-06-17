"""合成点云数据生成器

用于快速生成分类和分割训练数据，无需手动标注。
支持的基本体：box, cylinder, sphere, cone
"""

import argparse
import json
import math
import os
import random
import sys

import numpy as np


def generate_box_points(num_points: int, size: tuple = None) -> np.ndarray:
    """生成长方体表面点云

    Args:
        num_points: 点数
        size: (length, width, height) mm，默认随机 50-200
    Returns:
        points: [N, 3]
    """
    if size is None:
        size = tuple(np.random.uniform(50, 200, 3))
    l, w, h = size
    half_l, half_w, half_h = l/2, w/2, h/2

    # 在 6 个面上均匀采样
    points_per_face = num_points // 6
    remaining = num_points - points_per_face * 6
    faces = []

    for _ in range(6):
        n = points_per_face + (1 if remaining > 0 else 0)
        remaining -= 1

        face = np.random.uniform(-1, 1, (n, 3))
        face_id = len(faces) % 6
        if face_id == 0:  # +x
            face[:, 0] = half_l
            face[:, 1] *= half_w
            face[:, 2] *= half_h
        elif face_id == 1:  # -x
            face[:, 0] = -half_l
            face[:, 1] *= half_w
            face[:, 2] *= half_h
        elif face_id == 2:  # +y
            face[:, 1] = half_w
            face[:, 0] *= half_l
            face[:, 2] *= half_h
        elif face_id == 3:  # -y
            face[:, 1] = -half_w
            face[:, 0] *= half_l
            face[:, 2] *= half_h
        elif face_id == 4:  # +z
            face[:, 2] = half_h
            face[:, 0] *= half_l
            face[:, 1] *= half_w
        else:  # -z
            face[:, 2] = -half_h
            face[:, 0] *= half_l
            face[:, 1] *= half_w

        faces.append(face)

    return np.concatenate(faces, axis=0)[:num_points]


def generate_cylinder_points(num_points: int, radius: float = None,
                              height: float = None) -> np.ndarray:
    """生成圆柱表面点云"""
    if radius is None:
        radius = np.random.uniform(20, 100)
    if height is None:
        height = np.random.uniform(50, 200)

    # 侧面
    n_side = int(num_points * 0.7)
    theta = np.random.uniform(0, 2 * np.pi, n_side)
    z = np.random.uniform(-height/2, height/2, n_side)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    side = np.stack([x, y, z], axis=-1)

    # 顶面和底面
    n_cap = (num_points - n_side) // 2
    caps = []
    for sign in [1, -1]:
        r = np.random.uniform(0, radius, n_cap)
        theta = np.random.uniform(0, 2 * np.pi, n_cap)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        z = np.full(n_cap, sign * height / 2)
        caps.append(np.stack([x, y, z], axis=-1))

    points = np.concatenate([side, caps[0], caps[1]], axis=0)
    return points[:num_points]


def generate_sphere_points(num_points: int, radius: float = None) -> np.ndarray:
    """生成球面点云"""
    if radius is None:
        radius = np.random.uniform(20, 100)

    # 均匀球面采样
    theta = np.random.uniform(0, 2 * np.pi, num_points)
    phi = np.arccos(2 * np.random.uniform(0, 1, num_points) - 1)
    x = radius * np.sin(phi) * np.cos(theta)
    y = radius * np.sin(phi) * np.sin(theta)
    z = radius * np.cos(phi)
    return np.stack([x, y, z], axis=-1)


def generate_cone_points(num_points: int, radius: float = None,
                          height: float = None) -> np.ndarray:
    """生成圆锥表面点云"""
    if radius is None:
        radius = np.random.uniform(20, 100)
    if height is None:
        height = np.random.uniform(50, 200)

    # 侧面
    n_side = int(num_points * 0.75)
    theta = np.random.uniform(0, 2 * np.pi, n_side)
    t = np.random.uniform(0, 1, n_side)
    r = radius * (1 - t)
    z = height * t - height / 2
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    side = np.stack([x, y, z], axis=-1)

    # 底面
    n_cap = num_points - n_side
    r = np.random.uniform(0, radius, n_cap)
    theta = np.random.uniform(0, 2 * np.pi, n_cap)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.full(n_cap, -height / 2)
    cap = np.stack([x, y, z], axis=-1)

    return np.concatenate([side, cap], axis=0)[:num_points]


def generate_complex_points(num_points: int) -> np.ndarray:
    """生成复杂形状（L 形或 T 形）点云"""
    shape_type = random.choice(['L', 'T'])

    if shape_type == 'L':
        # L 形：两个长方体组合
        n1 = num_points // 2
        n2 = num_points - n1
        p1 = generate_box_points(n1, (100, 30, 80))
        p2 = generate_box_points(n2, (30, 100, 80))
        p2[:, 0] += 35
        p2[:, 1] -= 35
        return np.concatenate([p1, p2], axis=0)[:num_points]
    else:
        # T 形
        n1 = num_points // 2
        n2 = num_points - n1
        p1 = generate_box_points(n1, (100, 30, 60))
        p2 = generate_box_points(n2, (30, 100, 60))
        p2[:, 1] += 35
        return np.concatenate([p1, p2], axis=0)[:num_points]


GENERATORS = {
    'box': generate_box_points,
    'cylinder': generate_cylinder_points,
    'sphere': generate_sphere_points,
    'cone': generate_cone_points,
    'complex': generate_complex_points,
}


def save_ply(path: str, points: np.ndarray):
    """保存为 ASCII PLY 格式"""
    n = points.shape[0]
    with open(path, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for i in range(n):
            f.write(f"{points[i,0]:.4f} {points[i,1]:.4f} {points[i,2]:.4f}\n")


def main():
    parser = argparse.ArgumentParser(description='Generate synthetic point cloud data')
    parser.add_argument('--output', type=str, required=True, help='Output directory')
    parser.add_argument('--task', type=str, required=True, choices=['cls', 'seg'],
                        help='Task: cls (classification) or seg (segmentation)')
    parser.add_argument('--num-per-class', type=int, default=100,
                        help='Samples per class (for cls)')
    parser.add_argument('--num-samples', type=int, default=50,
                        help='Total samples (for seg)')
    parser.add_argument('--num-points', type=int, default=1024,
                        help='Points per sample')
    args = parser.parse_args()

    os.makedirs(os.path.join(args.output, 'data'), exist_ok=True)

    if args.task == 'cls':
        generate_classification_data(args)
    else:
        generate_segmentation_data(args)


def generate_classification_data(args):
    """生成分类数据集"""
    samples = []
    classes = list(GENERATORS.keys())

    for cls_name in classes:
        gen = GENERATORS[cls_name]
        for i in range(args.num_per_class):
            points = gen(args.num_points)
            filename = f"{cls_name}_{i:04d}.ply"
            save_ply(os.path.join(args.output, 'data', filename), points)

            samples.append({
                'instruction': '识别点云类型',
                'input': filename,
                'output': json.dumps({
                    'class_id': classes.index(cls_name),
                    'class_name': cls_name
                })
            })

    random.shuffle(samples)
    jsonl_path = os.path.join(args.output, 'dataset.jsonl')
    with open(jsonl_path, 'w', encoding='utf-8') as f:
        for s in samples:
            f.write(json.dumps(s, ensure_ascii=False) + '\n')

    print(f"分类数据集已生成: {args.output}")
    print(f"  类别数: {len(classes)}")
    print(f"  总样本: {len(samples)}")
    print(f"  每类: {args.num_per_class}")


def generate_segmentation_data(args):
    """生成分割数据集（合成多部件物体）"""
    samples = []
    num_classes = 4  # 0=背景, 1=主体, 2=部件A, 3=部件B

    for i in range(args.num_samples):
        # 生成一个多部件物体
        n_main = args.num_points // 2
        n_part_a = args.num_points // 4
        n_part_b = args.num_points - n_main - n_part_a

        main_pts = generate_box_points(n_main, (80, 80, 80))
        part_a = generate_cylinder_points(n_part_a, radius=15, height=60)
        part_a[:, 0] += 50
        part_b = generate_sphere_points(n_part_b, radius=20)
        part_b[:, 1] -= 50

        points = np.concatenate([main_pts, part_a, part_b], axis=0)
        labels = np.array([1]*n_main + [2]*n_part_a + [3]*n_part_b,
                         dtype=np.int64)[:args.num_points]

        points = points[:args.num_points]
        labels = labels[:args.num_points]

        filename = f"multi_part_{i:04d}.ply"
        save_ply(os.path.join(args.output, 'data', filename), points)

        label_filename = f"multi_part_{i:04d}_labels.npy"
        np.save(os.path.join(args.output, 'data', label_filename), labels)

        samples.append({
            'instruction': '分割网格部件',
            'input': filename,
            'output': json.dumps({
                'label_file': label_filename,
                'num_classes': num_classes
            })
        })

    random.shuffle(samples)
    jsonl_path = os.path.join(args.output, 'dataset.jsonl')
    with open(jsonl_path, 'w', encoding='utf-8') as f:
        for s in samples:
            f.write(json.dumps(s, ensure_ascii=False) + '\n')

    print(f"分割数据集已生成: {args.output}")
    print(f"  类别数: {num_classes}")
    print(f"  总样本: {len(samples)}")


if __name__ == '__main__':
    main()
