"""数据集验证工具

用法:
    python scripts/build_dataset.py <task_type> [--root <dataset_root>] [--fix]

task_type: classification / segmentation
"""

import argparse
import json
import os
import sys
from collections import Counter

from python_io import configure_stdio_utf8

configure_stdio_utf8()


def validate_classification(root: str, fix: bool = False):
    """验证分类数据集"""
    jsonl_path = os.path.join(root, 'dataset.jsonl')
    if not os.path.exists(jsonl_path):
        print(f"错误: 找不到 {jsonl_path}")
        return False

    data_dir = os.path.join(root, 'data')
    errors = []
    stats = Counter()
    samples = []

    with open(jsonl_path, 'r', encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                entry = json.loads(line)
            except json.JSONDecodeError as e:
                errors.append(f"行 {lineno}: JSON 解析失败 - {e}")
                continue

            # 检查必要字段
            if 'instruction' not in entry:
                errors.append(f"行 {lineno}: 缺少 instruction")
            if 'input' not in entry:
                errors.append(f"行 {lineno}: 缺少 input")
                continue
            if 'output' not in entry:
                errors.append(f"行 {lineno}: 缺少 output")
                continue

            # 检查文件是否存在
            input_path = os.path.join(data_dir, entry['input'])
            if not os.path.exists(input_path):
                errors.append(f"行 {lineno}: 文件不存在 - {entry['input']}")
                continue

            # 解析 output
            output = entry['output']
            if isinstance(output, str):
                try:
                    output = json.loads(output)
                except json.JSONDecodeError:
                    errors.append(f"行 {lineno}: output JSON 解析失败")
                    continue

            cls_name = output.get('class_name', 'unknown')
            stats[cls_name] += 1
            samples.append(entry)

    # 输出报告
    print(f"\n=== 分类数据集验证报告 ===")
    print(f"路径: {root}")
    print(f"总样本数: {len(samples)}")
    print(f"错误数: {len(errors)}")
    print(f"\n类别分布:")
    for cls, count in sorted(stats.items()):
        print(f"  {cls}: {count}")

    if errors:
        print(f"\n错误详情:")
        for e in errors[:20]:
            print(f"  {e}")
        if len(errors) > 20:
            print(f"  ... 共 {len(errors)} 个错误")

    return len(errors) == 0


def validate_segmentation(root: str, fix: bool = False):
    """验证分割数据集"""
    jsonl_path = os.path.join(root, 'dataset.jsonl')
    if not os.path.exists(jsonl_path):
        print(f"错误: 找不到 {jsonl_path}")
        return False

    data_dir = os.path.join(root, 'data')
    errors = []
    samples = []
    num_classes_set = set()

    with open(jsonl_path, 'r', encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                entry = json.loads(line)
            except json.JSONDecodeError as e:
                errors.append(f"行 {lineno}: JSON 解析失败 - {e}")
                continue

            if 'input' not in entry:
                errors.append(f"行 {lineno}: 缺少 input")
                continue
            if 'output' not in entry:
                errors.append(f"行 {lineno}: 缺少 output")
                continue

            input_path = os.path.join(data_dir, entry['input'])
            if not os.path.exists(input_path):
                errors.append(f"行 {lineno}: 点云文件不存在 - {entry['input']}")
                continue

            output = entry['output']
            if isinstance(output, str):
                try:
                    output = json.loads(output)
                except json.JSONDecodeError:
                    errors.append(f"行 {lineno}: output JSON 解析失败")
                    continue

            nc = output.get('num_classes', 0)
            num_classes_set.add(nc)

            label_file = output.get('label_file', '')
            labels_inline = output.get('labels', [])

            if label_file:
                label_path = os.path.join(data_dir, label_file)
                if not os.path.exists(label_path):
                    errors.append(f"行 {lineno}: 标签文件不存在 - {label_file}")
            elif not labels_inline:
                errors.append(f"行 {lineno}: 无标签数据（label_file 和 labels 均为空）")

            samples.append(entry)

    print(f"\n=== 分割数据集验证报告 ===")
    print(f"路径: {root}")
    print(f"总样本数: {len(samples)}")
    print(f"错误数: {len(errors)}")
    print(f"类别数集合: {num_classes_set}")

    if errors:
        print(f"\n错误详情:")
        for e in errors[:20]:
            print(f"  {e}")
        if len(errors) > 20:
            print(f"  ... 共 {len(errors)} 个错误")

    return len(errors) == 0


def main():
    parser = argparse.ArgumentParser(description='Validate PointNet++ dataset')
    parser.add_argument('task', type=str, choices=['classification', 'segmentation'],
                        help='Task type')
    parser.add_argument('--root', type=str, default=None,
                        help='Dataset root (default: datasets/<task>)')
    parser.add_argument('--fix', action='store_true', help='Auto-fix issues')
    args = parser.parse_args()

    root = args.root
    if root is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        root = os.path.join(script_dir, '..', 'datasets', args.task)

    if args.task == 'classification':
        ok = validate_classification(root, args.fix)
    else:
        ok = validate_segmentation(root, args.fix)

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
