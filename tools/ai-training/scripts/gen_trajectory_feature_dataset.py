#!/usr/bin/env python3
"""从示例 STEP 调用 enumerateFeatureCatalog 批量生成 trajectory.feature 训练样本（占位脚本）。

完整流水线需 CloudSim 导出 catalog JSON；本脚本提供合成 catalog 切片样本供 schema 校验与 prompt 调试。
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "domains" / "trajectory.feature" / "dataset.jsonl"


def synth_line_sample(user_text: str, ids: list[str]) -> dict:
    candidates = [
        {
            "displayIndex": i + 1,
            "candidateId": cid,
            "suggestedKind": "EdgeChain",
            "summary": f"边 {cid}",
        }
        for i, cid in enumerate(ids)
    ]
    return {
        "instruction": "根据 catalog 与用户文本输出 trajectory.feature JSON",
        "input": {
            "userText": user_text,
            "catalogSlice": {"featureAxis": "line", "candidates": candidates},
        },
        "output": json.dumps(
            {
                "version": 1,
                "featureAxis": "line",
                "selectedCandidateIds": ids,
                "features": [],
                "suggestedPipelineTemplate": "weld_default",
            },
            ensure_ascii=False,
        ),
    }


def main() -> int:
    rows = [
        synth_line_sample("识别焊缝", ["edge_1", "edge_2"]),
        synth_line_sample("选 2", ["edge_1", "edge_2"]),
    ]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
    print(f"wrote {len(rows)} samples to {OUT}")
    print("Run: python scripts/build_dataset.py trajectory.feature")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
