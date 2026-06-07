#!/usr/bin/env python3
"""为 TrajectoryAlgorithmBuiltins 源文件补充单行文件头注释（已存在则跳过）"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "Robot" / "TrajectoryAlgorithmBuiltins"

OP_DESC = {
    "Translate": "程序路点位姿平移",
    "Rotate": "程序路点位姿旋转",
    "Delete": "删除 scope 内路点",
    "Duplicate": "复制 scope 内路点",
    "Mirror": "镜像 scope 内路点",
    "Reorder": "重排 scope 内路点顺序",
    "Approach": "在路径首端插入进刀段",
    "Retract": "在路径尾端插入退刀段",
    "Resample": "按步长重采样 UnifiedTrajectory 折线",
    "OffsetAlongNormal": "沿法向偏移路径点",
    "OffsetLateral": "沿横向偏移路径点",
    "SmoothPose": "平滑路径点位姿",
    "AssignBlend": "为路径点写入 blend 半径",
    "AssignSpeedZone": "为路径点写入速度区",
    "Weave": "在路径上叠加摆动",
    "ReachabilityFilter": "剔除不可达路径点",
    "ExternalAxisSearch": "搜索外部轴以满足可达性",
}


def comment_for(path: Path) -> str | None:
    name = path.name
    stem = path.stem
    if name == "TrajectoryOpFormat.h":
        return "// 轨迹块参数在 JSON/UI 中的字段格式化"
    if name == "TrajectoryOpFormat.cpp":
        return "// TrajectoryOpFormat 实现"
    if name == "TrajectoryOpConfigImpl.h":
        return "// IOpParamConfig 通用实现：绑定 resource JSON schema"
    if name == "TrajectoryOpConfigImpl.cpp":
        return "// TrajectoryOpConfigImpl 实现"
    if name == "TrajectoryUnifiedScope.h":
        return "// 管道执行期 scope 解析与活动程序上下文"
    if name == "TrajectoryUnifiedScope.cpp":
        return "// TrajectoryUnifiedScope 实现"
    if name == "UnifiedTrajectoryPathMath.h":
        return "// UnifiedTrajectory 路径几何原语，供原子块复用"
    if name == "UnifiedTrajectoryPathMath.cpp":
        return "// UnifiedTrajectoryPathMath 实现"
    if name == "TrajectoryOpBuiltinsRegister.cpp":
        return "// 启动时注册全部内置原子块到 OpRegistry / ConfigRegistry"

    for op, desc in OP_DESC.items():
        if stem == f"{op}Op":
            return f"// {op} 原子块：{desc}"
        if stem == f"{op}OpConfig":
            return f"// {op} 块参数 schema 与默认 TrajectoryOpDescriptor"
        if stem == f"{op}OpParamAccess":
            return f"// {op} 块参数字段与 descriptor 读写"
    return None


def ensure_comment(path: Path) -> bool:
    comment = comment_for(path)
    if not comment:
        return False
    text = path.read_text(encoding="utf-8")
    if text.startswith("//") or text.startswith("/*"):
        return False
    path.write_text(comment + "\n" + text, encoding="utf-8")
    return True


def main() -> None:
    changed = 0
    for path in sorted(ROOT.rglob("*")):
        if path.suffix not in {".h", ".cpp"}:
            continue
        if ensure_comment(path):
            changed += 1
            print(path.relative_to(ROOT))
    print(f"annotated {changed} files")


if __name__ == "__main__":
    main()
