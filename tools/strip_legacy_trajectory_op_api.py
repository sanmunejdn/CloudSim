#!/usr/bin/env python3
"""从 Builtins Op 四件套中移除 contributePreviewTransform / buildApplyActions。"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "Robot" / "TrajectoryAlgorithmBuiltins" / "ops"

H_PATTERN = re.compile(
    r"\n\tbool contributePreviewTransform\([^;]+;\n"
    r"\tstd::vector<TrajectoryApplyAction> buildApplyActions\([^;]+;\n"
    r"\tbool processPath\(",
    re.MULTILINE | re.DOTALL,
)

CPP_PATTERN = re.compile(
    r"\nbool \w+Op::contributePreviewTransform\(\n.*?\n\}\n\n"
    r"std::vector<TrajectoryApplyAction> \w+Op::buildApplyActions\(\n.*?\n\}\n\n"
    r"bool \w+Op::processPath\(",
    re.MULTILINE | re.DOTALL,
)

CAP_REPLACEMENTS = {
    "PreviewPoseTransform | TrajectoryOpCapability::ApplyPoseTransform":
        "PreviewPoseTransform",
    "TrajectoryOpCapability::ApplyPoseTransform | TrajectoryOpCapability::PreviewPoseTransform":
        "PreviewPoseTransform",
    "TrajectoryOpCapability::ApplyStructuralEdit": "TrajectoryOpCapability::None",
}


def patch_file(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    orig = text
    if path.suffix == ".h":
        text = H_PATTERN.sub("\n\tbool processPath(", text)
    elif path.suffix == ".cpp":
        text = CPP_PATTERN.sub("\nbool {}Op::processPath(".format(path.parent.name.replace("Op", "") if False else ""), text)
        # fix broken substitution - use simpler approach per file
        text = orig
        start = text.find("bool ")
        idx = text.find("Op::contributePreviewTransform")
        if idx >= 0:
            end = text.find("bool ", idx + 1)
            proc = text.find("Op::processPath", idx)
            if proc > idx:
                # find line start before contributePreviewTransform
                line_start = text.rfind("\n", 0, idx) + 1
                proc_line_start = text.rfind("\n", 0, proc) + 1
                text = text[:line_start] + text[proc_line_start:]
        for old, new in CAP_REPLACEMENTS.items():
            text = text.replace(old, new)
    if text != orig:
        path.write_text(text, encoding="utf-8")
        return True
    return False


def strip_cpp_legacy(name: str, folder: Path) -> bool:
    cpp = folder / f"{name}Op.cpp"
    if not cpp.exists():
        return False
    text = cpp.read_text(encoding="utf-8")
    marker = f"bool {name}Op::contributePreviewTransform"
    proc = f"bool {name}Op::processPath"
    if marker not in text:
        for old, new in CAP_REPLACEMENTS.items():
            text = text.replace(old, new)
        cpp.write_text(text, encoding="utf-8")
        return False
    i = text.index(marker)
    j = text.index(proc, i)
    line_i = text.rfind("\n", 0, i) + 1
    line_j = text.rfind("\n", 0, j) + 1
    text = text[:line_i] + text[line_j:]
    for old, new in CAP_REPLACEMENTS.items():
        text = text.replace(old, new)
    cpp.write_text(text, encoding="utf-8")
    return True


def strip_h_legacy(name: str, folder: Path) -> bool:
    h = folder / f"{name}Op.h"
    if not h.exists():
        return False
    text = h.read_text(encoding="utf-8")
    old = text
    text = text.replace(
        "\tbool contributePreviewTransform(\n"
        "\t\tconst RobotInstruction::TrajectoryOpDescriptor& op,\n"
        "\t\tconst std::vector<std::string>& targetIds,\n"
        "\t\tPreviewTransformStep& out) const override;\n"
        "\tstd::vector<TrajectoryApplyAction> buildApplyActions(\n"
        "\t\tconst TrajectoryOpContext& ctx,\n"
        "\t\tconst RobotInstruction::TrajectoryOpDescriptor& op) const override;\n",
        "",
    )
    if text != old:
        h.write_text(text, encoding="utf-8")
        return True
    return False


def main():
    count = 0
    for folder in sorted(ROOT.iterdir()):
        if not folder.is_dir():
            continue
        name = folder.name
        if strip_h_legacy(name, folder):
            count += 1
        if strip_cpp_legacy(name, folder):
            count += 1
    print(f"stripped legacy methods in {count} files")


if __name__ == "__main__":
    main()
