#!/usr/bin/env python3
"""生成每算法块 OpConfig / OpParamAccess 头源文件。"""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "Robot"
BUILTINS = ROOT / "TrajectoryAlgorithmBuiltins" / "ops"

OPS = [
    ("Translate", "translate.", "Translate"),
    ("Rotate", "rotate.", "Rotate"),
    ("Mirror", "mirror.", "Mirror"),
    ("Delete", None, "Delete"),
    ("Duplicate", "structural.", "Duplicate"),
    ("Reorder", None, "Reorder"),
    ("Approach", "approach.", "Approach"),
    ("Retract", "retract.", "Retract"),
]

ACCESS_CPP = ROOT / "TrajectoryAlgorithm" / "source" / "TrajectoryOpParamAccess.cpp"


def extract_function_blocks(text: str, fn_name: str) -> str:
    marker = f"bool TrajectoryOpParamAccess::{fn_name}("
    start = text.find(marker)
    if start < 0:
        return ""
    brace = text.find("{", start)
    depth = 0
    for i in range(brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[brace + 1 : i]
    return ""


def extract_key_blocks(body: str, prefix: str) -> str:
    pattern = re.compile(
        r'\tif \(field\.key == "([^"]+)"\)\s*\{([^}]*(?:\{[^}]*\}[^}]*)*)\}',
        re.MULTILINE,
    )
    chunks = []
    for key, block in pattern.findall(body):
        if prefix and not key.startswith(prefix):
            continue
        chunks.append(f'\tif (field.key == "{key}")\n\t{{{block}\n\t}}')
    return "\n".join(chunks)


def gen_config(name: str, kind_token: str, folder: Path):
    h = folder / f"{name}OpConfig.h"
    cpp = folder / f"{name}OpConfig.cpp"
    h.write_text(
        f"""#pragma once

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{{

std::unique_ptr<IOpParamConfig> make{name}OpConfig();

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )
    cpp.write_text(
        f"""#include "{name}OpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{{

std::unique_ptr<IOpParamConfig> make{name}OpConfig()
{{
\treturn makeTrajectoryOpConfig(
\t\tRobotInstruction::TrajectoryOpKind::{kind_token},
\t\t"ops/{name}.json");
}}

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )


def gen_access(name: str, prefix: str | None, folder: Path, src: str):
    h = folder / f"{name}OpParamAccess.h"
    cpp = folder / f"{name}OpParamAccess.cpp"
    h.write_text(
        f"""#pragma once

#include "IOpParamAccess.h"

#include <memory>

namespace trajectory_algo
{{

class {name}OpParamAccess final : public IOpParamAccess
{{
public:
\tbool handlesKey(const std::string& key) const override;
\tbool read(
\t\tconst RobotInstruction::TrajectoryOpDescriptor& op,
\t\tconst TrajectoryOpParamField& field,
\t\tTrajectoryParamValue& out) const override;
\tbool write(
\t\tRobotInstruction::TrajectoryOpDescriptor& op,
\t\tconst TrajectoryOpParamField& field,
\t\tconst TrajectoryParamValue& in) const override;
}};

std::unique_ptr<IOpParamAccess> make{name}OpParamAccess();

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )
    if not prefix:
        cpp_body = f"""#include "{name}OpParamAccess.h"

namespace trajectory_algo
{{

bool {name}OpParamAccess::handlesKey(const std::string& key) const
{{
\t(void)key;
\treturn false;
}}

bool {name}OpParamAccess::read(
\tconst RobotInstruction::TrajectoryOpDescriptor& op,
\tconst TrajectoryOpParamField& field,
\tTrajectoryParamValue& out) const
{{
\t(void)op;
\t(void)field;
\t(void)out;
\treturn false;
}}

bool {name}OpParamAccess::write(
\tRobotInstruction::TrajectoryOpDescriptor& op,
\tconst TrajectoryOpParamField& field,
\tconst TrajectoryParamValue& in) const
{{
\t(void)op;
\t(void)field;
\t(void)in;
\treturn false;
}}

std::unique_ptr<IOpParamAccess> make{name}OpParamAccess()
{{
\treturn std::make_unique<{name}OpParamAccess>();
}}

}} // namespace trajectory_algo
"""
    else:
        read_body = extract_function_blocks(src, "read")
        write_body = extract_function_blocks(src, "write")
        read_block = extract_key_blocks(read_body, prefix)
        write_block = extract_key_blocks(write_body, prefix)
        cpp_body = f"""#include "{name}OpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{{

bool {name}OpParamAccess::handlesKey(const std::string& key) const
{{
\treturn key.rfind("{prefix}", 0) == 0;
}}

bool {name}OpParamAccess::read(
\tconst RobotInstruction::TrajectoryOpDescriptor& op,
\tconst TrajectoryOpParamField& field,
\tTrajectoryParamValue& out) const
{{
{read_block}
\treturn false;
}}

bool {name}OpParamAccess::write(
\tRobotInstruction::TrajectoryOpDescriptor& op,
\tconst TrajectoryOpParamField& field,
\tconst TrajectoryParamValue& in) const
{{
{write_block}
\treturn false;
}}

std::unique_ptr<IOpParamAccess> make{name}OpParamAccess()
{{
\treturn std::make_unique<{name}OpParamAccess>();
}}

}} // namespace trajectory_algo
"""
    cpp.write_text(cpp_body, encoding="utf-8")


def main():
    src = ACCESS_CPP.read_text(encoding="utf-8")
    for name, prefix, kind_token in OPS:
        folder = BUILTINS / name
        folder.mkdir(parents=True, exist_ok=True)
        gen_config(name, kind_token, folder)
        gen_access(name, prefix, folder, src)
        print(f"generated {name}")


if __name__ == "__main__":
    main()
