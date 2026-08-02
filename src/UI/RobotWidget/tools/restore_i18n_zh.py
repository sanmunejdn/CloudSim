# -*- coding: utf-8 -*-
"""按英文键恢复 RobotSimulationController.cpp 中被 ? 破坏的中文 i18n。"""
from __future__ import annotations

import re
from pathlib import Path

p = Path(__file__).resolve().parent.parent / "source" / "RobotSimulationController.cpp"
text = p.read_text(encoding="utf-8")

EN_TO_ZH = {
	"Stop simulation before TCP drag teach.": "请先停止仿真，再使用末端拖动示教。",
	"Failed to attach TCP drag gizmo.": "无法挂载 TCP 拖动示教罗盘。",
	"TCP drag IK exceeded joint limits; angles were clamped to URDF range.": "末端拖动 IK 超关节限位，已钳制到 URDF 范围。",
	"Simulation stopped.": "仿真已停止。",
	"Import a robot (URDF) first, then export the program.": "请先导入机器人(URDF)，再导出程序。",
	"No program to export.": "没有可导出的程序。",
	"Invalid brand selection.": "品牌选择无效。",
	"Selected program not found.": "未找到所选程序。",
	"No motion instructions to export.": "没有可导出的运动指令。",
	"All files (*.*)": "所有文件 (*.*)",
	"Save brand robot program": "保存品牌机器人程序",
	"Brand export script not found: %1": "未找到品牌导出脚本：%1",
	"Cannot create temporary Canonical file.": "无法创建临时 Canonical 文件。",
	"Brand export failed: %1": "品牌导出失败：%1",
	'Exported %1 program "%2" to %3 (flat motion refs: %4).': "已导出 %1 程序「%2」到 %3（展平运动引用：%4）。",
	"Robot simulation context is not ready.": "机器人仿真上下文尚未就绪。",
	"URDF path is empty.": "URDF 路径为空。",
	"URDF forward kinematics failed.": "URDF 正解计算失败。",
	"Cannot evaluate TCP: %1": "无法求 TCP：%1",
	"Flange link name is not configured.": "未配置法兰连杆名。",
	"Link '%1' not in URDF FK result (check tool frame flange link).": "连杆「%1」不在 URDF 正解结果中（请检查工具系法兰连杆）。",
	"Per-link robot has no joint scene node '%1'; use URDF FK path.": "每连杆机器人无关节场景节点「%1」；请使用 URDF 正解路径。",
	"Cannot evaluate TCP world transform.": "无法获取末端世界坐标。",
	"Preview IK failed.": "预览 IK 失败。",
	"Import a robot (URDF) first, then add simulation commands.": "请先导入机器人(URDF)，再添加仿真指令。",
	'No revolute joints in URDF (joints need type="revolute" or "continuous" and an axis).': "URDF 中无可旋转关节（需 type=“revolute/continuous” 及 axis）。",
	"Add at least one instruction row.": "请至少添加一条指令。",
	"Instruction row is invalid.": "指令行无效。",
	"All motion instructions failed to plan; simulation not started.": "所有运动指令规划失败，未启动仿真。",
	"Partial plan failure: will play until motion %1 (%2), then stop. Reason: %3": "部分规划失败：将播放至第 %1 条（%2）后停止。原因：%3",
	"Lazy planning: %1/%2 motions planned at start; rest on demand.": "懒加载规划：启动时已规划 %1/%2 条，其余按需规划。",
	"Invalid joint index in simulation command.": "仿真指令关节索引无效。",
	"Simulation started.": "仿真已开始。",
	"Forward kinematics failed: %1": "正解失败：%1",
	"Simulation finished.": "仿真已结束。",
	"Simulation stopped before failed motion%1. %2": "仿真已在失败运动前停止%1。%2",
	" (%1)": "（%1）",
	"Lazy plan seed unavailable for motion %1.": "运动 %1 的懒加载规划种子不可用。",
}

# i18n(QStringLiteral("en")[.arg...]*, QStringLiteral("zh")[.arg...]*)
i18n_pat = re.compile(
	r'i18n\(\s*QStringLiteral\("((?:\\.|[^"\\])*)"\)'
	r'((?:\s*\.arg\([^;]*?\))*)\s*,\s*'
	r'QStringLiteral\("((?:\\.|[^"\\])*)"\)',
	re.M | re.S,
)

replaced = 0
missing_map: list[str] = []


def repl_i18n(m: re.Match[str]) -> str:
	global replaced
	en, mid_args, zh = m.group(1), m.group(2), m.group(3)
	if "?" not in zh:
		return m.group(0)
	zh_new = EN_TO_ZH.get(en)
	if zh_new is None:
		missing_map.append(en)
		return m.group(0)
	replaced += 1
	return f'i18n(QStringLiteral("{en}"){mid_args}, QStringLiteral("{zh_new}")'


text2 = i18n_pat.sub(repl_i18n, text)

text2, n_tag = re.subn(
	r'QStringLiteral\("\?\?%1\?\?"\)',
	'QStringLiteral("（%1）")',
	text2,
)
replaced += n_tag

# 无英文对照的独立字面量（按出现上下文）
for old, new in [
	('QStringLiteral("????????????")', 'QStringLiteral("特征轨迹页不可用")'),
	('QStringLiteral("?????????????")', 'QStringLiteral("候选预览构建失败")'),
]:
	c = text2.count(old)
	if c:
		text2 = text2.replace(old, new)
		replaced += c

orphan = []
for m in re.finditer(r'QStringLiteral\("([^"]*\?[^"]*)"\)', text2):
	s = m.group(1)
	if "?" in s and not any("\u4e00" <= c <= "\u9fff" for c in s):
		# 保留合法含 ? 的通配/通配符文案
		if s in ("All files (*.*)",) or "*.*" in s:
			continue
		orphan.append(s)

p.write_text(text2, encoding="utf-8", newline="\r\n")
print(f"replaced={replaced}")
print(f"missing_map={len(missing_map)}")
for e in missing_map:
	print("  MISSING:", e)
print(f"orphan_qstrings={len(set(orphan))}")
for s in sorted(set(orphan)):
	print("  ORPHAN:", s)
