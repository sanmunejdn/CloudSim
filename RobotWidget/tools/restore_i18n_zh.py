# -*- coding: utf-8 -*-
"""Restore Chinese strings in RobotSimulationController.cpp (UTF-8)."""
from pathlib import Path

p = Path(__file__).resolve().parent.parent / "source" / "RobotSimulationController.cpp"
t = p.read_text(encoding="utf-8")

replacements = [
    (
        'm_host->i18n(QStringLiteral("Stop simulation before TCP drag teach."), QStringLiteral("..."))',
        'm_host->i18n(QStringLiteral("Stop simulation before TCP drag teach."), QStringLiteral("请先停止仿真，再使用末端拖动示教。"))',
    ),
    (
        'm_host->i18n(QStringLiteral("Failed to attach TCP drag gizmo."), QStringLiteral("..."))',
        'm_host->i18n(QStringLiteral("Failed to attach TCP drag gizmo."), QStringLiteral("无法挂载 TCP 拖动示教罗盘。"))',
    ),
    (
        'm_host->i18n(QStringLiteral("Simulation stopped."), QStringLiteral("..."))',
        'm_host->i18n(QStringLiteral("Simulation stopped."), QStringLiteral("仿真已停止。"))',
    ),
    (
        'QStringLiteral("Import a robot (URDF) first, then export the program."), QStringLiteral("...")',
        'QStringLiteral("Import a robot (URDF) first, then export the program."), QStringLiteral("请先导入机器人(URDF)，再导出程序。")',
    ),
    (
        'm_host->i18n(QStringLiteral("No motion instructions to export."), QStringLiteral("..."))',
        'm_host->i18n(QStringLiteral("No motion instructions to export."), QStringLiteral("没有可导出的运动指令。"))',
    ),
    (
        'm_host->i18n(QStringLiteral("Export robot program"), QStringLiteral("Export robot program"))',
        'm_host->i18n(QStringLiteral("Export robot program"), QStringLiteral("导出机器人程序"))',
    ),
    (
        'm_host->i18n(QStringLiteral("JSON (*.json);;CSV (*.csv)"), QStringLiteral("JSON (*.json);;CSV (*.csv)"))',
        'm_host->i18n(QStringLiteral("JSON (*.json);;CSV (*.csv)"), QStringLiteral("JSON (*.json);;CSV (*.csv)"))',
    ),
    (
        'QStringLiteral("Cannot write file: %1").arg(path),\n\t\t\t\t\tQStringLiteral("Cannot write file: %1").arg(path)',
        'QStringLiteral("Cannot write file: %1").arg(path),\n\t\t\t\t\tQStringLiteral("无法写入文件：%1").arg(path)',
    ),
    (
        'QStringLiteral("Exported program to %1 (%2 motion points, %3 IK failures).")\n\t\t\t\t\t.arg(path)\n\t\t\t\t\t.arg(exportResult.points.size())\n\t\t\t\t\t.arg(failedCount),\n\t\t\t\tQStringLiteral("Exported program to %1 (%2 motion points, %3 IK failures).")',
        'QStringLiteral("Exported program to %1 (%2 motion points, %3 IK failures).")\n\t\t\t\t\t.arg(path)\n\t\t\t\t\t.arg(exportResult.points.size())\n\t\t\t\t\t.arg(failedCount),\n\t\t\t\tQStringLiteral("已导出程序到 %1（%2 个运动点，%3 次 IK 失败）。")',
    ),
    (
        '*errMsg = m_host->i18n(QStringLiteral("Robot simulation context is not ready."), QStringLiteral("Robot simulation context is not ready."))',
        '*errMsg = m_host->i18n(QStringLiteral("Robot simulation context is not ready."), QStringLiteral("机器人仿真上下文尚未就绪。"))',
    ),
    (
        '*errMsg = m_host->i18n(QStringLiteral("URDF path is empty."), QStringLiteral("URDF path is empty."))',
        '*errMsg = m_host->i18n(QStringLiteral("URDF path is empty."), QStringLiteral("URDF 路径为空。"))',
    ),
    (
        '? m_host->i18n(QStringLiteral("URDF forward kinematics failed."), QStringLiteral("URDF forward kinematics failed."))',
        '? m_host->i18n(QStringLiteral("URDF forward kinematics failed."), QStringLiteral("URDF 正解计算失败。"))',
    ),
    (
        'QStringLiteral("Cannot evaluate TCP: %1").arg(detail),\n\t\t\t\t\tQStringLiteral("Cannot evaluate TCP: %1").arg(detail)',
        'QStringLiteral("Cannot evaluate TCP: %1").arg(detail),\n\t\t\t\t\tQStringLiteral("无法求 TCP：%1").arg(detail)',
    ),
    (
        'QStringLiteral("Flange link name is not configured."),\n\t\t\t\t\t\tQStringLiteral("Flange link name is not configured.")',
        'QStringLiteral("Flange link name is not configured."),\n\t\t\t\t\t\tQStringLiteral("未配置法兰连杆名。")',
    ),
    (
        'QStringLiteral("Link \'%1\' not in URDF FK result (check tool frame flange link).")\n\t\t\t\t\t\t\t.arg(flangeQ),\n\t\t\t\t\t\tQStringLiteral("Link \'%1\' not in URDF FK result (check tool frame flange link).")',
        'QStringLiteral("Link \'%1\' not in URDF FK result (check tool frame flange link).")\n\t\t\t\t\t\t\t.arg(flangeQ),\n\t\t\t\t\t\tQStringLiteral("连杆「%1」不在 URDF 正解结果中（请检查工具系法兰连杆）。")',
    ),
    (
        'QStringLiteral("Per-link robot has no joint scene node \'%1\'; use URDF FK path.")\n\t\t\t\t\t\t\t.arg(lastJointName),\n\t\t\t\t\t\tQStringLiteral("Per-link robot has no joint scene node \'%1\'; use URDF FK path.")',
        'QStringLiteral("Per-link robot has no joint scene node \'%1\'; use URDF FK path.")\n\t\t\t\t\t\t\t.arg(lastJointName),\n\t\t\t\t\t\tQStringLiteral("每连杆机器人无关节场景节点「%1」；请使用 URDF 正解路径。")',
    ),
    (
        '*errMsg = m_host->i18n(QStringLiteral("Cannot evaluate TCP world transform."), QStringLiteral("Cannot evaluate TCP world transform."))',
        '*errMsg = m_host->i18n(QStringLiteral("Cannot evaluate TCP world transform."), QStringLiteral("无法获取末端世界坐标。"))',
    ),
    (
        'QStringLiteral("Import a robot (URDF) first, then add simulation commands."), QStringLiteral("...")',
        'QStringLiteral("Import a robot (URDF) first, then add simulation commands."), QStringLiteral("请先导入机器人(URDF)，再添加仿真指令。")',
    ),
    (
        'QStringLiteral("No revolute joints in URDF (joints need type=\\"revolute\\" or \\"continuous\\" and an axis)."), QStringLiteral("...")',
        'QStringLiteral("No revolute joints in URDF (joints need type=\\"revolute\\" or \\"continuous\\" and an axis)."), QStringLiteral("URDF 中无可旋转关节（需 type=“revolute/continuous” 及 axis）。")',
    ),
    (
        'm_host->i18n(QStringLiteral("Add at least one instruction row."), QStringLiteral("..."))',
        'm_host->i18n(QStringLiteral("Add at least one instruction row."), QStringLiteral("请至少添加一条指令。"))',
    ),
    (
        'QStringLiteral("Instruction row is invalid."), QStringLiteral("...")',
        'QStringLiteral("Instruction row is invalid."), QStringLiteral("指令行无效。")',
    ),
    (
        'm_host->i18n(QStringLiteral("Instruction validation failed."), QStringLiteral("Instruction validation failed."))',
        'm_host->i18n(QStringLiteral("Instruction validation failed."), QStringLiteral("指令校验失败。"))',
    ),
    (
        'm_host->i18n(QStringLiteral("Instruction planning failed."), QStringLiteral("Instruction planning failed."))',
        'm_host->i18n(QStringLiteral("Instruction planning failed."), QStringLiteral("指令规划失败。"))',
    ),
    (
        'm_host->i18n(QStringLiteral("Invalid joint index in simulation command."), QStringLiteral("..."))',
        'm_host->i18n(QStringLiteral("Invalid joint index in simulation command."), QStringLiteral("仿真指令关节索引无效。"))',
    ),
    (
        'm_host->i18n(QStringLiteral("Simulation started."), QStringLiteral("..."))',
        'm_host->i18n(QStringLiteral("Simulation started."), QStringLiteral("仿真已开始。"))',
    ),
    (
        'm_host->appendRunWarning(QStringLiteral("...").arg(fkErr))',
        'm_host->appendRunWarning(m_host->i18n(QStringLiteral("Forward kinematics failed: %1").arg(fkErr), QStringLiteral("正解失败：%1").arg(fkErr)))',
    ),
    (
        'm_host->i18n(QStringLiteral("Simulation finished."), QStringLiteral("..."))',
        'm_host->i18n(QStringLiteral("Simulation finished."), QStringLiteral("仿真已结束。"))',
    ),
]

for old, new in replacements:
    if old not in t:
        print("MISSING:", old[:60])
    else:
        t = t.replace(old, new)

p.write_text(t, encoding="utf-8", newline="\r\n")
print("RobotSimulationController.cpp updated")
