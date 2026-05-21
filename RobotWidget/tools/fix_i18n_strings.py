from pathlib import Path
import re

p = Path(r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\RobotWidget\source\RobotSimulationController.cpp")
t = p.read_text(encoding="utf-8", errors="replace")

t = t.replace("m_host->runInfoPage()->appendInfo(", "m_host->appendRunInfo(")
t = t.replace("m_host->runInfoPage()->appendWarning(", "m_host->appendRunWarning(")

# i18n(english, broken_chinese) -> i18n(english, english) when chinese has mojibake
def fix_i18n(m):
    en = m.group(1)
    zh = m.group(2)
    if "?" in zh or "\ufffd" in zh or any(ord(c) > 0x4E00 and ord(c) < 0x9FFF for c in zh[:3]):
        return f"m_host->i18n({en}, {en})"
    return m.group(0)

t = re.sub(
    r"m_host->i18n\((QStringLiteral\([^)]+\)(?:\.arg\([^)]+\))*),\s*(QStringLiteral\([^)]*\)[^)]*)\)",
    fix_i18n,
    t,
)

# standalone broken QStringLiteral lines used as second arg
t = re.sub(
    r",\s*QStringLiteral\(\"[^\"]*\?[^\"]*\"\)",
    lambda m: ", " + m.group(0).split(",", 1)[1].strip() if False else "",
    t,
)

# simpler: replace known broken patterns with English duplicates
broken_pairs = [
    ('QStringLiteral("鏈哄櫒浜轰豢鐪熶笂涓嬫枃灏氭湭灏辩华銆?))', 'QStringLiteral("Robot simulation context is not ready."))'),
    ('QStringLiteral("URDF 璺緞涓虹┖銆?))', 'QStringLiteral("URDF path is empty."))'),
    ('QStringLiteral("URDF 姝ｈВ璁＄畻澶辫触銆?))', 'QStringLiteral("URDF forward kinematics failed."))'),
    ('QStringLiteral("鏈未閰缃硶鍏拌繛鏉嗗悕銆?))', 'QStringLiteral("Flange link name is not configured."))'),
    ('QStringLiteral("鏃犳硶鑾峰彇鏈綋涓栫晫鍧愭爣銆?))', 'QStringLiteral("Cannot evaluate TCP world transform."))'),
]
for old, new in broken_pairs:
    t = t.replace(old, new)

# fix i18n with broken zh: use same as en for lines containing ?
lines = t.splitlines()
out = []
for line in lines:
    if "m_host->i18n(" in line and "?" in line:
        # duplicate first QStringLiteral argument as second
        m = re.search(
            r"m_host->i18n\((QStringLiteral\([^)]+\)(?:\.arg\([^)]+\))*),\s*QStringLiteral\([^)]+\)[^)]*\)",
            line,
        )
        if m:
            line = line[: m.start()] + f"m_host->i18n({m.group(1)}, {m.group(1)})" + line[m.end() :]
    if "QStringLiteral(" in line and "?" in line and "m_host->i18n" not in line:
        line = re.sub(r"QStringLiteral\(\"[^\"]*\?[^\"]*\"\)", 'QStringLiteral("...")', line)
    out.append(line)
t = "\n".join(out) + "\n"

p.write_text(t, encoding="utf-8")
print("fixed")
