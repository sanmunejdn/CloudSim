from pathlib import Path

text = Path(r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\RobotWidget\source\RobotSimulationController.cpp").read_text(
    encoding="utf-8", errors="replace"
)
balance = 0
in_str = False
esc = False
quote = ""
for lineno, line in enumerate(text.splitlines(), 1):
    i = 0
    while i < len(line):
        c = line[i]
        if in_str:
            if esc:
                esc = False
            elif c == "\\":
                esc = True
            elif c == quote:
                in_str = False
            i += 1
            continue
        if c in "\"'":
            in_str = True
            quote = c
            i += 1
            continue
        if c == "{":
            balance += 1
        elif c == "}":
            balance -= 1
        i += 1
    if lineno in (1180, 1186, 1187, 2620, 2621) or balance < 0:
        print(f"line {lineno}: balance={balance}")
print("final", balance)
