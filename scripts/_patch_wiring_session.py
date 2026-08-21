# -*- coding: utf-8 -*-
from pathlib import Path

src = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim/src/UI/Widget/source")
candidates = list(src.glob("*SignalWiring*.cpp")) + list(src.glob("*SceneSignal*.cpp"))
print("candidates", [c.name for c in candidates])
p = None
for c in candidates:
    t = c.read_text(encoding="utf-8-sig", errors="replace")
    if "meshPickCommitted" in t:
        p = c
        break
if not p:
    raise SystemExit("no wiring file")
print("using", p)
t = p.read_text(encoding="utf-8-sig")
old = """\tQObject::connect(o, &OsgWidget::meshPickCommitted, &mw,
\t\t\t\t\t [robotHost](const PickResult pick, const int pickKindInt)
\t\t\t\t\t {
\t\t\t\t\t\t if (robotHost)
\t\t\t\t\t\t {
\t\t\t\t\t\t\t robotHost->notifyMeshPickCommitted(pick, static_cast<PickKind>(pickKindInt));
\t\t\t\t\t\t }
\t\t\t\t\t });"""
new = """\tQObject::connect(o, &OsgWidget::meshPickCommitted, &mw,
\t\t\t\t\t [robotHost, o](const PickResult pick, const int pickKindInt)
\t\t\t\t\t {
\t\t\t\t\t\t if (o && o->hasInteractionSession() && o->interactionController())
\t\t\t\t\t\t {
\t\t\t\t\t\t\t ViewportHit hit;
\t\t\t\t\t\t\t hit.phase = HitPhase::Commit;
\t\t\t\t\t\t\t hit.kind = static_cast<PickKind>(pickKindInt);
\t\t\t\t\t\t\t hit.raw = pick;
\t\t\t\t\t\t\t o->interactionController()->dispatchCommit(hit, HitResolveContext{});
\t\t\t\t\t\t\t return;
\t\t\t\t\t\t }
\t\t\t\t\t\t if (robotHost)
\t\t\t\t\t\t {
\t\t\t\t\t\t\t robotHost->notifyMeshPickCommitted(pick, static_cast<PickKind>(pickKindInt));
\t\t\t\t\t\t }
\t\t\t\t\t });"""
if old not in t:
    print("exact block missing; dumping nearby")
    lines = t.splitlines()
    for i, l in enumerate(lines):
        if "meshPickCommitted" in l:
            print("\n".join(f"{j+1}:{lines[j]}" for j in range(max(0, i - 1), min(len(lines), i + 12)))
            )
    raise SystemExit(1)
t = t.replace(old, new)
if "ViewportHit.h" not in t:
    t = t.replace(
        '#include "OsgWidget.h"',
        '#include "OsgWidget.h"\n'
        '#include "ViewportInteraction/ViewportHit.h"\n'
        '#include "ViewportInteraction/ViewportInteractionController.h"',
    )
p.write_bytes(t.replace("\n", "\r\n").encode("utf-8-sig"))
print("patched", p.name)

# PointPick include
pp = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim/src/UI/Widget/source/PointPickOperation.cpp")
pt = pp.read_text(encoding="utf-8-sig")
if "IViewportPickEngine.h" not in pt:
    pt = pt.replace(
        '#include "OsgWidget.h"',
        '#include "OsgWidget.h"\n#include "ViewportInteraction/IViewportPickEngine.h"',
    )
    pp.write_bytes(pt.replace("\n", "\r\n").encode("utf-8-sig"))
    print("PointPick include fixed")
