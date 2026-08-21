# -*- coding: utf-8 -*-
from pathlib import Path

# PointPick include
p = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim/src/UI/Widget/source/PointPickOperation.cpp")
t = p.read_text(encoding="utf-8-sig")
if "IViewportPickEngine.h" not in t:
    t = t.replace(
        '#include "OsgWidget.h"',
        '#include "OsgWidget.h"\n#include "ViewportInteraction/IViewportPickEngine.h"',
    )
    p.write_bytes(t.replace("\n", "\r\n").encode("utf-8-sig"))
    print("PointPick include ok")

# Dual-cast guard in MainWindowRobotHost::notifyMeshPickCommitted
host = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim/src/UI/Widget/source/MainWindowRobotHost.cpp")
ht = host.read_text(encoding="utf-8-sig")
needle = "if (m_meshPickHandler)"
guard = """if (m_mw)
	{
		if (auto* page = m_mw->currentDocumentPage())
		{
			if (auto* osg = page->osgWidget())
			{
				if (osg->hasInteractionSession())
				{
					return;
				}
			}
		}
	}
	if (m_meshPickHandler)"""
if "hasInteractionSession()" not in ht and needle in ht:
    # only replace the handler check at end of notifyMeshPickCommitted - find last occurrence after solid pick
    idx = ht.rfind(needle)
    ht = ht[:idx] + guard + ht[idx + len(needle) :]
    host.write_bytes(ht.replace("\n", "\r\n").encode("utf-8-sig"))
    print("dual-cast guard added")
else:
    print("host guard skip/exists")

# FeatureTrajectory: after setMeshFacePickMode(true) begin a MeshPickSession that forwards to onMeshPickCommitted
ft = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim/src/UI/RobotWidget/source/FeatureTrajectoryPageWidget.cpp")
ftt = ft.read_text(encoding="utf-8-sig")
if "MeshPickSession" not in ftt:
    ftt = ftt.replace(
        '#include "FeatureTrajectoryPageWidget.h"',
        '#include "FeatureTrajectoryPageWidget.h"\n#include "ViewportInteraction/MeshPickSession.h"\n#include "OsgWidget.h"',
        1,
    )
    # wrap face pick enable
    old = "osg->setMeshFacePickMode(true);"
    if old in ftt:
        new = """osg->setMeshFacePickMode(true);
	osg->beginInteractionSession(std::make_shared<MeshPickSession>(
		[this](const ViewportHit& hit) {
			onMeshPickCommitted(hit.raw, static_cast<int>(hit.kind));
		},
		[osg]() { osg->setMeshFacePickMode(false); }));"""
        # only replace first enable that is for picking - replace all true enables carefully
        ftt = ftt.replace(old, new, 1)
    # on disable end session
    old_off = "osg->setMeshFacePickMode(false);"
    # replace disables that don't already end session - add end before false
    ftt2 = []
    for line in ftt.splitlines(True):
        if "setMeshFacePickMode(false)" in line and "endInteractionSession" not in line:
            indent = line[: len(line) - len(line.lstrip())]
            ftt2.append(f"{indent}osg->endInteractionSession(true);\n")
        ftt2.append(line)
    ftt = "".join(ftt2)
    ft.write_bytes(ftt.replace("\n", "\r\n").encode("utf-8-sig"))
    print("FeatureTrajectory session wired")
else:
    print("FeatureTrajectory already has session")
