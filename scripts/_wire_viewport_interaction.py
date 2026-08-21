# -*- coding: utf-8 -*-
from pathlib import Path

ROOT = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim/src/UI/Widget")
HDR = ROOT / "inc/OsgWidget.h"
CPP = ROOT / "source/OsgWidget.cpp"
VCX = ROOT / "Widget.vcxproj"
FIL = ROOT / "Widget.vcxproj.filters"


def bom_write(path: Path, text: str) -> None:
    path.write_bytes(text.replace("\n", "\r\n").encode("utf-8-sig"))


h = HDR.read_text(encoding="utf-8-sig")
if "ViewportInteractionController" not in h:
    h = h.replace(
        "class OsgWidgetPickAnnotationController;",
        "class OsgWidgetPickAnnotationController;\n"
        "class ViewportInteractionController;\n"
        "class IViewportPickEngine;\n"
        "class IInteractionSession;",
    )
    h = h.replace(
        "PickResult queryPick(const PickQuery& query);",
        "PickResult queryPick(const PickQuery& query);\n"
        "\tViewportInteractionController* interactionController() { return m_interactionController.get(); }\n"
        "\tIViewportPickEngine* pickEngine();\n"
        "\tvoid beginInteractionSession(std::shared_ptr<IInteractionSession> session);\n"
        "\tvoid endInteractionSession(bool cancel = true);\n"
        "\tbool hasInteractionSession() const;\n"
        "\tvoid setupInteractionController();",
    )
    h = h.replace(
        "std::unique_ptr<SelectionOperation> m_labelingPickOperation;",
        "std::unique_ptr<SelectionOperation> m_labelingPickOperation;\n"
        "\tstd::unique_ptr<ViewportInteractionController> m_interactionController;",
    )
    if "#include <memory>" not in h:
        h = h.replace("#include <", "#include <memory>\n#include <", 1)
    bom_write(HDR, h)
    print("patched header")
else:
    print("header ok")

c = CPP.read_text(encoding="utf-8-sig")
inc_block = (
    '#include "ViewportInteraction/OsgWidgetPickEngine.h"\n'
    '#include "ViewportInteraction/ViewportInteractionController.h"\n'
    '#include "ViewportInteraction/Tools/SelectionOperationToolAdapter.h"\n'
    '#include "ViewportInteraction/Overlays/SelectionOperationOverlayAdapter.h"\n'
    '#include "ViewportInteraction/Policies/PassthroughHitPolicy.h"\n'
    '#include "ViewportInteraction/Policies/GizmoAxisHitPolicy.h"\n'
)
if "ViewportInteractionController.h" not in c:
    c = c.replace('#include "OsgWidget.h"', '#include "OsgWidget.h"\n' + inc_block, 1)

setup_fn = r'''
void OsgWidget::setupInteractionController()
{
	m_interactionController = std::make_unique<ViewportInteractionController>(
		std::make_unique<OsgWidgetPickEngine>(*this));
	m_interactionController->addOverlay(
		std::make_unique<SelectionOperationOverlayAdapter>("tcpDragTeach", m_tcpDragTeachOperation.get()));
	m_interactionController->addOverlay(
		std::make_unique<SelectionOperationOverlayAdapter>("meshSectionPlane", m_meshSectionPlaneOperation.get()));
	m_interactionController->addOverlay(
		std::make_unique<SelectionOperationOverlayAdapter>("objectTransform", m_objectTransformOperation.get()));
	m_interactionController->registerTool(
		std::make_unique<SelectionOperationToolAdapter>("pointCloud", m_pointPickOperation.get()));
	m_interactionController->registerTool(
		std::make_unique<SelectionOperationToolAdapter>("polyline", m_polylinePickOperation.get()));
	m_interactionController->registerTool(
		std::make_unique<SelectionOperationToolAdapter>("meshElement", m_meshElementPickOperation.get()));
	m_interactionController->registerTool(
		std::make_unique<SelectionOperationToolAdapter>("labeling", m_labelingPickOperation.get()));
	std::vector<std::unique_ptr<IHitResolvePolicy>> policies;
	policies.push_back(std::make_unique<GizmoAxisHitPolicy>());
	policies.push_back(std::make_unique<PassthroughHitPolicy>());
	m_interactionController->setHitPolicies(std::move(policies));
}

IViewportPickEngine* OsgWidget::pickEngine()
{
	return m_interactionController ? &m_interactionController->engine() : nullptr;
}

void OsgWidget::beginInteractionSession(std::shared_ptr<IInteractionSession> session)
{
	if (m_interactionController)
	{
		m_interactionController->beginSession(std::move(session));
	}
}

void OsgWidget::endInteractionSession(bool cancel)
{
	if (m_interactionController)
	{
		m_interactionController->endSession(cancel);
	}
}

bool OsgWidget::hasInteractionSession() const
{
	return m_interactionController && m_interactionController->hasSession();
}

'''

if "setupInteractionController()" not in c.split("void OsgWidget::setupInteractionController")[0]:
    pass

if "void OsgWidget::setupInteractionController" not in c:
    c = c.replace(
        "m_labelingPickOperation = std::make_unique<LabelingPickOperation>(this);",
        "m_labelingPickOperation = std::make_unique<LabelingPickOperation>(this);\n\tsetupInteractionController();",
    )
    needle = "void OsgWidget::clearImportedContent()"
    if needle in c:
        c = c.replace(needle, setup_fn + needle)
    else:
        c += setup_fn

old = """\tif (m_labelingPickOperation && m_labelingPickOperation->handleEvent(watched, event))
\t{
\t\treturn true;
\t}

\tif (m_pointPickOperation && m_pointPickOperation->handleEvent(watched, event))
\t{
\t\treturn true;
\t}

\tif (m_polylinePickOperation && m_polylinePickOperation->handleEvent(watched, event))
\t{
\t\treturn true;
\t}

\tif (m_meshElementPickOperation && m_meshElementPickOperation->handleEvent(watched, event))
\t{
\t\treturn true;
\t}

\tif (m_tcpDragTeachOperation && m_tcpDragTeachOperation->handleEvent(watched, event))
\t{
\t\treturn true;
\t}

\tif (m_meshSectionPlaneOperation && m_meshSectionPlaneOperation->handleEvent(watched, event))
\t{
\t\treturn true;
\t}

\tif (m_objectTransformOperation && m_objectTransformOperation->handleEvent(watched, event))
\t{
\t\treturn true;
\t}

\treturn QWidget::eventFilter(watched, event);
"""
new = """\tif (m_interactionController && m_interactionController->handleEvent(watched, event))
\t{
\t\treturn true;
\t}

\treturn QWidget::eventFilter(watched, event);
"""
if old in c:
    c = c.replace(old, new)
    print("eventFilter replaced")
elif "m_interactionController->handleEvent" in c:
    print("eventFilter already controller")
else:
    print("WARN eventFilter not replaced")

# sync tools on mode setters — match real function names from disk
syncs = [
    ("void OsgWidget::setObjectSelectionMode(bool enabled)", "objectSelect"),
    ("void OsgWidget::setPointPickMode(bool enabled)", "pointCloud"),
    ("void OsgWidget::setPolylinePickMode(bool enabled)", "polyline"),
    ("void OsgWidget::setMeshLinePickMode(bool enabled)", "meshElement"),
    ("void OsgWidget::setMeshFacePickMode(bool enabled)", "meshElement"),
]
for sig, tool in syncs:
    if sig not in c:
        print("missing setter", sig)
        continue
    idx = c.find(sig)
    window = c[idx : idx + 500]
    if f'setActiveTool("{tool}")' in window:
        continue
    brace = c.find("{", idx)
    snippet = f"""
\tif (m_interactionController)
\t{{
\t\tif (enabled)
\t\t{{
\t\t\tm_interactionController->setActiveTool("{tool}");
\t\t}}
\t\telse if (m_interactionController->activeToolId() &&
\t\t\t\t std::string(m_interactionController->activeToolId()) == "{tool}")
\t\t{{
\t\t\tm_interactionController->clearActiveTool();
\t\t}}
\t}}
"""
    c = c[: brace + 1] + snippet + c[brace + 1 :]
    print("synced", tool, "for", sig)

if "#include <string>" not in c[:2000]:
    c = c.replace('#include "OsgWidget.h"', '#include "OsgWidget.h"\n#include <string>', 1)

bom_write(CPP, c)
print("cpp patched")

# vcxproj
new_headers = [
    r"inc\ViewportInteraction\ViewportHit.h",
    r"inc\ViewportInteraction\IViewportPickEngine.h",
    r"inc\ViewportInteraction\OsgWidgetPickEngine.h",
    r"inc\ViewportInteraction\IPointerTool.h",
    r"inc\ViewportInteraction\IOverlayOp.h",
    r"inc\ViewportInteraction\IHitResolvePolicy.h",
    r"inc\ViewportInteraction\IInteractionSession.h",
    r"inc\ViewportInteraction\ViewportInteractionController.h",
    r"inc\ViewportInteraction\MeshPickSession.h",
    r"inc\ViewportInteraction\Policies\PassthroughHitPolicy.h",
    r"inc\ViewportInteraction\Policies\GizmoAxisHitPolicy.h",
    r"inc\ViewportInteraction\Policies\RobotObjectSelectPolicy.h",
    r"inc\ViewportInteraction\Tools\SelectionOperationToolAdapter.h",
    r"inc\ViewportInteraction\Overlays\SelectionOperationOverlayAdapter.h",
]
new_sources = [
    r"source\ViewportInteraction\OsgWidgetPickEngine.cpp",
    r"source\ViewportInteraction\ViewportInteractionController.cpp",
]
vcx = VCX.read_text(encoding="utf-8")
if "ViewportInteraction\\ViewportHit.h" not in vcx:
    insert_h = "\n".join(f'    <ClInclude Include="{p}" />' for p in new_headers)
    if 'Include="inc\\SelectionOperation.h"' in vcx:
        vcx = vcx.replace(
            '    <ClInclude Include="inc\\SelectionOperation.h" />',
            '    <ClInclude Include="inc\\SelectionOperation.h" />\n' + insert_h,
        )
    else:
        vcx = vcx.replace("</ItemGroup>", insert_h + "\n  </ItemGroup>", 1)
    insert_c = "\n".join(f'    <ClCompile Include="{p}" />' for p in new_sources)
    if 'Include="source\\ApplicationSettings.cpp"' in vcx:
        vcx = vcx.replace(
            '    <ClCompile Include="source\\ApplicationSettings.cpp" />',
            insert_c + "\n" + '    <ClCompile Include="source\\ApplicationSettings.cpp" />',
        )
    else:
        # find first ClCompile after pch
        vcx = vcx.replace(
            '    <ClCompile Include="source\\pch.cpp">',
            insert_c + "\n" + '    <ClCompile Include="source\\pch.cpp">',
        )
    VCX.write_text(vcx, encoding="utf-8")
    print("vcxproj patched")
else:
    print("vcxproj ok")

fil = FIL.read_text(encoding="utf-8")
if "inc\\ViewportInteraction>" not in fil and 'Include="inc\\ViewportInteraction"' not in fil:
    filter_defs = """    <Filter Include="inc\\ViewportInteraction">
      <UniqueIdentifier>{a1b2c3d4-e5f6-7890-abcd-ef1234567001}</UniqueIdentifier>
    </Filter>
    <Filter Include="inc\\ViewportInteraction\\Tools">
      <UniqueIdentifier>{a1b2c3d4-e5f6-7890-abcd-ef1234567002}</UniqueIdentifier>
    </Filter>
    <Filter Include="inc\\ViewportInteraction\\Overlays">
      <UniqueIdentifier>{a1b2c3d4-e5f6-7890-abcd-ef1234567003}</UniqueIdentifier>
    </Filter>
    <Filter Include="inc\\ViewportInteraction\\Policies">
      <UniqueIdentifier>{a1b2c3d4-e5f6-7890-abcd-ef1234567004}</UniqueIdentifier>
    </Filter>
    <Filter Include="src\\ViewportInteraction">
      <UniqueIdentifier>{a1b2c3d4-e5f6-7890-abcd-ef1234567005}</UniqueIdentifier>
    </Filter>
"""
    if 'Include="inc\\PickOperations"' in fil:
        fil = fil.replace(
            '    <Filter Include="inc\\PickOperations">',
            filter_defs + '    <Filter Include="inc\\PickOperations">',
        )
    else:
        fil = fil.replace("<ItemGroup>", "<ItemGroup>\n" + filter_defs, 1)
    mapping = {
        r"inc\ViewportInteraction\ViewportHit.h": r"inc\ViewportInteraction",
        r"inc\ViewportInteraction\IViewportPickEngine.h": r"inc\ViewportInteraction",
        r"inc\ViewportInteraction\OsgWidgetPickEngine.h": r"inc\ViewportInteraction",
        r"inc\ViewportInteraction\IPointerTool.h": r"inc\ViewportInteraction",
        r"inc\ViewportInteraction\IOverlayOp.h": r"inc\ViewportInteraction",
        r"inc\ViewportInteraction\IHitResolvePolicy.h": r"inc\ViewportInteraction",
        r"inc\ViewportInteraction\IInteractionSession.h": r"inc\ViewportInteraction",
        r"inc\ViewportInteraction\ViewportInteractionController.h": r"inc\ViewportInteraction",
        r"inc\ViewportInteraction\MeshPickSession.h": r"inc\ViewportInteraction",
        r"inc\ViewportInteraction\Policies\PassthroughHitPolicy.h": r"inc\ViewportInteraction\Policies",
        r"inc\ViewportInteraction\Policies\GizmoAxisHitPolicy.h": r"inc\ViewportInteraction\Policies",
        r"inc\ViewportInteraction\Policies\RobotObjectSelectPolicy.h": r"inc\ViewportInteraction\Policies",
        r"inc\ViewportInteraction\Tools\SelectionOperationToolAdapter.h": r"inc\ViewportInteraction\Tools",
        r"inc\ViewportInteraction\Overlays\SelectionOperationOverlayAdapter.h": r"inc\ViewportInteraction\Overlays",
        r"source\ViewportInteraction\OsgWidgetPickEngine.cpp": r"src\ViewportInteraction",
        r"source\ViewportInteraction\ViewportInteractionController.cpp": r"src\ViewportInteraction",
    }
    items = []
    for path, filt in mapping.items():
        tag = "ClCompile" if path.endswith(".cpp") else "ClInclude"
        items.append(f"    <{tag} Include=\"{path}\">\n      <Filter>{filt}</Filter>\n    </{tag}>")
    fil = fil.replace("</Project>", "\n".join(items) + "\n</Project>")
    FIL.write_text(fil, encoding="utf-8")
    print("filters patched")
else:
    print("filters ok")
