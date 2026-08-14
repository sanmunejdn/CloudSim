# -*- coding: utf-8 -*-
import re
from pathlib import Path

src = Path(r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\src\Host\CloudSimHost\CloudSimHost.vcxproj")
text = src.read_text(encoding="utf-8")

text = text.replace("{C1D2E3F4-A5B6-7890-CDEF-1234567890AB}", "{C1D2E3F4-A5B6-7890-CDEF-1234567890AC}")
text = text.replace("<RootNamespace>CloudSimHost</RootNamespace>", "<RootNamespace>CloudSimHostHeadless</RootNamespace>")
text = text.replace("$(CloudSimIntRoot)CloudSimHost\\", "$(CloudSimIntRoot)CloudSimHostHeadless\\")

text = re.sub(
    r"(<IntDir>\$\(CloudSimIntRoot\)CloudSimHostHeadless\\</IntDir>)",
    r"\1\n    <TargetName>CloudSimHostHeadless</TargetName>",
    text,
)

text = text.replace(
    "CLOUDSIM_HOST_LIB;CLOUDSIM_OSG_IN_HOST;WIDGET_LIB;AIBACKEND_LIB",
    "CLOUDSIM_HOST_HEADLESS_ONLY;CLOUDSIM_HOST_LIB;WIDGET_LIB;AIBACKEND_LIB",
)
text = text.replace(
    "CLOUDSIM_HOST_LIB;CLOUDSIM_OSG_IN_HOST;WIDGET_LIB;%(PreprocessorDefinitions)",
    "CLOUDSIM_HOST_HEADLESS_ONLY;CLOUDSIM_HOST_LIB;WIDGET_LIB;%(PreprocessorDefinitions)",
)

text = text.replace(
    "<AdditionalIncludeDirectories>inc;",
    "<AdditionalIncludeDirectories>inc\\headless_stub;inc;",
)

text = text.replace("OsgWidgetCore.lib;", "")

text = re.sub(
    r"\s*<ProjectReference Include=\"\.\.\\\.\.\\UI\\OsgWidgetCore\\OsgWidgetCore\.vcxproj\">.*?</ProjectReference>\s*",
    "\n",
    text,
    flags=re.S,
)

exclude_compile = {
    "GraphicsWindowQt1.cpp",
    "QWidgetViewer.cpp",
    "ObjectTransformOperation.cpp",
    "RobotTcpDragTeachOperation.cpp",
    "MeshSectionPlaneEditOperation.cpp",
    "PointPickOperation.cpp",
    "LabelingPickOperation.cpp",
    "PolylinePickOperation.cpp",
    "MeshEdgeFacePickOperation.cpp",
    "OsgRenderViewAdapter.cpp",
    "OsgWidgetSceneBridge.cpp",
    "OsgWidget.cpp",
    "OsgWidgetBackendLoadController.cpp",
    "OsgWidgetCaptureController.cpp",
    "OsgWidgetColorController.cpp",
    "OsgWidgetCameraFocusController.cpp",
    "OsgWidgetGizmoController.cpp",
    "OsgWidgetImportController.cpp",
    "OsgWidgetMeshSectionPlane.cpp",
    "OsgWidgetPickAnnotationController.cpp",
    "OsgWidgetTcpTeach.cpp",
    "OsgWidgetTransformHierarchyController.cpp",
}


def should_exclude(line: str) -> bool:
    if "<ClCompile Include=" not in line:
        return False
    return any(name in line for name in exclude_compile)


lines = []
skip_moc_block = False
for line in text.splitlines():
    if '<QtMoc Include="..\\..\\UI\\Widget\\inc\\OsgWidget.h">' in line:
        skip_moc_block = True
        continue
    if '<QtMoc Include="..\\..\\UI\\Widget\\inc\\QWidgetViewer.h">' in line:
        skip_moc_block = True
        continue
    if skip_moc_block:
        if "</QtMoc>" in line:
            skip_moc_block = False
        continue
    if should_exclude(line):
        continue
    lines.append(line)

out = "\n".join(lines)
if "OsgWidgetSceneBridge_Headless.cpp" not in out:
    out = out.replace(
        '    <ClCompile Include="source\\HostRenderViewFactory.cpp" />',
        '    <ClCompile Include="source\\HostRenderViewFactory.cpp" />\n'
        '    <ClCompile Include="source\\headless\\OsgWidgetSceneBridge_Headless.cpp" />',
    )

dst = Path(r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\src\Host\CloudSimHost\CloudSimHostHeadless.vcxproj")
dst.write_text(out + "\n", encoding="utf-8")
print("Wrote", dst)
