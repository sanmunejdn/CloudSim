# one-off: extract picking from OsgWidget.cpp -> OsgScenePicking.cpp
import re

path = r"D:\Project\VSprogram\CGAL5.5.2\CloudSim\Widget\source\OsgWidget.cpp"
with open(path, "r", encoding="utf-8") as f:
    lines = f.readlines()

parts = []
parts.append("".join(lines[1311:1377]))  # pickAndActivate
parts.append("".join(lines[1618:2592]))  # cache .. pickMeshEdge
parts.append("".join(lines[2593:2697]))  # show/hide mesh highlight
parts.append("".join(lines[2698:2838]))  # kd tree

text = "".join(parts)

text = text.replace(
    "bool OsgWidget::pickAndActivateBackendAtScreenPos(const QPoint& mousePos)",
    "bool OsgScene::pickAndActivateBackendAtScreenPos(double mouseX, double mouseY)",
)
text = text.replace(
    "OsgWidgetTransformHierarchyController::cacheSelectionGizmoPose(*this);",
    "cacheSelectionGizmoPose();",
)
text = re.sub(r"\s*refreshAnnotationTexts\(\);\s*\n", "\n", text)
text = re.sub(r"\s*setSelectionActive\(true\);\s*\n", "\n", text)

replacements = [
    ("void OsgWidget::cachePickablePointsFromNode", "void OsgScene::cachePickablePointsFromNode"),
    (
        "bool OsgWidget::pickPointAtScreenPos(const QPoint& mousePos,",
        "bool OsgScene::pickPointAtScreenPos(double mouseX, double mouseY,",
    ),
    (
        "bool OsgWidget::pickNearestPointAtScreenPos(const QPoint& mousePos,",
        "bool OsgScene::pickNearestPointAtScreenPos(double mouseX, double mouseY,",
    ),
    (
        "bool OsgWidget::pickPointByRayIntersection(const QPoint& mousePos,",
        "bool OsgScene::pickPointByRayIntersection(double mouseX, double mouseY,",
    ),
    (
        "bool OsgWidget::pickMeshFaceByRayIntersection(const QPoint& mousePos,",
        "bool OsgScene::pickMeshFaceByRayIntersection(double mouseX, double mouseY,",
    ),
    (
        "bool OsgWidget::pickMeshEdgeByRayIntersection(const QPoint& mousePos,",
        "bool OsgScene::pickMeshEdgeByRayIntersection(double mouseX, double mouseY,",
    ),
    ("void OsgWidget::showMeshFaceHighlight(const std::vector", "void OsgScene::showMeshFaceHighlight(const std::vector"),
    (
        "void OsgWidget::showMeshFaceHighlight(const osg::Vec3f& aWorld",
        "void OsgScene::showMeshFaceHighlight(const osg::Vec3f& aWorld",
    ),
    (
        "void OsgWidget::showMeshEdgeHighlight(const osg::Vec3f& aWorld",
        "void OsgScene::showMeshEdgeHighlight(const osg::Vec3f& aWorld",
    ),
    ("void OsgWidget::hideMeshElementHighlight()", "void OsgScene::hideMeshElementHighlight()"),
    ("void OsgWidget::rebuildPointKdTree()", "void OsgScene::rebuildPointKdTree()"),
    ("int OsgWidget::buildKdNode", "int OsgScene::buildKdNode"),
    ("int OsgWidget::nearestPointByKdTree", "int OsgScene::nearestPointByKdTree"),
    ("void OsgWidget::nearestCandidatesByKdTree", "void OsgScene::nearestCandidatesByKdTree"),
]
for a, b in replacements:
    text = text.replace(a, b)

text = text.replace("m_glWidget->height()", "viewportHeight()")
text = text.replace("m_glWidget->width()", "viewportWidth()")
text = text.replace("!m_glWidget ||", "!")  # remove gl widget check - use viewport
# Fix double space
text = text.replace("if (!m_viewer.valid() || !m_viewer->getCamera() || !m_root.valid())", "if (!m_viewer.valid() || !m_viewer->getCamera() || !m_root.valid())")

# pickPointByRay and others: first line x,y
text = re.sub(
    r"if \(!m_viewer\.valid\(\) \|\| !m_viewer->getCamera\(\) \|\| !m_root\.valid\(\)\)",
    "if (!m_viewer.valid() || !m_viewer->getCamera() || !m_root.valid())",
    text,
)

# mesh pick had !m_glWidget - already removed

# QPointF loop in pickNearest - remove "const QPointF target" block uses - already replaced dx/dy with mouseX/Y in for loop at 1820
text = text.replace("const QPointF target(mousePos);", "")
text = text.replace("target.x()", "mouseX")
text = text.replace("target.y()", "mouseY")

# pickMeshEdge: replace toScreen lambda and QPointF q
old_edge = """	const osg::Matrixd mvp = m_viewer->getCamera()->getViewMatrix() * m_viewer->getCamera()->getProjectionMatrix();
	const auto toScreen = [&](const osg::Vec3f& world) -> QPointF {
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		const double sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		const double sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
		return QPointF(sx, sy);
	};
	const QPointF q(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()));"""

new_edge = """	const osg::Matrixd mvp = m_viewer->getCamera()->getViewMatrix() * m_viewer->getCamera()->getProjectionMatrix();
	const auto toScreen = [&](const osg::Vec3f& world, double& sx, double& sy) {
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
	};
	const double qx = mouseX;
	const double qy = mouseY;"""

if old_edge in text:
    text = text.replace(old_edge, new_edge)
else:
    print("WARN: edge block not found, manual fix needed")

text = text.replace("const QPointF s0 = toScreen(ea);", "double s0x=0,s0y=0,s1x=0,s1y=0; toScreen(ea, s0x, s0y); toScreen(eb, s1x, s1y); const double s0sx=s0x")
# This is getting messy - manual fix pickMeshEdge section

out = r"D:\Project\VSprogram\CGAL5.5.2\CloudSim\OsgWidgetCore\source\OsgScenePicking.cpp"
header = r"""#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include "OsgScene.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>

#include <osg/GL>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Matrixd>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/PrimitiveSet>
#include <osg/Transform>
#include <osg/Vec3d>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/Viewer>

"""

with open(out, "w", encoding="utf-8") as f:
    f.write(header)
    f.write(text)

print("done", len(text))
