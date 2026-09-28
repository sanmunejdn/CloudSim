/// @file OsgWidgetPickEngine.cpp
/// @brief OsgWidgetPickEngine 实现

#include "ViewportInteraction/OsgWidgetPickEngine.h"

#include "OsgWidget.h"

OsgWidgetPickEngine::OsgWidgetPickEngine(OsgWidget& widget) : m_widget(widget) {}

PickResult OsgWidgetPickEngine::queryPick(const PickQuery& query)
{
	return m_widget.queryPick(query);
}

std::string OsgWidgetPickEngine::pickBackendIdAtScreenPos(double screenX, double screenY) const
{
	return m_widget.pickBackendIdAtScreenPos(screenX, screenY);
}

void OsgWidgetPickEngine::showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld)
{
	m_widget.showMeshFaceHighlight(vertsWorld);
}

void OsgWidgetPickEngine::showMeshEdgeHighlight(const std::vector<osg::Vec3f>& polylineWorld)
{
	m_widget.showMeshEdgeHighlight(polylineWorld);
}

void OsgWidgetPickEngine::showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld)
{
	m_widget.showMeshEdgeHighlight(aWorld, bWorld);
}

void OsgWidgetPickEngine::hideMeshElementHighlight()
{
	m_widget.hideMeshElementHighlight();
}

void OsgWidgetPickEngine::requestRedraw()
{
	m_widget.requestRedraw();
}

const std::string& OsgWidgetPickEngine::activeBackendId() const
{
	return m_widget.activeBackendId();
}

bool OsgWidgetPickEngine::crossObjectMeshPick() const
{
	return m_widget.crossObjectMeshPick();
}

bool OsgWidgetPickEngine::meshFacePickMode() const
{
	return m_widget.meshFacePickMode();
}

bool OsgWidgetPickEngine::meshLinePickMode() const
{
	return m_widget.meshLinePickMode();
}

bool OsgWidgetPickEngine::pointPickMode() const
{
	return m_widget.pointPickMode();
}

bool OsgWidgetPickEngine::objectSelectionMode() const
{
	return m_widget.objectSelectionMode();
}
