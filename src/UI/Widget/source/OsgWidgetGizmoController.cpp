/// @file OsgWidgetGizmoController.cpp
/// @brief OsgWidgetGizmo 控制

#include "OsgWidgetGizmoController.h"

#include "OsgWidget.h"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <osgViewer/Viewer>

osg::Node* OsgWidgetGizmoController::createCompassNode(OsgWidget& self)
{
	return self.OsgScene::createCompassNode();
}

void OsgWidgetGizmoController::attachCompassGraphics(OsgWidget& self)
{
	self.OsgScene::attachCompassGraphics();
}

void OsgWidgetGizmoController::detachCompassGraphics(OsgWidget& self)
{
	self.OsgScene::detachCompassGraphics();
}

void OsgWidgetGizmoController::refreshCompassDrawVisibility(OsgWidget& self)
{
	self.OsgScene::refreshCompassDrawVisibility();
}

void OsgWidgetGizmoController::updateCompassHighlight(OsgWidget& self, int axis)
{
	self.OsgScene::updateCompassHighlight(axis);
}

void OsgWidgetGizmoController::updateCompassScale(OsgWidget& self)
{
	self.OsgScene::updateCompassScale();
}

int OsgWidgetGizmoController::pickAxisAtScreenPos(const OsgWidget& self, const QPoint& mousePos, bool preferRing)
{
	return self.OsgScene::pickAxisAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
											  preferRing);
}
