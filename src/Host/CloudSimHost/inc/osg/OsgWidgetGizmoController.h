#ifndef CLOUDSIMHOST_OSGWIDGETGIZMOCONTROLLER_H
#define CLOUDSIMHOST_OSGWIDGETGIZMOCONTROLLER_H

/// @file OsgWidgetGizmoController.h
/// @brief 指南针/轴环 gizmo 的节点创建、显示/隐藏、高亮、缩放与拾取（从 OsgWidget 拆出）。

#include <QPoint>

namespace osg
{
class Node;
}

class OsgWidget;

/// 指南针/轴环 gizmo 的节点创建、显示/隐藏、高亮、缩放与拾取（从 OsgWidget 拆出）。
class OsgWidgetGizmoController
{
public:
	// Axis ids: 0=None, 1=X, 2=Y, 3=Z (must match OsgWidget::DragAxis order).
	static constexpr int kAxisNone = 0;
	static constexpr int kAxisX = 1;
	static constexpr int kAxisY = 2;
	static constexpr int kAxisZ = 3;

	static osg::Node* createCompassNode(OsgWidget& self);
	static int pickAxisAtScreenPos(const OsgWidget& self, const QPoint& mousePos, bool preferRing);

	static void updateCompassHighlight(OsgWidget& self, int axis);
	static void updateCompassScale(OsgWidget& self);

	static void refreshCompassDrawVisibility(OsgWidget& self);
	static void attachCompassGraphics(OsgWidget& self);
	static void detachCompassGraphics(OsgWidget& self);
};

#endif // CLOUDSIMHOST_OSGWIDGETGIZMOCONTROLLER_H
