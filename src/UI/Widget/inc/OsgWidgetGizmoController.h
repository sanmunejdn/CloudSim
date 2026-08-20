#ifndef WIDGET_OSGWIDGETGIZMOCONTROLLER_H
#define WIDGET_OSGWIDGETGIZMOCONTROLLER_H

/// @file OsgWidgetGizmoController.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 指南针/轴环 gizmo 的节点创建、显示/隐藏、高亮、缩放与拾取（自 OsgWidget 拆出

#include <QPoint>

namespace osg
{
class Node;
}

class OsgWidget;

/// 指南针/轴环 gizmo 的节点创建、显示/隐藏、高亮、缩放与拾取（自 OsgWidget 拆出
class OsgWidgetGizmoController
{
public:
	/// 轴编号 0=None,1=X,2=Y,3=Z（同 OsgWidget::DragAxis）
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

#endif // WIDGET_OSGWIDGETGIZMOCONTROLLER_H
