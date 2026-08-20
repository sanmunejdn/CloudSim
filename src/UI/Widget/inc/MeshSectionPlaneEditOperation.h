#ifndef WIDGET_MESHSECTIONPLANEEDITOPERATION_H
#define WIDGET_MESHSECTIONPLANEEDITOPERATION_H

/// @file MeshSectionPlaneEditOperation.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Mesh 截面罗盘：平移/旋转截面平面

#include "SelectionOperation.h"

/// Mesh 截面罗盘：平移/旋转截面平面
class MeshSectionPlaneEditOperation : public SelectionOperation
{
public:
	explicit MeshSectionPlaneEditOperation(class OsgWidget* owner);

	bool handleEvent(QObject* watched, QEvent* event) override;

private:
	int m_lastEmittedHoverAxis = -1;
	bool m_lastEmittedHoverRing = false;
};

#endif // WIDGET_MESHSECTIONPLANEEDITOPERATION_H
