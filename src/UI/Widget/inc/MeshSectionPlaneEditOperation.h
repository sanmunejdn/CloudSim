#pragma once

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
