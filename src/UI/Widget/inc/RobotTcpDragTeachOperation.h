#pragma once

#include "SelectionOperation.h"

/// TCP 示教罗盘交互：拾取轴、平移/旋转拖拽，经 \c OsgWidget 发示教位姿信号
class RobotTcpDragTeachOperation : public SelectionOperation
{
public:
	/// @param owner 三维视图，读写 TCP 示教成员
	explicit RobotTcpDragTeachOperation(OsgWidget* owner);

	bool handleEvent(QObject* watched, QEvent* event) override;

private:
	int m_lastEmittedHoverAxis = -1;
	bool m_lastEmittedHoverRing = false;
	bool m_sessionModified = false;
};
