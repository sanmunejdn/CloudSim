#ifndef CLOUDSIMHOST_ROBOTTCPDRAGTEACHOPERATION_H
#define CLOUDSIMHOST_ROBOTTCPDRAGTEACHOPERATION_H

/// @file RobotTcpDragTeachOperation.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief TCP 示教罗盘交互：拾取轴、平移/旋转拖拽，经 \c OsgWidget 发示教位姿信号

#include "SelectionOperation.h"

/// TCP 示教罗盘交互：拾取轴、平移/旋转拖拽，经 \c OsgWidget 发示教位姿信号
class RobotTcpDragTeachOperation : public SelectionOperation
{
public:
	/// @param owner 三维视图，读写 TCP 示教成员
	explicit RobotTcpDragTeachOperation(IViewportInteractionHost* host);

	bool handleEvent(QObject* watched, QEvent* event) override;

private:
	int m_lastEmittedHoverAxis = -1;
	bool m_lastEmittedHoverRing = false;
	bool m_sessionModified = false;
};

#endif // CLOUDSIMHOST_ROBOTTCPDRAGTEACHOPERATION_H
