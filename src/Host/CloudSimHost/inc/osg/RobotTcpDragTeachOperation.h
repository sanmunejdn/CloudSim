#pragma once

#include "SelectionOperation.h"

class RobotTcpDragTeachOperation : public SelectionOperation
{
public:
	explicit RobotTcpDragTeachOperation(OsgWidget* owner);

	bool handleEvent(QObject* watched, QEvent* event) override;

private:
	int m_lastEmittedHoverAxis = -1;
	bool m_lastEmittedHoverRing = false;
	bool m_sessionModified = false;
};
