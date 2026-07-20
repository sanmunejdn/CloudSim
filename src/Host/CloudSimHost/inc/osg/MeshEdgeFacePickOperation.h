#ifndef CLOUDSIMHOST_MESHEDGEFACEPICKOPERATION_H
#define CLOUDSIMHOST_MESHEDGEFACEPICKOPERATION_H

/// @file MeshEdgeFacePickOperation.h
/// @brief 网格线面拾取模式：悬停高亮边或面，点击确认，支持中键辅助移动视图。

#include "SelectionOperation.h"

#include <QElapsedTimer>
#include <QPoint>

/// 网格线面拾取模式：悬停高亮边或面，点击确认，支持中键辅助移动视图。
class MeshEdgeFacePickOperation final : public SelectionOperation
{
public:
	explicit MeshEdgeFacePickOperation(OsgWidget* owner);

private:
	bool m_leftPressed = false;
	QPoint m_pressPos;
	bool m_dragMoved = false;

	// After a click we briefly "hold" the last highlight so small
	// camera/mouse jitter does not immediately turn it into a miss.
	QElapsedTimer m_clickHoldTimer;

protected:
	bool canHandle(QObject* watched, QEvent* event) const override;
	bool onMouseMove(QMouseEvent* e) override;
	bool onMouseButtonPress(QMouseEvent* e) override;
	bool onMouseButtonRelease(QMouseEvent* e) override;
};

#endif // CLOUDSIMHOST_MESHEDGEFACEPICKOPERATION_H
