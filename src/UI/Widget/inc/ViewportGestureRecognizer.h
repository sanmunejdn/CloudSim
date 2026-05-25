#pragma once

#include <QElapsedTimer>
#include <QPoint>

#include "../../OsgWidgetCore/inc/OsgScene.h"

/// 视口手势：统一点/线/面拾取的 click/drag/release 判定
class ViewportGestureRecognizer
{
public:
	void onLeftPress(const QPoint& pos);
	void onLeftMove(const QPoint& pos);
	bool onLeftRelease(const QPoint& pos, bool* swallowReleaseOut);

	bool isLeftPressed() const { return m_leftPressed; }
	bool isDragging() const { return m_dragMoved; }
	bool wasClick() const { return m_wasClick; }

	bool inClickHold(const QElapsedTimer& holdTimer) const;
	void restartClickHold(QElapsedTimer& holdTimer) const;

	static bool shouldThrottleHover(const QElapsedTimer& feedbackTimer);

private:
	bool m_leftPressed = false;
	QPoint m_pressPos;
	bool m_dragMoved = false;
	bool m_wasClick = false;
};
