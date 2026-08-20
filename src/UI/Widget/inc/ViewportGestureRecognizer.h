#ifndef WIDGET_VIEWPORTGESTURERECOGNIZER_H
#define WIDGET_VIEWPORTGESTURERECOGNIZER_H

/// @file ViewportGestureRecognizer.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口手势：统一点/线/面拾取的 click/drag/release 判定

#include "../../OsgWidgetCore/inc/OsgScene.h"

#include <QElapsedTimer>
#include <QPoint>

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
	static bool shouldThrottleHover(const QElapsedTimer& feedbackTimer, int throttleMs);

private:
	bool m_leftPressed = false;
	QPoint m_pressPos;
	bool m_dragMoved = false;
	bool m_wasClick = false;
};

#endif // WIDGET_VIEWPORTGESTURERECOGNIZER_H
