/// @file ViewportGestureRecognizer.cpp
/// @brief ViewportGestureRecognizer 实现

#include "ViewportGestureRecognizer.h"

#include "OsgScene.h"

void ViewportGestureRecognizer::onLeftPress(const QPoint& pos)
{
	m_leftPressed = true;
	m_pressPos = pos;
	m_dragMoved = false;
	m_wasClick = false;
}

void ViewportGestureRecognizer::onLeftMove(const QPoint& pos)
{
	if (!m_leftPressed)
	{
		return;
	}
	const QPoint delta = pos - m_pressPos;
	if (delta.manhattanLength() > OsgScene::kPickDragMoveThresholdPx)
	{
		m_dragMoved = true;
	}
}

bool ViewportGestureRecognizer::onLeftRelease(const QPoint& pos, bool* swallowReleaseOut)
{
	if (swallowReleaseOut)
	{
		*swallowReleaseOut = false;
	}
	if (!m_leftPressed)
	{
		return false;
	}
	const QPoint delta = pos - m_pressPos;
	m_wasClick = !m_dragMoved && delta.manhattanLength() <= OsgScene::kPickClickMoveThresholdPx;
	m_leftPressed = false;
	m_dragMoved = false;
	if (swallowReleaseOut)
	{
		*swallowReleaseOut = m_wasClick;
	}
	return m_wasClick;
}

bool ViewportGestureRecognizer::inClickHold(const QElapsedTimer& holdTimer) const
{
	return holdTimer.isValid() && holdTimer.elapsed() < OsgScene::kPickClickHoldMs;
}

void ViewportGestureRecognizer::restartClickHold(QElapsedTimer& holdTimer) const
{
	holdTimer.restart();
}

bool ViewportGestureRecognizer::shouldThrottleHover(const QElapsedTimer& feedbackTimer)
{
	return shouldThrottleHover(feedbackTimer, OsgScene::kPickHoverThrottleMs);
}

bool ViewportGestureRecognizer::shouldThrottleHover(const QElapsedTimer& feedbackTimer, int throttleMs)
{
	return feedbackTimer.isValid() && feedbackTimer.elapsed() < throttleMs;
}
