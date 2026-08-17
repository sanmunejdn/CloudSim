/// @file PointPickOperation.cpp
/// @brief PointPick 操作

#include "PointPickOperation.h"

#include "OsgScene.h"
#include "OsgWidget.h"
#include "PickTypes.h"

#include <QEvent>
#include <QMouseEvent>
#include <QPoint>
#include <cmath>

PointPickOperation::PointPickOperation(OsgWidget* owner) : SelectionOperation(owner) {}

bool PointPickOperation::canHandle(QObject* watched, QEvent* event) const
{
	(void)event;
	return m_owner && watched == m_owner->m_glWidget && m_owner->m_pointPickMode;
}

bool PointPickOperation::onMouseButtonPress(QMouseEvent* mouseEvent)
{
	if (mouseEvent->button() == Qt::LeftButton)
	{
		m_gesture.onLeftPress(mouseEvent->pos());
		return false;
	}
	if (mouseEvent->button() == Qt::MiddleButton)
	{
		return false;
	}
	return true;
}

bool PointPickOperation::onMouseButtonRelease(QMouseEvent* mouseEvent)
{
	if (mouseEvent->button() != Qt::LeftButton)
	{
		return false;
	}

	bool swallowRelease = false;
	if (!m_gesture.onLeftRelease(mouseEvent->pos(), &swallowRelease))
	{
		return false;
	}

	PickQuery query;
	query.screenX = mouseEvent->pos().x();
	query.screenY = mouseEvent->pos().y();
	query.kind = PickKind::PointCloud;
	query.hitRadiusPx = OsgScene::kPointPickHitRadiusPx;
	const PickResult pick = m_owner->queryPick(query);

	if (pick.hit)
	{
		m_owner->updatePointPickMarker(pick.worldPoint, true);
	}
	else
	{
		m_owner->clearPointPickMarker();
	}
	emit m_owner->pointPickFeedback(QStringLiteral("%1 | nearest: %2 px")
										.arg(pick.hit ? QStringLiteral("Hit") : QStringLiteral("Miss"))
										.arg(pick.hit || pick.screenDistancePx > 0.0
												 ? QString::number(pick.screenDistancePx, 'f', 1)
												 : QStringLiteral("N/A")));
	if (pick.hit)
	{
		m_owner->addPointAnnotation(pick.worldPoint);
		m_owner->requestRedraw();
	}
	m_gesture.restartClickHold(m_clickHoldTimer);
	return swallowRelease;
}

bool PointPickOperation::onMouseDoubleClick(QMouseEvent*)
{
	return true;
}

bool PointPickOperation::onWheel(QWheelEvent*)
{
	return false;
}

bool PointPickOperation::onMouseMove(QMouseEvent* mouseEvent)
{
	if (mouseEvent->buttons().testFlag(Qt::LeftButton))
	{
		m_gesture.onLeftMove(mouseEvent->pos());
		return false;
	}
	if (mouseEvent->buttons().testFlag(Qt::MiddleButton))
	{
		return false;
	}
	if (ViewportGestureRecognizer::shouldThrottleHover(m_owner->m_feedbackTimer))
	{
		return true;
	}

	const QPoint pos = mouseEvent->pos();
	if ((pos - m_lastHoverPickPos).manhattanLength() < OsgScene::kPickHoverMinMovePx)
	{
		return true;
	}
	m_lastHoverPickPos = pos;

	const bool inClickHold = m_gesture.inClickHold(m_clickHoldTimer);

	PickQuery query;
	query.screenX = pos.x();
	query.screenY = pos.y();
	query.kind = PickKind::PointCloud;
	query.hitRadiusPx = OsgScene::kPointPickHitRadiusPx;
	query.hoverPick = true;
	const PickResult pick = m_owner->queryPick(query);

	const bool hadPreview = m_preview.valid && m_preview.result.hit &&
							m_preview.result.screenDistancePx <= OsgScene::kPointPickPreviewRadiusPx;
	const bool showPreview = pick.hit && pick.screenDistancePx <= OsgScene::kPointPickPreviewRadiusPx;
	bool needsRedraw = false;
	if (showPreview)
	{
		const bool samePoint = hadPreview && (m_preview.result.worldPoint - pick.worldPoint).length2() < 1e-3f;
		m_preview.valid = true;
		m_preview.result = pick;
		if (!samePoint)
		{
			m_owner->updatePointPickMarker(pick.worldPoint, true);
			needsRedraw = true;
		}
	}
	else if (!inClickHold)
	{
		if (hadPreview || m_preview.valid)
		{
			m_preview.valid = false;
			m_owner->clearPointPickMarker();
			needsRedraw = true;
		}
	}

	const bool feedbackChanged =
		(pick.hit != m_lastFeedbackHit) || (pick.hit && std::abs(pick.screenDistancePx - m_lastFeedbackDistPx) >= 0.5);
	if (feedbackChanged)
	{
		m_lastFeedbackHit = pick.hit;
		m_lastFeedbackDistPx = pick.screenDistancePx;
		emit m_owner->pointPickFeedback(QStringLiteral("%1 | nearest: %2 px | points: %3")
											.arg(pick.hit ? QStringLiteral("Hit") : QStringLiteral("Miss"))
											.arg(pick.screenDistancePx > 0.0
													 ? QString::number(pick.screenDistancePx, 'f', 1)
													 : QStringLiteral("N/A"))
											.arg(m_owner->m_pickablePointsLocal.size()));
	}
	m_owner->m_feedbackTimer.restart();
	if (needsRedraw)
	{
		m_owner->requestRedraw();
	}
	return true;
}
