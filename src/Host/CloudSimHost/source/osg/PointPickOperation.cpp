/// @file PointPickOperation.cpp
/// @brief PointPick 操作

#include "PointPickOperation.h"

#include "OsgScene.h"
#include "PickTypes.h"
#include "ViewportInteraction/IViewportPickEngine.h"

#include <QEvent>
#include <QMouseEvent>
#include <QPoint>
#include <cmath>

PointPickOperation::PointPickOperation(IViewportInteractionHost* host) : SelectionOperation(host) {}

bool PointPickOperation::canHandle(QObject* watched, QEvent* event) const
{
	(void)event;
	const IViewportInteractionHost* h = host();
	return h && watched == h->viewportGlWidget() && h->pointPickMode();
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

	IViewportInteractionHost* h = host();
	if (!h)
	{
		return swallowRelease;
	}

	PickQuery query;
	query.screenX = mouseEvent->pos().x();
	query.screenY = mouseEvent->pos().y();
	query.kind = PickKind::PointCloud;
	query.hitRadiusPx = OsgScene::kPointPickHitRadiusPx;
	const PickResult pick = (h->pickEngine() != nullptr) ? h->pickEngine()->queryPick(query) : h->queryPick(query);

	if (pick.hit)
	{
		h->updatePointPickMarker(pick.worldPoint, true);
	}
	else
	{
		h->clearPointPickMarker();
	}
	h->emitPointPickFeedback(QStringLiteral("%1 | nearest: %2 px")
							 .arg(pick.hit ? QStringLiteral("Hit") : QStringLiteral("Miss"))
							 .arg(pick.hit || pick.screenDistancePx > 0.0 ? QString::number(pick.screenDistancePx, 'f', 1)
																			: QStringLiteral("N/A")));
	if (pick.hit)
	{
		h->addPointAnnotation(pick.worldPoint);
		h->requestRedraw();
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
	IViewportInteractionHost* h = host();
	if (!h)
	{
		return false;
	}
	ViewportInteractionPointerState pointer = h->interactionPointerState();
	if (ViewportGestureRecognizer::shouldThrottleHover(pointer.feedbackTimer))
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
	const PickResult pick = (h->pickEngine() != nullptr) ? h->pickEngine()->queryPick(query) : h->queryPick(query);

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
			h->updatePointPickMarker(pick.worldPoint, true);
			needsRedraw = true;
		}
	}
	else if (!inClickHold)
	{
		if (hadPreview || m_preview.valid)
		{
			m_preview.valid = false;
			h->clearPointPickMarker();
			needsRedraw = true;
		}
	}

	const bool feedbackChanged =
		(pick.hit != m_lastFeedbackHit) || (pick.hit && std::abs(pick.screenDistancePx - m_lastFeedbackDistPx) >= 0.5);
	if (feedbackChanged)
	{
		m_lastFeedbackHit = pick.hit;
		m_lastFeedbackDistPx = pick.screenDistancePx;
		h->emitPointPickFeedback(QStringLiteral("%1 | nearest: %2 px | points: %3")
								 .arg(pick.hit ? QStringLiteral("Hit") : QStringLiteral("Miss"))
								 .arg(pick.screenDistancePx > 0.0 ? QString::number(pick.screenDistancePx, 'f', 1)
																  : QStringLiteral("N/A"))
								 .arg(h->pickablePointCount()));
	}
	pointer.feedbackTimer.restart();
	if (needsRedraw)
	{
		h->requestRedraw();
	}
	return true;
}
