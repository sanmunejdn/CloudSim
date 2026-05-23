#include "PointPickOperation.h"

#include <QEvent>
#include <QMouseEvent>

#include "OsgWidget.h"

PointPickOperation::PointPickOperation(OsgWidget* owner)
	: SelectionOperation(owner)
{
}

bool PointPickOperation::canHandle(QObject* watched, QEvent* event) const
{
	(void)event;
	if (!m_owner || watched != m_owner->m_glWidget || !m_owner->m_pointPickMode)
	{
		return false;
	}
	return true;
}

bool PointPickOperation::onMouseButtonPress(QMouseEvent* mouseEvent)
{
	if (mouseEvent->button() == Qt::LeftButton)
	{
		// Allow left-drag to rotate the view (TrackballManipulator).
		m_leftPressed = true;
		m_pressPos = mouseEvent->pos();
		m_dragMoved = false;
		return false;
	}

	// Allow middle mouse to move the view during pick modes.
	if (mouseEvent->button() == Qt::MiddleButton)
	{
		return false;
	}

	// Non-left/non-middle buttons: still consume to avoid unexpected asymmetric press/release.
	return true;
}

bool PointPickOperation::onMouseButtonRelease(QMouseEvent* mouseEvent)
{
	// Release is used for click selection; return false to let navigation handle release.
	if (mouseEvent->button() == Qt::LeftButton && m_leftPressed)
	{
		const QPoint delta = mouseEvent->pos() - m_pressPos;
		constexpr int kClickMoveThresholdPx = 10;
		const bool clicked = !m_dragMoved && delta.manhattanLength() <= kClickMoveThresholdPx;
		if (clicked)
		{
			osg::Vec3f worldPoint;
			double nearestDistance = 0.0;
			const bool hasNearest = m_owner->pickNearestPointAtScreenPos(mouseEvent->pos(), worldPoint, nearestDistance, false);
			const bool hit = hasNearest && nearestDistance <= 32.0;
			if (hasNearest)
			{
				m_owner->updatePointPickMarker(worldPoint, hit);
			}
			else
			{
				m_owner->clearPointPickMarker();
			}
			emit m_owner->pointPickFeedback(QStringLiteral("%1 | nearest: %2 px")
				.arg(hit ? QStringLiteral("Hit") : QStringLiteral("Miss"))
				.arg(hasNearest ? QString::number(nearestDistance, 'f', 1) : QStringLiteral("N/A")));
			if (hit)
			{
				m_owner->addPointAnnotation(worldPoint);
				m_owner->requestRedraw();
			}
		}

		m_leftPressed = false;
		m_dragMoved = false;
		return false;
	}
	return false;
}

bool PointPickOperation::onMouseDoubleClick(QMouseEvent*)
{
	// Preserve old behavior: consume dbl click.
	return true;
}

bool PointPickOperation::onWheel(QWheelEvent*)
{
	return false;
}

bool PointPickOperation::onMouseMove(QMouseEvent* mouseEvent)
{
	// While left button is pressed, forward events for navigation.
	if (mouseEvent->buttons().testFlag(Qt::LeftButton))
	{
		const QPoint delta = mouseEvent->pos() - m_pressPos;
		constexpr int kDragMoveThresholdPx = 10;
		if (delta.manhattanLength() > kDragMoveThresholdPx)
		{
			m_dragMoved = true;
		}
		return false;
	}

	// Forward middle mouse navigation to OSG.
	if (mouseEvent->buttons().testFlag(Qt::MiddleButton))
	{
		return false;
	}

	if (m_owner->m_feedbackTimer.isValid() && m_owner->m_feedbackTimer.elapsed() < 50)
	{
		return true;
	}

	osg::Vec3f worldPoint;
	double nearestDistance = 0.0;
	const bool hasNearest = m_owner->pickNearestPointAtScreenPos(mouseEvent->pos(), worldPoint, nearestDistance, true);
	const bool hit = hasNearest && nearestDistance <= 32.0;
	if (hasNearest)
	{
		m_owner->updatePointPickMarker(worldPoint, hit);
	}
	else
	{
		m_owner->clearPointPickMarker();
	}
	emit m_owner->pointPickFeedback(QStringLiteral("%1 | nearest: %2 px | preview/full: %3/%4")
		.arg(hit ? QStringLiteral("Hit") : QStringLiteral("Miss"))
		.arg(hasNearest ? QString::number(nearestDistance, 'f', 1) : QStringLiteral("N/A"))
		.arg(m_owner->m_pickablePointsPreviewLocal.size())
		.arg(m_owner->m_pickablePointsLocal.size()));
	m_owner->m_feedbackTimer.restart();
	m_owner->requestRedraw();
	return true; // hover preview
}

