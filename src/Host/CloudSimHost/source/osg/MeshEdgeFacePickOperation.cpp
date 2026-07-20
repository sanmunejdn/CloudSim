/// @file MeshEdgeFacePickOperation.cpp
/// @brief MeshEdgeFacePickOperation 实现

#include "MeshEdgeFacePickOperation.h"

#include "OsgWidget.h"

#include <QEvent>
#include <QMouseEvent>
#include <vector>

MeshEdgeFacePickOperation::MeshEdgeFacePickOperation(OsgWidget* owner) : SelectionOperation(owner) {}

bool MeshEdgeFacePickOperation::canHandle(QObject* watched, QEvent* event) const
{
	(void)event;
	return m_owner && watched == m_owner->m_glWidget && (m_owner->m_meshLinePickMode || m_owner->m_meshFacePickMode);
}

bool MeshEdgeFacePickOperation::onMouseButtonPress(QMouseEvent* mouseEvent)
{
	if (mouseEvent->button() != Qt::LeftButton)
	{
		// Allow middle mouse to move the view during pick modes.
		if (mouseEvent->button() == Qt::MiddleButton)
		{
			return false;
		}

		// Consume other buttons so osgGA never sees asymmetric press/release while edit mode is active.
		return true;
	}

	// Allow left-drag to rotate the view (TrackballManipulator).
	m_leftPressed = true;
	m_pressPos = mouseEvent->pos();
	m_dragMoved = false;
	return false;
}

bool MeshEdgeFacePickOperation::onMouseMove(QMouseEvent* mouseEvent)
{
	// While left button is pressed, forward events for navigation.
	if (mouseEvent->buttons().testFlag(Qt::LeftButton))
	{
		const QPoint delta = mouseEvent->pos() - m_pressPos;
		// If movement is significant, treat as navigation drag.
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

	// Hover picking when not dragging (left button not pressed).
	// Throttle preview computations.
	if (m_owner->m_feedbackTimer.isValid() && m_owner->m_feedbackTimer.elapsed() < 50)
	{
		return true;
	}

	constexpr int kClickHoldMs = 150;
	const bool inClickHold = m_clickHoldTimer.isValid() && m_clickHoldTimer.elapsed() < kClickHoldMs;

	if (m_owner->m_meshFacePickMode)
	{
		osg::Vec3f p, a, b, c, n;
		std::vector<osg::Vec3f> merged;
		const bool hit = m_owner->pickMeshFaceByRayIntersection(mouseEvent->pos(), p, a, b, c, n, &merged);
		if (hit)
		{
			m_owner->showMeshFaceHighlight(merged);
		}
		else if (!inClickHold)
		{
			// If we just clicked, keep the last highlight briefly to avoid flicker.
			m_owner->hideMeshElementHighlight();
		}
	}
	else if (m_owner->m_meshLinePickMode)
	{
		osg::Vec3f edgeA, edgeB, p;
		const bool hit = m_owner->pickMeshEdgeByRayIntersection(mouseEvent->pos(), p, edgeA, edgeB);
		if (hit)
		{
			m_owner->showMeshEdgeHighlight(edgeA, edgeB);
		}
		else if (!inClickHold)
		{
			m_owner->hideMeshElementHighlight();
		}
	}

	m_owner->m_feedbackTimer.restart();
	m_owner->requestRedraw();
	return true; // consume hover moves to keep UI smooth
}

bool MeshEdgeFacePickOperation::onMouseButtonRelease(QMouseEvent* mouseEvent)
{
	if (mouseEvent->button() == Qt::LeftButton && m_leftPressed)
	{
		const QPoint delta = mouseEvent->pos() - m_pressPos;
		// On Windows high-DPI + input smoothing, a "click" can still move
		// more than a few pixels between press/release.
		constexpr int kClickMoveThresholdPx = 25;
		const bool clicked = !m_dragMoved && delta.manhattanLength() <= kClickMoveThresholdPx;

		if (clicked)
		{
			if (m_owner->m_meshFacePickMode)
			{
				osg::Vec3f p, a, b, c, n;
				std::vector<osg::Vec3f> merged;
				const bool hit = m_owner->pickMeshFaceByRayIntersection(mouseEvent->pos(), p, a, b, c, n, &merged);
				// If miss immediately after click (tiny jitter), keep previous hover highlight.
				if (hit)
					m_owner->showMeshFaceHighlight(merged);
			}
			else if (m_owner->m_meshLinePickMode)
			{
				osg::Vec3f edgeA, edgeB, p;
				const bool hit = m_owner->pickMeshEdgeByRayIntersection(mouseEvent->pos(), p, edgeA, edgeB);
				if (hit)
					m_owner->showMeshEdgeHighlight(edgeA, edgeB);
			}

			m_clickHoldTimer.restart();
			m_owner->requestRedraw();
		}

		m_leftPressed = false;
		m_dragMoved = false;
		// Important: if it's a "click" (not a drag), swallow release event to prevent
		// TrackballManipulator from applying a small rotation that immediately turns
		// hover/mode highlights into a miss.
		return clicked ? true : false; // drag => allow navigation
	}
	return false;
}
