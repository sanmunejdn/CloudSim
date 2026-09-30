/// @file PolylinePickOperation.cpp
/// @brief PolylinePick 操作

#include "PolylinePickOperation.h"

#include <QEvent>
#include <QKeyEvent>
#include <QMouseEvent>

namespace
{
constexpr int kMinPolygonVertices = 3;

} // namespace

PolylinePickOperation::PolylinePickOperation(IViewportInteractionHost* host) : SelectionOperation(host) {}

void PolylinePickOperation::refreshOverlay() const
{
	IViewportInteractionHost* h = host();
	if (!h)
	{
		return;
	}
	h->updatePolylinePickOverlay(m_vertices, m_hasCursor ? &m_cursorPos : nullptr);
}

bool PolylinePickOperation::tryCommitPolygon()
{
	IViewportInteractionHost* h = host();
	if (!h || static_cast<int>(m_vertices.size()) < kMinPolygonVertices)
	{
		return false;
	}
	h->commitPolylinePick(m_vertices);
	m_vertices.clear();
	m_hasCursor = false;
	refreshOverlay();
	return true;
}

bool PolylinePickOperation::canHandle(QObject* watched, QEvent* event) const
{
	(void)event;
	const IViewportInteractionHost* h = host();
	return h && watched == h->viewportGlWidget() && h->polylinePickMode();
}

bool PolylinePickOperation::onMouseButtonPress(QMouseEvent* e)
{
	IViewportInteractionHost* h = host();
	if (!h)
	{
		return false;
	}
	if (e->button() == Qt::LeftButton)
	{
		m_vertices.push_back(e->pos());
		m_cursorPos = e->pos();
		m_hasCursor = true;
		refreshOverlay();
		h->requestRedraw();
		h->emitPolylinePickFeedback(
			QStringLiteral("Vertices: %1 (right-click or double-click to close, Esc to cancel)")
				.arg(static_cast<int>(m_vertices.size())));
		return true;
	}
	if (e->button() == Qt::RightButton)
	{
		return tryCommitPolygon();
	}
	return e->button() != Qt::MiddleButton;
}

bool PolylinePickOperation::onMouseButtonRelease(QMouseEvent* e)
{
	if (e->button() == Qt::LeftButton)
	{
		return true;
	}
	return false;
}

bool PolylinePickOperation::onMouseDoubleClick(QMouseEvent* e)
{
	if (e->button() == Qt::LeftButton && !m_vertices.empty())
	{
		m_vertices.pop_back();
		refreshOverlay();
	}
	return tryCommitPolygon();
}

bool PolylinePickOperation::onWheel(QWheelEvent* e)
{
	(void)e;
	return false;
}

bool PolylinePickOperation::onMouseMove(QMouseEvent* e)
{
	if (e->buttons().testFlag(Qt::LeftButton))
	{
		return true;
	}
	IViewportInteractionHost* h = host();
	if (!h)
	{
		return false;
	}
	m_cursorPos = e->pos();
	m_hasCursor = true;
	refreshOverlay();
	h->requestRedraw();
	return true;
}
