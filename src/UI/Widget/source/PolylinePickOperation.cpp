/// @file PolylinePickOperation.cpp
/// @brief PolylinePickOperation 实现

#include "PolylinePickOperation.h"

#include "OsgWidget.h"

#include <QEvent>
#include <QKeyEvent>
#include <QMouseEvent>

namespace
{
constexpr int kMinPolygonVertices = 3;

} // namespace

PolylinePickOperation::PolylinePickOperation(OsgWidget* owner) : SelectionOperation(owner) {}

void PolylinePickOperation::refreshOverlay() const
{
	if (!m_owner)
	{
		return;
	}
	m_owner->updatePolylinePickOverlay(m_vertices, m_hasCursor ? &m_cursorPos : nullptr);
}

bool PolylinePickOperation::tryCommitPolygon()
{
	if (!m_owner || static_cast<int>(m_vertices.size()) < kMinPolygonVertices)
	{
		return false;
	}
	m_owner->commitPolylinePick(m_vertices);
	m_vertices.clear();
	m_hasCursor = false;
	refreshOverlay();
	return true;
}

bool PolylinePickOperation::canHandle(QObject* watched, QEvent* event) const
{
	(void)event;
	return m_owner && watched == m_owner->m_glWidget && m_owner->m_polylinePickMode;
}

bool PolylinePickOperation::onMouseButtonPress(QMouseEvent* e)
{
	if (e->button() == Qt::LeftButton)
	{
		m_vertices.push_back(e->pos());
		m_cursorPos = e->pos();
		m_hasCursor = true;
		refreshOverlay();
		m_owner->requestRedraw();
		emit m_owner->polylinePickFeedback(
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
	// 双击序列会先触发第二次 press 加点，闭合前去掉该重复点
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
	m_cursorPos = e->pos();
	m_hasCursor = true;
	refreshOverlay();
	m_owner->requestRedraw();
	return true;
}
