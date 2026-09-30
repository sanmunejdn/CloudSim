/// @file LabelingPickOperation.cpp
/// @brief LabelingPick 操作

#include "LabelingPickOperation.h"

#include "OsgScene.h"
#include "PickTypes.h"

#include <QEvent>
#include <QMouseEvent>

LabelingPickOperation::LabelingPickOperation(IViewportInteractionHost* host) : SelectionOperation(host) {}

bool LabelingPickOperation::canHandle(QObject* watched, QEvent* event) const
{
	(void)event;
	const IViewportInteractionHost* h = host();
	return h && watched == h->viewportGlWidget() &&
		   (h->labelingClickPickMode() || h->labelingBrushPickMode());
}

void LabelingPickOperation::emitClickPick(const QPoint& pos)
{
	IViewportInteractionHost* h = host();
	if (!h)
	{
		return;
	}
	PickQuery query;
	query.screenX = pos.x();
	query.screenY = pos.y();
	if (h->labelingMeshFaceMode())
	{
		query.kind = PickKind::MeshFace;
	}
	else
	{
		query.kind = PickKind::PointCloud;
		query.hitRadiusPx = OsgScene::kPointPickHitRadiusPx;
	}
	const PickResult pick = h->queryPick(query);
	h->emitLabelingClickCommitted(pick);
}

void LabelingPickOperation::emitBrushStroke(const QPoint& pos)
{
	IViewportInteractionHost* h = host();
	if (!h || !h->labelingBrushPickMode())
	{
		return;
	}
	QVector<int> indices;
	if (h->labelingMeshFaceMode())
	{
		PickQuery query;
		query.screenX = pos.x();
		query.screenY = pos.y();
		query.kind = PickKind::MeshFace;
		const PickResult pick = h->queryPick(query);
		if (pick.hit && pick.meshTriangleIndex >= 0)
		{
			indices.push_back(pick.meshTriangleIndex);
		}
	}
	else
	{
		std::vector<int> raw;
		h->collectPointIndicesInScreenRadius(pos.x(), pos.y(), h->labelingBrushRadiusPx(), raw);
		indices.reserve(static_cast<int>(raw.size()));
		for (int idx : raw)
		{
			indices.push_back(idx);
		}
	}
	for (int idx : indices)
	{
		m_brushAccumulated.insert(idx);
	}
	if (!indices.isEmpty())
	{
		h->emitLabelingBrushStroke(indices);
	}
}

bool LabelingPickOperation::onMouseButtonPress(QMouseEvent* mouseEvent)
{
	IViewportInteractionHost* h = host();
	if (!h)
	{
		return false;
	}
	if (mouseEvent->button() == Qt::LeftButton)
	{
		m_gesture.onLeftPress(mouseEvent->pos());
		if (h->labelingBrushPickMode())
		{
			m_brushAccumulated.clear();
			emitBrushStroke(mouseEvent->pos());
		}
		return h->labelingBrushPickMode();
	}
	return false;
}

bool LabelingPickOperation::onMouseButtonRelease(QMouseEvent* mouseEvent)
{
	IViewportInteractionHost* h = host();
	if (!h)
	{
		return false;
	}
	if (mouseEvent->button() != Qt::LeftButton)
	{
		return false;
	}
	bool swallowRelease = false;
	if (!m_gesture.onLeftRelease(mouseEvent->pos(), &swallowRelease))
	{
		return false;
	}
	if (h->labelingClickPickMode())
	{
		emitClickPick(mouseEvent->pos());
		return swallowRelease;
	}
	if (h->labelingBrushPickMode())
	{
		h->emitLabelingBrushFinished();
		return true;
	}
	return swallowRelease;
}

bool LabelingPickOperation::onMouseDoubleClick(QMouseEvent*)
{
	return true;
}

bool LabelingPickOperation::onWheel(QWheelEvent*)
{
	return false;
}

bool LabelingPickOperation::onMouseMove(QMouseEvent* mouseEvent)
{
	IViewportInteractionHost* h = host();
	if (!h)
	{
		return false;
	}
	if (mouseEvent->buttons().testFlag(Qt::LeftButton))
	{
		if (h->labelingBrushPickMode())
		{
			emitBrushStroke(mouseEvent->pos());
			return true;
		}
		m_gesture.onLeftMove(mouseEvent->pos());
		return false;
	}
	return false;
}
