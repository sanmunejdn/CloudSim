/// @file LabelingPickOperation.cpp
/// @brief LabelingPick 操作

#include "LabelingPickOperation.h"

#include "OsgScene.h"
#include "OsgWidget.h"
#include "PickTypes.h"

#include <QEvent>
#include <QMouseEvent>

LabelingPickOperation::LabelingPickOperation(OsgWidget* owner) : SelectionOperation(owner) {}

bool LabelingPickOperation::canHandle(QObject* watched, QEvent* event) const
{
	(void)event;
	return m_owner && watched == m_owner->m_glWidget &&
		   (m_owner->m_labelingClickPickMode || m_owner->m_labelingBrushPickMode);
}

void LabelingPickOperation::emitClickPick(const QPoint& pos)
{
	PickQuery query;
	query.screenX = pos.x();
	query.screenY = pos.y();
	if (m_owner->m_labelingMeshFaceMode)
	{
		query.kind = PickKind::MeshFace;
	}
	else
	{
		query.kind = PickKind::PointCloud;
		query.hitRadiusPx = OsgScene::kPointPickHitRadiusPx;
	}
	const PickResult pick = m_owner->queryPick(query);
	emit m_owner->labelingClickCommitted(pick);
}

void LabelingPickOperation::emitBrushStroke(const QPoint& pos)
{
	if (!m_owner->m_labelingBrushPickMode)
	{
		return;
	}
	QVector<int> indices;
	if (m_owner->m_labelingMeshFaceMode)
	{
		PickQuery query;
		query.screenX = pos.x();
		query.screenY = pos.y();
		query.kind = PickKind::MeshFace;
		const PickResult pick = m_owner->queryPick(query);
		if (pick.hit && pick.meshTriangleIndex >= 0)
		{
			indices.push_back(pick.meshTriangleIndex);
		}
	}
	else
	{
		std::vector<int> raw;
		m_owner->collectPointIndicesInScreenRadius(pos.x(), pos.y(), m_owner->m_labelingBrushRadiusPx, raw);
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
		emit m_owner->labelingBrushStroke(indices);
	}
}

bool LabelingPickOperation::onMouseButtonPress(QMouseEvent* mouseEvent)
{
	if (mouseEvent->button() == Qt::LeftButton)
	{
		m_gesture.onLeftPress(mouseEvent->pos());
		if (m_owner->m_labelingBrushPickMode)
		{
			m_brushAccumulated.clear();
			emitBrushStroke(mouseEvent->pos());
		}
		return m_owner->m_labelingBrushPickMode;
	}
	return false;
}

bool LabelingPickOperation::onMouseButtonRelease(QMouseEvent* mouseEvent)
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
	if (m_owner->m_labelingClickPickMode)
	{
		emitClickPick(mouseEvent->pos());
		return swallowRelease;
	}
	if (m_owner->m_labelingBrushPickMode)
	{
		emit m_owner->labelingBrushFinished();
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
	if (mouseEvent->buttons().testFlag(Qt::LeftButton))
	{
		if (m_owner->m_labelingBrushPickMode)
		{
			emitBrushStroke(mouseEvent->pos());
			return true;
		}
		m_gesture.onLeftMove(mouseEvent->pos());
		return false;
	}
	return false;
}
