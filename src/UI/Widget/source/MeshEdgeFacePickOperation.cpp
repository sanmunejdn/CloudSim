#include "MeshEdgeFacePickOperation.h"

#include <QEvent>
#include <QMouseEvent>

#include "OsgScene.h"
#include "OsgWidget.h"
#include "PickTypes.h"

namespace
{

bool hoverPickUnchanged(const PickResult& pick, const PickPreviewState& preview, bool faceMode)
{
	if (!preview.valid || !preview.result.hit || !pick.hit)
	{
		return false;
	}
	const PickResult& prev = preview.result;
	if (pick.backendId != prev.backendId)
	{
		return false;
	}
	if (faceMode)
	{
		if (pick.brepNativePick)
		{
			return pick.brepFaceIndex == prev.brepFaceIndex;
		}
		return pick.pickedTriangleIndex >= 0 && pick.pickedTriangleIndex == prev.pickedTriangleIndex;
	}
	if (pick.brepNativePick)
	{
		return pick.brepEdgeIndex == prev.brepEdgeIndex;
	}
	return pick.meshEdgeA == prev.meshEdgeA && pick.meshEdgeB == prev.meshEdgeB;
}

} // namespace

MeshEdgeFacePickOperation::MeshEdgeFacePickOperation(OsgWidget* owner)
	: SelectionOperation(owner)
{
}

bool MeshEdgeFacePickOperation::canHandle(QObject* watched, QEvent* event) const
{
	(void)event;
	return m_owner
		&& watched == m_owner->m_glWidget
		&& (m_owner->m_meshLinePickMode || m_owner->m_meshFacePickMode);
}

PickQuery MeshEdgeFacePickOperation::makePickQuery(const QPoint& pos) const
{
	PickQuery query;
	query.screenX = pos.x();
	query.screenY = pos.y();
	query.hoverPick = true;
	query.kind = m_owner->m_meshFacePickMode ? PickKind::MeshFace : PickKind::MeshEdge;
	if (!m_owner->m_activeBackendId.empty())
	{
		query.scopeBackendId = m_owner->m_activeBackendId;
	}
	return query;
}

void MeshEdgeFacePickOperation::applyPickResult(const PickResult& pick)
{
	if (!pick.hit)
	{
		return;
	}
	if (m_owner->m_meshFacePickMode)
	{
		m_owner->showMeshFaceHighlight(pick.meshFaceVertsWorld);
	}
	else if (!pick.meshEdgePolylineWorld.empty())
	{
		m_owner->showMeshEdgeHighlight(pick.meshEdgePolylineWorld);
	}
	else
	{
		m_owner->showMeshEdgeHighlight(pick.meshEdgeA, pick.meshEdgeB);
	}
}

void MeshEdgeFacePickOperation::emitMeshFeedback(bool click, const PickResult& pick) const
{
	const QString phase = click ? QStringLiteral("Click") : QStringLiteral("Hover");
	const QString kind = m_owner->m_meshFacePickMode ? QStringLiteral("face") : QStringLiteral("edge");
	QString detail;
	if (m_owner->m_meshLinePickMode && pick.hit)
	{
		detail = QStringLiteral(" | edge: %1 px").arg(pick.screenDistancePx, 0, 'f', 1);
	}
	emit m_owner->meshPickFeedback(QStringLiteral("%1 %2 %3%4")
		.arg(phase)
		.arg(pick.hit ? QStringLiteral("Hit") : QStringLiteral("Miss"))
		.arg(kind)
		.arg(detail));
}

bool MeshEdgeFacePickOperation::onMouseButtonPress(QMouseEvent* mouseEvent)
{
	if (mouseEvent->button() != Qt::LeftButton)
	{
		return mouseEvent->button() != Qt::MiddleButton;
	}
	m_gesture.onLeftPress(mouseEvent->pos());
	return false;
}

bool MeshEdgeFacePickOperation::onMouseMove(QMouseEvent* mouseEvent)
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
	const int hoverThrottleMs = m_owner->m_meshFacePickMode
		? OsgScene::kPickHoverThrottleMs
		: OsgScene::kPickHoverEdgeThrottleMs;
	if (ViewportGestureRecognizer::shouldThrottleHover(m_owner->m_feedbackTimer, hoverThrottleMs))
	{
		return true;
	}

	const bool inClickHold = m_gesture.inClickHold(m_clickHoldTimer);
	const PickResult pick = m_owner->queryPick(makePickQuery(mouseEvent->pos()));

	if (pick.hit && hoverPickUnchanged(pick, m_preview, m_owner->m_meshFacePickMode))
	{
		emitMeshFeedback(false, pick);
		m_owner->m_feedbackTimer.restart();
		return true;
	}

	if (pick.hit)
	{
		m_preview.valid = true;
		m_preview.result = pick;
		applyPickResult(pick);
	}
	else if (!inClickHold)
	{
		m_preview.valid = false;
		m_owner->hideMeshElementHighlight();
	}

	emitMeshFeedback(false, pick);
	m_owner->m_feedbackTimer.restart();
	m_owner->requestRedraw();
	return true;
}

bool MeshEdgeFacePickOperation::onMouseButtonRelease(QMouseEvent* mouseEvent)
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

	PickResult pick = (m_preview.valid && m_preview.result.hit) ? m_preview.result : PickResult{};
	if (!pick.hit)
	{
		PickQuery query = makePickQuery(mouseEvent->pos());
		query.hoverPick = false;
		pick = m_owner->queryPick(query);
	}
	if (pick.hit)
	{
		m_preview.valid = true;
		m_preview.result = pick;
		applyPickResult(pick);
	}

	emitMeshFeedback(true, pick);
	if (pick.hit)
	{
		const int kindInt = m_owner->m_meshFacePickMode
			? static_cast<int>(PickKind::MeshFace)
			: static_cast<int>(PickKind::MeshEdge);
		emit m_owner->meshPickCommitted(pick, kindInt);
	}
	m_gesture.restartClickHold(m_clickHoldTimer);
	m_owner->requestRedraw();
	return swallowRelease;
}
