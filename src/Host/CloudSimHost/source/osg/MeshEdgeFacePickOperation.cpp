/// @file MeshEdgeFacePickOperation.cpp

/// @brief MeshEdgeFacePick 操作



#include "MeshEdgeFacePickOperation.h"



#include "OsgScene.h"

#include "PickTypes.h"

#include "ViewportInteraction/IViewportPickEngine.h"



#include <QEvent>

#include <QMouseEvent>



namespace

{

IViewportPickEngine* meshPickEngineOf(IViewportInteractionHost* owner)

{

	return owner ? owner->pickEngine() : nullptr;

}



PickResult meshQueryPick(IViewportInteractionHost* owner, const PickQuery& query)

{

	if (IViewportPickEngine* eng = meshPickEngineOf(owner))

	{

		return eng->queryPick(query);

	}

	return owner->queryPick(query);

}



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



MeshEdgeFacePickOperation::MeshEdgeFacePickOperation(IViewportInteractionHost* host) : SelectionOperation(host) {}



bool MeshEdgeFacePickOperation::canHandle(QObject* watched, QEvent* event) const

{

	(void)event;

	const IViewportInteractionHost* h = host();

	return h && watched == h->viewportGlWidget() && (h->meshLinePickMode() || h->meshFacePickMode());

}



PickQuery MeshEdgeFacePickOperation::makePickQuery(const QPoint& pos) const

{

	const IViewportInteractionHost* h = host();

	PickQuery query;

	query.screenX = pos.x();

	query.screenY = pos.y();

	query.hoverPick = true;

	query.kind = h->meshFacePickMode() ? PickKind::MeshFace : PickKind::MeshEdge;

	if (!h->crossObjectMeshPick() && !h->activeBackendId().empty())

	{

		query.scopeBackendId = h->activeBackendId();

	}

	return query;

}



void MeshEdgeFacePickOperation::applyPickResult(const PickResult& pick)

{

	IViewportInteractionHost* h = host();

	if (!h || !pick.hit)

	{

		return;

	}

	if (h->meshFacePickMode())

	{

		h->showMeshFaceHighlight(pick.meshFaceVertsWorld);

	}

	else if (!pick.meshEdgePolylineWorld.empty())

	{

		h->showMeshEdgeHighlight(pick.meshEdgePolylineWorld);

	}

	else

	{

		h->showMeshEdgeHighlight(pick.meshEdgeA, pick.meshEdgeB);

	}

}



void MeshEdgeFacePickOperation::emitMeshFeedback(bool click, const PickResult& pick)

{

	IViewportInteractionHost* h = host();

	if (!h)

	{

		return;

	}

	const QString phase = click ? QStringLiteral("Click") : QStringLiteral("Hover");

	const QString kind = h->meshFacePickMode() ? QStringLiteral("face") : QStringLiteral("edge");

	QString detail;

	if (h->meshLinePickMode() && pick.hit)

	{

		detail = QStringLiteral(" | edge: %1 px").arg(pick.screenDistancePx, 0, 'f', 1);

	}

	h->emitMeshPickFeedback(QStringLiteral("%1 %2 %3%4")

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

	IViewportInteractionHost* h = host();

	if (!h)

	{

		return false;

	}

	if (mouseEvent->buttons().testFlag(Qt::LeftButton))

	{

		m_gesture.onLeftMove(mouseEvent->pos());

		return false;

	}

	if (mouseEvent->buttons().testFlag(Qt::MiddleButton))

	{

		return false;

	}

	if (h->originPlanePickActive() && h->originPlaneHoverIndex() >= 0)

	{

		m_preview.valid = false;

		h->hideMeshElementHighlight();

		h->interactionPointerState().feedbackTimer.restart();

		return true;

	}

	const int hoverThrottleMs =

		h->meshFacePickMode() ? OsgScene::kPickHoverThrottleMs : OsgScene::kPickHoverEdgeThrottleMs;

	ViewportInteractionPointerState pointer = h->interactionPointerState();

	if (ViewportGestureRecognizer::shouldThrottleHover(pointer.feedbackTimer, hoverThrottleMs))

	{

		return true;

	}



	const bool inClickHold = m_gesture.inClickHold(m_clickHoldTimer);

	const PickResult pick = meshQueryPick(h, makePickQuery(mouseEvent->pos()));



	if (pick.hit && hoverPickUnchanged(pick, m_preview, h->meshFacePickMode()))

	{

		emitMeshFeedback(false, pick);

		pointer.feedbackTimer.restart();

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

		h->hideMeshElementHighlight();

	}



	emitMeshFeedback(false, pick);

	pointer.feedbackTimer.restart();

	h->requestRedraw();

	return true;

}



bool MeshEdgeFacePickOperation::onMouseButtonRelease(QMouseEvent* mouseEvent)

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



	if (h->originPlanePickActive() && h->resolveSketchSupportOriginIndex(mouseEvent->x(), mouseEvent->y()) >= 0)

	{

		m_preview.valid = false;

		h->hideMeshElementHighlight();

		h->requestRedraw();

		return swallowRelease;

	}



	PickResult pick = (m_preview.valid && m_preview.result.hit) ? m_preview.result : PickResult{};

	if (!pick.hit)

	{

		PickQuery query = makePickQuery(mouseEvent->pos());

		query.hoverPick = false;

		pick = meshQueryPick(h, query);

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

		const int kindInt = h->meshFacePickMode() ? static_cast<int>(PickKind::MeshFace) : static_cast<int>(PickKind::MeshEdge);

		h->emitMeshPickCommitted(pick, kindInt);

	}

	m_gesture.restartClickHold(m_clickHoldTimer);

	h->requestRedraw();

	return swallowRelease;

}

