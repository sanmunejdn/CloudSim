/// @file ViewportInteractionController.cpp
/// @brief ViewportInteractionController 实现

#include "ViewportInteraction/ViewportInteractionController.h"

#include <string>
#include <utility>

ViewportInteractionController::ViewportInteractionController(std::unique_ptr<IViewportPickEngine> engine)
	: m_engine(std::move(engine))
{
}

void ViewportInteractionController::addOverlay(std::unique_ptr<IOverlayOp> overlay)
{
	if (overlay)
	{
		m_overlays.push_back(std::move(overlay));
	}
}

void ViewportInteractionController::registerTool(std::unique_ptr<IPointerTool> tool)
{
	if (tool)
	{
		m_tools.push_back(std::move(tool));
	}
}

bool ViewportInteractionController::setActiveTool(const char* toolId)
{
	if (!toolId)
	{
		clearActiveTool();
		return false;
	}
	IPointerTool* found = nullptr;
	for (const auto& tool : m_tools)
	{
		if (tool && tool->toolId() && std::string(tool->toolId()) == toolId)
		{
			found = tool.get();
			break;
		}
	}
	if (!found)
	{
		return false;
	}
	if (m_activeTool == found)
	{
		return true;
	}
	if (m_activeTool)
	{
		m_activeTool->onDeactivated();
	}
	m_activeTool = found;
	m_activeTool->onActivated();
	return true;
}

void ViewportInteractionController::clearActiveTool()
{
	if (m_activeTool)
	{
		m_activeTool->onDeactivated();
		m_activeTool = nullptr;
	}
}

const char* ViewportInteractionController::activeToolId() const
{
	return m_activeTool ? m_activeTool->toolId() : nullptr;
}

void ViewportInteractionController::beginSession(std::shared_ptr<IInteractionSession> session)
{
	if (m_session)
	{
		m_session->onCancel();
	}
	m_session = std::move(session);
}

void ViewportInteractionController::endSession(bool cancel)
{
	if (!m_session)
	{
		return;
	}
	if (cancel)
	{
		m_session->onCancel();
	}
	m_session.reset();
}

void ViewportInteractionController::setHitPolicies(std::vector<std::unique_ptr<IHitResolvePolicy>> policies)
{
	m_policies = std::move(policies);
}

ViewportHit ViewportInteractionController::resolveHit(ViewportHit hit, const HitResolveContext& ctx) const
{
	if (hit.resolvedBackendId.empty() && hit.raw.hit)
	{
		hit.resolvedBackendId = hit.raw.backendId;
	}
	for (const auto& policy : m_policies)
	{
		if (policy)
		{
			policy->apply(hit, ctx);
		}
	}
	return hit;
}

void ViewportInteractionController::dispatchHover(ViewportHit hit, const HitResolveContext& ctx)
{
	hit.phase = HitPhase::Hover;
	hit = resolveHit(std::move(hit), ctx);
	if (m_session)
	{
		m_session->onHover(hit);
		hit.consumedBySession = true;
	}
}

void ViewportInteractionController::dispatchCommit(ViewportHit hit, const HitResolveContext& ctx)
{
	hit.phase = HitPhase::Commit;
	hit = resolveHit(std::move(hit), ctx);
	if (m_session)
	{
		m_session->onCommit(hit);
		hit.consumedBySession = true;
	}
}

bool ViewportInteractionController::handleEvent(QObject* watched, QEvent* event)
{
	for (const auto& overlay : m_overlays)
	{
		if (overlay && overlay->handleEvent(watched, event))
		{
			return true;
		}
	}
	if (m_activeTool && m_activeTool->handleEvent(watched, event))
	{
		return true;
	}
	return false;
}
