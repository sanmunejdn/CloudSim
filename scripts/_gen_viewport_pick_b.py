# -*- coding: utf-8 -*-
from pathlib import Path

ROOT = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim/src/UI/Widget")


def write(rel: str, text: str) -> None:
    path = ROOT / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(text.replace("\n", "\r\n").encode("utf-8-sig"))
    print("wrote", rel)


write(
    "inc/ViewportInteraction/IPointerTool.h",
    r"""#ifndef WIDGET_IPOINTERTOOL_H
#define WIDGET_IPOINTERTOOL_H

/// @file IPointerTool.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 指针工具：手势与命中，不直接改业务树

#include "ViewportHit.h"

class QObject;
class QEvent;

class IPointerTool
{
public:
	virtual ~IPointerTool() = default;
	virtual const char* toolId() const = 0;
	virtual void onActivated() {}
	virtual void onDeactivated() {}
	virtual bool handleEvent(QObject* watched, QEvent* event) = 0;
};

#endif // WIDGET_IPOINTERTOOL_H
""",
)

write(
    "inc/ViewportInteraction/IOverlayOp.h",
    r"""#ifndef WIDGET_IOVERLAYOP_H
#define WIDGET_IOVERLAYOP_H

/// @file IOverlayOp.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 叠加操作：罗盘/TCP/截面，不产业务 Hit

class QObject;
class QEvent;

class IOverlayOp
{
public:
	virtual ~IOverlayOp() = default;
	virtual const char* overlayId() const = 0;
	virtual bool handleEvent(QObject* watched, QEvent* event) = 0;
};

#endif // WIDGET_IOVERLAYOP_H
""",
)

write(
    "inc/ViewportInteraction/IHitResolvePolicy.h",
    r"""#ifndef WIDGET_IHITRESOLVEPOLICY_H
#define WIDGET_IHITRESOLVEPOLICY_H

/// @file IHitResolvePolicy.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 命中语义归并策略

#include "ViewportHit.h"

struct HitResolveContext
{
	std::string currentSelectedBackendId;
	bool objectSelectTool = false;
};

class IHitResolvePolicy
{
public:
	virtual ~IHitResolvePolicy() = default;
	virtual bool apply(ViewportHit& hit, const HitResolveContext& ctx) const = 0;
};

#endif // WIDGET_IHITRESOLVEPOLICY_H
""",
)

write(
    "inc/ViewportInteraction/IInteractionSession.h",
    r"""#ifndef WIDGET_IINTERACTIONSESSION_H
#define WIDGET_IINTERACTIONSESSION_H

/// @file IInteractionSession.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 交互会话：业务消费 Hover/Commit/Cancel

#include "ViewportHit.h"

class IInteractionSession
{
public:
	virtual ~IInteractionSession() = default;
	virtual void onHover(const ViewportHit& hit) { (void)hit; }
	virtual void onCommit(const ViewportHit& hit) = 0;
	virtual void onCancel() {}
};

#endif // WIDGET_IINTERACTIONSESSION_H
""",
)

write(
    "inc/ViewportInteraction/ViewportInteractionController.h",
    r"""#ifndef WIDGET_VIEWPORTINTERACTIONCONTROLLER_H
#define WIDGET_VIEWPORTINTERACTIONCONTROLLER_H

/// @file ViewportInteractionController.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口交互控制器：activeTool + overlays + Session

#include "IHitResolvePolicy.h"
#include "IInteractionSession.h"
#include "IOverlayOp.h"
#include "IPointerTool.h"
#include "IViewportPickEngine.h"

#include <memory>
#include <string>
#include <vector>

class QObject;
class QEvent;

class ViewportInteractionController
{
public:
	explicit ViewportInteractionController(std::unique_ptr<IViewportPickEngine> engine);

	IViewportPickEngine& engine() { return *m_engine; }
	const IViewportPickEngine& engine() const { return *m_engine; }

	void addOverlay(std::unique_ptr<IOverlayOp> overlay);
	void registerTool(std::unique_ptr<IPointerTool> tool);

	bool setActiveTool(const char* toolId);
	void clearActiveTool();
	const char* activeToolId() const;

	void beginSession(std::shared_ptr<IInteractionSession> session);
	void endSession(bool cancel);
	bool hasSession() const { return static_cast<bool>(m_session); }

	void setHitPolicies(std::vector<std::unique_ptr<IHitResolvePolicy>> policies);
	ViewportHit resolveHit(ViewportHit hit, const HitResolveContext& ctx) const;
	void dispatchHover(ViewportHit hit, const HitResolveContext& ctx);
	void dispatchCommit(ViewportHit hit, const HitResolveContext& ctx);

	bool handleEvent(QObject* watched, QEvent* event);

private:
	std::unique_ptr<IViewportPickEngine> m_engine;
	std::vector<std::unique_ptr<IOverlayOp>> m_overlays;
	std::vector<std::unique_ptr<IPointerTool>> m_tools;
	IPointerTool* m_activeTool = nullptr;
	std::shared_ptr<IInteractionSession> m_session;
	std::vector<std::unique_ptr<IHitResolvePolicy>> m_policies;
};

#endif // WIDGET_VIEWPORTINTERACTIONCONTROLLER_H
""",
)

write(
    "source/ViewportInteraction/ViewportInteractionController.cpp",
    r"""/// @file ViewportInteractionController.cpp
/// @brief ViewportInteractionController 实现

#include "ViewportInteraction/ViewportInteractionController.h"

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
""",
)

print("controller ok")
