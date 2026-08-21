#ifndef WIDGET_VIEWPORTINTERACTIONCONTROLLER_H
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
