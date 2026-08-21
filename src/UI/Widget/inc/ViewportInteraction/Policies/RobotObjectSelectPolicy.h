#ifndef WIDGET_ROBOTOBJECTSELECTPOLICY_H
#define WIDGET_ROBOTOBJECTSELECTPOLICY_H

/// @file RobotObjectSelectPolicy.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 对象选：整机归并 + gizmo 锚点（由外部注入 DocumentPage 解析）

#include "../IHitResolvePolicy.h"

#include <functional>
#include <string>

class RobotObjectSelectPolicy final : public IHitResolvePolicy
{
public:
	using ResolveFn = std::function<std::string(const std::string& rawBackendId)>;

	RobotObjectSelectPolicy(ResolveFn resolveSelection, ResolveFn resolveGizmoAnchor)
		: m_resolveSelection(std::move(resolveSelection)), m_resolveGizmoAnchor(std::move(resolveGizmoAnchor))
	{
	}

	bool apply(ViewportHit& hit, const HitResolveContext& ctx) const override
	{
		if (!ctx.objectSelectTool || !hit.raw.hit)
		{
			return false;
		}
		const std::string raw = hit.raw.backendId;
		if (m_resolveSelection)
		{
			hit.resolvedBackendId = m_resolveSelection(raw);
		}
		if (hit.resolvedBackendId.empty())
		{
			hit.resolvedBackendId = raw;
		}
		if (m_resolveGizmoAnchor)
		{
			hit.gizmoAnchorBackendId = m_resolveGizmoAnchor(hit.resolvedBackendId);
		}
		if (hit.gizmoAnchorBackendId.empty())
		{
			hit.gizmoAnchorBackendId = hit.resolvedBackendId;
		}
		return true;
	}

private:
	ResolveFn m_resolveSelection;
	ResolveFn m_resolveGizmoAnchor;
};

#endif // WIDGET_ROBOTOBJECTSELECTPOLICY_H
