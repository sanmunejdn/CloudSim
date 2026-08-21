#ifndef WIDGET_PASSTHROUGHHITPOLICY_H
#define WIDGET_PASSTHROUGHHITPOLICY_H

/// @file PassthroughHitPolicy.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 默认透传：resolved 填 raw.backendId

#include "../IHitResolvePolicy.h"

class PassthroughHitPolicy final : public IHitResolvePolicy
{
public:
	bool apply(ViewportHit& hit, const HitResolveContext& ctx) const override
	{
		(void)ctx;
		if (hit.resolvedBackendId.empty() && hit.raw.hit)
		{
			hit.resolvedBackendId = hit.raw.backendId;
		}
		if (hit.gizmoAnchorBackendId.empty())
		{
			hit.gizmoAnchorBackendId = hit.resolvedBackendId;
		}
		return true;
	}
};

#endif // WIDGET_PASSTHROUGHHITPOLICY_H
