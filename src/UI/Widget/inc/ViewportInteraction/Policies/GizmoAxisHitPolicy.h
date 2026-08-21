#ifndef WIDGET_GIZMOAXISHITPOLICY_H
#define WIDGET_GIZMOAXISHITPOLICY_H

/// @file GizmoAxisHitPolicy.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 罗盘轴命中标记：对象选 Commit 应忽略

#include "../IHitResolvePolicy.h"

class GizmoAxisHitPolicy final : public IHitResolvePolicy
{
public:
	bool apply(ViewportHit& hit, const HitResolveContext& ctx) const override
	{
		(void)ctx;
		if (hit.kind == PickKind::GizmoAxis)
		{
			hit.kind = PickKind::GizmoAxis;
			return true;
		}
		return false;
	}
};

#endif // WIDGET_GIZMOAXISHITPOLICY_H
