#ifndef WIDGET_IHITRESOLVEPOLICY_H
#define WIDGET_IHITRESOLVEPOLICY_H

/// @file IHitResolvePolicy.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 命中语义归并策略

#include "ViewportHit.h"

#include <string>

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
