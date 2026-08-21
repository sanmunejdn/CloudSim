#ifndef WIDGET_VIEWPORTHIT_H
#define WIDGET_VIEWPORTHIT_H

/// @file ViewportHit.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口命中 DTO：raw/resolved 与 Hover/Commit

#include "../../../OsgWidgetCore/inc/PickTypes.h"

#include <string>

enum class HitPhase
{
	Hover,
	Commit
};

struct ViewportHit
{
	HitPhase phase = HitPhase::Hover;
	PickKind kind = PickKind::BackendObject;
	PickResult raw{};
	std::string resolvedBackendId;
	std::string gizmoAnchorBackendId;
	bool consumedBySession = false;
};

#endif // WIDGET_VIEWPORTHIT_H
