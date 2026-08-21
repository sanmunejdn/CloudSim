# -*- coding: utf-8 -*-
from pathlib import Path

ROOT = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim/src/UI/Widget")


def write(rel: str, text: str) -> None:
    path = ROOT / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(text.replace("\n", "\r\n").encode("utf-8-sig"))
    print("wrote", rel)


write(
    "inc/ViewportInteraction/Policies/PassthroughHitPolicy.h",
    r"""#ifndef WIDGET_PASSTHROUGHHITPOLICY_H
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
""",
)

write(
    "inc/ViewportInteraction/Policies/RobotObjectSelectPolicy.h",
    r"""#ifndef WIDGET_ROBOTOBJECTSELECTPOLICY_H
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
""",
)

write(
    "inc/ViewportInteraction/Policies/GizmoAxisHitPolicy.h",
    r"""#ifndef WIDGET_GIZMOAXISHITPOLICY_H
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
		if (hit.kind == PickKind::GizmoAxis || hit.raw.kind == PickKind::GizmoAxis)
		{
			hit.kind = PickKind::GizmoAxis;
			return true;
		}
		return false;
	}
};

#endif // WIDGET_GIZMOAXISHITPOLICY_H
""",
)

write(
    "inc/ViewportInteraction/Tools/SelectionOperationToolAdapter.h",
    r"""#ifndef WIDGET_SELECTIONOPERATIONTOOLADAPTER_H
#define WIDGET_SELECTIONOPERATIONTOOLADAPTER_H

/// @file SelectionOperationToolAdapter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 将既有 SelectionOperation 适配为 IPointerTool

#include "../IPointerTool.h"

#include <string>

class SelectionOperation;

class SelectionOperationToolAdapter final : public IPointerTool
{
public:
	SelectionOperationToolAdapter(const char* id, SelectionOperation* operation)
		: m_id(id ? id : ""), m_operation(operation)
	{
	}

	const char* toolId() const override { return m_id.c_str(); }

	bool handleEvent(QObject* watched, QEvent* event) override
	{
		return m_operation ? m_operation->handleEvent(watched, event) : false;
	}

private:
	std::string m_id;
	SelectionOperation* m_operation = nullptr;
};

#endif // WIDGET_SELECTIONOPERATIONTOOLADAPTER_H
""",
)

write(
    "inc/ViewportInteraction/Overlays/SelectionOperationOverlayAdapter.h",
    r"""#ifndef WIDGET_SELECTIONOPERATIONOVERLAYADAPTER_H
#define WIDGET_SELECTIONOPERATIONOVERLAYADAPTER_H

/// @file SelectionOperationOverlayAdapter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 将既有 SelectionOperation 适配为 IOverlayOp

#include "../IOverlayOp.h"

#include <string>

class SelectionOperation;

class SelectionOperationOverlayAdapter final : public IOverlayOp
{
public:
	SelectionOperationOverlayAdapter(const char* id, SelectionOperation* operation)
		: m_id(id ? id : ""), m_operation(operation)
	{
	}

	const char* overlayId() const override { return m_id.c_str(); }

	bool handleEvent(QObject* watched, QEvent* event) override
	{
		return m_operation ? m_operation->handleEvent(watched, event) : false;
	}

private:
	std::string m_id;
	SelectionOperation* m_operation = nullptr;
};

#endif // WIDGET_SELECTIONOPERATIONOVERLAYADAPTER_H
""",
)

write(
    "inc/ViewportInteraction/MeshPickSession.h",
    r"""#ifndef WIDGET_MESHPICKSESSION_H
#define WIDGET_MESHPICKSESSION_H

/// @file MeshPickSession.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Mesh 边/面拾取会话：Commit 回调给业务

#include "../IInteractionSession.h"

#include <functional>

class MeshPickSession final : public IInteractionSession
{
public:
	using CommitFn = std::function<void(const ViewportHit&)>;
	using CancelFn = std::function<void()>;

	explicit MeshPickSession(CommitFn onCommit, CancelFn onCancel = {})
		: m_onCommit(std::move(onCommit)), m_onCancel(std::move(onCancel))
	{
	}

	void onCommit(const ViewportHit& hit) override
	{
		if (m_onCommit)
		{
			m_onCommit(hit);
		}
	}

	void onCancel() override
	{
		if (m_onCancel)
		{
			m_onCancel();
		}
	}

private:
	CommitFn m_onCommit;
	CancelFn m_onCancel;
};

#endif // WIDGET_MESHPICKSESSION_H
""",
)

print("policies+adapters ok")
