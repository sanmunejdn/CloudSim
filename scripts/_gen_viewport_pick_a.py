# -*- coding: utf-8 -*-
from pathlib import Path

ROOT = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim/src/UI/Widget")


def write(rel: str, text: str) -> None:
    path = ROOT / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(text.replace("\n", "\r\n").encode("utf-8-sig"))
    print("wrote", rel)


write(
    "inc/ViewportInteraction/ViewportHit.h",
    r"""#ifndef WIDGET_VIEWPORTHIT_H
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
""",
)

write(
    "inc/ViewportInteraction/IViewportPickEngine.h",
    r"""#ifndef WIDGET_IVIEWPORTPICKENGINE_H
#define WIDGET_IVIEWPORTPICKENGINE_H

/// @file IViewportPickEngine.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口拾取引擎：唯一 queryPick / 高亮入口

#include "../../../OsgWidgetCore/inc/PickTypes.h"

#include <osg/Vec3f>
#include <string>
#include <vector>

class IViewportPickEngine
{
public:
	virtual ~IViewportPickEngine() = default;

	virtual PickResult queryPick(const PickQuery& query) = 0;
	virtual std::string pickBackendIdAtScreenPos(double screenX, double screenY) const = 0;

	virtual void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld) = 0;
	virtual void showMeshEdgeHighlight(const std::vector<osg::Vec3f>& polylineWorld) = 0;
	virtual void showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld) = 0;
	virtual void hideMeshElementHighlight() = 0;
	virtual void requestRedraw() = 0;

	virtual const std::string& activeBackendId() const = 0;
	virtual bool crossObjectMeshPick() const = 0;
	virtual bool meshFacePickMode() const = 0;
	virtual bool meshLinePickMode() const = 0;
	virtual bool pointPickMode() const = 0;
	virtual bool objectSelectionMode() const = 0;
};

#endif // WIDGET_IVIEWPORTPICKENGINE_H
""",
)

write(
    "inc/ViewportInteraction/OsgWidgetPickEngine.h",
    r"""#ifndef WIDGET_OSGWIDGETPICKENGINE_H
#define WIDGET_OSGWIDGETPICKENGINE_H

/// @file OsgWidgetPickEngine.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief OsgWidget 上的 PickEngine 薄封装

#include "IViewportPickEngine.h"

class OsgWidget;

class OsgWidgetPickEngine final : public IViewportPickEngine
{
public:
	explicit OsgWidgetPickEngine(OsgWidget& widget);

	PickResult queryPick(const PickQuery& query) override;
	std::string pickBackendIdAtScreenPos(double screenX, double screenY) const override;

	void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld) override;
	void showMeshEdgeHighlight(const std::vector<osg::Vec3f>& polylineWorld) override;
	void showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld) override;
	void hideMeshElementHighlight() override;
	void requestRedraw() override;

	const std::string& activeBackendId() const override;
	bool crossObjectMeshPick() const override;
	bool meshFacePickMode() const override;
	bool meshLinePickMode() const override;
	bool pointPickMode() const override;
	bool objectSelectionMode() const override;

private:
	OsgWidget& m_widget;
};

#endif // WIDGET_OSGWIDGETPICKENGINE_H
""",
)

write(
    "source/ViewportInteraction/OsgWidgetPickEngine.cpp",
    r"""/// @file OsgWidgetPickEngine.cpp
/// @brief OsgWidgetPickEngine 实现

#include "ViewportInteraction/OsgWidgetPickEngine.h"

#include "OsgWidget.h"

OsgWidgetPickEngine::OsgWidgetPickEngine(OsgWidget& widget) : m_widget(widget) {}

PickResult OsgWidgetPickEngine::queryPick(const PickQuery& query)
{
	return m_widget.queryPick(query);
}

std::string OsgWidgetPickEngine::pickBackendIdAtScreenPos(double screenX, double screenY) const
{
	return m_widget.pickBackendIdAtScreenPos(screenX, screenY);
}

void OsgWidgetPickEngine::showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld)
{
	m_widget.showMeshFaceHighlight(vertsWorld);
}

void OsgWidgetPickEngine::showMeshEdgeHighlight(const std::vector<osg::Vec3f>& polylineWorld)
{
	m_widget.showMeshEdgeHighlight(polylineWorld);
}

void OsgWidgetPickEngine::showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld)
{
	m_widget.showMeshEdgeHighlight(aWorld, bWorld);
}

void OsgWidgetPickEngine::hideMeshElementHighlight()
{
	m_widget.hideMeshElementHighlight();
}

void OsgWidgetPickEngine::requestRedraw()
{
	m_widget.requestRedraw();
}

const std::string& OsgWidgetPickEngine::activeBackendId() const
{
	return m_widget.activeBackendId();
}

bool OsgWidgetPickEngine::crossObjectMeshPick() const
{
	return m_widget.crossObjectMeshPick();
}

bool OsgWidgetPickEngine::meshFacePickMode() const
{
	return m_widget.meshFacePickMode();
}

bool OsgWidgetPickEngine::meshLinePickMode() const
{
	return m_widget.meshLinePickMode();
}

bool OsgWidgetPickEngine::pointPickMode() const
{
	return m_widget.pointPickMode();
}

bool OsgWidgetPickEngine::objectSelectionMode() const
{
	return m_widget.objectSelectionMode();
}
""",
)

print("engine done")
