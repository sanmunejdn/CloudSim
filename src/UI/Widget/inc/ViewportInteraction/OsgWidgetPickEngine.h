#ifndef WIDGET_OSGWIDGETPICKENGINE_H
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
