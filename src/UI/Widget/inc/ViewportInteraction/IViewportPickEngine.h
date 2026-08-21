#ifndef WIDGET_IVIEWPORTPICKENGINE_H
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
