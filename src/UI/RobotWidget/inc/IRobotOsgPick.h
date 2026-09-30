#ifndef ROBOTWIDGET_IROBOTOSGPICK_H
#define ROBOTWIDGET_IROBOTOSGPICK_H

/// @file IRobotOsgPick.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人 UI 视口拾取模式（IRobotOsgViewHost 切面）

#include "robotwidget_global.h"

#include <functional>
#include <string>
#include <vector>

#include <osg/Vec3f>

enum class MeshTrianglePickTool
{
	None = 0,
	Click,
	Brush,
	Polyline
};

/// 点/线/面/三角标注与路点拾取
class ROBOTWIDGET_EXPORT IRobotOsgPick
{
public:
	virtual ~IRobotOsgPick() = default;

	virtual void setPointPickMode(bool enabled) = 0;
	virtual bool pointPickMode() const = 0;

	virtual void setMeshLinePickMode(bool enabled) = 0;
	virtual void setMeshFacePickMode(bool enabled) = 0;
	virtual bool meshLinePickMode() const = 0;
	virtual bool meshFacePickMode() const = 0;
	virtual void setMeshPickScopeBackendId(const std::string& backendId) = 0;
	virtual void syncSelectionForBackendId(const std::string& backendId) = 0;

	virtual void setMeshTrianglePickTool(MeshTrianglePickTool tool, float brushRadiusPx = 12.f) = 0;
	virtual void cancelMeshTrianglePick() = 0;
	virtual MeshTrianglePickTool meshTrianglePickTool() const = 0;

	virtual void setPolylinePickMode(bool enabled) = 0;
	virtual bool polylinePickMode() const = 0;

	virtual void setInstructionWaypointPickMode(bool enabled) = 0;
	virtual bool instructionWaypointPickMode() const = 0;
	virtual void
	setInstructionWaypointPickCallbacks(std::function<void(const std::string& instructionId, bool isArcVia)> onPicked,
										std::function<void()> onCanceled) = 0;

	virtual void setCrossObjectMeshPick(bool enabled) = 0;
	virtual void showPinnedMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld) = 0;
	virtual void hidePinnedMeshFaceHighlight() = 0;
};

#endif // ROBOTWIDGET_IROBOTOSGPICK_H
