#ifndef ROBOTWIDGET_IROBOTOSGSCENEOPS_H
#define ROBOTWIDGET_IROBOTOSGSCENEOPS_H

/// @file IRobotOsgSceneOps.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人 UI 场景矩阵、选择与截面（IRobotOsgViewHost 切面）

#include "robotwidget_global.h"

#include "CoreTypes.h"

#include <functional>
#include <string>

class IRobotBackendPoseSink;

/// 后端世界矩阵、对象选择与截面编辑
class ROBOTWIDGET_EXPORT IRobotOsgSceneOps
{
public:
	virtual ~IRobotOsgSceneOps() = default;

	virtual IRobotBackendPoseSink* poseSink() = 0;
	virtual void requestRedraw() = 0;

	virtual bool objectSelectionMode() const = 0;
	virtual void setObjectSelectionMode(bool enabled) = 0;
	virtual void clearBackendObjectSelection() = 0;
	virtual void setSelectionActive(bool active) = 0;
	virtual void setTransformGizmoFrame(int worldOrLocal) = 0;
	virtual bool transformGizmoFrameIsLocal() const = 0;

	virtual bool hasBackendObjectBranch(const std::string& backendId) const = 0;
	virtual bool getBackendRootWorldMatrix(const std::string& backendId, cloudsim::core::Mat4& outWorld) const = 0;
	virtual bool tryGetBackendModelCenterMm(const std::string& backendId, double& cx, double& cy, double& cz) const = 0;
	virtual std::string resolvePickScopeBackendId(const std::string& backendId) const = 0;
	virtual bool backendSkipsInnerModelCenterRebase(const std::string& backendId) const = 0;

	virtual void setCameraFollowBackendId(const std::string& backendId) = 0;

	virtual void showMeshSectionPlane(const std::string& backendIdUtf8, const double originModelMm[3],
									  const double normalModel[3]) = 0;
	virtual void
	beginMeshSectionPlaneEdit(const std::string& backendIdUtf8, const double originModelMm[3],
							  const double normalModel[3],
							  std::function<void(const double origin[3], const double normal[3])> onChanged) = 0;
	virtual void updateMeshSectionPlanePose(const double originModelMm[3], const double normalModel[3]) = 0;
	virtual void endMeshSectionPlaneEdit() = 0;
	virtual void hideMeshSectionPlane() = 0;
	virtual void setMeshSectionPlanePreviewVisible(bool visible) = 0;
	virtual bool getCameraViewDirectionInBackendModel(const std::string& backendIdUtf8,
													  double outDirModel[3]) const = 0;
};

#endif // ROBOTWIDGET_IROBOTOSGSCENEOPS_H
