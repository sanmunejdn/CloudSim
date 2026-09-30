#ifndef CLOUDSIMHOST_IVIEWPORTSCENEOPS_H
#define CLOUDSIMHOST_IVIEWPORTSCENEOPS_H

/// @file IViewportSceneOps.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口场景装载/捕获/后端视觉与矩阵（IOsgWidgetView 切面）

#include "MeshCapturedPart.h"

#include <QString>
#include <string>
#include <vector>

#include <osg/Matrixd>
#include <osg/Node>
#include <osg/Vec3d>
#include <osg/Vec3f>
#include <osg/Vec4>

class BackendDataBase;
class BackendDataManager;
class MeshBackendData;
class PointCloudBackendData;
class IRobotBackendPoseSink;

/// 导入/加载/暂存/清空、捕获、后端视觉与矩阵、相机聚焦
class IViewportSceneOps
{
public:
	virtual ~IViewportSceneOps() = default;

	virtual bool importModelFile(const QString& filePath, QString* errorMessage = nullptr) = 0;
	virtual bool importPointCloudFile(const QString& filePath, QString* errorMessage = nullptr) = 0;
	virtual bool captureImportedPointCloudBackend(PointCloudBackendData& out, QString* errorMessage = nullptr) = 0;
	virtual bool capturePointCloudBackendFromScene(const std::string& backendId, PointCloudBackendData& out,
												   QString* errorMessage = nullptr) = 0;
	virtual bool captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage = nullptr) = 0;
	virtual bool captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts,
													 QString* errorMessage = nullptr) = 0;
	virtual bool loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage = nullptr,
											   bool resetViewToHome = true) = 0;
	virtual bool loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage = nullptr,
										 bool resetViewToHome = true, bool showWireOutline = true,
										 bool useSceneLighting = true) = 0;
	virtual bool loadBackendFromBackendData(const BackendDataBase& data, QString* errorMessage = nullptr,
											bool resetViewToHome = true, bool showWireOutline = true,
											bool useSceneLighting = true) = 0;
	virtual void clearStagingGeometry() = 0;
	virtual void setStagingMeshPreview(const std::vector<float>& xyzTriangles, const osg::Vec4& rgba) = 0;

	virtual void setSelectedPosition(const osg::Vec3f& position) = 0;
	virtual void setSelectedRotationEulerDeg(const osg::Vec3f& eulerDeg) = 0;
	virtual void setSelectedColor(float r, float g, float b, float a = 1.0f) = 0;

	virtual void applyColorToBackendObject(const std::string& backendId, const osg::Vec4& color) = 0;
	virtual void setBackendObjectVisible(const std::string& backendId, bool visible) = 0;
	virtual void setBackendParent(const std::string& backendId, const std::string& parentBackendId) = 0;
	virtual void setBackendLogicalParent(const std::string& backendId, const std::string& parentBackendId) = 0;
	virtual void removeBackendObjectVisual(const std::string& backendId) = 0;
	virtual bool hasBackendObjectBranch(const std::string& backendId) const = 0;
	virtual osg::Node* backendObjectRootNode(const std::string& backendId) const = 0;
	virtual void syncSelectionFromBackend(const PointCloudBackendData& data) = 0;
	virtual void syncSelectionFromBackend(const MeshBackendData& data) = 0;
	virtual void syncSelectionForBackendId(const std::string& backendId) = 0;
	virtual bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const = 0;
	virtual void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat) = 0;
	virtual bool applyWorldMatrixToOsg(const std::string& backendId, BackendDataManager& mgr) = 0;
	virtual bool syncOuterPatFromBackend(const BackendDataBase& data) = 0;

	virtual bool isTransformGizmoDragging() const = 0;
	virtual bool isTcpDragTeachActive() const = 0;
	virtual std::string activeBackendId() const = 0;
	virtual void requestRedraw() const = 0;
	virtual void focusCameraOnBackend(const std::string& backendId) = 0;
	virtual void focusCameraOnAllVisibleBackends() = 0;
	virtual void orientViewToPlane(const osg::Vec3d& focusMm, const osg::Vec3d& normal, const osg::Vec3d& upHint) = 0;
	virtual void setCameraViewDirection(const osg::Vec3d& eyeDirectionFromCenter, const osg::Vec3d& upHint) = 0;

	virtual bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
											double& outCz) const = 0;
	virtual bool tryGetBackendPointLocalToWorldMatrix(const std::string& backendId,
													  double outColMajor16[16]) const = 0;
	virtual std::string resolvePickScopeBackendId(const std::string& backendId) const = 0;

	/// 真/桩 OsgWidget 同时实现 IRobotBackendPoseSink；窄接口侧经此桥接
	virtual IRobotBackendPoseSink* asPoseSink() = 0;

	/// 打开工程前清空视口导入预览与场景挂载（保留 Document 数据层由调用方处理）
	virtual void clearImportedContent() = 0;
};

#endif // CLOUDSIMHOST_IVIEWPORTSCENEOPS_H
