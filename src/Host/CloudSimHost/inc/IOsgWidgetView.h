#ifndef CLOUDSIMHOST_IOSGWIDGETVIEW_H
#define CLOUDSIMHOST_IOSGWIDGETVIEW_H

/// @file IOsgWidgetView.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 共享 Host 源依赖的 OsgWidget 窄接口（桌面/Headless 各自实现）

#include "PickTypes.h"

#include <QEvent>
#include <QList>
#include <QMetaObject>
#include <QObject>
#include <QPoint>
#include <QString>
#include <QVector>
#include <functional>
#include <string>
#include <vector>

#include <osg/Matrixd>
#include <osg/Node>
#include <osg/Vec3d>
#include <osg/Vec3f>
#include <osg/Vec4>

#include <RobotOsgUiTypes.h>

class BackendDataBase;
class BackendDataManager;
class MeshBackendData;
class PointCloudBackendData;
class IRobotBackendPoseSink;
struct MeshCapturedPart;

/// 共享库抽取前置：共享源只依赖此接口，不直接 include 真/桩 OsgWidget.h
/// vtable 仅允许在末尾追加新方法，禁止插入或改签名
class IOsgWidgetView
{
public:
	struct AnnotationSnapshot
	{
		QString id;
		QString displayText;
		QString backendId;
		osg::Vec3f localCentered;
		osg::Vec3f worldAnchor{};
		bool hasWorldAnchor = false;
		bool visible = true;
	};

	struct SketchSupportExtraPlane
	{
		osg::Vec3d origin{0, 0, 0};
		osg::Vec3d axisX{1, 0, 0};
		osg::Vec3d axisY{0, 1, 0};
		osg::Vec3d normal{0, 0, 1};
		float halfMm = 40.f;
	};

	using OriginPlanePickedFn = std::function<void(bool ok, int planeIndex)>;
	using SketchPlaneInputHandler = std::function<bool(QObject* watched, QEvent* event)>;

	virtual ~IOsgWidgetView() = default;

	virtual QList<AnnotationSnapshot> annotationSnapshots() const = 0;
	virtual void restoreAnnotations(const QList<AnnotationSnapshot>& snapshots) = 0;
	virtual void setCameraFollowBackendId(std::string backendId) = 0;
	virtual std::string cameraFollowBackendId() const = 0;

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

	virtual void setSelectionActive(bool active) = 0;
	virtual void setMeshLinePickMode(bool enabled) = 0;
	virtual void setMeshFacePickMode(bool enabled) = 0;
	virtual void setPolylinePickMode(bool enabled) = 0;
	virtual void setLabelingClickPickMode(bool enabled, bool meshFace) = 0;
	virtual void setLabelingBrushPickMode(bool enabled, bool meshFace, float radiusPx) = 0;

	virtual bool intersectScreenWithPlaneMm(int screenX, int screenY, const osg::Vec3d& planeOrigin,
											const osg::Vec3d& planeNormal, osg::Vec3d& outHitWorldMm,
											QString* outError = nullptr) const = 0;

	virtual void setSketchLineOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
									  const std::vector<std::size_t>& segmentEndExclusive,
									  const std::vector<osg::Vec4>& segmentColors,
									  const std::vector<float>& segmentWidthsPx = {}) = 0;
	virtual void clearSketchLineOverlay() = 0;
	virtual void setSketchPlaneInputHandler(SketchPlaneInputHandler handler) = 0;
	virtual void clearSketchPlaneInputHandler() = 0;
	virtual void setSketchSupportExtraPlanes(std::vector<SketchSupportExtraPlane> planes) = 0;
	virtual void clearSketchSupportExtraPlanes() = 0;
	virtual void beginOriginPlaneSelection(OriginPlanePickedFn onFinished, float halfSizeMm = 60.f) = 0;
	virtual void cancelOriginPlaneSelection() = 0;
	virtual int resolveSketchSupportOriginIndex(int screenX, int screenY) const = 0;
	virtual QPoint lastMousePos() const = 0;
	virtual void setOriginReferenceVisibility(bool originPoint, bool planeXY, bool planeXZ, bool planeYZ,
												float halfSizeMm = 60.f) = 0;

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

	virtual void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld) = 0;
	virtual void hideMeshElementHighlight() = 0;

	virtual bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
											double& outCz) const = 0;
	virtual bool tryGetBackendPointLocalToWorldMatrix(const std::string& backendId,
													  double outColMajor16[16]) const = 0;
	virtual std::string resolvePickScopeBackendId(const std::string& backendId) const = 0;

	/// 真/桩 OsgWidget 同时实现 IRobotBackendPoseSink；窄接口侧经此桥接
	virtual IRobotBackendPoseSink* asPoseSink() = 0;

	virtual QMetaObject::Connection observeMeshPickCommitted(QObject* ctx,
														   std::function<void(PickResult, int)> handler) = 0;
	virtual QMetaObject::Connection observePolylinePickCommitted(
		QObject* ctx, std::function<void(QVector<float>, QVector<double>, int, int)> handler) = 0;
	virtual QMetaObject::Connection observePolylinePickCanceled(QObject* ctx, std::function<void()> handler) = 0;
	virtual QMetaObject::Connection observeLabelingClickCommitted(QObject* ctx,
																  std::function<void(PickResult)> handler) = 0;
	virtual QMetaObject::Connection observeLabelingBrushStroke(QObject* ctx,
															   std::function<void(QVector<int>)> handler) = 0;
	virtual QMetaObject::Connection observeLabelingBrushFinished(QObject* ctx, std::function<void()> handler) = 0;
	virtual QMetaObject::Connection observeLabelingPickCanceled(QObject* ctx, std::function<void()> handler) = 0;

	/// 打开工程前清空视口导入预览与场景挂载（保留 Document 数据层由调用方处理）
	virtual void clearImportedContent() = 0;
};

#endif // CLOUDSIMHOST_IOSGWIDGETVIEW_H
