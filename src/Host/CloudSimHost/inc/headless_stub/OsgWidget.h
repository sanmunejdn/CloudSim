#ifndef CLOUDSIMHOST_OSGWIDGET_H
#define CLOUDSIMHOST_OSGWIDGET_H

/// @file OsgWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Headless 桩：与桌面 OsgWidget 同头卫，避免双定义；满足 Host/PluginHost 编译

#include "cloudsim_host_global.h"

#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "../../OsgWidgetCore/inc/RobotOsgUiTypes.h"
#include "CoreTypes.h"
#include "IOsgWidgetView.h"
#include "IRobotBackendPoseSink.h"

#include <QEvent>
#include <QList>
#include <QMetaType>
#include <QObject>
#include <QPoint>
#include <QString>
#include <QVector>
#include <QWidget>
#include <functional>
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

struct MeshCapturedPart;

Q_DECLARE_METATYPE(PickResult)

/// Web Headless：不继承 OsgScene，方法均为空操作
class CLOUDSIM_HOST_EXPORT OsgWidget : public QWidget, public IRobotBackendPoseSink, public IOsgWidgetView
{
	Q_OBJECT
public:
	using AnnotationSnapshot = IOsgWidgetView::AnnotationSnapshot;

	explicit OsgWidget(QWidget* parent = nullptr) : QWidget(parent) {}

	/// 与桌面同签名，供共享 DocumentHost 编译期通过（Headless 运行时不建 OSG）
	void setPoseSyncBackendManager(BackendDataManager* mgr) { (void)mgr; }
	using VisualSyncMarkDirtyFn = std::function<void(const std::string&, std::uint32_t)>;
	void setVisualSyncMarkDirty(VisualSyncMarkDirtyFn fn) { (void)fn; }

	QList<AnnotationSnapshot> annotationSnapshots() const override { return {}; }
	void restoreAnnotations(const QList<AnnotationSnapshot>& snapshots) override { (void)snapshots; }
	void setCameraFollowBackendId(std::string backendId) override { (void)backendId; }
	std::string cameraFollowBackendId() const override { return {}; }

	void setSelectedPosition(const osg::Vec3f& position) override { (void)position; }
	void setSelectedRotationEulerDeg(const osg::Vec3f& eulerDeg) override { (void)eulerDeg; }
	void setSelectedColor(float r, float g, float b, float a = 1.0f) override
	{
		(void)r;
		(void)g;
		(void)b;
		(void)a;
	}

	bool importModelFile(const QString& filePath, QString* errorMessage = nullptr) override
	{
		(void)filePath;
		(void)errorMessage;
		return false;
	}
	bool importPointCloudFile(const QString& filePath, QString* errorMessage = nullptr) override
	{
		(void)filePath;
		(void)errorMessage;
		return false;
	}
	bool captureImportedPointCloudBackend(PointCloudBackendData& out, QString* errorMessage = nullptr) override
	{
		(void)out;
		(void)errorMessage;
		return false;
	}
	bool capturePointCloudBackendFromScene(const std::string& backendId, PointCloudBackendData& out,
										   QString* errorMessage = nullptr) override
	{
		(void)backendId;
		(void)out;
		(void)errorMessage;
		return false;
	}
	bool captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage = nullptr) override
	{
		(void)out;
		(void)errorMessage;
		return false;
	}
	bool captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts, QString* errorMessage = nullptr) override
	{
		(void)outParts;
		(void)errorMessage;
		return false;
	}

	bool loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage = nullptr,
									   bool resetViewToHome = true) override
	{
		(void)data;
		(void)errorMessage;
		(void)resetViewToHome;
		return false;
	}
	bool loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage = nullptr,
								 bool resetViewToHome = true, bool showWireOutline = true, bool useSceneLighting = true) override
	{
		(void)data;
		(void)errorMessage;
		(void)resetViewToHome;
		(void)showWireOutline;
		(void)useSceneLighting;
		return false;
	}
	bool loadBackendFromBackendData(const BackendDataBase& data, QString* errorMessage = nullptr,
									bool resetViewToHome = true, bool showWireOutline = true,
									bool useSceneLighting = true) override
	{
		(void)data;
		(void)errorMessage;
		(void)resetViewToHome;
		(void)showWireOutline;
		(void)useSceneLighting;
		return false;
	}

	void clearStagingGeometry()  override{}
	void setStagingMeshPreview(const std::vector<float>& xyzTriangles, const osg::Vec4& rgba) override
	{
		(void)xyzTriangles;
		(void)rgba;
	}

	void setSelectionActive(bool active)  override{ (void)active; }
	void setPolylinePickMode(bool enabled)  override{ (void)enabled; }
	bool polylinePickMode() const { return false; }
	void setMeshLinePickMode(bool enabled)  override{ (void)enabled; }
	void setMeshFacePickMode(bool enabled)  override{ (void)enabled; }
	void setLabelingClickPickMode(bool enabled, bool meshFace) override
	{
		(void)enabled;
		(void)meshFace;
	}
	void setLabelingBrushPickMode(bool enabled, bool meshFace, float radiusPx) override
	{
		(void)enabled;
		(void)meshFace;
		(void)radiusPx;
	}

	using OriginPlanePickedFn = std::function<void(bool ok, int planeIndex)>;
	void beginOriginPlaneSelection(OriginPlanePickedFn onFinished, float halfSizeMm = 60.f) override
	{
		(void)onFinished;
		(void)halfSizeMm;
	}
	void cancelOriginPlaneSelection()  override{}

	using SketchSupportExtraPlane = IOsgWidgetView::SketchSupportExtraPlane;
	void setSketchSupportExtraPlanes(std::vector<SketchSupportExtraPlane> planes) override { (void)planes; }
	void clearSketchSupportExtraPlanes()  override{}
	int resolveSketchSupportOriginIndex(int screenX, int screenY) const override
	{
		(void)screenX;
		(void)screenY;
		return -1;
	}
	QPoint lastMousePos() const  override{ return {}; }

	using SketchPlaneInputHandler = std::function<bool(QObject* watched, QEvent* event)>;
	void setSketchPlaneInputHandler(SketchPlaneInputHandler handler)  override{ (void)handler; }
	void clearSketchPlaneInputHandler()  override{}

	void setSketchLineOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
							  const std::vector<std::size_t>& segmentEndExclusive,
							  const std::vector<osg::Vec4>& segmentColors,
							  const std::vector<float>& segmentWidthsPx = {}) override
	{
		(void)points;
		(void)segmentEndExclusive;
		(void)segmentColors;
		(void)segmentWidthsPx;
	}
	void clearSketchLineOverlay()  override{}

	bool intersectScreenWithPlaneMm(int screenX, int screenY, const osg::Vec3d& planeOrigin,
									const osg::Vec3d& planeNormal, osg::Vec3d& outHitWorldMm,
									QString* outError = nullptr) const override
	{
		(void)screenX;
		(void)screenY;
		(void)planeOrigin;
		(void)planeNormal;
		(void)outHitWorldMm;
		(void)outError;
		return false;
	}

	void setOriginReferenceVisibility(bool originPoint, bool planeXY, bool planeXZ, bool planeYZ,
									  float halfSizeMm = 60.f) override
	{
		(void)originPoint;
		(void)planeXY;
		(void)planeXZ;
		(void)planeYZ;
		(void)halfSizeMm;
	}

	void applyColorToBackendObject(const std::string& backendId, const osg::Vec4& color) override
	{
		(void)backendId;
		(void)color;
	}
	void setBackendObjectVisible(const std::string& backendId, bool visible) override
	{
		(void)backendId;
		(void)visible;
	}
	void setBackendParent(const std::string& backendId, const std::string& parentBackendId) override
	{
		(void)backendId;
		(void)parentBackendId;
	}
	void setBackendLogicalParent(const std::string& backendId, const std::string& parentBackendId) override
	{
		(void)backendId;
		(void)parentBackendId;
	}
	void removeBackendObjectVisual(const std::string& backendId)  override{ (void)backendId; }
	bool hasBackendObjectBranch(const std::string& backendId) const override
	{
		(void)backendId;
		return false;
	}
	osg::Node* backendObjectRootNode(const std::string& backendId) const override
	{
		(void)backendId;
		return nullptr;
	}

	void syncSelectionFromBackend(const PointCloudBackendData& data)  override{ (void)data; }
	void syncSelectionFromBackend(const MeshBackendData& data)  override{ (void)data; }
	void syncSelectionForBackendId(const std::string& backendId)  override{ (void)backendId; }
	void setPickVisualAlias(const std::string& logicalBackendId, const std::string& visualBackendId)
	{
		(void)logicalBackendId;
		(void)visualBackendId;
	}
	bool syncOuterPatFromBackend(const BackendDataBase& data) override
	{
		(void)data;
		return false;
	}

	void requestRedraw() const  override{}
	void focusCameraOnBackend(const std::string& backendId)  override{ (void)backendId; }
	void focusCameraOnAllVisibleBackends()  override{}
	void orientViewToPlane(const osg::Vec3d& focusMm, const osg::Vec3d& normal, const osg::Vec3d& upHint) override
	{
		(void)focusMm;
		(void)normal;
		(void)upHint;
	}
	void setCameraViewDirection(const osg::Vec3d& eyeDirectionFromCenter, const osg::Vec3d& upHint) override
	{
		(void)eyeDirectionFromCenter;
		(void)upHint;
	}
	bool getCameraViewDirectionInBackendModel(const std::string& backendIdUtf8, double outDirModel[3]) const
	{
		(void)backendIdUtf8;
		(void)outDirModel;
		return false;
	}

	void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld)  override{ (void)vertsWorld; }
	void showMeshFaceHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld, const osg::Vec3f& cWorld)
	{
		(void)aWorld;
		(void)bWorld;
		(void)cWorld;
	}
	void hideMeshElementHighlight()  override{}
	void showPinnedMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld){ (void)vertsWorld; }
	void hidePinnedMeshFaceHighlight(){}
	void setCrossObjectMeshPick(bool){}
	void showMeshFittedSurfacePreview(const std::vector<osg::Vec3f>& triangleVertsWorld){ (void)triangleVertsWorld; }
	void clearMeshFittedSurfacePreview(){}

	void setRawTrajectoryOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
								 const RobotOsgUi::RawTrajectoryPreviewOptions& options = {})
	{
		(void)points;
		(void)options;
	}
	void setRawTrajectoryOverlayAxisComponents(bool showX, bool showY, bool showZ)
	{
		(void)showX;
		(void)showY;
		(void)showZ;
	}
	void setReachableWorkspaceOverlay(const RobotOsgUi::ReachableWorkspaceOverlay& overlay){ (void)overlay; }
	void clearReachableWorkspaceOverlay(){}
	void setPlaybackCursorOverlay(const RobotOsgUi::PlaybackCursorOverlay& cursor){ (void)cursor; }
	void clearPlaybackCursorOverlay(){}
	void setWaypointIndexLabels(const std::vector<RobotOsgUi::WaypointIndexLabel>& labels){ (void)labels; }
	void clearWaypointIndexLabels(){}
	void setInstructionWaypointPickMode(bool enabled){ (void)enabled; }
	bool instructionWaypointPickMode() const{ return false; }
	void
	setInstructionWaypointPickCallbacks(std::function<void(const std::string& instructionId, bool isArcVia)> onPicked,
										std::function<void()> onCanceled)
	{
		(void)onPicked;
		(void)onCanceled;
	}

	void showMeshSectionPlane(const std::string& backendIdUtf8, const double originModelMm[3],
							  const double normalModel[3])
	{
		(void)backendIdUtf8;
		(void)originModelMm;
		(void)normalModel;
	}
	void beginMeshSectionPlaneEdit(const std::string& backendIdUtf8, const double originModelMm[3],
								   const double normalModel[3],
								   std::function<void(const double origin[3], const double normal[3])> onChanged)
	{
		(void)backendIdUtf8;
		(void)originModelMm;
		(void)normalModel;
		(void)onChanged;
	}
	void updateMeshSectionPlanePose(const double originModelMm[3], const double normalModel[3])
	{
		(void)originModelMm;
		(void)normalModel;
	}
	void endMeshSectionPlaneEdit(){}
	void hideMeshSectionPlane(){}
	void setMeshSectionPlanePreviewVisible(bool visible){ (void)visible; }

	bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const override
	{
		(void)backendId;
		(void)outWorld;
		return false;
	}
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat) override
	{
		(void)backendId;
		(void)worldMat;
	}
	bool getBackendRootWorldMatrix(const std::string& backendId, cloudsim::core::Mat4& outWorld) const
	{
		(void)backendId;
		(void)outWorld;
		return false;
	}
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId,
											const cloudsim::core::Mat4& worldColumnMajor)
	{
		(void)backendId;
		(void)worldColumnMajor;
	}
	bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
									double& outCz) const override
	{
		(void)backendId;
		(void)outCx;
		(void)outCy;
		(void)outCz;
		return false;
	}
	bool tryGetBackendPointLocalToWorldMatrix(const std::string& backendId, double outColMajor16[16]) const override
	{
		(void)backendId;
		(void)outColMajor16;
		return false;
	}
	std::string resolvePickScopeBackendId(const std::string& backendId) const  override{ return backendId; }
	bool isTransformGizmoDragging() const  override{ return false; }
	bool isTcpDragTeachActive() const  override{ return false; }
	std::string activeBackendId() const  override{ return {}; }
	bool applyWorldMatrixToOsg(const std::string& backendId, BackendDataManager& mgr) override
	{
		(void)backendId;
		(void)mgr;
		return false;
	}
	IRobotBackendPoseSink* asPoseSink() override { return this; }

	QMetaObject::Connection observeMeshPickCommitted(QObject* ctx,
													 std::function<void(PickResult, int)> handler) override
	{
		(void)ctx;
		(void)handler;
		return {};
	}
	QMetaObject::Connection observePolylinePickCommitted(
		QObject* ctx, std::function<void(QVector<float>, QVector<double>, int, int)> handler) override
	{
		(void)ctx;
		(void)handler;
		return {};
	}
	QMetaObject::Connection observePolylinePickCanceled(QObject* ctx, std::function<void()> handler) override
	{
		(void)ctx;
		(void)handler;
		return {};
	}
	QMetaObject::Connection observeLabelingClickCommitted(QObject* ctx,
														  std::function<void(PickResult)> handler) override
	{
		(void)ctx;
		(void)handler;
		return {};
	}
	QMetaObject::Connection observeLabelingBrushStroke(QObject* ctx,
													   std::function<void(QVector<int>)> handler) override
	{
		(void)ctx;
		(void)handler;
		return {};
	}
	QMetaObject::Connection observeLabelingBrushFinished(QObject* ctx, std::function<void()> handler) override
	{
		(void)ctx;
		(void)handler;
		return {};
	}
	QMetaObject::Connection observeLabelingPickCanceled(QObject* ctx, std::function<void()> handler) override
	{
		(void)ctx;
		(void)handler;
		return {};
	}

	void clearImportedContent() override {}

signals:
	void polylinePickCommitted(QVector<float> polylineScreenXy, QVector<double> mvpMatrix, int viewportWidth,
							   int viewportHeight);
	void polylinePickCanceled();
	void meshPickCommitted(PickResult pick, int pickKind);
	void labelingClickCommitted(PickResult pick);
	void labelingBrushStroke(QVector<int> indices);
	void labelingBrushFinished();
	void labelingPickCanceled();
};

#endif // CLOUDSIMHOST_OSGWIDGET_H
