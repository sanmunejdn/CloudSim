#ifndef CLOUDSIMHOST_OSGWIDGET_H
#define CLOUDSIMHOST_OSGWIDGET_H

/// @file OsgWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 三维视图控件（Qt + \c OsgScene）

#include "widget_global.h"

#include "OsgScene.h"
#include "PickTypes.h"
#include "RobotOsgUiTypes.h"
#include "GraphicsWindowQt1.h"
#include "IOsgWidgetView.h"
#include "IRobotBackendPoseSink.h"
#include "IViewportSceneSignals.h"
#include "ViewportInteraction/IViewportInteractionHost.h"

#include <QElapsedTimer>
#include <QEvent>
#include <QList>
#include <QMetaType>
#include <QPoint>
#include <QString>
#include <QTimer>
#include <QVector>
#include <QWidget>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <RigidTransform.h>
#include <osg/Array>
#include <osg/AutoTransform>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/PositionAttitudeTransform>
#include <osg/Quat>
#include <osg/Vec3>
#include <osg/Vec3f>
#include <osg/Vec4>
#include <osg/ref_ptr>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>

Q_DECLARE_METATYPE(PickResult)

namespace osg
{
class Group;
class Node;
class Geometry;
class Material;
} // namespace osg

namespace osgViewer
{
class Viewer;
}

class QWidgetViewer;
class SelectionOperation;
class PointPickOperation;
class ObjectTransformOperation;
class RobotTcpDragTeachOperation;
class MeshEdgeFacePickOperation;
class MeshSectionPlaneEditOperation;
class BackendDataBase;
class BackendDataManager;
class PointCloudBackendData;
class MeshBackendData;
class OsgWidgetImportController;
class OsgWidgetBackendLoadController;
class OsgWidgetCaptureController;
class OsgWidgetPickAnnotationController;
class ViewportInteractionController;
class IViewportPickEngine;
class IInteractionSession;
class OsgWidgetColorController;
class OsgWidgetTransformHierarchyController;
struct MeshCapturedPart;

/// 三维视图控件（Qt + \c OsgScene）
///
/// Viewer/相机、后端导入显示、拾取标注；对象变换与 TCP 示教罗盘（后者挂场景 overlay，
/// 位姿经 syncTcpTeachWorldPatFromMount 与 mount PAT 对齐）
class OSG_WIDGET_API OsgWidget : public QWidget,
								 public IRobotBackendPoseSink,
								 public OsgScene,
								 public IOsgWidgetView,
								 public IViewportSceneSignals,
								 public IViewportInteractionHost
{
	Q_OBJECT
public:
	using DragAxis = OsgScene::DragAxis;
	friend class OsgWidgetImportController;
	friend class OsgWidgetBackendLoadController;
	friend class OsgWidgetCaptureController;
	friend class OsgWidgetPickAnnotationController;
	friend class OsgWidgetColorController;
	friend class OsgWidgetTransformHierarchyController;

	using AnnotationEntry = OsgScene::AnnotationEntry;

public:
	using AnnotationSnapshot = IOsgWidgetView::AnnotationSnapshot;

public:
	explicit OsgWidget(QWidget* parent = nullptr);
	~OsgWidget() override;
	bool importModelFile(const QString& filePath, QString* errorMessage = nullptr) override;
	bool importPointCloudFile(const QString& filePath, QString* errorMessage = nullptr) override;
	bool captureImportedPointCloudBackend(PointCloudBackendData& out, QString* errorMessage = nullptr) override;
	bool capturePointCloudBackendFromScene(const std::string& backendId, PointCloudBackendData& out,
										   QString* errorMessage = nullptr) override;
	bool captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage = nullptr) override;
	bool captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts, QString* errorMessage = nullptr) override;
	bool captureViewportPng(QByteArray& outPng, QString* errorMessage = nullptr, int maxWidth = 768,
							int maxHeight = 768);
	bool loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage = nullptr,
									   bool resetViewToHome = true) override;
	bool loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage = nullptr,
								 bool resetViewToHome = true, bool showWireOutline = true,
								 bool useSceneLighting = true) override;
	bool loadBackendFromBackendData(const BackendDataBase& data, QString* errorMessage = nullptr,
									bool resetViewToHome = true, bool showWireOutline = true,
									bool useSceneLighting = true) override;
	/// 受光网格后端（如 URDF 连杆）；改色时保留光照材质
	bool isBackendMeshLit(const std::string& backendId) const;
	void clearImportedContent() override;
	/// 仅清导入预览，保留已注册后端可视
	void clearStagingGeometry() override;
	/// 半透明三角网预览（xyz 交错，9 floats/三角）
	void setStagingMeshPreview(const std::vector<float>& xyzTriangles, const osg::Vec4& rgba) override;
	/// 草图折线 overlay：关深度测试，避免贴面被遮挡；rgba 按段着色
	void setSketchLineOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
							  const std::vector<std::size_t>& segmentEndExclusive,
							  const std::vector<osg::Vec4>& segmentColors,
							  const std::vector<float>& segmentWidthsPx = {}) override;
	void clearSketchLineOverlay() override;
	void setSelectionActive(bool active) override;
	void setObjectSelectionMode(bool enabled);
	bool objectSelectionMode() const override;
	/// 物体变换罗盘：物体系沿当前罗盘轴（与模型姿态一致）；世界系沿世界 X/Y/Z。
	using TransformGizmoFrame = OsgScene::TransformGizmoFrame;
	void setTransformGizmoFrame(TransformGizmoFrame frame);
	TransformGizmoFrame transformGizmoFrame() const override { return m_transformGizmoFrame; }
	void setPointPickMode(bool enabled);
	bool pointPickMode() const override;
	void setPolylinePickMode(bool enabled) override;
	bool polylinePickMode() const override;
	void updatePolylinePickOverlay(const std::vector<QPoint>& vertices, const QPoint* cursorPos) override;
	void commitPolylinePick(const std::vector<QPoint>& vertices) override;
	void clearPolylinePickOverlay();
	void setMeshLinePickMode(bool enabled) override;
	bool meshLinePickMode() const override;
	void setMeshFacePickMode(bool enabled) override;
	bool meshFacePickMode() const override;

	/// 屏幕点 → 世界射线与平面求交（逻辑像素，与拾取一致）
	bool intersectScreenWithPlaneMm(int screenX, int screenY, const osg::Vec3d& planeOrigin,
									const osg::Vec3d& planeNormal, osg::Vec3d& outHitWorldMm,
									QString* outError = nullptr) const override;

	/// 草图编辑：消费视口鼠标/键，抑制轨道（handler 返回 true 表示已处理）
	using SketchPlaneInputHandler = std::function<bool(QObject* watched, QEvent* event)>;
	void setSketchPlaneInputHandler(SketchPlaneInputHandler handler) override;
	void clearSketchPlaneInputHandler() override;

	/// 新建草图：显示 XY/XZ/YZ 半透明基准面，点击回调 index（0/1/2）；取消 ok=false
	/// index>=100 表示 setSketchSupportExtraPlanes 中的用户面（100+i）
	using OriginPlanePickedFn = std::function<void(bool ok, int planeIndex)>;
	void beginOriginPlaneSelection(OriginPlanePickedFn onFinished, float halfSizeMm = 60.f) override;
	void cancelOriginPlaneSelection() override;
	bool isOriginPlaneSelectionActive() const { return m_originPlanePickActive; }

	using SketchSupportExtraPlane = IOsgWidgetView::SketchSupportExtraPlane;
	void setSketchSupportExtraPlanes(std::vector<SketchSupportExtraPlane> planes) override;
	void clearSketchSupportExtraPlanes() override;
	/// 命中用户候选面；outDist2 为到相机距离平方
	int hitTestSupportExtra(int screenX, int screenY, double* outDist2 = nullptr) const;
	/// 基面/用户面与模型面更近者胜；胜出基面 0..2，用户面 100+i，否则 -1 交给网格
	int resolveSketchSupportOriginIndex(int screenX, int screenY) const override;
	QPoint lastMousePos() const override { return m_lastMousePos; }

	/// 持久显示世界原点三轴 + 三基准面（拾取会话期间自动隐藏，结束后按标志恢复）
	void setOriginReferenceVisibility(bool originPoint, bool planeXY, bool planeXZ, bool planeYZ,
									  float halfSizeMm = 60.f) override;

	void setLabelingClickPickMode(bool enabled, bool meshFace) override;
	void setLabelingBrushPickMode(bool enabled, bool meshFace, float radiusPx) override;
	PickResult queryPick(const PickQuery& query) override;
	ViewportInteractionController* interactionController() { return m_interactionController.get(); }
	IViewportPickEngine* pickEngine() override;
	void beginInteractionSession(std::shared_ptr<IInteractionSession> session);
	void endInteractionSession(bool cancel = true);
	bool hasInteractionSession() const;
	void setupInteractionController();
	osg::Vec3f selectedPosition() const override;
	void setSelectedPosition(const osg::Vec3f& position) override;
	osg::Vec3f selectedRotationEulerDeg() const override;
	void setSelectedRotationEulerDeg(const osg::Vec3f& eulerDeg) override;
	void setSelectedColor(float r, float g, float b, float a = 1.0f) override;
	/// 按 backendId 刷新场景颜色，不发 selectedObjectColorChanged
	void applyColorToBackendObject(const std::string& backendId, const osg::Vec4& color) override;
	QString pointCloudPluginReport() const;
	/// 按后端树行显隐 OSG 分支
	void setBackendObjectVisible(const std::string& backendId, bool visible) override;
	/// 同步逻辑父子链（顶层标注跟踪等）
	void setBackendParent(const std::string& backendId, const std::string& parentBackendId) override;
	void setBackendLogicalParent(const std::string& backendId, const std::string& parentBackendId) override;
	void removeBackendObjectVisual(const std::string& backendId) override;
	/// 后端几何已在场景中（非导入预览）
	bool hasBackendObjectBranch(const std::string& backendId) const override;
	/// 增量几何更新用：已挂载后端的 outer 根节点，未挂载返回 nullptr
	osg::Node* backendObjectRootNode(const std::string& backendId) const override
	{
		const auto it = m_backendObjectRoots.find(backendId);
		return (it != m_backendObjectRoots.end() && it->second.valid()) ? it->second.get() : nullptr;
	}
	/// 不重载几何同步 gizmo/拾取缓存，保留标注
	void syncSelectionFromBackend(const PointCloudBackendData& data) override;
	void syncSelectionFromBackend(const MeshBackendData& data) override;
	/// 无自有几何的后端行也可选中（如装配父节点）
	void syncSelectionForBackendId(const std::string& backendId) override;
	/// 逻辑节点映射到已挂载 OSG 分支（装配子件共用父级 visual）
	void setPickVisualAlias(const std::string& logicalBackendId, const std::string& visualBackendId);
	bool backendSkipsInnerModelCenterRebase(const std::string& backendId) const;
	bool setAnnotationVisible(const QString& annotationId, bool visible);
	bool removeAnnotation(const QString& annotationId);
	void clearAllAnnotations();
	QList<AnnotationSnapshot> annotationSnapshots() const override;
	void restoreAnnotations(const QList<AnnotationSnapshot>& snapshots) override;
	/// 随 Qt 深/浅主题设置 OSG 背景色
	void setViewerBackgroundForDarkUi(bool dark);
	/// GL 视口控件，供浮动工具栏等 overlay 挂载
	QWidget* viewportWidget() const { return m_glWidget; }
	/// 线框/实体切换
	void setWireframeMode(bool enabled) override;
	bool wireframeMode() const { return m_wireframeMode; }
	QWidget* viewportOverlayHostWidget() const override;
	/// 至少一个后端有几何或存在导入预览
	bool hasImportedContent() const;
	/// Viewer 根节点（\c setSceneData），供场景树 UI/调试
	const osg::Group* sceneGraphRoot() const { return m_root.get(); }
	/// 绕 \a pivotWorld 刚体旋转各后端根（左乘姿态）
	void applyRigidRotationAboutWorldPivot(const std::vector<std::string>& backendIds, const osg::Vec3f& pivotWorld,
										   const osg::Quat& deltaRotation);
	osg::Vec3f averageBackendRootPositionWorld(const std::vector<std::string>& backendIds) const;
	/// \a backendId 外层 PAT 世界矩阵（含父链）；OSG 形态供 OsgWidget 内部使用
	bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const override;
	/// 设外层 PAT 世界矩阵为 \a worldMat（含父链）
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat) override;
	/// 单轨：Data worldMatrix → OSG outer local（逻辑父 world 优先读 Data）
	bool applyWorldMatrixToOsg(const std::string& backendId, BackendDataManager& mgr) override;
	bool syncTransformFromBackendData(const BackendDataBase& data, BackendDataManager& mgr);
	void setPoseSyncBackendManager(BackendDataManager* mgr);
	using VisualSyncMarkDirtyFn = std::function<void(const std::string&, std::uint32_t)>;
	void setVisualSyncMarkDirty(VisualSyncMarkDirtyFn fn);
	/// IRobotBackendPoseSink：列主序 Mat4
	bool getBackendRootWorldMatrix(const std::string& backendId, cloudsim::core::Mat4& outWorld) const override;
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId,
											const cloudsim::core::Mat4& worldColumnMajor) override;
	bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
									double& outCz) const override;
	/// 将 target 内层去心质心改为与 source 一致（两者均需 skipInnerModelCenterRebase=false）
	bool alignBackendInnerModelCenterFrom(const std::string& targetBackendId, const std::string& sourceBackendId);
	void syncRobotMeshBackendPoseAfterKinematics(const BackendDataBase& mesh) override;

	/// 添加层级机器人场景，返回后端 id
	/// @param robotAssembly 机器人场景根（UrdfRobotLoader::buildHierarchicalRobotScene）
	/// @param displayName 后端树显示名
	/// @return 机器人后端 id，失败为空
	QString addHierarchicalRobotScene(osg::Group* robotAssembly, const QString& displayName);

	/// 移除层级化机器人场景图。
	void removeHierarchicalRobotScene(const QString& backendId);
	void setInstructionPoseAxes(const std::vector<RobotOsgUi::InstructionPoseAxis>& axes);
	void clearInstructionPoseAxes();
	void setRawTrajectoryOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
								 const std::vector<std::size_t>& segmentEndExclusive = {});
	void clearRawTrajectoryOverlay();
	void setRawTrajectoryOverlayFrames(const std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& frames);
	void setRawTrajectoryOverlayAxisComponents(bool showX, bool showY, bool showZ);
	void clearRawTrajectoryOverlayFrames();
	void setRobotFrameOverlays(const RobotOsgUi::RobotFrameOverlayUpdate& update);
	void clearRobotFrameOverlays(const std::string& robotRootBackendId);
	void setFeatureCatalogOverlay(const std::vector<RobotOsgUi::FeatureCatalogOverlayItem>& items);
	void clearFeatureCatalogOverlay();
	void setReachableWorkspaceOverlay(const RobotOsgUi::ReachableWorkspaceOverlay& overlay);
	void clearReachableWorkspaceOverlay();
	void setPlaybackCursorOverlay(const RobotOsgUi::PlaybackCursorOverlay& cursor);
	void clearPlaybackCursorOverlay();
	void setWaypointIndexLabels(const std::vector<RobotOsgUi::WaypointIndexLabel>& labels);
	void clearWaypointIndexLabels();
	void setInstructionWaypointPickMode(bool enabled);
	bool instructionWaypointPickMode() const { return m_instructionWaypointPickMode; }
	void
	setInstructionWaypointPickCallbacks(std::function<void(const std::string& instructionId, bool isArcVia)> onPicked,
										std::function<void()> onCanceled);

	/// TCP 末端拖动示教：场景 overlay 罗盘，拖动发位姿信号（不写指令）
	bool isTcpDragTeachActive() const override { return m_tcpTeachActive; }
	bool isTcpDragGizmoDragging() const { return m_tcpTeachDragging || m_tcpTeachRotating; }
	/// 进入 TCP 示教
	/// @param mountBackendId TCP 挂载后端 PAT id
	/// @param T_base_target 机器人基座系目标 TCP 位姿
	/// @param modelDiagonalMm 参考模型对角线 mm，罗盘屏幕缩放
	/// @param resolveRobotBaseWorld 可选，解析基座世界矩阵；基座/工具坐标换算
	/// @param toolLocalOnFlange 非空则按法兰局部工具矩阵放置 TCP
	void beginTcpDragTeach(const std::string& mountBackendId, const engine::RigidTransform& T_base_target,
						   float modelDiagonalMm = 1000.0f,
						   std::function<bool(osg::Matrixd& outRobotBaseWorld)> resolveRobotBaseWorld = nullptr,
						   const osg::Matrixd* toolLocalOnFlange = nullptr);
	void endTcpDragTeach();
	/// 外部同步示教目标（IK/属性面板）
	/// @param T_base_target 基座系目标位姿
	/// @param syncTargetInBase 为 true 时刷新内部 \c m_tcpTeachTargetInBase
	void updateTcpDragTeachFromTarget(const engine::RigidTransform& T_base_target, bool syncTargetInBase = true);
	/// 示教中更新法兰局部工具矩阵（如切换当前工具坐标系）
	/// @param toolLocalOnFlange 法兰系下工具位姿
	void updateTcpDragTeachToolLocalOnFlange(const osg::Matrixd& toolLocalOnFlange);
	engine::RigidTransform tcpDragTeachTargetInBase() const { return m_tcpTeachTargetInBase; }

	/// Mesh 轨迹截面编辑（模型系原点+法向，罗盘拖动）
	bool isMeshSectionPlaneEditActive() const;
	void showMeshSectionPlane(const std::string& backendIdUtf8, const double originModelMm[3],
							  const double normalModel[3]);
	void beginMeshSectionPlaneEdit(const std::string& backendIdUtf8, const double originModelMm[3],
								   const double normalModel[3],
								   std::function<void(const double origin[3], const double normal[3])> onChanged);
	void updateMeshSectionPlanePose(const double originModelMm[3], const double normalModel[3]);
	void endMeshSectionPlaneEdit();
	void hideMeshSectionPlane();
	void setMeshSectionPlanePreviewVisible(bool visible);
	bool getCameraViewDirectionWorld(double outDirUnit[3]) const;
	bool getCameraViewDirectionInBackendModel(const std::string& backendIdUtf8, double outDirModel[3]) const;

	/// 帧定时器回调（如跟随求解）
	void setPerFrameHook(std::function<void(OsgWidget*)> fn);
	/// per-link 机器人对象 gizmo：intercept 为 true 时跳过逻辑子孙传播
	using RobotObjectGizmoSyncFn = std::function<bool(const ObjectGizmoFrame&, bool dragging)>;
	using RobotObjectGizmoFkRefreshFn = std::function<void(const ObjectGizmoFrame&, bool dragging)>;
	void setRobotObjectGizmoSyncHook(RobotObjectGizmoSyncFn fn);
	void setRobotObjectGizmoFkRefreshHook(RobotObjectGizmoFkRefreshFn fn);
	/// 写活动外层 PAT；per-link 机器人走 FK 钩子而非逻辑父子传播
	void syncActiveBackendRootFromObjectFrame(const ObjectGizmoFrame& cur, bool dragging) override;
	/// 对象 gizmo 拖拽中，跳过对该选中跟随者的位姿覆写
	bool isTransformGizmoDragging() const override;
	/// 按缓存质心将 \a data 位姿写到外层 PAT
	bool syncOuterPatFromBackend(const BackendDataBase& data) override;
	/// 非拖拽时将 ObjectGizmoFrame 同步到活动后端根
	void syncActiveBackendRootFromSelectedTransform() override;
	/// OSG 位姿写回后端（跟随求解前）
	bool writeActiveBackendPoseFromOsg(BackendDataBase& data);
	/// 轨道相机中心跟随此后端世界原点（空则关闭）
	void setCameraFollowBackendId(std::string backendId) override;
	void clearCameraFollowBackendId();
	std::string cameraFollowBackendId() const override { return m_cameraFollowBackendId; }

	void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld) override
	{
		OsgScene::showMeshFaceHighlight(vertsWorld);
	}
	void hideMeshElementHighlight() override { OsgScene::hideMeshElementHighlight(); }

	using OsgScene::showMeshFittedSurfacePreview;

	std::string activeBackendId() const override { return OsgScene::activeBackendId(); }
	void requestRedraw() const override { OsgScene::requestRedraw(); }
	void focusCameraOnBackend(const std::string& backendId) override { OsgScene::focusCameraOnBackend(backendId); }
	void focusCameraOnAllVisibleBackends() override { OsgScene::focusCameraOnAllVisibleBackends(); }
	void orientViewToPlane(const osg::Vec3d& focusMm, const osg::Vec3d& normal, const osg::Vec3d& upHint) override
	{
		OsgScene::orientViewToPlane(focusMm, normal, upHint);
	}
	void setCameraViewDirection(const osg::Vec3d& eyeDirectionFromCenter, const osg::Vec3d& upHint) override
	{
		OsgScene::setCameraViewDirection(eyeDirectionFromCenter, upHint);
	}
	std::string resolvePickScopeBackendId(const std::string& backendId) const override
	{
		return OsgScene::resolvePickScopeBackendId(backendId);
	}
	bool tryGetBackendPointLocalToWorldMatrix(const std::string& backendId, double outColMajor16[16]) const override
	{
		return OsgScene::tryGetBackendPointLocalToWorldMatrix(backendId, outColMajor16);
	}
	IRobotBackendPoseSink* asPoseSink() override { return this; }

	QMetaObject::Connection observeMeshPickCommitted(QObject* ctx,
													 std::function<void(PickResult, int)> handler) override
	{
		return QObject::connect(this, &OsgWidget::meshPickCommitted, ctx, std::move(handler));
	}
	QMetaObject::Connection observePolylinePickCommitted(
		QObject* ctx, std::function<void(QVector<float>, QVector<double>, int, int)> handler) override
	{
		return QObject::connect(this, &OsgWidget::polylinePickCommitted, ctx, std::move(handler));
	}
	QMetaObject::Connection observePolylinePickCanceled(QObject* ctx, std::function<void()> handler) override
	{
		return QObject::connect(this, &OsgWidget::polylinePickCanceled, ctx, std::move(handler));
	}
	QMetaObject::Connection observeLabelingClickCommitted(QObject* ctx,
														  std::function<void(PickResult)> handler) override
	{
		return QObject::connect(this, &OsgWidget::labelingClickCommitted, ctx, std::move(handler));
	}
	QMetaObject::Connection observeLabelingBrushStroke(QObject* ctx,
													   std::function<void(QVector<int>)> handler) override
	{
		return QObject::connect(this, &OsgWidget::labelingBrushStroke, ctx, std::move(handler));
	}
	QMetaObject::Connection observeLabelingBrushFinished(QObject* ctx, std::function<void()> handler) override
	{
		return QObject::connect(this, &OsgWidget::labelingBrushFinished, ctx, std::move(handler));
	}
	QMetaObject::Connection observeLabelingPickCanceled(QObject* ctx, std::function<void()> handler) override
	{
		return QObject::connect(this, &OsgWidget::labelingPickCanceled, ctx, std::move(handler));
	}

	bool dispatchMeshPickCommitToInteractionSession(const PickResult& pick, int pickKindInt) override;

	QMetaObject::Connection observeSelectedObjectPoseChanged(
		QObject* ctx, std::function<void(float, float, float)> handler) override
	{
		return QObject::connect(this, &OsgWidget::selectedObjectPoseChanged, ctx, std::move(handler));
	}
	QMetaObject::Connection observeSelectedObjectRotationChanged(
		QObject* ctx, std::function<void(float, float, float)> handler) override
	{
		return QObject::connect(this, &OsgWidget::selectedObjectRotationChanged, ctx, std::move(handler));
	}
	QMetaObject::Connection observeSelectedObjectColorChanged(
		QObject* ctx, std::function<void(float, float, float, float)> handler) override
	{
		return QObject::connect(this, &OsgWidget::selectedObjectColorChanged, ctx, std::move(handler));
	}
	QMetaObject::Connection observeTransformGizmoCommitted(QObject* ctx, std::function<void()> handler) override
	{
		return QObject::connect(this, &OsgWidget::transformGizmoCommitted, ctx, std::move(handler));
	}
	QMetaObject::Connection observeTcpDragTeachPoseChanged(
		QObject* ctx, std::function<void(double, double, double, double, double, double)> handler) override
	{
		return QObject::connect(this, &OsgWidget::tcpDragTeachPoseChanged, ctx, std::move(handler));
	}
	QMetaObject::Connection observeTcpDragTeachEnded(QObject* ctx, std::function<void()> handler) override
	{
		return QObject::connect(this, &OsgWidget::tcpDragTeachEnded, ctx, std::move(handler));
	}
	QMetaObject::Connection observeActiveAxisChanged(QObject* ctx,
													 std::function<void(const QString&)> handler) override
	{
		return QObject::connect(this, &OsgWidget::activeAxisChanged, ctx, std::move(handler));
	}
	QMetaObject::Connection observeSelectionCanceledByEsc(QObject* ctx, std::function<void()> handler) override
	{
		return QObject::connect(this, &OsgWidget::selectionCanceledByEsc, ctx, std::move(handler));
	}
	QMetaObject::Connection observeAnnotationCreated(QObject* ctx,
													 std::function<void(const QString&, const QString&)> handler)
		override
	{
		return QObject::connect(this, &OsgWidget::annotationCreated, ctx, std::move(handler));
	}
	QMetaObject::Connection observeAnnotationRemoved(QObject* ctx, std::function<void(const QString&)> handler) override
	{
		return QObject::connect(this, &OsgWidget::annotationRemoved, ctx, std::move(handler));
	}
	QMetaObject::Connection observeAnnotationVisibilityChanged(
		QObject* ctx, std::function<void(const QString&, bool)> handler) override
	{
		return QObject::connect(this, &OsgWidget::annotationVisibilityChanged, ctx, std::move(handler));
	}
	QMetaObject::Connection observePointPickFeedback(QObject* ctx, std::function<void(const QString&)> handler) override
	{
		return QObject::connect(this, &OsgWidget::pointPickFeedback, ctx, std::move(handler));
	}
	QMetaObject::Connection observeMeshPickFeedback(QObject* ctx, std::function<void(const QString&)> handler) override
	{
		return QObject::connect(this, &OsgWidget::meshPickFeedback, ctx, std::move(handler));
	}
	QMetaObject::Connection observeBackendObjectPicked(QObject* ctx,
													   std::function<void(const QString&)> handler) override
	{
		return QObject::connect(this, &OsgWidget::backendObjectPicked, ctx, std::move(handler));
	}
	QMetaObject::Connection observeInstructionWaypointPicked(QObject* ctx,
														   std::function<void(const QString&, bool)> handler) override
	{
		return QObject::connect(this, &OsgWidget::instructionWaypointPicked, ctx, std::move(handler));
	}
	QMetaObject::Connection observeInstructionWaypointPickCanceled(QObject* ctx,
																   std::function<void()> handler) override
	{
		return QObject::connect(this, &OsgWidget::instructionWaypointPickCanceled, ctx, std::move(handler));
	}

	using OsgScene::clearMeshFittedSurfacePreview;

	/// TCP 示教罗盘（同 OsgScene 对象 gizmo）
	void updateTcpTeachCompassHighlight(DragAxis axis, bool highlightRing = false) override;
	void updateTcpTeachCompassScale();
	/// @param outPivotWorld 出参，TCP 枢轴世界坐标 mm
	void computeTcpTeachPivotWorld(osg::Vec3f& outPivotWorld) const override;
	/// @param axis 拖拽轴
	/// @param outAxisWorld 出参，单位轴方向（世界系）
	bool tcpTeachCompassUnitAxisWorld(DragAxis axis, osg::Vec3d& outAxisWorld) const;
	bool beginTcpTeachScreenDrag() override;
	/// @param curPos 当前鼠标（控件逻辑像素）
	/// @param lastPos 上一帧鼠标
	/// @return 沿冻结屏幕轴的位移 mm
	double tcpTeachScreenDragDsMm(const QPoint& curPos, const QPoint& lastPos) const override;
	/// @param mousePos 屏幕拾取点
	/// @param preferRing 优先拾取旋转环
	/// @param outPickedRing 非空时区分环/轴线
	int pickTcpTeachAxisAtScreenPos(const QPoint& mousePos, bool preferRing, bool* outPickedRing = nullptr) const override;
	void applyTcpTeachTranslationWorld(int axisIndex, double dsWorld);
	void applyTcpTeachTranslationBody(int axisIndex, double dsWorld) override;
	void applyTcpTeachRotationWorld(int axisIndex, double deltaRad);
	void applyTcpTeachRotationBody(int axisIndex, double deltaRad) override;
	void syncTcpTeachCompassAttitude();
	/// 从 \c m_tcpTeachMountPat 同步 \c m_tcpTeachWorldPat（兜底；示教 overlay 优先 FromTarget）
	void syncTcpTeachWorldPatFromMount();
	/// 按 T_base_target·P 刷新罗盘世界位姿（法兰挂载时 mount 仍跟 FK，overlay 跟目标）
	void syncTcpTeachWorldPatFromTarget();
	bool tcpTeachResolveBaseWorld(osg::Matrixd& outBaseWorld) const;
	bool tcpTeachToolWorldMatrix(osg::Matrixd& outToolWorld) const;
	void tcpTeachSetTargetFromToolWorld(const osg::Matrixd& toolWorld);
	bool m_tcpTeachActive = false;
	std::string m_tcpTeachMountBackendId;
	std::function<bool(osg::Matrixd&)> m_tcpTeachResolveRobotBaseWorld;
	bool m_tcpTeachUseFlangeLocalPlacement = false;
	osg::Matrixd m_tcpTeachToolLocalOnFlange;
	engine::RigidTransform m_tcpTeachTargetInBase;
	float m_tcpTeachModelDiagonal = 1000.0f;
	double m_tcpTeachGizmoRefDistance = -1.0;
	double m_tcpTeachGizmoRefScale = 1.0;
	bool m_tcpTeachDragging = false;
	bool m_tcpTeachRotating = false;
	DragAxis m_tcpTeachDragAxis = DragAxis::None;
	DragAxis m_tcpTeachHoverAxis = DragAxis::None;
	osg::Vec3d m_tcpTeachDragAxisWorld{};
	double m_tcpTeachDragScreenAxisUx = 1.0;
	double m_tcpTeachDragScreenAxisUy = 0.0;
	double m_tcpTeachDragMmPerPixel = 1.0;
	osg::Vec3d m_tcpTeachTransDragPlaneO{};
	osg::Vec3d m_tcpTeachTransDragPlaneN{};
	osg::Vec3d m_tcpTeachDragLastHitWorld{};
	bool m_tcpTeachTransDragPlaneActive = false;
	bool m_tcpTeachRotatePivotActive = false;
	osg::Vec3d m_tcpTeachRotatePivotWorld{};
	osg::ref_ptr<osg::MatrixTransform> m_tcpTeachMountPat;
	/// 场景 overlay 上的世界位姿节点（罗盘挂此，不挂在受光机器人子树下）
	osg::ref_ptr<osg::MatrixTransform> m_tcpTeachWorldPat;
	osg::ref_ptr<osg::Group> m_tcpTeachOverlayGroup;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_tcpTeachCompassTransform;
	osg::ref_ptr<osg::MatrixTransform> m_tcpTeachCompassScaleTransform;
	osg::ref_ptr<osg::Node> m_tcpTeachCompassNode;
	osg::ref_ptr<osg::MatrixTransform> m_tcpTeachAxisBranch[3];
	osg::ref_ptr<osg::MatrixTransform> m_tcpTeachRingBranch[3];

	/// Mesh 截面罗盘
	void setMeshSectionPlaneCompassVisible(bool visible);
	void ensureMeshSectionPlaneOverlay(const std::string& backendIdUtf8);
	void notifyMeshSectionPlaneChanged() override;
	void syncMeshSectionPlaneOverlayFromModel();
	void updateMeshSectionPlaneCompassHighlight(DragAxis axis, bool highlightRing = false) override;
	void updateMeshSectionPlaneCompassScale();
	void computeMeshSectionPlanePivotWorld(osg::Vec3d& outPivotWorld) const override;
	bool meshSectionPlaneCompassUnitAxisWorld(DragAxis axis, osg::Vec3d& outAxisWorld) const override;
	bool beginMeshSectionPlaneScreenDrag() override;
	double meshSectionPlaneScreenDragDsMm(const QPoint& curPos, const QPoint& lastPos) const override;
	void applyMeshSectionPlaneTranslationAxis(int axisIndex, double dsWorld) override;
	void applyMeshSectionPlaneTranslationWorld(const osg::Vec3d& hitWorld, const osg::Vec3d& lastHitWorld) override;
	void applyMeshSectionPlaneRotationAxis(int axisIndex, double deltaRad) override;
	bool pickMeshSectionPlaneDragPoint(const QPoint& mousePos, osg::Vec3d& outHitWorld) const override;
	int pickMeshSectionPlaneAxisAtScreenPos(const QPoint& mousePos, bool preferRing,
											bool* outPickedRing = nullptr) const override;
	bool m_sectionPlaneVisible = false;
	bool m_sectionPlaneEditActive = false;
	std::string m_sectionPlaneBackendId;
	std::function<void(const double[3], const double[3])> m_sectionPlaneOnChanged;
	osg::Vec3d m_sectionPlaneOriginModel{};
	osg::Vec3d m_sectionPlaneNormalModel{0.0, 0.0, 1.0};
	osg::Vec3d m_sectionPlaneAxisUModel{1.0, 0.0, 0.0};
	float m_sectionPlaneModelDiagonal = 1000.f;
	double m_sectionPlaneGizmoRefDistance = -1.0;
	double m_sectionPlaneGizmoRefScale = 1.0;
	bool m_sectionPlaneDragging = false;
	bool m_sectionPlaneRotating = false;
	bool m_sectionPlanePlaneDragging = false;
	DragAxis m_sectionPlaneDragAxis = DragAxis::None;
	DragAxis m_sectionPlaneHoverAxis = DragAxis::None;
	osg::Vec3d m_sectionPlaneScreenDragAxisWorld{};
	double m_sectionPlaneDragScreenAxisUx = 1.0;
	double m_sectionPlaneDragScreenAxisUy = 0.0;
	double m_sectionPlaneDragMmPerPixel = 1.0;
	osg::Vec3d m_sectionPlaneTransDragPlaneO{};
	osg::Vec3d m_sectionPlaneTransDragPlaneN{};
	osg::Vec3d m_sectionPlaneDragLastHitWorld{};
	osg::Vec3d m_sectionPlaneRotatePivotWorld{};
	osg::ref_ptr<osg::Group> m_sectionPlaneOverlayGroup;
	osg::ref_ptr<osg::MatrixTransform> m_sectionPlaneWorldPat;
	osg::ref_ptr<osg::Node> m_sectionPlaneQuadNode;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_sectionPlaneCompassTransform;
	osg::ref_ptr<osg::MatrixTransform> m_sectionPlaneCompassScaleTransform;
	osg::ref_ptr<osg::Node> m_sectionPlaneCompassNode;
	osg::ref_ptr<osg::MatrixTransform> m_sectionPlaneAxisBranch[3];
	osg::ref_ptr<osg::MatrixTransform> m_sectionPlaneRingBranch[3];

	// IViewportInteractionHost（SelectionOperation 专用，其余见上文已有 override）
	QObject* viewportGlWidget() const override;
	ViewportInteractionPointerState interactionPointerState() override;
	bool tcpTeachActive() const override { return m_tcpTeachActive; }
	bool meshSectionPlaneEditActive() const override;
	bool labelingClickPickMode() const override { return m_labelingClickPickMode; }
	bool labelingBrushPickMode() const override { return m_labelingBrushPickMode; }
	bool labelingMeshFaceMode() const override { return m_labelingMeshFaceMode; }
	float labelingBrushRadiusPx() const override { return m_labelingBrushRadiusPx; }
	bool crossObjectMeshPick() const override { return OsgScene::crossObjectMeshPick(); }
	bool originPlanePickActive() const override { return m_originPlanePickActive; }
	int originPlaneHoverIndex() const override { return m_originPlaneHoverIndex; }
	std::size_t pickablePointCount() const override;
	void updatePointPickMarker(const osg::Vec3f& pointWorld, bool hit) override;
	void clearPointPickMarker() override;
	void addPointAnnotation(const osg::Vec3f& pointWorld) override;
	void showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld) override;
	void showMeshEdgeHighlight(const std::vector<osg::Vec3f>& polylineWorld) override;
	void collectPointIndicesInScreenRadius(int screenX, int screenY, float radiusPx,
										   std::vector<int>& outIndices) const override;
	ViewportObjectGizmoDragState objectGizmoDragState() override;
	bool hasActiveObjectOuterPat() const override;
	osg::MatrixTransform* activeObjectOuterPat() override;
	DragAxis pickObjectGizmoAxisAtScreenPos(const QPoint& mousePos, bool preferRing,
											bool* outPickedRing = nullptr) override;
	bool pickAndActivateBackendAtScreenPos(const QPoint& mousePos) override;
	bool beginGizmoScreenDrag(DragAxis axis) override;
	bool beginGizmoScreenRotate(DragAxis axis, double mouseX, double mouseY) override;
	double gizmoScreenDragDs(double mouseXCur, double mouseYCur, double mouseXLast, double mouseYLast) const override;
	double gizmoScreenRotateDeltaRad(double mouseX, double mouseY) override;
	bool readActiveObjectGizmoFrame(ObjectGizmoFrame& out) const override;
	void syncCompassGizmoOrientation() override;
	void updateObjectGizmoCompassHighlight(DragAxis axis, bool highlightRing = false) override;
	void cacheSelectionGizmoPose() override;
	void refreshAnnotationTexts() override;
	void logGizmoPivotDiagnostics(const char* reasonTag) const override;
	QString gizmoAxisToString(DragAxis axis) const override;
	void resetObjectGizmoDragSession() override;
	bool cacheObjectGizmoRotatePivot() override;
	double objectGizmoMaxTranslateStepWorld() const override;
	osg::Vec3d gizmoWorldUnitAxis(DragAxis axis) const override;
	bool computeCameraScreenRayWorld(double mouseX, double mouseY, osg::Vec3d& outRayOriginWorld,
									 osg::Vec3d& outRayDirUnitWorld) const override;
	void computeGizmoPivotWorld(osg::Vec3f& outPivotWorld) const override;
	ViewportTcpTeachDragState tcpTeachDragState() override;
	osg::Vec3d tcpTeachWorldUnitAxis(DragAxis axis) const override;
	void resetTcpTeachDragSession() override;
	double tcpTeachMaxTranslateStep() const override;
	void emitTcpDragTeachPoseChanged() override;
	ViewportMeshSectionPlaneDragState meshSectionPlaneDragState() override;
	void emitPointPickFeedback(const QString& text) override;
	void emitPolylinePickFeedback(const QString& text) override;
	void emitActiveAxisChanged(const QString& axisName) override;
	void emitSelectedObjectPoseChanged(float x, float y, float z) override;
	void emitSelectedObjectRotationChanged(float rx, float ry, float rz) override;
	void emitTransformGizmoCommitted() override;
	void emitMeshPickFeedback(const QString& text) override;
	void emitMeshPickCommitted(const PickResult& pick, int pickKindInt) override;
	void emitLabelingClickCommitted(const PickResult& pick) override;
	void emitLabelingBrushStroke(const QVector<int>& indices) override;
	void emitLabelingBrushFinished() override;

signals:
	void selectedObjectPoseChanged(float x, float y, float z);
	void selectedObjectRotationChanged(float rx, float ry, float rz);
	void selectedObjectColorChanged(float r, float g, float b, float a);
	/// 平移/旋转 gizmo 拖拽结束
	void transformGizmoCommitted();
	/// TCP 示教拖动中位姿更新（基座系 mm + 欧拉 deg）
	void tcpDragTeachPoseChanged(double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg, double ezDeg);
	void tcpDragTeachEnded();
	void instructionWaypointPicked(const QString& instructionId, bool isArcVia);
	void instructionWaypointPickCanceled();
	void backendObjectPicked(const QString& backendId);
	void activeAxisChanged(const QString& axisName);
	void selectionCanceledByEsc();
	void pointPickFeedback(const QString& text);
	void polylinePickFeedback(const QString& text);
	void polylinePickCommitted(QVector<float> polylineScreenXy, QVector<double> mvpMatrix, int viewportWidth,
							   int viewportHeight);
	void polylinePickCanceled();
	void meshPickFeedback(const QString& text);
	void meshPickCommitted(PickResult pick, int pickKind);
	void labelingClickCommitted(PickResult pick);
	void labelingBrushStroke(QVector<int> indices);
	void labelingBrushFinished();
	void labelingPickCanceled();
	void annotationCreated(const QString& annotationId, const QString& displayText);
	void annotationRemoved(const QString& annotationId);
	void annotationVisibilityChanged(const QString& annotationId, bool visible);
	/// 随 OsgScene::requestRedraw 发出（场景或相机变更）
	void sceneRedrawRequested();

protected:
	void showEvent(QShowEvent* event) override;
	void resizeEvent(QResizeEvent* event) override;

private:
	void initViewer();
	void initUi();
	void syncViewportFromGlWidget();
	void syncViewportLayoutFromFramebuffer(int framebufferWidth, int framebufferHeight);
	void scheduleDeferredViewportLayoutSync();
	osg::Node* loadXyzPointCloud(const QString& filePath, QString* errorMessage);
	osg::Node* loadAsciiPlyPointCloud(const QString& filePath, QString* errorMessage);
	osg::Node* createCompassNode();
	bool eventFilter(QObject* watched, QEvent* event) override;
	/// \param outPickedRing 若非空：命中旋转环时为 true，命中轴线段时为 false。
	DragAxis pickAxisAtScreenPos(const QPoint& mousePos, bool preferRing, bool* outPickedRing = nullptr) const;
	void applyColorToActiveBackendObject(const osg::Vec4& color);
	void applyColorToStagingGeometry(const osg::Vec4& color);
	osg::ref_ptr<osg::Geode> buildPointCloudGeode(const PointCloudBackendData& data, QString* errorMessage) const;
	bool upsertPointCloudBranchInScene(const PointCloudBackendData& data, QString* errorMessage, bool resetViewToHome);
	osg::ref_ptr<osg::Node> buildMeshGeode(const MeshBackendData& data, QString* errorMessage,
										   bool showWireOutline = true, bool useSceneLighting = false) const;
	bool upsertMeshBranchInScene(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome,
								 bool showWireOutline = true, bool useSceneLighting = false);
	bool upsertBackendBranchInScene(const BackendDataBase& data, QString* errorMessage, bool resetViewToHome,
									bool showWireOutline = true, bool useSceneLighting = false);
	osg::Node* stagingGeometryRoot() const;
	void applyVisibilityMaskForBackend(const std::string& backendId);
	void updateCompassHighlight(DragAxis axis, bool highlightRing = false);
	QString axisToString(DragAxis axis) const;
	void updateCompassScale();
	void refreshCompassDrawVisibility();
	/// World：轴对齐世界 XYZ；Local：轴随物体；枢轴在模型原点
	void attachCompassGraphics();
	void detachCompassGraphics();
	void syncCameraManipulatorForModes();
	void clearPointAnnotations();
	bool pickPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld) const;
	bool pickNearestPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx,
									 bool previewOnly) const;
	bool pickPointByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx) const;

public slots:
	void onViewportFocusRequested() override;
	void onViewportScreenshotRequested() override;

private:
	QWidgetViewer* m_glWidget = nullptr;
	QTimer m_frameTimer;
	QTimer m_idleRenderTimer;
	mutable QElapsedTimer m_feedbackTimer;
	QPoint m_lastMousePos;
	std::unique_ptr<OsgWidgetImportController> m_importController;
	std::unique_ptr<OsgWidgetBackendLoadController> m_backendLoadController;
	std::unique_ptr<OsgWidgetCaptureController> m_captureController;
	std::unique_ptr<OsgWidgetPickAnnotationController> m_pickAnnotationController;
	std::unique_ptr<SelectionOperation> m_pointPickOperation;
	std::unique_ptr<SelectionOperation> m_polylinePickOperation;
	std::unique_ptr<SelectionOperation> m_objectTransformOperation;
	std::unique_ptr<SelectionOperation> m_tcpDragTeachOperation;
	std::unique_ptr<SelectionOperation> m_meshSectionPlaneOperation;
	std::unique_ptr<SelectionOperation> m_meshElementPickOperation;
	std::unique_ptr<SelectionOperation> m_labelingPickOperation;
	std::unique_ptr<ViewportInteractionController> m_interactionController;
	SketchPlaneInputHandler m_sketchPlaneInputHandler;
	bool m_originPlanePickActive = false;
	float m_originPlaneHalfMm = 60.f;
	int m_originPlaneHoverIndex = -1;
	OriginPlanePickedFn m_originPlanePickedFn;
	std::vector<SketchSupportExtraPlane> m_supportExtras;
	osg::ref_ptr<osg::Group> m_originPlanePickGroup;
	osg::ref_ptr<osg::Vec4Array> m_originPlaneFillColors[3];
	osg::ref_ptr<osg::Vec4Array> m_originPlaneEdgeColors[3];
	osg::ref_ptr<osg::Geometry> m_originPlaneFillGeoms[3];
	osg::ref_ptr<osg::Geometry> m_originPlaneEdgeGeoms[3];
	osg::ref_ptr<osg::Material> m_originPlaneFillMaterials[3];
	/// 持久原点/基准面（与拾取会话分离）
	bool m_originRefPointVisible = false;
	bool m_originRefPlaneVisible[3] = {false, false, false};
	float m_originRefHalfMm = 60.f;
	osg::ref_ptr<osg::Group> m_originRefGroup;
	osg::ref_ptr<osg::Group> m_originRefPointGroup;
	osg::ref_ptr<osg::Group> m_originRefPlaneGroups[3];
	void ensureOriginReferenceGroup();
	void syncOriginReferenceNodeMasks();
	/// 返回命中基面 index；outDist2 为到相机距离平方
	int hitTestOriginPlane(int screenX, int screenY, double* outDist2 = nullptr) const;
	void applyOriginPlaneHover(int hoverIndex);
	void updateSketchSupportHover(int screenX, int screenY);
	/// 使用场景光照加载的网格后端（如 URDF 连杆），改色时保留 Material+LIGHTING。
	std::unordered_set<std::string> m_litMeshBackendIds;
	osg::ref_ptr<osg::Group> m_instructionPoseAxesGroup;
	osg::ref_ptr<osg::Geode> m_rawTrajectoryOverlayGeode;
	osg::ref_ptr<osg::Geode> m_sketchLineOverlayGeode;
	osg::ref_ptr<osg::Group> m_rawTrajectoryFramesGroup;
	osg::ref_ptr<osg::Geode> m_reachableWorkspaceOverlayGeode;
	osg::ref_ptr<osg::MatrixTransform> m_playbackCursorMt;
	/// 路点拾取悬停圈（朝向屏幕）
	osg::ref_ptr<osg::AutoTransform> m_waypointPickHoverRingAt;
	std::string m_waypointPickHoverInstructionId;
	osg::ref_ptr<osg::Group> m_waypointIndexLabelsGroup;
	bool m_instructionWaypointPickMode = false;
	struct InstructionWaypointPickTarget
	{
		std::string instructionId;
		cloudsim::core::Vec3 positionMm{};
		bool isArcVia = false;
	};
	std::vector<InstructionWaypointPickTarget> m_instructionWaypointPickTargets;
	std::function<void(const std::string&, bool)> m_instructionWaypointPicked;
	std::function<void()> m_instructionWaypointPickCanceled;
	bool tryPickInstructionWaypointAt(int mouseX, int mouseY, std::string& outInstructionId,
									  cloudsim::core::Vec3* outPositionMm = nullptr, bool* outIsArcVia = nullptr) const;
	void ensureWaypointPickHoverRing();
	void updateWaypointPickHoverAt(int mouseX, int mouseY);
	void clearWaypointPickHover();
	cloudsim::core::Vec3 m_waypointPickHoverPositionMm{};
	bool m_rawTrajShowAxisX = true;
	bool m_rawTrajShowAxisY = true;
	bool m_rawTrajShowAxisZ = true;
	Qt::CursorShape m_lastViewportCursor = Qt::ArrowCursor;
	struct RobotFrameOverlayNodes
	{
		std::vector<osg::ref_ptr<osg::MatrixTransform>> toolNodes;
		std::vector<osg::ref_ptr<osg::MatrixTransform>> userNodes;
	};
	std::unordered_map<std::string, RobotFrameOverlayNodes> m_robotFrameOverlayNodes;

	std::function<void(OsgWidget*)> m_perFrameHook;
	RobotObjectGizmoSyncFn m_robotObjectGizmoSyncHook;
	RobotObjectGizmoFkRefreshFn m_robotObjectGizmoFkRefreshHook;
	std::string m_cameraFollowBackendId;

	// 渐变背景
	osg::ref_ptr<osg::Camera> m_gradientBackgroundCamera;
	osg::ref_ptr<osg::Geode> m_gradientBackgroundGeode;
	osg::ref_ptr<osg::Geometry> m_gradientBackgroundGeom;
	bool m_darkUiTheme = false;
	bool m_wireframeMode = false;
	void applyViewportWireframeToBackendBranch(osg::Node* outerBranch);
	void applyViewportWireframeToAllBackends();
	void createGradientBackground();
	void updateGradientColors(bool dark);

	void updateCameraFollowCenter();

	void noteViewportInteraction();

	void applyViewCubeFaceLabelImagesFromQt();

	bool pickMeshFaceByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld, osg::Vec3f& outAWorld,
									   osg::Vec3f& outBWorld, osg::Vec3f& outCWorld, osg::Vec3f& outNormalWorld,
									   std::vector<osg::Vec3f>* outMergedCoplanarVertsWorld = nullptr,
									   const std::string* scopeBackendId = nullptr) const;

	bool pickMeshEdgeByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld, osg::Vec3f& outEdgeAWorld,
									   osg::Vec3f& outEdgeBWorld, double* outEdgeDistancePx = nullptr,
									   const std::string* scopeBackendId = nullptr) const;

	BackendDataManager* m_poseSyncBackendMgr = nullptr;
	VisualSyncMarkDirtyFn m_visualSyncMarkDirty;
};

#endif // CLOUDSIMHOST_OSGWIDGET_H
