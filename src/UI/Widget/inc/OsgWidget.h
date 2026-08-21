#ifndef WIDGET_OSGWIDGET_H
#define WIDGET_OSGWIDGET_H

/// @file OsgWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 三维视图控件（Qt + \c OsgScene）

#include "widget_global.h"

#include "../../OsgWidgetCore/inc/OsgScene.h"
#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "../../RobotWidget/inc/RobotOsgUiTypes.h"
#include "GraphicsWindowQt1.h"
#include "IRobotBackendPoseSink.h"

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
class OSG_WIDGET_API OsgWidget : public QWidget, public IRobotBackendPoseSink, public OsgScene
{
	Q_OBJECT
public:
	using DragAxis = OsgScene::DragAxis;
	friend class PointPickOperation;
	friend class PolylinePickOperation;
	friend class ObjectTransformOperation;
	friend class RobotTcpDragTeachOperation;
	friend class MeshEdgeFacePickOperation;
	friend class MeshSectionPlaneEditOperation;
	friend class LabelingPickOperation;
	friend class OsgWidgetImportController;
	friend class OsgWidgetBackendLoadController;
	friend class OsgWidgetCaptureController;
	friend class OsgWidgetPickAnnotationController;
	friend class OsgWidgetColorController;
	friend class OsgWidgetTransformHierarchyController;

	using AnnotationEntry = OsgScene::AnnotationEntry;

public:
	struct AnnotationSnapshot
	{
		QString id;
		QString displayText;
		QString backendId; /// 空表示旧版/未知
		/// 旧工程：保存时对象 gizmo 局部偏移
		osg::Vec3f localCentered;
		/// 世界锚点；切换后端对象后位置不变
		osg::Vec3f worldAnchor{};
		bool hasWorldAnchor = false;
		bool visible = true;
	};

public:
	explicit OsgWidget(QWidget* parent = nullptr);
	~OsgWidget() override;
	bool importModelFile(const QString& filePath, QString* errorMessage = nullptr);
	bool importPointCloudFile(const QString& filePath, QString* errorMessage = nullptr);
	bool captureImportedPointCloudBackend(PointCloudBackendData& out, QString* errorMessage = nullptr);
	bool capturePointCloudBackendFromScene(const std::string& backendId, PointCloudBackendData& out,
										   QString* errorMessage = nullptr);
	bool captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage = nullptr);
	bool captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts, QString* errorMessage = nullptr);
	bool captureViewportPng(QByteArray& outPng, QString* errorMessage = nullptr, int maxWidth = 768,
							int maxHeight = 768);
	bool loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage = nullptr,
									   bool resetViewToHome = true);
	bool loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage = nullptr,
								 bool resetViewToHome = true, bool showWireOutline = true,
								 bool useSceneLighting = true);
	bool loadBackendFromBackendData(const BackendDataBase& data, QString* errorMessage = nullptr,
									bool resetViewToHome = true, bool showWireOutline = true,
									bool useSceneLighting = true);
	/// 受光网格后端（如 URDF 连杆）；改色时保留光照材质
	bool isBackendMeshLit(const std::string& backendId) const;
	void clearImportedContent();
	/// 仅清导入预览，保留已注册后端可视
	void clearStagingGeometry();
	/// 半透明三角网预览（xyz 交错，9 floats/三角）
	void setStagingMeshPreview(const std::vector<float>& xyzTriangles, const osg::Vec4& rgba);
	/// 草图折线 overlay：关深度测试，避免贴面被遮挡；rgba 按段着色
	void setSketchLineOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
							  const std::vector<std::size_t>& segmentEndExclusive,
							  const std::vector<osg::Vec4>& segmentColors,
							  const std::vector<float>& segmentWidthsPx = {});
	void clearSketchLineOverlay();
	void setSelectionActive(bool active);
	void setObjectSelectionMode(bool enabled);
	bool objectSelectionMode() const;
	/// 物体变换罗盘：物体系沿当前罗盘轴（与模型姿态一致）；世界系沿世界 X/Y/Z。
	using TransformGizmoFrame = OsgScene::TransformGizmoFrame;
	void setTransformGizmoFrame(TransformGizmoFrame frame);
	TransformGizmoFrame transformGizmoFrame() const { return m_transformGizmoFrame; }
	void setPointPickMode(bool enabled);
	bool pointPickMode() const;
	void setPolylinePickMode(bool enabled);
	bool polylinePickMode() const;
	void updatePolylinePickOverlay(const std::vector<QPoint>& vertices, const QPoint* cursorPos);
	void commitPolylinePick(const std::vector<QPoint>& vertices);
	void clearPolylinePickOverlay();
	void setMeshLinePickMode(bool enabled);
	bool meshLinePickMode() const;
	void setMeshFacePickMode(bool enabled);
	bool meshFacePickMode() const;

	/// 屏幕点 → 世界射线与平面求交（逻辑像素，与拾取一致）
	bool intersectScreenWithPlaneMm(int screenX, int screenY, const osg::Vec3d& planeOrigin,
									const osg::Vec3d& planeNormal, osg::Vec3d& outHitWorldMm,
									QString* outError = nullptr) const;

	/// 草图编辑：消费视口鼠标/键，抑制轨道（handler 返回 true 表示已处理）
	using SketchPlaneInputHandler = std::function<bool(QObject* watched, QEvent* event)>;
	void setSketchPlaneInputHandler(SketchPlaneInputHandler handler);
	void clearSketchPlaneInputHandler();

	/// 新建草图：显示 XY/XZ/YZ 半透明基准面，点击回调 index（0/1/2）；取消 ok=false
	/// index>=100 表示 setSketchSupportExtraPlanes 中的用户面（100+i）
	using OriginPlanePickedFn = std::function<void(bool ok, int planeIndex)>;
	void beginOriginPlaneSelection(OriginPlanePickedFn onFinished, float halfSizeMm = 60.f);
	void cancelOriginPlaneSelection();
	bool isOriginPlaneSelectionActive() const { return m_originPlanePickActive; }

	struct SketchSupportExtraPlane
	{
		osg::Vec3d origin{0, 0, 0};
		osg::Vec3d axisX{1, 0, 0};
		osg::Vec3d axisY{0, 1, 0};
		osg::Vec3d normal{0, 0, 1};
		float halfMm = 40.f;
	};
	void setSketchSupportExtraPlanes(std::vector<SketchSupportExtraPlane> planes);
	void clearSketchSupportExtraPlanes();
	/// 命中用户候选面；outDist2 为到相机距离平方
	int hitTestSupportExtra(int screenX, int screenY, double* outDist2 = nullptr) const;
	/// 基面/用户面与模型面更近者胜；胜出基面 0..2，用户面 100+i，否则 -1 交给网格
	int resolveSketchSupportOriginIndex(int screenX, int screenY) const;
	QPoint lastMousePos() const { return m_lastMousePos; }

	/// 持久显示世界原点三轴 + 三基准面（拾取会话期间自动隐藏，结束后按标志恢复）
	void setOriginReferenceVisibility(bool originPoint, bool planeXY, bool planeXZ, bool planeYZ,
									  float halfSizeMm = 60.f);

	void setLabelingClickPickMode(bool enabled, bool meshFace);
	void setLabelingBrushPickMode(bool enabled, bool meshFace, float radiusPx);
	PickResult queryPick(const PickQuery& query);
	ViewportInteractionController* interactionController() { return m_interactionController.get(); }
	IViewportPickEngine* pickEngine();
	void beginInteractionSession(std::shared_ptr<IInteractionSession> session);
	void endInteractionSession(bool cancel = true);
	bool hasInteractionSession() const;
	void setupInteractionController();
	osg::Vec3f selectedPosition() const;
	void setSelectedPosition(const osg::Vec3f& position);
	osg::Vec3f selectedRotationEulerDeg() const;
	void setSelectedRotationEulerDeg(const osg::Vec3f& eulerDeg);
	void setSelectedColor(float r, float g, float b, float a = 1.0f);
	/// 按 backendId 刷新场景颜色，不发 selectedObjectColorChanged
	void applyColorToBackendObject(const std::string& backendId, const osg::Vec4& color);
	QString pointCloudPluginReport() const;
	/// 按后端树行显隐 OSG 分支
	void setBackendObjectVisible(const std::string& backendId, bool visible);
	/// 同步逻辑父子链（顶层标注跟踪等）
	void setBackendParent(const std::string& backendId, const std::string& parentBackendId);
	void setBackendLogicalParent(const std::string& backendId, const std::string& parentBackendId);
	void removeBackendObjectVisual(const std::string& backendId);
	/// 后端几何已在场景中（非导入预览）
	bool hasBackendObjectBranch(const std::string& backendId) const;
	/// 不重载几何同步 gizmo/拾取缓存，保留标注
	void syncSelectionFromBackend(const PointCloudBackendData& data);
	void syncSelectionFromBackend(const MeshBackendData& data);
	/// 无自有几何的后端行也可选中（如装配父节点）
	void syncSelectionForBackendId(const std::string& backendId);
	/// 逻辑节点映射到已挂载 OSG 分支（装配子件共用父级 visual）
	void setPickVisualAlias(const std::string& logicalBackendId, const std::string& visualBackendId);
	bool backendSkipsInnerModelCenterRebase(const std::string& backendId) const;
	bool setAnnotationVisible(const QString& annotationId, bool visible);
	bool removeAnnotation(const QString& annotationId);
	void clearAllAnnotations();
	QList<AnnotationSnapshot> annotationSnapshots() const;
	void restoreAnnotations(const QList<AnnotationSnapshot>& snapshots);
	/// 随 Qt 深/浅主题设置 OSG 背景色
	void setViewerBackgroundForDarkUi(bool dark);
	/// GL 视口控件，供浮动工具栏等 overlay 挂载
	QWidget* viewportWidget() const { return m_glWidget; }
	/// 线框/实体切换
	void setWireframeMode(bool enabled);
	bool wireframeMode() const { return m_wireframeMode; }
	/// 至少一个后端有几何或存在导入预览
	bool hasImportedContent() const;
	/// Viewer 根节点（\c setSceneData），供场景树 UI/调试
	const osg::Group* sceneGraphRoot() const { return m_root.get(); }
	/// 绕 \a pivotWorld 刚体旋转各后端根（左乘姿态）
	void applyRigidRotationAboutWorldPivot(const std::vector<std::string>& backendIds, const osg::Vec3f& pivotWorld,
										   const osg::Quat& deltaRotation);
	osg::Vec3f averageBackendRootPositionWorld(const std::vector<std::string>& backendIds) const;
	/// \a backendId 外层 PAT 世界矩阵（含父链）；OSG 形态供 OsgWidget 内部使用
	bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const;
	/// 设外层 PAT 世界矩阵为 \a worldMat（含父链）
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat);
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
	void setInstructionWaypointPickCallbacks(
		std::function<void(const std::string& instructionId, bool isArcVia)> onPicked,
		std::function<void()> onCanceled);

	/// TCP 末端拖动示教：场景 overlay 罗盘，拖动发位姿信号（不写指令）
	bool isTcpDragTeachActive() const { return m_tcpTeachActive; }
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
	void syncActiveBackendRootFromObjectFrame(const ObjectGizmoFrame& cur, bool dragging);
	/// 对象 gizmo 拖拽中，跳过对该选中跟随者的位姿覆写
	bool isTransformGizmoDragging() const;
	/// 按缓存质心将 \a data 位姿写到外层 PAT
	bool syncOuterPatFromBackend(const BackendDataBase& data);
	/// 非拖拽时将 ObjectGizmoFrame 同步到活动后端根
	void syncActiveBackendRootFromSelectedTransform();
	/// OSG 位姿写回后端（跟随求解前）
	bool writeActiveBackendPoseFromOsg(BackendDataBase& data);
	/// 轨道相机中心跟随此后端世界原点（空则关闭）
	void setCameraFollowBackendId(std::string backendId);
	void clearCameraFollowBackendId();
	const std::string& cameraFollowBackendId() const { return m_cameraFollowBackendId; }

	using OsgScene::clearMeshFittedSurfacePreview;
	using OsgScene::hideMeshElementHighlight;
	using OsgScene::showMeshEdgeHighlight;
	using OsgScene::showMeshFaceHighlight;
	using OsgScene::showMeshFittedSurfacePreview;

	/// TCP 示教罗盘（RobotTcpDragTeachOperation 友元，同 OsgScene 对象 gizmo）
	void updateTcpTeachCompassHighlight(DragAxis axis, bool highlightRing = false);
	void updateTcpTeachCompassScale();
	/// @param outPivotWorld 出参，TCP 枢轴世界坐标 mm
	void computeTcpTeachPivotWorld(osg::Vec3f& outPivotWorld) const;
	/// @param axis 拖拽轴
	/// @param outAxisWorld 出参，单位轴方向（世界系）
	bool tcpTeachCompassUnitAxisWorld(DragAxis axis, osg::Vec3d& outAxisWorld) const;
	bool beginTcpTeachScreenDrag();
	/// @param curPos 当前鼠标（控件逻辑像素）
	/// @param lastPos 上一帧鼠标
	/// @return 沿冻结屏幕轴的位移 mm
	double tcpTeachScreenDragDsMm(const QPoint& curPos, const QPoint& lastPos) const;
	/// @param mousePos 屏幕拾取点
	/// @param preferRing 优先拾取旋转环
	/// @param outPickedRing 非空时区分环/轴线
	int pickTcpTeachAxisAtScreenPos(const QPoint& mousePos, bool preferRing, bool* outPickedRing = nullptr) const;
	void applyTcpTeachTranslationWorld(int axisIndex, double dsWorld);
	void applyTcpTeachTranslationBody(int axisIndex, double dsWorld);
	void applyTcpTeachRotationWorld(int axisIndex, double deltaRad);
	void applyTcpTeachRotationBody(int axisIndex, double deltaRad);
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

	/// Mesh 截面罗盘（MeshSectionPlaneEditOperation 友元）
	void setMeshSectionPlaneCompassVisible(bool visible);
	void ensureMeshSectionPlaneOverlay(const std::string& backendIdUtf8);
	void notifyMeshSectionPlaneChanged();
	void syncMeshSectionPlaneOverlayFromModel();
	void updateMeshSectionPlaneCompassHighlight(DragAxis axis, bool highlightRing = false);
	void updateMeshSectionPlaneCompassScale();
	void computeMeshSectionPlanePivotWorld(osg::Vec3d& outPivotWorld) const;
	bool meshSectionPlaneCompassUnitAxisWorld(DragAxis axis, osg::Vec3d& outAxisWorld) const;
	bool beginMeshSectionPlaneScreenDrag();
	double meshSectionPlaneScreenDragDsMm(const QPoint& curPos, const QPoint& lastPos) const;
	void applyMeshSectionPlaneTranslationAxis(int axisIndex, double dsWorld);
	void applyMeshSectionPlaneTranslationWorld(const osg::Vec3d& hitWorld, const osg::Vec3d& lastHitWorld);
	void applyMeshSectionPlaneRotationAxis(int axisIndex, double deltaRad);
	bool pickMeshSectionPlaneDragPoint(const QPoint& mousePos, osg::Vec3d& outHitWorld) const;
	int pickMeshSectionPlaneAxisAtScreenPos(const QPoint& mousePos, bool preferRing,
											bool* outPickedRing = nullptr) const;
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
	void syncCompassGizmoOrientation();
	/// 环境变量 \c POINTCLOUD_GIZMO_PIVOT_DIAG 非空且不为 \c "0" 时：经 RunLogger 输出枢轴与场景图文件原点对比（调试用）。
	void logGizmoPivotDiagnostics(const char* reasonTag) const;
	void attachCompassGraphics();
	void detachCompassGraphics();
	void syncCameraManipulatorForModes();
	bool pickAndActivateBackendAtScreenPos(const QPoint& mousePos);
	void clearPointAnnotations();
	bool pickPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld) const;
	bool pickNearestPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx,
									 bool previewOnly) const;
	bool pickPointByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx) const;
	void addPointAnnotation(const osg::Vec3f& pointWorld);
	void updatePointPickMarker(const osg::Vec3f& pointWorld, bool hit);
	void clearPointPickMarker();
	void refreshAnnotationTexts();
	void emitTcpDragTeachPoseChanged();

public slots:
	void onViewportFocusRequested();
	void onViewportScreenshotRequested();

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
									  cloudsim::core::Vec3* outPositionMm = nullptr,
									  bool* outIsArcVia = nullptr) const;
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
};

#endif // WIDGET_OSGWIDGET_H
