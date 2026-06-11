#pragma once

#include <QWidget>
#include <QTimer>
#include <QString>
#include <QPoint>
#include <QElapsedTimer>
#include <vector>
#include <string>
#include <cstddef>
#include <utility>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <QList>
#include <functional>
#include <osgGA/TrackballManipulator>
#include <osg/Camera>
#include <osg/Array>
#include <osg/AutoTransform>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Quat>
#include <osg/Vec3f>
#include <osg/ref_ptr>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osgText/Text>

#include "widget_global.h"
#include "GraphicsWindowQt1.h"
#include "IRobotBackendPoseSink.h"
#include "../../../../UI/OsgWidgetCore/inc/OsgScene.h"

#include <RigidTransform.h>

namespace osg {
class Group;
class Node;
class Geometry;
}

namespace osgViewer {
class Viewer;
}

class QWidgetViewer;
class SelectionOperation;
class PointPickOperation;
class ObjectTransformOperation;
class RobotTcpDragTeachOperation;
class MeshEdgeFacePickOperation;
class BackendDataBase;
class PointCloudBackendData;
class MeshBackendData;
class OsgWidgetImportController;
class OsgWidgetBackendLoadController;
class OsgWidgetCaptureController;
class OsgWidgetPickAnnotationController;
class OsgWidgetColorController;
class OsgWidgetTransformHierarchyController;
struct MeshCapturedPart;

/// 三维视图控件（Qt + \c OsgScene）
///
/// Viewer/相机、后端导入显示、拾取标注；对象变换与 TCP 示教罗盘（后者挂场景 overlay，
/// 位姿经 \c syncTcpTeachWorldPatFromMount 与 mount PAT 对齐）。
class OSG_WIDGET_API OsgWidget : public QWidget, public IRobotBackendPoseSink, public OsgScene
{
	Q_OBJECT
public:
	using DragAxis = OsgScene::DragAxis;
	friend class PointPickOperation;
	friend class ObjectTransformOperation;
	friend class RobotTcpDragTeachOperation;
	friend class MeshEdgeFacePickOperation;
	friend class OsgWidgetImportController;
	friend class OsgWidgetBackendLoadController;
	friend class OsgWidgetCaptureController;
	friend class OsgWidgetPickAnnotationController;
	friend class OsgWidgetColorController;
	friend class OsgWidgetTransformHierarchyController;

	using AnnotationEntry = OsgScene::AnnotationEntry;

public:
	struct InstructionPoseAxis
	{
		osg::Vec3f positionMm;
		osg::Vec3f eulerDeg;
		bool lineMotion = false;
		/// IK reachable for this waypoint (origin marker green/red).
		bool reachable = true;
		std::string robotBackendId; // Local frame parent. Empty means world overlay.
		/// per-link：T_base_tcp 直接挂在 \a robotBackendId 的 PAT 下（勿用其子装配子图，避免漂到场景原点）。
		bool mountTcpOnPatRoot = false;
		bool hasLocalMatrix = false;
		double localMatrix[16]{};
		/// URDF link name (e.g. "link_6"). When set, \a localMatrix is inv(T_link)*T_tcp in that link's
		/// container frame and the axis is parented under \c "{name}_Container" in the robot scene graph.
		std::string urdfTcpAttachLinkName;
	};

	struct AnnotationSnapshot
	{
		QString id;
		QString displayText;
		QString backendId; // empty => legacy/unknown
		/// Legacy: offset in object gizmo (center+pose) space at save time (older projects only).
		osg::Vec3f localCentered;
		/// Authoritative world-space anchor; when set, position survives switching backend objects.
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
	bool captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage = nullptr);
	bool captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts, QString* errorMessage = nullptr);
	bool loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage = nullptr,
		bool resetViewToHome = true);
	bool loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage = nullptr, bool resetViewToHome = true,
		bool showWireOutline = true, bool useSceneLighting = true);
	/// URDF 等光照网格；改色时保留材质
	bool isBackendMeshLit(const std::string& backendId) const;
	void clearImportedContent();
	/// 仅清导入预览，已注册分支保留
	void clearStagingGeometry();
	void setSelectionActive(bool active);
	void setObjectSelectionMode(bool enabled);
	bool objectSelectionMode() const;
	/// 物体变换罗盘：物体系沿当前罗盘轴（与模型姿态一致）；世界系沿世界 X/Y/Z。
	using TransformGizmoFrame = OsgScene::TransformGizmoFrame;
	void setTransformGizmoFrame(TransformGizmoFrame frame);
	TransformGizmoFrame transformGizmoFrame() const { return m_transformGizmoFrame; }
	void setPointPickMode(bool enabled);
	bool pointPickMode() const;
	void setMeshLinePickMode(bool enabled);
	bool meshLinePickMode() const;
	void setMeshFacePickMode(bool enabled);
	bool meshFacePickMode() const;
	PickResult queryPick(const PickQuery& query);
	osg::Vec3f selectedPosition() const;
	void setSelectedPosition(const osg::Vec3f& position);
	osg::Vec3f selectedRotationEulerDeg() const;
	void setSelectedRotationEulerDeg(const osg::Vec3f& eulerDeg);
	void setSelectedColor(float r, float g, float b, float a = 1.0f);
	QString pointCloudPluginReport() const;
	/// Per backend tree row: show/hide that object's OSG branch (multiple clouds/meshes per document).
	void setBackendObjectVisible(const std::string& backendId, bool visible);
	/// Sync logical backend hierarchy for features like top-parent annotation tracking.
	void setBackendParent(const std::string& backendId, const std::string& parentBackendId);
	void removeBackendObjectVisual(const std::string& backendId);
	/// True if geometry for this backend id is already in the scene (not import staging).
	bool hasBackendObjectBranch(const std::string& backendId) const;
	/// Align gizmo / pick cache with backend pose without reloading geometry (preserves annotations).
	void syncSelectionFromBackend(const PointCloudBackendData& data);
	void syncSelectionFromBackend(const MeshBackendData& data);
	/// Select backend row even when it has no own geometry (e.g. assembly parent).
	void syncSelectionForBackendId(const std::string& backendId);
	bool setAnnotationVisible(const QString& annotationId, bool visible);
	bool removeAnnotation(const QString& annotationId);
	void clearAllAnnotations();
	QList<AnnotationSnapshot> annotationSnapshots() const;
	void restoreAnnotations(const QList<AnnotationSnapshot>& snapshots);
	// Match Qt dark/light theme: clears OSG camera to a light gray (dark UI) or near-white (light UI).
	void setViewerBackgroundForDarkUi(bool dark);
	/// True when at least one backend object has scene geometry (or import staging is present).
	bool hasImportedContent() const;
	/// Root passed to the viewer (\c setSceneData); use for scene-graph hierarchy UI / debugging.
	const osg::Group* sceneGraphRoot() const { return m_root.get(); }
	/// Rigid-body rotation: each backend root rotates about \a pivotWorld by \a deltaRotation (left-multiply attitude).
	void applyRigidRotationAboutWorldPivot(const std::vector<std::string>& backendIds, const osg::Vec3f& pivotWorld,
		const osg::Quat& deltaRotation);
	osg::Vec3f averageBackendRootPositionWorld(const std::vector<std::string>& backendIds) const;
	/// World matrix of the outer PAT for \a backendId (respects parent chain under the scene).
	bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const override;
	/// Sets outer PAT pose so its world matrix equals \a worldMat (respects parent chain).
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat) override;
	bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy, double& outCz) const override;
	void syncRobotMeshBackendPoseAfterKinematics(const BackendDataBase& mesh) override;

	/// 挂接 URDF 层级场景，返回 robot 根 backendId
	/// @param robotAssembly UrdfRobotLoader::buildHierarchicalRobotScene 根节点
	/// @param displayName 后端树显示名
	/// @return 失败为空
	QString addHierarchicalRobotScene(osg::Group* robotAssembly, const QString& displayName);

	/// 移除 URDF 层级场景
	void removeHierarchicalRobotScene(const QString& backendId);
	void setInstructionPoseAxes(const std::vector<InstructionPoseAxis>& axes);
	void clearInstructionPoseAxes();

	struct RobotFrameOverlayUpdate
	{
		std::string robotRootBackendId;
		bool showToolFrames = false;
		struct ToolEntry
		{
			std::string name;
			/// 非空时挂到该连杆/后端 PAT；空则挂到 robotRoot 下 URDF 装配子图（层级导入）。
			std::string mountBackendId;
			osg::Matrixd localMatrix;
			bool active = false;
		};
		std::vector<ToolEntry> toolFrames;
		bool showUserFrames = false;
		struct UserEntry
		{
			std::string name;
			std::string mountBackendId;
			osg::Matrixd localMatrix;
		};
		std::vector<UserEntry> userFrames;
	};
	void setRobotFrameOverlays(const RobotFrameOverlayUpdate& update);
	void clearRobotFrameOverlays(const std::string& robotRootBackendId);

	/// TCP 末端拖动示教：场景 overlay 罗盘，拖动发位姿信号（不写指令）
	bool isTcpDragTeachActive() const { return m_tcpTeachActive; }
	bool isTcpDragGizmoDragging() const { return m_tcpTeachDragging || m_tcpTeachRotating; }
	/// 进入 TCP 示教
	/// @param mountBackendId TCP 挂载后端 PAT id
	/// @param T_base_target 机器人基座系目标 TCP 位姿
	/// @param modelDiagonalMm 模型对角线 mm，罗盘屏幕缩放
	/// @param resolveRobotBaseWorld 可选，解析基座世界矩阵；基座/工具坐标换算
	/// @param toolLocalOnFlange 非空则按法兰局部工具矩阵放置 TCP
	void beginTcpDragTeach(
		const std::string& mountBackendId,
		const engine::RigidTransform& T_base_target,
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

	/// Optional per-frame callback (e.g. follow-attachment solve); runs on the viewer frame timer.
	void setPerFrameHook(std::function<void(OsgWidget*)> fn);
	/// True while object or TCP-teach gizmo translate/rotate drag is active (skip automatic follower pose overwrite).
	bool isTransformGizmoDragging() const;
	/// Apply \a data pose/rotation to the outer PAT using cached model center (backend as pose authority).
	bool syncOuterPatFromBackend(const BackendDataBase& data);
	/// Push current \c ObjectGizmoFrame (from active outer PAT) through \c syncActiveBackendRootFromObjectFrame (non-drag).
	void syncActiveBackendRootFromSelectedTransform();
	/// Read active backend outer world matrix into backend pose/rotation (OSG authority before follow solve).
	bool writeActiveBackendPoseFromOsg(BackendDataBase& data);
	/// Orbit manipulator center tracks the world origin of this backend (empty disables).
	void setCameraFollowBackendId(std::string backendId);
	void clearCameraFollowBackendId();
	const std::string& cameraFollowBackendId() const { return m_cameraFollowBackendId; }

	// TCP 示教罗盘（\ref RobotTcpDragTeachOperation 友元，约定同 OsgScene 对象 gizmo）
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
	/// 从 \c m_tcpTeachMountPat 同步 \c m_tcpTeachWorldPat（overlay 不随机器人子树光照）
	void syncTcpTeachWorldPatFromMount();
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
	osg::ref_ptr<osg::Group> m_tcpTeachOverlayGroup;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_tcpTeachCompassTransform;
	osg::ref_ptr<osg::MatrixTransform> m_tcpTeachCompassScaleTransform;
	osg::ref_ptr<osg::Node> m_tcpTeachCompassNode;
	osg::ref_ptr<osg::MatrixTransform> m_tcpTeachAxisBranch[3];
	osg::ref_ptr<osg::MatrixTransform> m_tcpTeachRingBranch[3];

signals:
	void selectedObjectPoseChanged(float x, float y, float z);
	void selectedObjectRotationChanged(float rx, float ry, float rz);
	void selectedObjectColorChanged(float r, float g, float b, float a);
	/// Emitted once when translate/rotate gizmo drag ends (left/right release after an active drag).
	void transformGizmoCommitted();
	/// TCP 示教拖动中位姿更新（基座系工具原点 mm + 欧拉 deg）。
	void tcpDragTeachPoseChanged(double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg, double ezDeg);
	void tcpDragTeachEnded();
	void backendObjectPicked(const QString& backendId);
	void activeAxisChanged(const QString& axisName);
	void selectionCanceledByEsc();
	void pointPickFeedback(const QString& text);
	void meshPickFeedback(const QString& text);
	void annotationCreated(const QString& annotationId, const QString& displayText);
	void annotationRemoved(const QString& annotationId);
	void annotationVisibilityChanged(const QString& annotationId, bool visible);
	/// Emitted together with each \ref OsgScene::requestRedraw (scene graph or camera changed).
	void sceneRedrawRequested();

private:
	void initViewer();
	void initUi();
	osg::Node* loadXyzPointCloud(const QString& filePath, QString* errorMessage);
	osg::Node* loadAsciiPlyPointCloud(const QString& filePath, QString* errorMessage);
	osg::Node* createCompassNode();
	bool eventFilter(QObject* watched, QEvent* event) override;
	/// \param outPickedRing 若非空：命中旋转环时为 true，命中轴线段时为 false。
	DragAxis pickAxisAtScreenPos(const QPoint& mousePos, bool preferRing, bool* outPickedRing = nullptr) const;
	void applyColorToActiveBackendObject(const osg::Vec4& color);
	void applyColorToBackendObject(const std::string& backendId, const osg::Vec4& color);
	void applyColorToStagingGeometry(const osg::Vec4& color);
	osg::ref_ptr<osg::Geode> buildPointCloudGeode(const PointCloudBackendData& data, QString* errorMessage) const;
	bool upsertPointCloudBranchInScene(const PointCloudBackendData& data, QString* errorMessage, bool resetViewToHome);
	osg::ref_ptr<osg::Node> buildMeshGeode(const MeshBackendData& data, QString* errorMessage,
		bool showWireOutline = true, bool useSceneLighting = false) const;
	bool upsertMeshBranchInScene(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome,
		bool showWireOutline = true, bool useSceneLighting = false);
	osg::Node* stagingGeometryRoot() const;
	void applyVisibilityMaskForBackend(const std::string& backendId);
	void updateCompassHighlight(DragAxis axis, bool highlightRing = false);
	QString axisToString(DragAxis axis) const;
	void updateCompassScale();
	void refreshCompassDrawVisibility();
	/// World: gizmo axes align with world X/Y/Z; Local: axes follow object. Pivot stays at model origin (compass).
	void syncCompassGizmoOrientation();
	/// 环境变量 \c POINTCLOUD_GIZMO_PIVOT_DIAG 非空且不为 \c "0" 时：经 RunLogger 输出枢轴与场景图文件原点对比（调试用）。
	void logGizmoPivotDiagnostics(const char* reasonTag) const;
	void attachCompassGraphics();
	void detachCompassGraphics();
	void syncCameraManipulatorForModes();
	bool pickAndActivateBackendAtScreenPos(const QPoint& mousePos);
	void clearPointAnnotations();
	bool pickPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld) const;
	bool pickNearestPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx, bool previewOnly) const;
	bool pickPointByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx) const;
	void addPointAnnotation(const osg::Vec3f& pointWorld);
	void updatePointPickMarker(const osg::Vec3f& pointWorld, bool hit);
	void clearPointPickMarker();
	void refreshAnnotationTexts();
	void emitTcpDragTeachPoseChanged();

private:
	QWidgetViewer* m_glWidget = nullptr;
	QTimer m_frameTimer;
	mutable QElapsedTimer m_feedbackTimer;
	QPoint m_lastMousePos;
	std::unique_ptr<OsgWidgetImportController> m_importController;
	std::unique_ptr<OsgWidgetBackendLoadController> m_backendLoadController;
	std::unique_ptr<OsgWidgetCaptureController> m_captureController;
	std::unique_ptr<OsgWidgetPickAnnotationController> m_pickAnnotationController;
	std::unique_ptr<SelectionOperation> m_pointPickOperation;
	std::unique_ptr<SelectionOperation> m_objectTransformOperation;
	std::unique_ptr<SelectionOperation> m_tcpDragTeachOperation;
	std::unique_ptr<SelectionOperation> m_meshElementPickOperation;
	/// 使用场景光照加载的网格后端（如 URDF 连杆），改色时保留 Material+LIGHTING。
	std::unordered_set<std::string> m_litMeshBackendIds;
	osg::ref_ptr<osg::Group> m_instructionPoseAxesGroup;
	/// Instruction TCP axes parented directly under link containers (must detach before rebuild/clear).
	std::vector<osg::ref_ptr<osg::MatrixTransform>> m_instructionPoseAxisNodes;
	struct RobotFrameOverlayNodes
	{
		std::vector<osg::ref_ptr<osg::MatrixTransform>> toolNodes;
		std::vector<osg::ref_ptr<osg::MatrixTransform>> userNodes;
	};
	std::unordered_map<std::string, RobotFrameOverlayNodes> m_robotFrameOverlayNodes;

	std::function<void(OsgWidget*)> m_perFrameHook;
	std::string m_cameraFollowBackendId;

	void updateCameraFollowCenter();

	bool pickMeshFaceByRayIntersection(const QPoint& mousePos,
		osg::Vec3f& outPointWorld,
		osg::Vec3f& outAWorld,
		osg::Vec3f& outBWorld,
		osg::Vec3f& outCWorld,
		osg::Vec3f& outNormalWorld,
		std::vector<osg::Vec3f>* outMergedCoplanarVertsWorld = nullptr) const;

	bool pickMeshEdgeByRayIntersection(const QPoint& mousePos,
		osg::Vec3f& outPointWorld,
		osg::Vec3f& outEdgeAWorld,
		osg::Vec3f& outEdgeBWorld,
		double* outEdgeDistancePx = nullptr) const;

	using OsgScene::showMeshFaceHighlight;
	using OsgScene::showMeshEdgeHighlight;
	using OsgScene::hideMeshElementHighlight;
};

