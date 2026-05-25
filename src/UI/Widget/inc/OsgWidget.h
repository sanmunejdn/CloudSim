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
#include "../../OsgWidgetCore/inc/OsgScene.h"

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
/// 位姿经 syncTcpTeachWorldPatFromMount 与 mount PAT 对齐）
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
		/// 路点 IK 可达性（原点标记绿/红）
		bool reachable = true;
		std::string robotBackendId; /// 局部父节点；空表示世界 overlay
		/// per-link：T_base_tcp 挂 robotBackendId 的 PAT 下（勿用子装配子图，避免漂到场景原点）
		bool mountTcpOnPatRoot = false;
		bool hasLocalMatrix = false;
		double localMatrix[16]{};
		/// URDF 连杆名；非空时 \a localMatrix 为该连杆容器系 inv(T_link)*T_tcp，轴挂于 \c "{name}_Container"
		std::string urdfTcpAttachLinkName;
	};

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
	bool captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage = nullptr);
	bool captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts, QString* errorMessage = nullptr);
	bool loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage = nullptr,
		bool resetViewToHome = true);
	bool loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage = nullptr, bool resetViewToHome = true,
		bool showWireOutline = true, bool useSceneLighting = true, bool skipInnerModelCenterRebase = false);
	/// 受光网格后端（如 URDF 连杆）；改色时保留光照材质
	bool isBackendMeshLit(const std::string& backendId) const;
	void clearImportedContent();
	/// 仅清导入预览，保留已注册后端可视
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
	bool setAnnotationVisible(const QString& annotationId, bool visible);
	bool removeAnnotation(const QString& annotationId);
	void clearAllAnnotations();
	QList<AnnotationSnapshot> annotationSnapshots() const;
	void restoreAnnotations(const QList<AnnotationSnapshot>& snapshots);
/// 随 Qt 深/浅主题设置 OSG 背景色
	void setViewerBackgroundForDarkUi(bool dark);
	/// 至少一个后端有几何或存在导入预览
	bool hasImportedContent() const;
	/// Viewer 根节点（\c setSceneData），供场景树 UI/调试
	const osg::Group* sceneGraphRoot() const { return m_root.get(); }
	/// 绕 \a pivotWorld 刚体旋转各后端根（左乘姿态）
	void applyRigidRotationAboutWorldPivot(const std::vector<std::string>& backendIds, const osg::Vec3f& pivotWorld,
		const osg::Quat& deltaRotation);
	osg::Vec3f averageBackendRootPositionWorld(const std::vector<std::string>& backendIds) const;
	/// \a backendId 外层 PAT 世界矩阵（含父链）
	bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const override;
	/// 设外层 PAT 世界矩阵为 \a worldMat（含父链）
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat) override;
	bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy, double& outCz) const override;
	void syncRobotMeshBackendPoseAfterKinematics(const BackendDataBase& mesh) override;

	/// 添加层级机器人场景，返回后端 id
	/// @param robotAssembly 机器人场景根（UrdfRobotLoader::buildHierarchicalRobotScene）
	/// @param displayName 后端树显示名
	/// @return 机器人后端 id，失败为空
	QString addHierarchicalRobotScene(osg::Group* robotAssembly, const QString& displayName);

	/// 移除层级化机器人场景图。
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
	/// @param modelDiagonalMm 参考模型对角线 mm，罗盘屏幕缩放
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

	/// 帧定时器回调（如跟随求解）
	void setPerFrameHook(std::function<void(OsgWidget*)> fn);
	/// 对象/TCP 示教 gizmo 拖拽中，跳过跟随位姿覆写
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
	/// 场景 overlay 上的世界位姿节点（罗盘挂此，不挂在受光机器人子树下）
	osg::ref_ptr<osg::MatrixTransform> m_tcpTeachWorldPat;
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
	/// 平移/旋转 gizmo 拖拽结束
	void transformGizmoCommitted();
	/// TCP 示教拖动中位姿更新（基座系 mm + 欧拉 deg）
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
	/// 随 OsgScene::requestRedraw 发出（场景或相机变更）
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
		bool showWireOutline = true, bool useSceneLighting = false, bool skipInnerModelCenterRebase = false);
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
	/// 指令 TCP 轴直接挂连杆容器，重建/清空前须 detach
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
		std::vector<osg::Vec3f>* outMergedCoplanarVertsWorld = nullptr,
		const std::string* scopeBackendId = nullptr) const;

	bool pickMeshEdgeByRayIntersection(const QPoint& mousePos,
		osg::Vec3f& outPointWorld,
		osg::Vec3f& outEdgeAWorld,
		osg::Vec3f& outEdgeBWorld,
		double* outEdgeDistancePx = nullptr,
		const std::string* scopeBackendId = nullptr) const;

	using OsgScene::showMeshFaceHighlight;
	using OsgScene::showMeshEdgeHighlight;
	using OsgScene::hideMeshElementHighlight;
};

