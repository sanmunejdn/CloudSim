#pragma once

#include "osgwidgetcore_global.h"
#include "BackendPickIndexRegistry.h"
#include "BackendVisualBindingIndex.h"
#include "ObjectGizmoFrame.h"
#include "PickTypes.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <osgGA/TrackballManipulator>
#include <osg/Camera>
#include <osg/Light>
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

namespace osg {
class Group;
class Node;
class Geometry;
class Image;
}

#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>

class BackendDataBase;

namespace geoalgo {
struct Point3d;
struct BrepImportArtifacts;
}

/// OSG 场景图与相机（无 Qt）
///
/// 后端对象绑定、拾取、对象变换罗盘（\c m_gizmoOverlayGroup）及 TCP 示教场景 overlay
/// （\c m_tcpTeachSceneOverlayGroup，几何由 \c OsgWidget 挂载）。
/// 重绘经 \ref setRequestRedraw 注入；视口经 \ref setViewportPixels 与 DPR 同步。
class OSGWIDGETCORE_EXPORT OsgScene
{
public:
	enum class DragAxis { None, X, Y, Z };
	/// Local：物体轴；World：枢轴处对齐世界轴
	enum class TransformGizmoFrame { World, Local };
	/// 标准相机视角（Z-up，保持当前 pivot 与视距）
	enum class CameraViewPreset { Front, Back, Left, Right, Top, Bottom, Iso };

/// Gizmo 轴编号（与历史 OsgWidgetGizmoController 一致）：0=None,1=X,2=Y,3=Z
	static constexpr int kGizmoAxisNone = 0;
	static constexpr int kGizmoAxisX = 1;
	static constexpr int kGizmoAxisY = 2;
	static constexpr int kGizmoAxisZ = 3;

	/// 屏幕拾取共用阈值（hover/click 须一致，避免预览与提交分裂）
	static constexpr double kPointPickHitRadiusPx = 32.0;
	/// 仅在此半径内显示绿色预览环（小于 hit 半径，避免环与光标相距过远）
	static constexpr double kPointPickPreviewRadiusPx = 18.0;
	static constexpr double kMeshEdgeHitRadiusPx = 18.0;
	static constexpr int kPickClickMoveThresholdPx = 25;
	static constexpr int kPickDragMoveThresholdPx = 25;
	static constexpr int kPickClickHoldMs = 150;
	static constexpr int kPickHoverThrottleMs = 16;
	static constexpr int kPickHoverEdgeThrottleMs = 40;
	static constexpr int kPickHoverMinMovePx = 4;
	static constexpr unsigned int kMaskPickContent = 0x1u;
	static constexpr unsigned int kMaskPickOverlay = 0x2u;

	struct AnnotationEntry
	{
		std::string id;
		std::string displayText;
		osg::ref_ptr<osg::AutoTransform> transform;
		osg::ref_ptr<osg::MatrixTransform> scaleBranch;
		osg::ref_ptr<osgText::Text> textDrawable;
		std::string backendId;
		osg::Vec3f localCentered = osg::Vec3f(0.0f, 0.0f, 0.0f);
		osg::Vec3f worldAnchor{};
		bool hasWorldAnchor = false;
		bool visible = true;
	};

	OsgScene();
	virtual ~OsgScene() = default;

	void setRequestRedraw(std::function<void()> fn) { m_requestRedraw = std::move(fn); }
	void requestRedraw() const;
	void setViewportPixels(int w, int h);
	void setDevicePixelRatio(double dpr) { m_devicePixelRatio = dpr > 0.0 ? dpr : 1.0; }
	int viewportWidth() const { return m_viewportWidth; }
	int viewportHeight() const { return m_viewportHeight; }
	double devicePixelRatio() const { return m_devicePixelRatio; }

	void initScene();
	osg::Group* sceneContentRoot() const { return m_sceneContentGroup.get(); }
	osg::Group* backendObjectsRoot() const { return m_backendObjectsGroup.get(); }
	osg::Group* robotAssemblyRoot() const { return m_robotAssemblyGroup.get(); }
	osg::Group* trajectoryOverlayRoot() const { return m_trajectoryOverlayGroup.get(); }
	/// 将头灯绑定到 Viewer（osg::View::HEADLIGHT，每帧随主相机视点更新；主场景由 setSceneData 提供，不宜把 LightSource 直接挂到 Camera 子图）。
	void applyHeadlightToViewer(osgViewer::Viewer* viewer);
	void initWorldAxesHud();
	/// @a framebufferWidth/@a framebufferHeight 为 GL 帧缓冲设备像素
	void updateWorldAxesHudViewport(int framebufferWidth, int framebufferHeight);
	void initViewCubeHud();
	void updateViewCubeHudViewport(int framebufferWidth, int framebufferHeight);
	/// 用预渲染位图替换 osgText 面标签（OsgWidgetCore 无 freetype 插件时 osgText 无法加载 CJK）
	void applyViewCubeFaceLabelImages(const osg::ref_ptr<osg::Image> images[6]);
	/// 视口逻辑坐标（Qt 左上角原点）点击视角立方体；命中则切换相机并返回 true
	bool tryPickViewCubeAtLogicalMouse(double logicalX, double logicalY);
	void setCameraViewDirection(const osg::Vec3d& eyeDirectionFromCenter, const osg::Vec3d& upHint);

	static osg::Quat eulerDegToQuat(const osg::Vec3f& eulerDeg);
	static osg::Vec3f quatToEulerDeg(const osg::Quat& q);

	/// 清除 osgGA 事件队列中卡住的按键/按钮状态（相机漫游恢复）。
	void resetNavigationInputQueues();

	/// 帧定时器：非空标注每帧缩放
	bool hasPointAnnotations() const { return !m_annotations.empty(); }

	bool isBackendDescendantOf(const std::string& backendId, const std::string& ancestorId) const;
	/// \a childBackendId 外层 PAT 是否为 \a ancestorBackendId 子节点（走真实父链，避免重复 gizmo 增量）
	bool backendOuterPatIsUnderOuterPatInSceneGraph(const std::string& childBackendId, const std::string& ancestorBackendId) const;
	void cacheSelectionGizmoPose();
	const std::string& activeBackendId() const { return m_activeBackendId; }
	/// 写活动外层 PAT；非拖拽时将旋转增量传播到子孙根
	void syncActiveBackendRootFromObjectFrame(const ObjectGizmoFrame& cur, bool dragging);
	bool readActiveObjectGizmoFrame(ObjectGizmoFrame& out) const;
	void attachGizmoOverlayToActiveBackend();
	void detachGizmoOverlay();

	void syncGizmoAndPickFromBackend(const BackendDataBase& data);

	osg::Vec3f computePointCloudCenterFromXyz(const std::vector<float>& xyz) const;
	float computePointCloudDiagonalFromXyz(const std::vector<float>& xyz) const;
	osg::Vec3f computeMeshCenterFromSoup(const std::vector<float>& soup) const;
	float computeMeshDiagonalFromSoup(const std::vector<float>& soup) const;

	/// 构建对象变换罗盘几何（轴+环），StateSet 走 \c osg_compass::applyUnlitHighlitStateSet
	osg::Node* createCompassNode();
	/// 将罗盘挂到当前选中后端 \c m_gizmoOverlayGroup
	void attachCompassGraphics();
	void detachCompassGraphics();
	void refreshCompassDrawVisibility();
	/// \param highlightRing 为 true 时仅放大对应旋转环；为 false 时放大对应正半轴（平移拾取）。
	void updateCompassHighlight(int axis, bool highlightRing = false);
	void syncCompassGizmoOrientation();
	void updateCompassScale();
	/// \param outPickedRing 若非空：命中旋转环时为 true，命中轴线段时为 false。
	int pickAxisAtScreenPos(double mouseX, double mouseY, bool preferRing, bool* outPickedRing = nullptr) const;
	/// Qt 逻辑像素；内部乘 DPR 对齐 OSG 视口
	bool computeCameraScreenRayWorld(double mouseX, double mouseY, osg::Vec3d& outRayOriginWorld, osg::Vec3d& outRayDirUnitWorld) const;
	void computeGizmoPivotWorld(osg::Vec3f& outPivotWorld) const;
	/// 与罗盘显示一致的单位轴方向（世界坐标）。
	bool gizmoCompassUnitAxisWorld(DragAxis axis, osg::Vec3d& outAxisWorld) const;
	/// 平移拖动：冻结屏幕轴 + mm/px（与 TCP 示教相同，避免移动 pivot 时平面求交发散）。
	bool beginGizmoScreenDrag(DragAxis axis);
	double gizmoScreenDragDs(double mouseXCur, double mouseYCur, double mouseXLast, double mouseYLast) const;
	/// 旋转拖动：绕罗盘枢轴的屏幕角（与平移相同，避免姿态变化时平面求交发散）。
	bool beginGizmoScreenRotate(DragAxis axis, double mouseX, double mouseY);
	double gizmoScreenRotateDeltaRad(double mouseX, double mouseY);
	bool gizmoScreenAngleAtMouse(DragAxis axis, double mouseX, double mouseY, double& outAngleRad) const;
	/// POINTCLOUD_GIZMO_PIVOT_DIAG 非空且非 0 时经 RunLogger 输出枢轴诊断
	void logGizmoPivotDiagnostics(const char* reasonTag) const;

	void focusCameraOnBackend(const std::string& backendId);
	void setCameraViewPreset(CameraViewPreset preset);
	/// 仅写入逻辑父 id（不改 OSG 场景父链），供分件导入后 focusCameraOnBackend 聚合子树包围球
	void setBackendLogicalParent(const std::string& backendId, const std::string& parentBackendId);

	bool pickAndActivateBackendAtScreenPos(double mouseX, double mouseY);
	void cachePickablePointsFromNode(osg::Node* node);
	bool pickPointAtScreenPos(double mouseX, double mouseY, osg::Vec3f& outPointWorld) const;
	bool pickNearestPointAtScreenPos(double mouseX, double mouseY, osg::Vec3f& outPointWorld, double& outDistancePx,
		bool previewOnly) const;
	bool pickPointByRayIntersection(double mouseX, double mouseY, osg::Vec3f& outPointWorld, double& outDistancePx) const;
	bool pickMeshFaceByRayIntersection(double mouseX, double mouseY, osg::Vec3f& outPointWorld, osg::Vec3f& outAWorld,
		osg::Vec3f& outBWorld, osg::Vec3f& outCWorld, osg::Vec3f& outNormalWorld,
		std::vector<osg::Vec3f>* outMergedCoplanarVertsWorld = nullptr,
		const std::string* scopeBackendId = nullptr,
		int* outPickedTriangleIndex = nullptr) const;
	bool pickMeshEdgeByRayIntersection(double mouseX, double mouseY, osg::Vec3f& outPointWorld, osg::Vec3f& outEdgeAWorld,
		osg::Vec3f& outEdgeBWorld, double* outEdgeDistancePx = nullptr,
		const std::string* scopeBackendId = nullptr) const;
	PickResult queryPick(const PickQuery& query);
	void setPickVisualAlias(const std::string& logicalBackendId, const std::string& visualBackendId);
	std::string resolvePickScopeBackendId(const std::string& backendId) const;
	/// pickVisualAlias 反向：visual id → 唯一 logical id；多 alias 时回退 visualId
	std::string resolveLogicalBackendIdFromVisualPick(const std::string& visualBackendId, int brepFaceIndex = -1) const;
	void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld);
	void showMeshFaceHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld, const osg::Vec3f& cWorld);
	void showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld);
	void showMeshEdgeHighlight(const std::vector<osg::Vec3f>& polylineWorld);
	void hideMeshElementHighlight();

	struct FeatureCatalogOverlayItem
	{
		int displayIndex = 0;
		osg::Vec3f anchorWorldMm;
		osg::Vec3f labelWorldMm;
		bool hasEdgeSegment = false;
		osg::Vec3f edgeAWorldMm;
		osg::Vec3f edgeBWorldMm;
	};
	void setFeatureCatalogOverlay(const std::vector<FeatureCatalogOverlayItem>& items);
	void clearFeatureCatalogOverlay();

	void bindBackendVisualRoot(const std::string& backendId, osg::Node* rootNode);
	void bindBackendVisualRoot(const std::string& backendId, osg::Node* rootNode,
		const std::shared_ptr<geoalgo::BrepImportArtifacts>& brepArtifacts);
	void unbindBackendVisualRoot(const std::string& backendId);
	void clearBackendVisualBindings();
	bool resolveBackendIdFromPickedPath(const osg::NodePath& path, std::string& outBackendId) const;

	void rebuildPointKdTree();
	void nearestCandidatesByKdTree(const osg::Vec3f& queryLocalCentered, int k, std::vector<int>& outIndices) const;
	void nearestCandidatesByPickIndex(
		const PickSpatialIndex& index,
		const osg::Vec3f& queryLocalCentered,
		int k,
		std::vector<int>& outIndices) const;
	void importPickSpatialIndexForActiveBackend(const PickSpatialIndex& index);
	const PickSpatialIndex* activePointPickIndex() const;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;
	/// 与 GL_LIGHT0 对应；通过 View::HEADLIGHT 随相机移动（等价于头灯挂在视点）。
	osg::ref_ptr<osg::Light> m_headlight;
	osg::ref_ptr<osgGA::TrackballManipulator> m_trackballManipulator;
	osg::ref_ptr<osgViewer::GraphicsWindow> m_graphicsWindow;

	osg::ref_ptr<osg::Group> m_root;
	/// 主场景内容分层父节点（标注、导入物、机器人、轨迹等），子节点顺序影响默认绘制先后。
	osg::ref_ptr<osg::Group> m_sceneContentGroup;
	/// 导入网格/点云对应的后端根 \c osg::MatrixTransform 挂接于此（与机器人装配分离，便于仿真与资源管理）。
	osg::ref_ptr<osg::Group> m_backendObjectsGroup;
	/// URDF / 关节层级机器人场景（参见 OsgWidget::addHierarchicalRobotScene）。
	osg::ref_ptr<osg::Group> m_robotAssemblyGroup;
	/// 预留：轨迹、路径、调试曲线等覆盖几何。
	osg::ref_ptr<osg::Group> m_trajectoryOverlayGroup;
	/// TCP 示教罗盘：挂在此组下，避免成为受光机器人子节点后随视角变暗。
	osg::ref_ptr<osg::Group> m_tcpTeachSceneOverlayGroup;
	osg::ref_ptr<osg::Group> m_stagingGroup;
	std::unordered_map<std::string, osg::ref_ptr<osg::MatrixTransform>> m_backendObjectRoots;
	std::unordered_map<std::string, std::string> m_backendParentIds;
	std::unordered_map<std::string, osg::Vec3f> m_backendModelCenters;
	std::unordered_map<std::string, bool> m_backendSkipCenterRebase;
	std::unordered_map<std::string, bool> m_backendVisibility;
	BackendVisualBindingIndex m_backendVisualBindings;
	BackendPickIndexRegistry m_backendPickIndexes;
	std::unordered_map<std::string, std::string> m_pickVisualAliases;
	std::string m_activeBackendId;
	osg::ref_ptr<osg::MatrixTransform> m_activeBackendOuterPat;
	/// gizmo overlay 挂内层 PAT 文件原点，attach 前不在场景
	osg::ref_ptr<osg::Group> m_gizmoOverlayGroup;
	osg::ref_ptr<osg::Node> m_gizmoAttachedInner;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_compassTransform;
	/// 仅缩放罗盘几何；挂在 \c m_compassTransform 下，避免 PAT 的 scale 与 position 组合把枢轴拉离模型原点。
	osg::ref_ptr<osg::MatrixTransform> m_compassScaleTransform;
	osg::ref_ptr<osg::Group> m_annotationGroup;
	osg::ref_ptr<osg::AutoTransform> m_pickFeedbackTransform;
	osg::ref_ptr<osg::Node> m_compassNode;
	osg::ref_ptr<osg::Node> m_pickFeedbackNode;
	/// 各轴正半轴与旋转环分支，悬停/拖拽局部缩放高亮
	osg::ref_ptr<osg::MatrixTransform> m_compassAxisBranch[3];
	osg::ref_ptr<osg::MatrixTransform> m_compassRingBranch[3];
	osg::ref_ptr<osg::Camera> m_worldAxesHudCamera;
	osg::ref_ptr<osg::Camera> m_viewCubeHudCamera;
	osg::ref_ptr<osg::Group> m_viewCubeLabelsGroup;
	int m_viewCubeHudMargin = 12;
	int m_viewCubeHudSize = 220;
	int m_worldAxesHudMargin = 10;
	int m_worldAxesHudSize = 120;
	int m_viewCubeHudEffectiveSize = 220;
	int m_worldAxesHudEffectiveSize = 120;

	bool m_selectionActive = false;
	bool m_objectSelectionMode = false;
	bool m_pointPickMode = false;
	bool m_meshLinePickMode = false;
	bool m_meshFacePickMode = false;
	bool m_dragging = false;
	bool m_rotating = false;
	DragAxis m_dragAxis = DragAxis::None;
	DragAxis m_hoverAxis = DragAxis::None;
	/// 平移拖拽冻结平面（过枢轴、含拖拽轴、朝向按下时相机）
	osg::Vec3d m_gizmoTransDragPlaneO{};
	osg::Vec3d m_gizmoTransDragPlaneN{};
	osg::Vec3d m_gizmoDragLastHitWorld{};
	bool m_gizmoTransDragPlaneActive = false;
	double m_gizmoDragScreenAxisUx = 1.0;
	double m_gizmoDragScreenAxisUy = 0.0;
	double m_gizmoDragMmPerPixel = 1.0;
	osg::Vec3d m_gizmoScreenDragAxisWorld{0.0, 0.0, 1.0};
	double m_gizmoRotateLastScreenAngle = 0.0;
	bool m_gizmoRotateScreenActive = false;
	/// 旋转拖拽起始缓存文件原点（外层父局部），保枢轴
	bool m_gizmoRotatePivotActive = false;
	osg::Vec3d m_gizmoRotatePivotInParent{};
	osg::Vec3d m_gizmoRotatePivotWorld{};
	mutable osg::Vec3f m_modelCenter = osg::Vec3f(0.0f, 0.0f, 0.0f);
	double m_gizmoReferenceDistance = -1.0;
	double m_gizmoReferenceScale = 1.0;
	float m_activeModelDiagonal = 1.0f;
	bool m_darkUiTheme = false;
	bool m_hasLastSelectionPose = false;
	/// 上次提交的 center+pose 与姿态，供子孙传播
	osg::Vec3f m_lastGizmoCenterPlusPose = osg::Vec3f(0.0f, 0.0f, 0.0f);
	osg::Quat m_lastGizmoAttitude;
	std::vector<osg::Vec3f> m_pickablePointsLocal;
	std::vector<osg::Vec3f> m_pickablePointsPreviewLocal;
	struct KdNode
	{
		int pointIndex = -1;
		int axis = 0;
		int left = -1;
		int right = -1;
	};
	std::vector<KdNode> m_kdNodes;
	int m_kdRoot = -1;
	std::vector<osg::Vec3f> m_pickablePointsCenteredLocal;
	std::vector<AnnotationEntry> m_annotations;
	int m_annotationCounter = 0;

	osg::ref_ptr<osg::Group> m_meshPickOverlayGroup;
	osg::ref_ptr<osg::Group> m_featureCatalogOverlayGroup;
	osg::ref_ptr<osg::Geometry> m_meshPickedFaceGeom;
	osg::ref_ptr<osg::Geometry> m_meshPickedEdgeGeom;
	osg::ref_ptr<osg::Vec3Array> m_meshPickedFaceVertices;
	osg::ref_ptr<osg::Vec3Array> m_meshPickedEdgeVertices;
	osg::ref_ptr<osg::Vec4Array> m_meshPickedFaceColors;
	osg::ref_ptr<osg::Vec4Array> m_meshPickedEdgeColors;

private:
	struct HudCornerViewport
	{
		int x = 0;
		int y = 0;
		int width = 0;
		int height = 0;
		int effectiveLogicalSize = 0;
	};
	HudCornerViewport computeHudCornerViewport(
		int framebufferWidth,
		int framebufferHeight,
		int marginLogical,
		int nominalSizeLogical,
		bool topRight) const;
	void applyHudSquareOrthoProjection(
		osg::Camera* camera,
		float halfExtent,
		int viewportWidth,
		int viewportHeight) const;
	void logicalMouseToDeviceCoords(double logicalX, double logicalY, double& outDeviceX, double& outDeviceY) const;
	void logicalMouseToPickWindowCoords(double logicalX, double logicalY, double& outWindowX, double& outWindowY) const;
	bool isBrepPickBackend(const std::string& backendId) const;
	bool tryQueryBrepPick(const PickQuery& query, bool pickFace, PickResult& out) const;
	bool getWorldPickRay(double mouseX, double mouseY, osg::Vec3d& outStart, osg::Vec3d& outEnd) const;
	bool backendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const;
	bool worldPointToStepModelMm(const std::string& backendId, const osg::Vec3d& worldMm, geoalgo::Point3d& outModel) const;
	bool stepModelPointToWorldMm(const std::string& backendId, const geoalgo::Point3d& modelMm, osg::Vec3f& outWorld) const;

	int buildKdNode(std::vector<int>& indices, int begin, int end, int depth);
	int nearestPointByKdTree(const osg::Vec3f& queryLocalCentered) const;

protected:
	std::function<void()> m_requestRedraw;
	int m_viewportWidth = 640;
	int m_viewportHeight = 480;
	double m_devicePixelRatio = 1.0;
	TransformGizmoFrame m_transformGizmoFrame = TransformGizmoFrame::Local;
};
