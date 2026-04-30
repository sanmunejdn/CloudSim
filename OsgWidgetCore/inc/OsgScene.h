#pragma once

#include "osgwidgetcore_global.h"
#include "BackendVisualBindingIndex.h"

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
}

namespace osgViewer {
class Viewer;
class GraphicsWindow;
}

class BackendDataBase;

/// OSG 场景图与相机状态（无 Qt）。重绘通过 \ref setRequestRedraw 注入；视口像素通过 \ref setViewportPixels 同步。
class OSGWIDGETCORE_EXPORT OsgScene
{
public:
	enum class DragAxis { None, X, Y, Z };
	/// Gizmo axes: Local = object/body; World = world-aligned at pivot (see syncCompassGizmoOrientation).
	enum class TransformGizmoFrame { World, Local };

	// Gizmo 轴编号（与历史 OsgWidgetGizmoController 一致）：0=None,1=X,2=Y,3=Z
	static constexpr int kGizmoAxisNone = 0;
	static constexpr int kGizmoAxisX = 1;
	static constexpr int kGizmoAxisY = 2;
	static constexpr int kGizmoAxisZ = 3;

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
	void updateWorldAxesHudViewport(int widgetWidth, int widgetHeight);

	static osg::Quat eulerDegToQuat(const osg::Vec3f& eulerDeg);
	static osg::Vec3f quatToEulerDeg(const osg::Quat& q);

	/// 清除 osgGA 事件队列中卡住的按键/按钮状态（相机漫游恢复）。
	void resetNavigationInputQueues();

	bool isBackendDescendantOf(const std::string& backendId, const std::string& ancestorId) const;
	void cacheSelectionPoseFromSelectedTransform();
	void syncActiveBackendRootFromSelectedTransform();

	void syncGizmoAndPickFromBackend(const BackendDataBase& data);

	osg::Vec3f computePointCloudCenterFromXyz(const std::vector<float>& xyz) const;
	float computePointCloudDiagonalFromXyz(const std::vector<float>& xyz) const;
	osg::Vec3f computeMeshCenterFromSoup(const std::vector<float>& soup) const;
	float computeMeshDiagonalFromSoup(const std::vector<float>& soup) const;

	osg::Node* createCompassNode();
	void attachCompassGraphics();
	void detachCompassGraphics();
	void refreshCompassDrawVisibility();
	void updateCompassHighlight(int axis);
	/// Places the compass at mesh/point model origin (file 0,0,0), not bbox center, when geometry uses inner PAT at -m_modelCenter.
	void updateCompassLocalOffsetForModelOrigin();
	void updateCompassScale();
	int pickAxisAtScreenPos(double mouseX, double mouseY, bool preferRing) const;

	void focusCameraOnBackend(const std::string& backendId);

	bool pickAndActivateBackendAtScreenPos(double mouseX, double mouseY);
	void cachePickablePointsFromNode(osg::Node* node);
	bool pickPointAtScreenPos(double mouseX, double mouseY, osg::Vec3f& outPointWorld) const;
	bool pickNearestPointAtScreenPos(double mouseX, double mouseY, osg::Vec3f& outPointWorld, double& outDistancePx,
		bool previewOnly) const;
	bool pickPointByRayIntersection(double mouseX, double mouseY, osg::Vec3f& outPointWorld, double& outDistancePx) const;
	bool pickMeshFaceByRayIntersection(double mouseX, double mouseY, osg::Vec3f& outPointWorld, osg::Vec3f& outAWorld,
		osg::Vec3f& outBWorld, osg::Vec3f& outCWorld, osg::Vec3f& outNormalWorld,
		std::vector<osg::Vec3f>* outMergedCoplanarVertsWorld = nullptr) const;
	bool pickMeshEdgeByRayIntersection(double mouseX, double mouseY, osg::Vec3f& outPointWorld, osg::Vec3f& outEdgeAWorld,
		osg::Vec3f& outEdgeBWorld) const;
	void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld);
	void showMeshFaceHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld, const osg::Vec3f& cWorld);
	void showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld);
	void hideMeshElementHighlight();
	void bindBackendVisualRoot(const std::string& backendId, osg::Node* rootNode);
	void unbindBackendVisualRoot(const std::string& backendId);
	void clearBackendVisualBindings();
	bool resolveBackendIdFromPickedPath(const osg::NodePath& path, std::string& outBackendId) const;

	void rebuildPointKdTree();
	void nearestCandidatesByKdTree(const osg::Vec3f& queryLocalCentered, int k, std::vector<int>& outIndices) const;

	osg::ref_ptr<osgViewer::Viewer> m_viewer;
	/// 与 GL_LIGHT0 对应；通过 View::HEADLIGHT 随相机移动（等价于头灯挂在视点）。
	osg::ref_ptr<osg::Light> m_headlight;
	osg::ref_ptr<osgGA::TrackballManipulator> m_trackballManipulator;
	osg::ref_ptr<osgViewer::GraphicsWindow> m_graphicsWindow;

	osg::ref_ptr<osg::Group> m_root;
	/// 主场景内容分层父节点（标注、导入物、机器人、轨迹等），子节点顺序影响默认绘制先后。
	osg::ref_ptr<osg::Group> m_sceneContentGroup;
	/// 导入网格/点云对应的后端根 PAT 挂接于此（与机器人装配分离，便于仿真与资源管理）。
	osg::ref_ptr<osg::Group> m_backendObjectsGroup;
	/// URDF / 关节层级机器人场景（参见 OsgWidget::addHierarchicalRobotScene）。
	osg::ref_ptr<osg::Group> m_robotAssemblyGroup;
	/// 预留：轨迹、路径、调试曲线等覆盖几何。
	osg::ref_ptr<osg::Group> m_trajectoryOverlayGroup;
	osg::ref_ptr<osg::Group> m_stagingGroup;
	std::unordered_map<std::string, osg::ref_ptr<osg::PositionAttitudeTransform>> m_backendObjectRoots;
	std::unordered_map<std::string, std::string> m_backendParentIds;
	std::unordered_map<std::string, osg::Vec3f> m_backendModelCenters;
	std::unordered_map<std::string, bool> m_backendVisibility;
	BackendVisualBindingIndex m_backendVisualBindings;
	std::string m_activeBackendId;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_activeBackendOuterPat;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_selectedTransform;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_compassTransform;
	osg::ref_ptr<osg::Group> m_annotationGroup;
	osg::ref_ptr<osg::AutoTransform> m_pickFeedbackTransform;
	osg::ref_ptr<osg::Node> m_compassNode;
	osg::ref_ptr<osg::Node> m_pickFeedbackNode;
	osg::ref_ptr<osg::Vec4Array> m_compassColors;
	osg::ref_ptr<osg::Vec4Array> m_ringColorX;
	osg::ref_ptr<osg::Vec4Array> m_ringColorY;
	osg::ref_ptr<osg::Vec4Array> m_ringColorZ;
	osg::ref_ptr<osg::Camera> m_worldAxesHudCamera;

	bool m_selectionActive = false;
	bool m_objectSelectionMode = false;
	bool m_pointPickMode = false;
	bool m_meshLinePickMode = false;
	bool m_meshFacePickMode = false;
	bool m_dragging = false;
	bool m_rotating = false;
	DragAxis m_dragAxis = DragAxis::None;
	DragAxis m_hoverAxis = DragAxis::None;
	osg::Vec3f m_modelCenter = osg::Vec3f(0.0f, 0.0f, 0.0f);
	double m_gizmoReferenceDistance = -1.0;
	double m_gizmoReferenceScale = 1.0;
	float m_activeModelDiagonal = 1.0f;
	bool m_darkUiTheme = false;
	bool m_hasLastSelectionPose = false;
	osg::Vec3f m_lastSelectionPos = osg::Vec3f(0.0f, 0.0f, 0.0f);
	osg::Quat m_lastSelectionAtt;
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
	osg::ref_ptr<osg::Geometry> m_meshPickedFaceGeom;
	osg::ref_ptr<osg::Geometry> m_meshPickedEdgeGeom;
	osg::ref_ptr<osg::Vec3Array> m_meshPickedFaceVertices;
	osg::ref_ptr<osg::Vec3Array> m_meshPickedEdgeVertices;
	osg::ref_ptr<osg::Vec4Array> m_meshPickedFaceColors;
	osg::ref_ptr<osg::Vec4Array> m_meshPickedEdgeColors;

private:
	int buildKdNode(std::vector<int>& indices, int begin, int end, int depth);
	int nearestPointByKdTree(const osg::Vec3f& queryLocalCentered) const;

protected:
	std::function<void()> m_requestRedraw;
	int m_viewportWidth = 640;
	int m_viewportHeight = 480;
	double m_devicePixelRatio = 1.0;
	TransformGizmoFrame m_transformGizmoFrame = TransformGizmoFrame::Local;
};
