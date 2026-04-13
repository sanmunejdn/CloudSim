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
#include "../OsgWidgetCore/inc/OsgScene.h"

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
class MeshEdgeFacePickOperation;
class PointCloudBackendData;
class MeshBackendData;
class OsgWidgetImportController;
class OsgWidgetBackendLoadController;
class OsgWidgetCaptureController;
class OsgWidgetPickAnnotationController;
class OsgWidgetColorController;
class OsgWidgetTransformHierarchyController;
struct MeshCapturedPart;

/// 三维视图核心控件：封装 OSG Viewer、相机与场景根节点，负责导入、后端对象显示、拾取与标注等。
class WIDGET_EXPORT OsgWidget : public QWidget, public IRobotBackendPoseSink, public OsgScene
{
	Q_OBJECT
public:
	using DragAxis = OsgScene::DragAxis;
	friend class PointPickOperation;
	friend class ObjectTransformOperation;
	friend class MeshEdgeFacePickOperation;
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
		QString backendId; // empty => legacy/unknown
		/// Legacy: offset in m_selectedTransform space at save time (older projects only).
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
	/// 该后端网格是否以场景光照渲染（如 URDF 连杆），用于改色时保留光照材质。
	bool isBackendMeshLit(const std::string& backendId) const;
	void clearImportedContent();
	/// Clears import preview only (keeps already registered backend visuals).
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
	/// Rigid-body rotation: each backend root rotates about \a pivotWorld by \a deltaRotation (left-multiply attitude).
	void applyRigidRotationAboutWorldPivot(const std::vector<std::string>& backendIds, const osg::Vec3f& pivotWorld,
		const osg::Quat& deltaRotation);
	osg::Vec3f averageBackendRootPositionWorld(const std::vector<std::string>& backendIds) const;
	/// World matrix of the outer PAT for \a backendId (respects parent chain under the scene).
	bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const override;
	/// Sets outer PAT pose so its world matrix equals \a worldMat (respects parent chain).
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat) override;

	/// 【中文】添加层级化机器人场景图（动态层级法），返回场景节点的 ID 用于后续管理。
	/// 【English】Add hierarchical robot scene graph (dynamic hierarchy method).
	/// @param robotAssembly The root node of the robot scene (e.g., from UrdfRobotLoader::buildHierarchicalRobotScene)
	/// @param displayName Display name for the robot in the backend tree
	/// @return Backend ID assigned to the robot scene, empty if failed
	QString addHierarchicalRobotScene(osg::Group* robotAssembly, const QString& displayName);

	/// 【中文】移除层级化机器人场景图。
	void removeHierarchicalRobotScene(const QString& backendId);

signals:
	void selectedObjectPoseChanged(float x, float y, float z);
	void selectedObjectRotationChanged(float rx, float ry, float rz);
	void selectedObjectColorChanged(float r, float g, float b, float a);
	void activeAxisChanged(const QString& axisName);
	void selectionCanceledByEsc();
	void pointPickFeedback(const QString& text);
	void annotationCreated(const QString& annotationId, const QString& displayText);
	void annotationRemoved(const QString& annotationId);
	void annotationVisibilityChanged(const QString& annotationId, bool visible);

private:
	void initViewer();
	void initUi();
	osg::Node* loadXyzPointCloud(const QString& filePath, QString* errorMessage);
	osg::Node* loadAsciiPlyPointCloud(const QString& filePath, QString* errorMessage);
	osg::Node* createCompassNode();
	bool eventFilter(QObject* watched, QEvent* event) override;
	DragAxis pickAxisAtScreenPos(const QPoint& mousePos, bool preferRing) const;
	void applyColorToActiveBackendObject(const osg::Vec4& color);
	void applyColorToBackendObject(const std::string& backendId, const osg::Vec4& color);
	void applyColorToStagingGeometry(const osg::Vec4& color);
	void syncActiveBackendRootFromSelectedTransform();
	osg::ref_ptr<osg::Geode> buildPointCloudGeode(const PointCloudBackendData& data, QString* errorMessage) const;
	bool upsertPointCloudBranchInScene(const PointCloudBackendData& data, QString* errorMessage, bool resetViewToHome);
	osg::ref_ptr<osg::Node> buildMeshGeode(const MeshBackendData& data, QString* errorMessage,
		bool showWireOutline = true, bool useSceneLighting = false) const;
	bool upsertMeshBranchInScene(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome,
		bool showWireOutline = true, bool useSceneLighting = false);
	osg::Node* stagingGeometryRoot() const;
	void applyVisibilityMaskForBackend(const std::string& backendId);
	void updateCompassHighlight(DragAxis axis);
	QString axisToString(DragAxis axis) const;
	void updateCompassScale();
	void refreshCompassDrawVisibility();
	/// World: gizmo axes align with world X/Y/Z; Local: axes follow object. Pivot stays at model origin (compass).
	void syncCompassGizmoOrientation();
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
	std::unique_ptr<SelectionOperation> m_meshElementPickOperation;
	/// 使用场景光照加载的网格后端（如 URDF 连杆），改色时保留 Material+LIGHTING。
	std::unordered_set<std::string> m_litMeshBackendIds;

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
		osg::Vec3f& outEdgeBWorld) const;

	using OsgScene::showMeshFaceHighlight;
	using OsgScene::showMeshEdgeHighlight;
	using OsgScene::hideMeshElementHighlight;
};

