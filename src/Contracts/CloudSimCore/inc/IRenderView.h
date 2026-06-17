#pragma once

#include "CoreTypes.h"
#include "cloudsim_core_global.h"

#include <functional>
#include <memory>

#include <QByteArray>

class QWidget;

namespace cloudsim::core {

using PickHandler = std::function<void(const ObjectId& backendId)>;

/// 文档渲染视口
class CLOUDSIM_CORE_EXPORT IRenderView
{
public:
	virtual ~IRenderView() = default;

	virtual QWidget* widget() = 0;
	virtual const QWidget* widget() const = 0;

	virtual void setWorldMatrix(const ObjectId& id, const Mat4& columnMajor) = 0;
	virtual bool getWorldMatrix(const ObjectId& id, Mat4& outColumnMajor) const = 0;

	virtual void setVisible(const ObjectId& id, bool visible) = 0;
	virtual void removeVisual(const ObjectId& id) = 0;
	virtual bool hasVisualBranch(const ObjectId& id) const = 0;

	virtual bool tryGetModelCenterMm(const ObjectId& id, double& outCx, double& outCy, double& outCz) const = 0;

	virtual void setPickHandler(PickHandler handler) = 0;
	virtual void clearPickHandler() = 0;

	virtual void requestRedraw() = 0;

	virtual void setSelectionActive(bool active) = 0;
	virtual void clearInstructionPoseAxes() = 0;
	virtual bool hasImportedContent() const = 0;
	virtual bool isTcpDragTeachActive() const = 0;
	virtual bool isTransformGizmoDragging() const = 0;
	virtual void setAnnotationVisible(const ObjectId& annotationId, bool visible) = 0;
	virtual bool removeAnnotation(const ObjectId& annotationId) = 0;
	virtual void clearAllAnnotations() = 0;
	virtual QVector<AnnotationSnapshotDto> annotationSnapshots() const = 0;

	/// 聚焦后端子树
	virtual void focusCameraOnBackend(const ObjectId& id) = 0;
	/// 逻辑父链（不改 OSG）
	virtual void setBackendLogicalParent(const ObjectId& childId, const ObjectId& parentId) = 0;

	// 场景树快照（用于调试树视图）
	struct SceneNodeInfo
	{
		QString className;
		QString name;
		QString localMatrixSummary;
		std::vector<SceneNodeInfo> children;
	};
	virtual SceneNodeInfo sceneGraphSnapshot(int maxDepth = 8) const = 0;

	// 选中对象位姿查询
	virtual bool selectedPosition(float& outX, float& outY, float& outZ) const = 0;
	virtual bool selectedRotationEulerDeg(float& outRx, float& outRy, float& outRz) const = 0;

	virtual void ensureSelectionVisualForBackend(const ObjectId& id, bool urdfLinkMesh = false) = 0;
	virtual bool syncOuterPatFromBackend(const ObjectId& id) = 0;
	virtual GeometryKind geometryKindForBackend(const ObjectId& id) const = 0;
	/// gizmo 松手：OSG 选中态写回后端位姿
	virtual bool commitGizmoPoseToBackend(const ObjectId& id) = 0;

	virtual void setViewerBackgroundForDarkUi(bool dark) = 0;
	virtual void setPerFrameHook(std::function<void()> hook) = 0;
	virtual QString pointCloudPluginReport() const = 0;

	virtual void setCameraFollowBackendId(const ObjectId& id) = 0;
	virtual void clearCameraFollowBackendId() = 0;
	virtual void setObjectSelectionMode(bool enabled) = 0;
	virtual bool objectSelectionMode() const = 0;
	virtual void setPointPickMode(bool enabled) = 0;
	virtual bool pointPickMode() const = 0;
	virtual void setMeshLinePickMode(bool enabled) = 0;
	virtual bool meshLinePickMode() const = 0;
	virtual void setMeshFacePickMode(bool enabled) = 0;
	virtual bool meshFacePickMode() const = 0;
	virtual void syncSelectionForBackend(const ObjectId& id) = 0;
	virtual bool captureViewportPng(QByteArray& outPng, QString* outError = nullptr, int maxWidth = 768,
		int maxHeight = 768) = 0;

	virtual void setTransformGizmoFrame(TransformGizmoFrameDto frame) = 0;
	virtual TransformGizmoFrameDto transformGizmoFrame() const = 0;

	virtual void endTcpDragTeach() = 0;
	virtual void beginTcpDragTeach(const ObjectId& mountBackendId, const Mat4& targetInBaseColumnMajor,
		float modelDiagonalMm, RobotBaseWorldResolver resolveRobotBaseWorld = nullptr,
		const Mat4* toolLocalOnFlangeColumnMajor = nullptr) = 0;
	virtual void updateTcpDragTeachFromTarget(const Mat4& targetInBaseColumnMajor, bool syncTargetInBase = true) = 0;
	virtual void updateTcpDragTeachToolLocalOnFlange(const Mat4& toolLocalOnFlangeColumnMajor) = 0;

	virtual void setInstructionPoseAxes(const QVector<InstructionPoseAxisDto>& axes) = 0;
	virtual void setRawTrajectoryOverlay(const QVector<RawTrajectoryOverlayVertexDto>& points) = 0;
	virtual void clearRawTrajectoryOverlay() = 0;
	virtual void setRawTrajectoryOverlayFrames(const QVector<RawTrajectoryOverlayFrameDto>& frames) = 0;
	virtual void clearRawTrajectoryOverlayFrames() = 0;
	virtual void setRobotFrameOverlays(const RobotFrameOverlayUpdateDto& update) = 0;
	virtual void clearRobotFrameOverlays(const ObjectId& robotRootBackendId) = 0;
	virtual void setFeatureCatalogOverlay(const QVector<FeatureCatalogOverlayItemDto>& items) = 0;
	virtual void clearFeatureCatalogOverlay() = 0;

	/// pick alias / skip-rebase 查询（供 WidgetOsgViewHost 避免直连 OsgWidget）
	virtual std::string resolvePickScopeBackendId(const std::string& backendId) const = 0;
	virtual bool backendSkipsInnerModelCenterRebase(const std::string& backendId) const = 0;

	/// 机器人 gizmo / active id（Phase B 收口）
	virtual std::string activeBackendId() const = 0;
	virtual void setRobotObjectGizmoSyncHook(std::function<bool()> hook) = 0;
	virtual void setRobotObjectGizmoFkRefreshHook(std::function<void()> hook) = 0;
};

/// 渲染视口工厂
class CLOUDSIM_CORE_EXPORT IRenderViewFactory
{
public:
	virtual ~IRenderViewFactory() = default;
	virtual std::unique_ptr<IRenderView> createView(QWidget* parent) = 0;
};

} // namespace cloudsim::core
