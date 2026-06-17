#pragma once

#include "IRenderView.h"

class OsgWidget;

namespace cloudsim::host {

class DocumentHost;

/// OsgWidget 渲染适配
class OsgRenderViewAdapter final : public core::IRenderView
{
public:
	explicit OsgRenderViewAdapter(OsgWidget& widget);
	explicit OsgRenderViewAdapter(OsgWidget& widget, DocumentHost& host);
	explicit OsgRenderViewAdapter(DocumentHost& host);

	QWidget* widget() override;
	const QWidget* widget() const override;

	void setWorldMatrix(const core::ObjectId& id, const core::Mat4& columnMajor) override;
	bool getWorldMatrix(const core::ObjectId& id, core::Mat4& outColumnMajor) const override;

	void setVisible(const core::ObjectId& id, bool visible) override;
	void removeVisual(const core::ObjectId& id) override;
	bool hasVisualBranch(const core::ObjectId& id) const override;

	bool tryGetModelCenterMm(const core::ObjectId& id, double& outCx, double& outCy, double& outCz) const override;

	void setPickHandler(core::PickHandler handler) override;
	void clearPickHandler() override;

	void requestRedraw() override;

	void setSelectionActive(bool active) override;
	void clearInstructionPoseAxes() override;
	bool hasImportedContent() const override;
	bool isTcpDragTeachActive() const override;
	bool isTransformGizmoDragging() const override;
	void setAnnotationVisible(const core::ObjectId& annotationId, bool visible) override;
	bool removeAnnotation(const core::ObjectId& annotationId) override;
	void clearAllAnnotations() override;
	QVector<core::AnnotationSnapshotDto> annotationSnapshots() const override;

	void focusCameraOnBackend(const core::ObjectId& id) override;
	void setBackendLogicalParent(const core::ObjectId& childId, const core::ObjectId& parentId) override;

	SceneNodeInfo sceneGraphSnapshot(int maxDepth = 8) const override;

	bool selectedPosition(float& outX, float& outY, float& outZ) const override;
	bool selectedRotationEulerDeg(float& outRx, float& outRy, float& outRz) const override;

	void ensureSelectionVisualForBackend(const core::ObjectId& id, bool urdfLinkMesh = false) override;
	bool syncOuterPatFromBackend(const core::ObjectId& id) override;
	core::GeometryKind geometryKindForBackend(const core::ObjectId& id) const override;
	bool commitGizmoPoseToBackend(const core::ObjectId& id) override;

	void setViewerBackgroundForDarkUi(bool dark) override;
	void setPerFrameHook(std::function<void()> hook) override;
	QString pointCloudPluginReport() const override;

	void setCameraFollowBackendId(const core::ObjectId& id) override;
	void clearCameraFollowBackendId() override;
	void setObjectSelectionMode(bool enabled) override;
	bool objectSelectionMode() const override;
	void setPointPickMode(bool enabled) override;
	bool pointPickMode() const override;
	void setMeshLinePickMode(bool enabled) override;
	bool meshLinePickMode() const override;
	void setMeshFacePickMode(bool enabled) override;
	bool meshFacePickMode() const override;
	void syncSelectionForBackend(const core::ObjectId& id) override;
	bool captureViewportPng(QByteArray& outPng, QString* outError, int maxWidth, int maxHeight) override;

	void setTransformGizmoFrame(core::TransformGizmoFrameDto frame) override;
	core::TransformGizmoFrameDto transformGizmoFrame() const override;
	void endTcpDragTeach() override;
	void beginTcpDragTeach(const core::ObjectId& mountBackendId, const core::Mat4& targetInBaseColumnMajor,
		float modelDiagonalMm, core::RobotBaseWorldResolver resolveRobotBaseWorld,
		const core::Mat4* toolLocalOnFlangeColumnMajor) override;
	void updateTcpDragTeachFromTarget(const core::Mat4& targetInBaseColumnMajor, bool syncTargetInBase) override;
	void updateTcpDragTeachToolLocalOnFlange(const core::Mat4& toolLocalOnFlangeColumnMajor) override;

	void setInstructionPoseAxes(const QVector<core::InstructionPoseAxisDto>& axes) override;
	void setRawTrajectoryOverlay(const QVector<core::RawTrajectoryOverlayVertexDto>& points) override;
	void clearRawTrajectoryOverlay() override;
	void setRawTrajectoryOverlayFrames(const QVector<core::RawTrajectoryOverlayFrameDto>& frames) override;
	void clearRawTrajectoryOverlayFrames() override;
	void setRobotFrameOverlays(const core::RobotFrameOverlayUpdateDto& update) override;
	void clearRobotFrameOverlays(const core::ObjectId& robotRootBackendId) override;
	void setFeatureCatalogOverlay(const QVector<core::FeatureCatalogOverlayItemDto>& items) override;
	void clearFeatureCatalogOverlay() override;

	std::string resolvePickScopeBackendId(const std::string& backendId) const override;
	bool backendSkipsInnerModelCenterRebase(const std::string& backendId) const override;

	std::string activeBackendId() const override;
	void setRobotObjectGizmoSyncHook(std::function<bool()> hook) override;
	void setRobotObjectGizmoFkRefreshHook(std::function<void()> hook) override;

private:
	DocumentHost* m_host = nullptr;
	OsgWidget& m_widget;
	core::PickHandler m_pickHandler;
};

} // namespace cloudsim::host
