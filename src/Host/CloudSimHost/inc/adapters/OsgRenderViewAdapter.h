#pragma once

#include "IRenderView.h"

class OsgWidget;

namespace cloudsim::host {

/// OsgWidget 渲染适配
class OsgRenderViewAdapter final : public core::IRenderView
{
public:
	explicit OsgRenderViewAdapter(OsgWidget& widget);

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

	void focusCameraOnBackend(const core::ObjectId& id) override;
	void setBackendLogicalParent(const core::ObjectId& childId, const core::ObjectId& parentId) override;

	SceneNodeInfo sceneGraphSnapshot(int maxDepth = 8) const override;

	bool selectedPosition(float& outX, float& outY, float& outZ) const override;
	bool selectedRotationEulerDeg(float& outRx, float& outRy, float& outRz) const override;

private:
	OsgWidget& m_widget;
	core::PickHandler m_pickHandler;
};

} // namespace cloudsim::host
