#pragma once

#include "IRenderView.h"

class OsgWidget;

namespace cloudsim::host {

/// OsgWidget → IRenderView 映射；列主序 Mat4 与 osg::Matrixd 在此转换，Adapter 内不写业务分支
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

	void focusCameraOnBackend(const core::ObjectId& id) override; ///< 逻辑子树聚合包围球
	void setBackendLogicalParent(const core::ObjectId& childId, const core::ObjectId& parentId) override; ///< 仅旁路表，不改 OSG 父链

private:
	OsgWidget& m_widget;
	core::PickHandler m_pickHandler;
};

} // namespace cloudsim::host
