/// @file IndustrialCameraPlugin.cpp
/// @brief 注册工业相机侧栏（内嵌相机/手眼/视觉抓取 Tab）

#include "IndustrialCameraPlugin.h"

#include "CameraResourceStore.h"
#include "IPluginHostContext.h"
#include "IPluginProjectContext.h"
#include "IPluginUiContext.h"
#include "IndustrialCameraDockWidget.h"
#include "VisionGraspPanelWidget.h"

#include <QJsonObject>

QString IndustrialCameraPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.industrialcamera");
}

QString IndustrialCameraPlugin::displayName() const
{
	return QStringLiteral("Industrial Camera");
}

bool IndustrialCameraPlugin::initialize(IPluginHostContext* host)
{
	if (!host)
		return false;
	if (host->hostVersion() < 0x00013800)
	{
		host->logError(QStringLiteral("IndustrialCameraPlugin requires host 1.56.0+ (narrow contexts)"));
		return false;
	}
	IPluginUiContext* uiCtx = host->uiContext();
	IPluginProjectContext* projCtx = host->projectContext();
	if (!uiCtx || !projCtx || !uiCtx->sidePanelTabParent())
		return false;
	host_ = host;
	industrial_camera_ui::ensureIndustrialCameraRoot(nullptr);

	auto* dock = new IndustrialCameraDockWidget(host, nullptr);
	dock->setUseChinese(host->useChinese());
	dock->applyLanguage();
	panel_ = dock;

	const bool zh = host->useChinese();
	if (uiCtx->registerSidePanelTab(zh ? "工业相机" : "Camera", panel_) < 0)
	{
		panel_ = nullptr;
		return false;
	}

	host->onLanguageChanged([this](const bool) { applyLanguage(); });
	projCtx->onProjectAboutToSave(
		[this](const QString&, QJsonObject& root)
		{
			auto* dock = qobject_cast<IndustrialCameraDockWidget*>(panel_);
			if (!dock || !dock->visionGraspPanel())
				return;
			QJsonObject vg;
			dock->visionGraspPanel()->saveOffsetsToJson(vg);
			root.insert(QStringLiteral("industrialCameraVisionGrasp"), vg);
		});
	projCtx->onProjectLoaded(
		[this](const QString&, const QJsonObject& root)
		{
			auto* dock = qobject_cast<IndustrialCameraDockWidget*>(panel_);
			if (!dock || !dock->visionGraspPanel())
				return;
			const QJsonObject vg = root.value(QStringLiteral("industrialCameraVisionGrasp")).toObject();
			if (!vg.isEmpty())
				dock->visionGraspPanel()->loadOffsetsFromJson(vg);
		});
	host->logInfo(zh ? QStringLiteral("工业相机插件已加载。") : QStringLiteral("Industrial camera plugin loaded."));
	return true;
}

void IndustrialCameraPlugin::shutdown()
{
	if (host_ && panel_)
	{
		if (IPluginUiContext* uiCtx = host_->uiContext())
			uiCtx->unregisterSidePanelTab(panel_);
	}
	panel_ = nullptr;
	host_ = nullptr;
}

void IndustrialCameraPlugin::applyLanguage()
{
	if (!host_ || !panel_)
		return;
	const bool zh = host_->useChinese();
	if (auto* dock = qobject_cast<IndustrialCameraDockWidget*>(panel_))
	{
		dock->setUseChinese(zh);
		dock->applyLanguage();
	}
	if (IPluginUiContext* uiCtx = host_->uiContext())
		uiCtx->setSidePanelTabTitle(panel_, zh ? "工业相机" : "Camera");
}
