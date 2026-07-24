/// @file IndustrialCameraPlugin.cpp
/// @brief 注册工业相机侧栏（内嵌相机/手眼 Tab）

#include "IndustrialCameraPlugin.h"

#include "CameraResourceStore.h"
#include "IndustrialCameraDockWidget.h"
#include "IPluginHostContext.h"

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
	if (!host || !host->sidePanelTabParent())
		return false;
	host_ = host;
	industrial_camera_ui::ensureIndustrialCameraRoot(nullptr);

	auto* dock = new IndustrialCameraDockWidget(host, nullptr);
	dock->setUseChinese(host->useChinese());
	dock->applyLanguage();
	panel_ = dock;

	const bool zh = host->useChinese();
	if (host->registerSidePanelTab(zh ? "工业相机" : "Camera", panel_) < 0)
	{
		panel_ = nullptr;
		return false;
	}

	host->onLanguageChanged([this](const bool) { applyLanguage(); });
	host->logInfo(zh ? QStringLiteral("工业相机插件已加载。") : QStringLiteral("Industrial camera plugin loaded."));
	return true;
}

void IndustrialCameraPlugin::shutdown()
{
	if (host_ && panel_)
		host_->unregisterSidePanelTab(panel_);
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
	host_->setSidePanelTabTitle(panel_, zh ? "工业相机" : "Camera");
}
