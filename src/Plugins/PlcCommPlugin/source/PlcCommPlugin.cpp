/// @file PlcCommPlugin.cpp
/// @brief PlcCommPlugin 实现

#include "PlcCommPlugin.h"

#include "IPluginHostContext.h"
#include "PlcCommWidget.h"

QString PlcCommPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.plccomm");
}

QString PlcCommPlugin::displayName() const
{
	return QStringLiteral("PLC");
}

bool PlcCommPlugin::initialize(IPluginHostContext* host)
{
	if (!host || !host->sidePanelTabParent())
	{
		return false;
	}
	host_ = host;

	panel_ = createPlcCommWidget(nullptr);
	auto* plcPanel = qobject_cast<PlcCommWidget*>(panel_);
	if (!plcPanel)
	{
		panel_ = nullptr;
		return false;
	}

	plcPanel->setUseChinese(host->useChinese());
	plcPanel->applyLanguage();

	const char* tabTitle = host->useChinese() ? "PLC 通讯" : "PLC";
	if (host->registerSidePanelTab(tabTitle, panel_) < 0)
	{
		panel_ = nullptr;
		return false;
	}

	host->onLanguageChanged([this](const bool) { applyLanguage(); });

	host->logInfo(host->useChinese() ? QStringLiteral("PLC 通讯插件已加载。")
									 : QStringLiteral("PLC comm plugin initialized."));
	return true;
}

void PlcCommPlugin::shutdown()
{
	if (host_ && panel_)
	{
		host_->unregisterSidePanelTab(panel_);
	}
	panel_ = nullptr;
	host_ = nullptr;
}

void PlcCommPlugin::applyLanguage()
{
	if (!host_ || !panel_)
	{
		return;
	}
	const bool zh = host_->useChinese();
	if (auto* plcPanel = qobject_cast<PlcCommWidget*>(panel_))
	{
		plcPanel->setUseChinese(zh);
		plcPanel->applyLanguage();
	}
	host_->setSidePanelTabTitle(panel_, zh ? "PLC 通讯" : "PLC");
}
