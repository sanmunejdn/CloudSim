/// @file PlcCommPlugin.cpp
/// @brief PlcCommPlugin 实现

#include "PlcCommPlugin.h"

#include "IPluginHostContext.h"
#include "IPluginUiContext.h"
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
	if (!host)
	{
		return false;
	}
	if (host->hostVersion() < 0x00013800)
	{
		host->logError(QStringLiteral("PlcCommPlugin requires host 1.56.0+ (narrow contexts)"));
		return false;
	}
	IPluginUiContext* uiCtx = host->uiContext();
	if (!uiCtx || !uiCtx->sidePanelTabParent())
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
	if (uiCtx->registerSidePanelTab(tabTitle, panel_) < 0)
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
		if (IPluginUiContext* uiCtx = host_->uiContext())
		{
			uiCtx->unregisterSidePanelTab(panel_);
		}
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
	if (IPluginUiContext* uiCtx = host_->uiContext())
	{
		uiCtx->setSidePanelTabTitle(panel_, zh ? "PLC 通讯" : "PLC");
	}
}
