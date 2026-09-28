/// @file HelloAiPlugin.cpp
/// @brief HelloAiPlugin 实现

#include "HelloAiPlugin.h"

#include "IPluginAiContext.h"
#include "IPluginDocumentContext.h"
#include "IPluginHostContext.h"
#include "IPluginUiContext.h"

QString HelloAiPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.helloai");
}

QString HelloAiPlugin::displayName() const
{
	return QStringLiteral("Hello AI");
}

bool HelloAiPlugin::initialize(IPluginHostContext* host)
{
	if (!host)
	{
		return false;
	}
	if (host->hostVersion() < 0x00013800)
	{
		return false;
	}
	if (!host->documentContext() || !host->uiContext() || !host->aiContext())
	{
		return false;
	}
	m_host = host;
	return true;
}

void HelloAiPlugin::shutdown()
{
	shutdownAi();
	m_host = nullptr;
}

QString HelloAiPlugin::aiPluginId() const
{
	return pluginId();
}

bool HelloAiPlugin::initializeAi(IPluginHostContext* host, IAiAssistantHost* aiHost)
{
	if (!host || host->hostVersion() < 0x00013800 || !host->aiContext())
	{
		return false;
	}
	m_host = host;
	m_aiHost = aiHost;
	(void)m_aiHost;
	(void)host->aiContext()->aiAssistantHost();
	return true;
}

void HelloAiPlugin::shutdownAi()
{
	m_aiHost = nullptr;
}
