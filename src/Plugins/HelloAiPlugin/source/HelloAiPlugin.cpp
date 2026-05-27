#include "HelloAiPlugin.h"

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
	m_host = host;
	m_aiHost = aiHost;
	(void)m_aiHost;
	return true;
}

void HelloAiPlugin::shutdownAi()
{
	m_aiHost = nullptr;
}
