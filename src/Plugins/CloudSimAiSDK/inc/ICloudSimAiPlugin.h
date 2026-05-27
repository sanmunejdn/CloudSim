#pragma once

#include "cloudsim_ai_sdk_global.h"

#include <QtPlugin>
#include <QString>

class IPluginHostContext;
class IAiAssistantHost;
class QWidget;

class ICloudSimAiPlugin
{
public:
	virtual ~ICloudSimAiPlugin() = default;

	virtual QString aiPluginId() const = 0;

	virtual bool initializeAi(IPluginHostContext* host, IAiAssistantHost* aiHost) = 0;
	virtual void shutdownAi() = 0;

	virtual QWidget* createAssistantWidget(QWidget* parent) { return nullptr; }
	virtual int assistantPanelPriority() const { return 0; }
};

#define CloudSimAiPlugin_iid "com.cloudsim.ICloudSimAiPlugin/1.0"
Q_DECLARE_INTERFACE(ICloudSimAiPlugin, CloudSimAiPlugin_iid)
