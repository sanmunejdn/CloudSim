#ifndef CLOUDSIMPLUGINHOST_PLUGINROBOTHOSTIMPL_H
#define CLOUDSIMPLUGINHOST_PLUGINROBOTHOSTIMPL_H

/// @file PluginRobotHostImpl.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief IPluginRobotHost：转发到 IPluginMainWindowHost

#include "IPluginRobotHost.h"

class PluginHostContext;

class PluginRobotHostImpl : public IPluginRobotHost
{
public:
	explicit PluginRobotHostImpl(PluginHostContext* hostContext);

	bool getActiveRobotTcpPose(PluginPose6d& outMmDeg, QString* err) override;
	bool getSelectedBackendWorldPose(PluginPose6d& outMmDeg, QString* err) override;
	bool planAndConfirmTcpWaypoints(const QVector<PluginPose6d>& goalsMmDeg, QString* err) override;

private:
	PluginHostContext* m_host = nullptr;
};

#endif // CLOUDSIMPLUGINHOST_PLUGINROBOTHOSTIMPL_H
