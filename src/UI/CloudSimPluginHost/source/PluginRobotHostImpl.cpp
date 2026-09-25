/// @file PluginRobotHostImpl.cpp
/// @brief 插件机器人宿主：转发到 MainWindow 仿真桥

#include "PluginRobotHostImpl.h"

#include "IPluginMainWindowHost.h"
#include "PluginHostContext.h"

namespace
{
IPluginMainWindowHost* requireMainWindow(PluginHostContext* host, QString* err)
{
	if (!host || !host->mainWindowHost())
	{
		if (err)
			*err = QStringLiteral("宿主未就绪");
		return nullptr;
	}
	return host->mainWindowHost();
}
} // namespace

PluginRobotHostImpl::PluginRobotHostImpl(PluginHostContext* hostContext) : m_host(hostContext) {}

bool PluginRobotHostImpl::getActiveRobotTcpPose(PluginPose6d& outMmDeg, QString* err)
{
	IPluginMainWindowHost* mw = requireMainWindow(m_host, err);
	if (!mw)
		return false;
	return mw->getActiveRobotTcpPoseForPlugin(outMmDeg.xMm, outMmDeg.yMm, outMmDeg.zMm, outMmDeg.rxDeg, outMmDeg.ryDeg,
											 outMmDeg.rzDeg, err);
}

bool PluginRobotHostImpl::getSelectedBackendWorldPose(PluginPose6d& outMmDeg, QString* err)
{
	IPluginMainWindowHost* mw = requireMainWindow(m_host, err);
	if (!mw)
		return false;
	return mw->getSelectedBackendPoseInRobotBaseForPlugin(outMmDeg.xMm, outMmDeg.yMm, outMmDeg.zMm, outMmDeg.rxDeg,
														  outMmDeg.ryDeg, outMmDeg.rzDeg, err);
}

bool PluginRobotHostImpl::planAndConfirmTcpWaypoints(const QVector<PluginPose6d>& goalsMmDeg, QString* err)
{
	IPluginMainWindowHost* mw = requireMainWindow(m_host, err);
	if (!mw)
		return false;
	QVector<QVector<double>> goals6;
	goals6.reserve(goalsMmDeg.size());
	for (const PluginPose6d& p : goalsMmDeg)
		goals6.push_back({p.xMm, p.yMm, p.zMm, p.rxDeg, p.ryDeg, p.rzDeg});
	return mw->planAndConfirmTcpWaypointsForPlugin(goals6, err);
}
