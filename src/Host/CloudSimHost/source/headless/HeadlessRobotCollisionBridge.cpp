/// @file HeadlessRobotCollisionBridge.cpp

#include "headless/HeadlessRobotCollisionBridge.h"

#include "DocumentHost.h"

namespace cloudsim::host
{
HeadlessRobotCollisionBridge::HeadlessRobotCollisionBridge(DocumentHost& host) : m_host(host)
{
	(void)m_host;
}

QJsonObject HeadlessRobotCollisionBridge::getSettings(const QJsonObject& body)
{
	(void)body;
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("stub"), true);
	return o;
}

QJsonObject HeadlessRobotCollisionBridge::putSettings(const QJsonObject& body)
{
	(void)body;
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("stub"), true);
	return o;
}

QJsonObject HeadlessRobotCollisionBridge::plan(const QJsonObject& body)
{
	(void)body;
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("stub"), true);
	return o;
}

QJsonObject HeadlessRobotCollisionBridge::confirm(const QJsonObject& body)
{
	(void)body;
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("stub"), true);
	return o;
}

} // namespace cloudsim::host
