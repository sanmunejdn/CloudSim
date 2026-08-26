/// @file HeadlessDrawingBridge.cpp

#include "headless/HeadlessDrawingBridge.h"

#include "BackendTypeIds.h"
#include "DocumentHost.h"

namespace cloudsim::host
{
HeadlessDrawingBridge::HeadlessDrawingBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessDrawingBridge::exportDrawing(const QJsonObject& body)
{
	(void)body;
	(void)m_host;

	const QString svg = QStringLiteral(
		R"(<?xml version="1.0" encoding="UTF-8"?>)"
		R"(<svg xmlns="http://www.w3.org/2000/svg" width="420" height="297" viewBox="0 0 420 297">)"
		R"(<rect width="420" height="297" fill="#fafafa" stroke="#ccc"/>)"
		R"(<text x="12" y="24" font-family="sans-serif" font-size="12" fill="#666">CloudSim drawing placeholder</text>)"
		R"(</svg>)");

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("svg"), svg);
	o.insert(QStringLiteral("engineeringDrawing"), m_engineeringDrawing);
	return o;
}

void HeadlessDrawingBridge::loadSidecarFromProject(const QJsonObject& projectRoot)
{
	m_engineeringDrawing =
		projectRoot.value(QLatin1String(backend_type::kProjectKeyEngineeringDrawing)).toObject();
}

void HeadlessDrawingBridge::mergeSidecarIntoProject(QJsonObject& projectRoot) const
{
	if (!m_engineeringDrawing.isEmpty())
		projectRoot.insert(QLatin1String(backend_type::kProjectKeyEngineeringDrawing), m_engineeringDrawing);
}

} // namespace cloudsim::host
