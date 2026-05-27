#include "Ai/MeshCreateDomainHandler.h"

#include "Ai/AiActionPlanExecutor.h"
#include "AiDomainTypes.h"
#include "AiCommandSchema.h"
#include "IAiAssistantHost.h"
#include "PluginHostContext.h"

#include <json.hpp>

QString MeshCreateDomainHandler::domainId() const
{
	return AiDomainIds::meshCreate();
}

bool MeshCreateDomainHandler::validateOutput(const QByteArray& jsonUtf8, QString* err) const
{
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("Invalid JSON.");
		return false;
	}
	if (j.contains("action") && j.value("action", "") == "create_mesh")
	{
		BackendPrimitiveGeometry::PrimitiveMeshParams p;
		BackendPrimitiveGeometry::PrimitiveMeshQuality q;
		std::string name, src, e;
		if (!AiCommandSchema::parseCreateMeshCommand(j, p, q, name, src, e))
		{
			if (err)
				*err = QString::fromStdString(e);
			return false;
		}
		return true;
	}
	if (j.value("version", 0) == 2 && j.contains("steps"))
		return true;
	if (err)
		*err = QStringLiteral("Expected create_mesh or action plan v2.");
	return false;
}

bool MeshCreateDomainHandler::execute(const QByteArray& jsonUtf8, IPluginHostContext* host, IAiAssistantHost* aiHost,
	QString* summary, QString* err)
{
	(void)aiHost;
	auto* ph = dynamic_cast<PluginHostContext*>(host);
	if (!ph)
	{
		if (err)
			*err = QStringLiteral("Plugin host unavailable.");
		return false;
	}
	return AiActionPlanExecutor::execute(*ph, jsonUtf8, summary, err);
}
