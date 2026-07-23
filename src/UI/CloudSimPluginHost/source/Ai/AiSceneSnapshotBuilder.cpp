/// @file AiSceneSnapshotBuilder.cpp
/// @brief 枚举活动文档后端供 Agent 注入

#include "Ai/AiSceneSnapshotBuilder.h"

#include "PluginHostContext.h"
#include "IPluginDocument.h"

#include <json.hpp>

namespace AiSceneSnapshotBuilder
{
QByteArray buildJson(PluginHostContext& host)
{
	nlohmann::json root;
	root["selected_backend_id"] = host.selectedBackendId().toStdString();
	nlohmann::json objs = nlohmann::json::array();
	IPluginDocument* doc = host.activeDocument();
	if (doc)
	{
		root["document_label"] = doc->documentLabel();
		root["document_id"] = doc->documentId();
		for (const std::string& id : doc->backendIds())
		{
			nlohmann::json o;
			o["id"] = id;
			o["name"] = doc->backendDisplayName(id);
			o["class"] = doc->backendClassName(id);
			objs.push_back(o);
		}
	}
	root["objects"] = objs;
	return QByteArray::fromStdString(root.dump());
}
} // namespace AiSceneSnapshotBuilder
