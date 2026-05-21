#pragma once

#include "widget_global.h"

// AiCommandSchema lives in AiBackend.dll

#include <json.hpp>
#include <QString>

class MainWindow;

namespace AiCreateMeshRunner
{
WIDGET_EXPORT bool executeFromJson(MainWindow& mw, const nlohmann::json& cmd, QString& outAssistantReply, QString& outError);
}
