#pragma once

#include "aibackend_global.h"

#include <json.hpp>
#include <QString>
#include <optional>

namespace AiIntentParser
{
struct ParseResult
{
	bool ok = false;
	nlohmann::json command;
	QString errorMessage;
	QString hintMessage;
};

AIBACKEND_EXPORT ParseResult tryParseUserText(const QString& text);

/// mesh.compose：长方体 + 通孔等固定句式，避免小模型把坯料建成圆柱
AIBACKEND_EXPORT ParseResult tryParseComposeUserText(const QString& text);
}
