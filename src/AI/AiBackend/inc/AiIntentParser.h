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
}
