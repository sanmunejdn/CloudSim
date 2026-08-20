#ifndef CLOUDSIMPLUGINHOST_AIINTENTPARSER_H
#define CLOUDSIMPLUGINHOST_AIINTENTPARSER_H

/// @file AiIntentParser.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief mesh.compose：长方体 + 通孔等固定句式，避免小模型把坯料建成圆柱

#include "aibackend_global.h"

#include <QString>
#include <optional>

#include <json.hpp>

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

/// feature.compose：简单板/块 → Pad 特征链
AIBACKEND_EXPORT ParseResult tryParseFeatureComposeUserText(const QString& text);
} // namespace AiIntentParser

#endif // CLOUDSIMPLUGINHOST_AIINTENTPARSER_H
