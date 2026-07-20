#ifndef CLOUDSIMPLUGINHOST_AICONFIGLOADER_H
#define CLOUDSIMPLUGINHOST_AICONFIGLOADER_H

/// @file AiConfigLoader.h
/// @brief AiConfigLoader 接口

#include "AiConfigDto.h"

#include <QString>
#include <optional>

std::optional<AiConfigDto> loadAiConfigDto(const QString& filePath = QString());
bool saveAiConfigDto(const AiConfigDto& config, const QString& filePath = QString(), QString* errorMessage = nullptr);

#endif // CLOUDSIMPLUGINHOST_AICONFIGLOADER_H
