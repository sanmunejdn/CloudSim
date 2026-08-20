#ifndef CLOUDSIMPLUGINHOST_AICONFIGLOADER_H
#define CLOUDSIMPLUGINHOST_AICONFIGLOADER_H

/// @file AiConfigLoader.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AiConfigLoader 接口

#include "AiConfigDto.h"

#include <QString>
#include <optional>

std::optional<AiConfigDto> loadAiConfigDto(const QString& filePath = QString());
bool saveAiConfigDto(const AiConfigDto& config, const QString& filePath = QString(), QString* errorMessage = nullptr);

#endif // CLOUDSIMPLUGINHOST_AICONFIGLOADER_H
