#pragma once

#include "AiConfigDto.h"

#include <optional>
#include <QString>

std::optional<AiConfigDto> loadAiConfigDto(const QString& filePath = QString());
bool saveAiConfigDto(const AiConfigDto& config, const QString& filePath = QString(), QString* errorMessage = nullptr);
