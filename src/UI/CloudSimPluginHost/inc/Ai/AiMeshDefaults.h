#ifndef CLOUDSIMPLUGINHOST_AIMESHDEFAULTS_H
#define CLOUDSIMPLUGINHOST_AIMESHDEFAULTS_H

/// @file AiMeshDefaults.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief create_mesh：仅补缺失或非正尺寸；usedDefaults 表示至少写入过一个默认字段

#include "aibackend_global.h"

#include <QString>

#include <json.hpp>

namespace AiMeshDefaults
{
struct MeshCreateDefaults
{
	double boxLengthMm = 100.0;
	double boxWidthMm = 100.0;
	double boxHeightMm = 100.0;
	double cylinderRadiusMm = 50.0;
	double cylinderHeightMm = 100.0;
	double coneRadiusMm = 50.0;
	double coneHeightMm = 100.0;
	double sphereRadiusMm = 50.0;
};

AIBACKEND_EXPORT MeshCreateDefaults builtinDefaults();
AIBACKEND_EXPORT MeshCreateDefaults activeDefaults();
AIBACKEND_EXPORT void setConfigOverrides(const MeshCreateDefaults& overrides);
AIBACKEND_EXPORT void loadFromConfigJson(const nlohmann::json& root);

/// create_mesh：仅补缺失或非正尺寸；usedDefaults 表示至少写入过一个默认字段
AIBACKEND_EXPORT bool applyMissingDimensions(nlohmann::json& cmd, bool* usedDefaults = nullptr);

AIBACKEND_EXPORT QString summarizeDimensionsMm(const nlohmann::json& cmd);
AIBACKEND_EXPORT QString defaultsAppliedNote(const nlohmann::json& cmd, bool usedDefaults);

} // namespace AiMeshDefaults

#endif // CLOUDSIMPLUGINHOST_AIMESHDEFAULTS_H
