#ifndef TRAJECTORYALGORITHMBUILTINS_TRANSLATEOPCONFIG_H
#define TRAJECTORYALGORITHMBUILTINS_TRANSLATEOPCONFIG_H

/// @file TranslateOpConfig.h
/// @brief TranslateOpConfig 接口

// Translate 块参数 schema 与默认 TrajectoryOpDescriptor
#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{
std::unique_ptr<IOpParamConfig> makeTranslateOpConfig();

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_TRANSLATEOPCONFIG_H
