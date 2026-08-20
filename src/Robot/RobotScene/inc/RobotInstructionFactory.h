#ifndef ROBOTSCENE_ROBOTINSTRUCTIONFACTORY_H
#define ROBOTSCENE_ROBOTINSTRUCTIONFACTORY_H

/// @file RobotInstructionFactory.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RobotInstructionFactory 接口

#include "robot_scene_global.h"

#include "RobotInstructionModel.h"

#include <memory>
#include <string>
#include <vector>

#include <json.hpp>

namespace RobotInstruction
{
ROBOT_SCENE_API std::shared_ptr<Base> createFromJson(const nlohmann::json& j, std::string* errMsg = nullptr);
ROBOT_SCENE_API nlohmann::json toJson(const Base& ins);
ROBOT_SCENE_API std::shared_ptr<Base> cloneInstruction(const Base& ins);
ROBOT_SCENE_API std::shared_ptr<Base> cloneInstructionPreservingId(const Base& ins);
ROBOT_SCENE_API std::vector<std::shared_ptr<Base>> createListFromJson(const nlohmann::json& arr,
																	  std::string* errMsg = nullptr);

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONFACTORY_H
