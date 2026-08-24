#ifndef ROBOTSCENE_KINEMATICMODELREGISTRY_H
#define ROBOTSCENE_KINEMATICMODELREGISTRY_H

#include "robot_scene_global.h"

#include "IKinematicModel.h"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace KinematicModelRegistry
{
inline std::string keyCustomDevice(const std::string& backendId) { return "custom:" + backendId; }
inline std::string keyRobotInstance(const std::string& sceneBackendId) { return "robot:" + sceneBackendId; }

ROBOT_SCENE_API void clear();
ROBOT_SCENE_API void registerModel(const std::string& key, std::shared_ptr<kinematic_core::IKinematicModel> model);
ROBOT_SCENE_API std::shared_ptr<kinematic_core::IKinematicModel> modelForKey(const std::string& key);
}

#endif // ROBOTSCENE_KINEMATICMODELREGISTRY_H
