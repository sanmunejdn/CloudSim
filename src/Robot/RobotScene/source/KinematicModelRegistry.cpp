#include "KinematicModelRegistry.h"

namespace KinematicModelRegistry
{
namespace
{
std::mutex g_mutex;
std::unordered_map<std::string, std::shared_ptr<kinematic_core::IKinematicModel>> g_models;
} // namespace

void clear()
{
	std::lock_guard<std::mutex> lock(g_mutex);
	g_models.clear();
}

void registerModel(const std::string& key, std::shared_ptr<kinematic_core::IKinematicModel> model)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	g_models[key] = std::move(model);
}

std::shared_ptr<kinematic_core::IKinematicModel> modelForKey(const std::string& key)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	const auto it = g_models.find(key);
	if (it == g_models.end())
	{
		return {};
	}
	return it->second;
}

} // namespace KinematicModelRegistry
