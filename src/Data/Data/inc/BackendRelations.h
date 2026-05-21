#pragma once

#include <memory>
#include <vector>

#include "BackendDataBase.h"
#include "BackendDataManager.h"

namespace backend_relations
{

inline std::vector<std::shared_ptr<BackendDataBase>> parents(
	const BackendDataBase& object,
	const BackendDataManager& manager)
{
	return object.parentObjects(manager);
}

inline std::vector<std::shared_ptr<BackendDataBase>> children(
	const BackendDataBase& object,
	const BackendDataManager& manager)
{
	return object.childObjects(manager);
}

inline std::vector<std::shared_ptr<BackendDataBase>> descendants(
	const BackendDataBase& object,
	const BackendDataManager& manager)
{
	return object.descendantObjects(manager);
}

inline std::vector<std::string> parentIds(
	const BackendDataBase& object,
	const BackendDataManager& manager)
{
	return manager.parentsOf(object.id());
}

inline std::vector<std::string> childIds(
	const BackendDataBase& object,
	const BackendDataManager& manager)
{
	return manager.childrenOf(object.id());
}

inline std::vector<std::string> descendantIds(
	const BackendDataBase& object,
	const BackendDataManager& manager)
{
	return manager.descendantsOf(object.id());
}

} // namespace backend_relations
