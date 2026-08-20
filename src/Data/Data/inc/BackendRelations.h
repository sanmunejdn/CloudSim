#ifndef DATA_BACKENDRELATIONS_H
#define DATA_BACKENDRELATIONS_H

/// @file BackendRelations.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 对象图关系查询薄封装（委托 BackendDataBase / Manager）

#include "BackendDataBase.h"
#include "BackendDataManager.h"

#include <memory>
#include <vector>

/// 对象图关系查询薄封装（委托 BackendDataBase / Manager）
namespace backend_relations
{
inline std::vector<std::shared_ptr<BackendDataBase>> parents(const BackendDataBase& object,
															 const BackendDataManager& manager)
{
	return object.parentObjects(manager);
}

inline std::vector<std::shared_ptr<BackendDataBase>> children(const BackendDataBase& object,
															  const BackendDataManager& manager)
{
	return object.childObjects(manager);
}

inline std::vector<std::shared_ptr<BackendDataBase>> descendants(const BackendDataBase& object,
																 const BackendDataManager& manager)
{
	return object.descendantObjects(manager);
}

inline std::vector<std::string> parentIds(const BackendDataBase& object, const BackendDataManager& manager)
{
	return manager.parentsOf(object.id());
}

inline std::vector<std::string> childIds(const BackendDataBase& object, const BackendDataManager& manager)
{
	return manager.childrenOf(object.id());
}

inline std::vector<std::string> descendantIds(const BackendDataBase& object, const BackendDataManager& manager)
{
	return manager.descendantsOf(object.id());
}

} // namespace backend_relations

#endif // DATA_BACKENDRELATIONS_H
