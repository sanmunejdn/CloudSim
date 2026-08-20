#ifndef DATA_BACKENDCOMPONENT_H
#define DATA_BACKENDCOMPONENT_H

/// @file BackendComponent.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后端组件类型擦除契约（Follow 等挂到 BackendDataBase）

#include "data_global.h"

#include <memory>
#include <string>

/// 后端组件类型擦除契约（Follow 等挂到 BackendDataBase）
class DATA_EXPORT IBackendComponent
{
public:
	virtual ~IBackendComponent() = default;
	virtual std::string componentType() const = 0;
};

using BackendComponentPtr = std::shared_ptr<IBackendComponent>;

#endif // DATA_BACKENDCOMPONENT_H
