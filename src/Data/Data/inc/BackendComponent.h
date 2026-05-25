#pragma once

#include <memory>
#include <string>

#include "data_global.h"

/// 后端组件类型擦除契约（Follow 等挂到 BackendDataBase）
class DATA_EXPORT IBackendComponent
{
public:
	virtual ~IBackendComponent() = default;
	virtual std::string componentType() const = 0;
};

using BackendComponentPtr = std::shared_ptr<IBackendComponent>;
