#pragma once

#include <memory>
#include <string>

#include "data_global.h"

/// Type-erased backend component contract.
class DATA_EXPORT IBackendComponent
{
public:
	virtual ~IBackendComponent() = default;
	virtual std::string componentType() const = 0;
};

using BackendComponentPtr = std::shared_ptr<IBackendComponent>;
