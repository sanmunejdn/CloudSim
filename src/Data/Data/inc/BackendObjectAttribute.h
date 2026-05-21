#pragma once

#include <json.hpp>
#include <memory>
#include <string>

#include "data_global.h"
#include "../../PropertyCore/inc/PropertyAttribute.h"

class BackendDataBase;

// Stateless property plugins: JSON row definitions for the property panel; apply writes back to BackendDataBase.
class DATA_EXPORT BackendAttributeBase : public property_core::PropertyAttribute<BackendDataBase>
{
public:
	virtual ~BackendAttributeBase() = default;
};

using BackendAttributePtr = std::shared_ptr<BackendAttributeBase>;

DATA_EXPORT BackendAttributePtr makeBackendPoseAttribute();
DATA_EXPORT BackendAttributePtr makeBackendRotationAttribute();
DATA_EXPORT BackendAttributePtr makeBackendDisplayColorAttribute();
