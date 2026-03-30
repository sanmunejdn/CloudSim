#pragma once

#include <json.hpp>
#include <string>

#include "data_global.h"

class BackendDataBase;

// Stateless property plugins: JSON row definitions for the property panel; apply writes back to BackendDataBase.
class DATA_EXPORT BackendAttributeBase
{
public:
	virtual ~BackendAttributeBase() = default;

	virtual void appendRows(const BackendDataBase& data, nlohmann::json& rows) const = 0;
	virtual bool handlesKey(const BackendDataBase& data, const std::string& key) const = 0;
	virtual bool apply(BackendDataBase& data, const std::string& key, const std::string& value, std::string* errMsg) const = 0;
};

class DATA_EXPORT BackendPoseAttribute final : public BackendAttributeBase
{
public:
	void appendRows(const BackendDataBase& data, nlohmann::json& rows) const override;
	bool handlesKey(const BackendDataBase& data, const std::string& key) const override;
	bool apply(BackendDataBase& data, const std::string& key, const std::string& value, std::string* errMsg) const override;
};

class DATA_EXPORT BackendRotationAttribute final : public BackendAttributeBase
{
public:
	void appendRows(const BackendDataBase& data, nlohmann::json& rows) const override;
	bool handlesKey(const BackendDataBase& data, const std::string& key) const override;
	bool apply(BackendDataBase& data, const std::string& key, const std::string& value, std::string* errMsg) const override;
};

class DATA_EXPORT BackendDisplayColorAttribute final : public BackendAttributeBase
{
public:
	void appendRows(const BackendDataBase& data, nlohmann::json& rows) const override;
	bool handlesKey(const BackendDataBase& data, const std::string& key) const override;
	bool apply(BackendDataBase& data, const std::string& key, const std::string& value, std::string* errMsg) const override;
};
