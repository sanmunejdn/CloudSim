#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <json.hpp>

#include "data_global.h"
#include "BackendObjectAttribute.h"

// 3D vector (double) for pose, bounds, etc.
struct BackendVec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

// Axis-aligned bounding box.
struct BackendBoundingBox
{
	BackendVec3 min;
	BackendVec3 max;
	bool valid = false;
};

// Display color RGBA, components in 0..1.
struct BackendColor
{
	float r = 1.0f;
	float g = 1.0f;
	float b = 1.0f;
	float a = 1.0f;
};

// Abstract backend: id, name, geometry bounds, property panel JSON, pose/color hooks.
class DATA_EXPORT BackendDataBase
{
public:
	BackendDataBase();
	virtual ~BackendDataBase() = default;

	const std::string& id() const;
	void setId(const std::string& id);

	const std::string& name() const;
	void setName(const std::string& name);

	virtual std::string className() const = 0;

	virtual bool hasGeometry() const = 0;
	virtual BackendBoundingBox geometryBounds() const = 0;
	virtual std::size_t geometryElementCount() const = 0;
	virtual void clearGeometry() = 0;

	virtual BackendVec3 pose() const { return BackendVec3{}; }
	virtual void setPose(const BackendVec3& position) { (void)position; }
	virtual BackendVec3 rotation() const { return BackendVec3{}; }
	virtual void setRotation(const BackendVec3& eulerDeg) { (void)eulerDeg; }
	virtual BackendColor color() const { return BackendColor{}; }
	virtual void setColor(const BackendColor& c) { (void)c; }

	virtual bool hasPoseProperty() const { return false; }
	virtual bool hasRotationProperty() const { return false; }
	virtual bool hasColorProperty() const { return false; }

	// Property panel: JSON array; row shape in BackendPropertyRow.h (backend_property_json).
	virtual nlohmann::json snapshotPropertyRows() const;
	virtual bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg);

protected:
	static std::string generateId();

	std::vector<std::shared_ptr<BackendAttributeBase>> m_attributes;

private:
	std::string m_id;
	std::string m_name;
};
