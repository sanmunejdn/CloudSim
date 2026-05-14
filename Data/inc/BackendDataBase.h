#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <json.hpp>

#include "data_global.h"
#include "BackendComponent.h"
#include "BackendObjectAttribute.h"

class BackendDataManager;

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

	/// Narrow transform API: world-space pose authority for types that expose pose/rotation (see ARCHITECTURE §6.2.1).
	virtual bool supportsBackendTransform() const { return hasPoseProperty(); }
	virtual void applyBackendWorldPose(const BackendVec3& centerWorld, const BackendVec3& eulerDegWorld);

	// Property panel: JSON array; row shape in BackendPropertyRow.h (backend_property_json).
	// \a mgr resolves follow target id to display name when present.
	virtual nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const;
	virtual bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
		const BackendDataManager* mgr = nullptr);

	bool addComponent(const BackendComponentPtr& component);
	bool removeComponent(const std::string& componentType);
	BackendComponentPtr getComponent(const std::string& componentType) const;
	std::vector<BackendComponentPtr> listComponents() const;
	bool hasComponent(const std::string& componentType) const;

	std::vector<std::shared_ptr<BackendDataBase>> parentObjects(const BackendDataManager& manager) const;
	std::vector<std::shared_ptr<BackendDataBase>> childObjects(const BackendDataManager& manager) const;
	std::vector<std::shared_ptr<BackendDataBase>> descendantObjects(const BackendDataManager& manager) const;

protected:
	static std::string generateId();

	std::vector<std::shared_ptr<BackendAttributeBase>> m_attributes;

private:
	std::string m_id;
	std::string m_name;
	mutable std::mutex m_componentMutex;
	std::unordered_map<std::string, BackendComponentPtr> m_components;
};
