#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <typeindex>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <json.hpp>

#include "data_global.h"
#include "BackendComponent.h"
#include "BackendObjectAttribute.h"
#include "BackendFollowMath.h"
#include "PropertyBag.h"

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

struct BackendPoseValue
{
	BackendVec3 position{};
	BackendVec3 eulerDeg{};
};

enum class BackendPoseReferenceFrame
{
	World = 0,
	Parent = 1
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

	BackendPoseReferenceFrame poseReferenceFrame() const;
	void setPoseReferenceFrame(BackendPoseReferenceFrame frame);
	BackendVec3 poseInFrame(BackendPoseReferenceFrame frame, const BackendDataManager* mgr = nullptr) const;
	BackendVec3 rotationInFrame(BackendPoseReferenceFrame frame, const BackendDataManager* mgr = nullptr) const;
	void setPoseInFrame(const BackendVec3& value, BackendPoseReferenceFrame frame, const BackendDataManager* mgr = nullptr);
	void setRotationInFrame(const BackendVec3& value, BackendPoseReferenceFrame frame, const BackendDataManager* mgr = nullptr);
	BackendPoseValue poseValue(BackendPoseReferenceFrame frame, const BackendDataManager* mgr = nullptr) const;
	void setPoseValue(const BackendPoseValue& value, BackendPoseReferenceFrame frame, const BackendDataManager* mgr = nullptr);
	BackendMat4 worldMatrix(const BackendDataManager* mgr = nullptr) const;
	void setWorldMatrix(const BackendMat4& world, const BackendDataManager* mgr = nullptr);
	bool validatePoseFrameRoundTrip(const BackendDataManager* mgr, double epsilon = 1e-6) const;

	PropertyBag& propertyBag() { return m_propertyBag; }
	const PropertyBag& propertyBag() const { return m_propertyBag; }
	nlohmann::json saveToJson() const;
	bool loadFromJson(const nlohmann::json& in, std::string* errMsg = nullptr);

	// Property panel: JSON array; row shape in BackendPropertyRow.h (backend_property_json).
	// \a mgr resolves follow target id to display name when present.
	virtual nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const;
	virtual bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
		const BackendDataManager* mgr = nullptr);

	bool addComponent(const BackendComponentPtr& component);
	bool removeComponent(const std::string& componentType);
	BackendComponentPtr getComponent(const std::string& componentType) const;
	template <typename T>
	std::shared_ptr<T> getComponent() const
	{
		static_assert(std::is_base_of<IBackendComponent, T>::value, "T must derive from IBackendComponent");
		std::lock_guard<std::mutex> lock(m_componentMutex);
		const auto it = m_componentsByType.find(std::type_index(typeid(T)));
		if (it == m_componentsByType.end())
		{
			return nullptr;
		}
		return std::dynamic_pointer_cast<T>(it->second);
	}

	template <typename T, typename... Args>
	std::shared_ptr<T> emplaceComponent(Args&&... args)
	{
		static_assert(std::is_base_of<IBackendComponent, T>::value, "T must derive from IBackendComponent");
		const std::shared_ptr<T> comp = std::make_shared<T>(std::forward<Args>(args)...);
		if (!addComponent(comp))
		{
			return nullptr;
		}
		return comp;
	}
	std::vector<BackendComponentPtr> listComponents() const;
	bool hasComponent(const std::string& componentType) const;

	std::vector<std::shared_ptr<BackendDataBase>> parentObjects(const BackendDataManager& manager) const;
	std::vector<std::shared_ptr<BackendDataBase>> childObjects(const BackendDataManager& manager) const;
	std::vector<std::shared_ptr<BackendDataBase>> descendantObjects(const BackendDataManager& manager) const;

protected:
	static std::string generateId();
	virtual void saveDerivedJson(nlohmann::json& out) const;
	virtual bool loadDerivedJson(const nlohmann::json& in, std::string* errMsg);

	std::vector<std::shared_ptr<BackendAttributeBase>> m_attributes;

private:
	std::string m_id;
	std::string m_name;
	BackendPoseReferenceFrame m_poseReferenceFrame = BackendPoseReferenceFrame::World;
	mutable std::shared_mutex m_worldMatrixMutex;
	mutable bool m_worldMatrixDirty = true;
	mutable BackendMat4 m_worldMatrixCache = BackendMat4::identity();
	mutable PropertyBag m_propertyBag;
	mutable std::mutex m_componentMutex;
	std::unordered_map<std::string, BackendComponentPtr> m_components;
	std::unordered_map<std::type_index, BackendComponentPtr> m_componentsByType;
};
