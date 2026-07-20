#ifndef DATA_BACKENDDATABASE_H
#define DATA_BACKENDDATABASE_H

/// @file BackendDataBase.h
/// @brief 后端基类：id、几何包围、属性面板、位姿/颜色钩子

#include "data_global.h"

#include "BackendComponent.h"
#include "BackendFollowMath.h"
#include "BackendObjectAttribute.h"
#include "PropertyBag.h"

#include <cstddef>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <json.hpp>

class BackendDataManager;

struct BackendVec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct BackendBoundingBox
{
	BackendVec3 min;
	BackendVec3 max;
	bool valid = false;
};

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

/// 后端基类：id、几何包围、属性面板、位姿/颜色钩子
class DATA_EXPORT BackendDataBase
{
public:
	BackendDataBase();
	virtual ~BackendDataBase() = default;

	const std::string& id() const;
	void setId(const std::string& id);

	const std::string& name() const;
	void setName(const std::string& name);

	/// 场景显示态真源（工程 JSON 字段 visible；OSG/树为派生视图）
	bool isVisible() const;
	void setVisible(bool visible);

	virtual std::string className() const = 0;

	virtual bool hasGeometry() const = 0;
	virtual BackendBoundingBox geometryBounds() const = 0;
	virtual std::size_t geometryElementCount() const = 0;
	virtual void clearGeometry() = 0;

	virtual BackendVec3 pose() const;
	virtual void setPose(const BackendVec3& position);
	virtual BackendVec3 rotation() const;
	virtual void setRotation(const BackendVec3& eulerDeg);
	virtual BackendColor color() const { return BackendColor{}; }
	virtual void setColor(const BackendColor& c) { (void)c; }

	virtual bool hasPoseProperty() const { return false; }
	virtual bool hasRotationProperty() const { return false; }
	virtual bool hasColorProperty() const { return false; }

	/// 窄变换 API：暴露 pose/rotation 的类型以世界位姿为准
	virtual bool supportsBackendTransform() const { return hasPoseProperty(); }
	virtual void applyBackendWorldPose(const BackendVec3& centerWorld, const BackendVec3& eulerDegWorld);

	BackendPoseReferenceFrame poseReferenceFrame() const;
	void setPoseReferenceFrame(BackendPoseReferenceFrame frame);
	BackendVec3 poseInFrame(BackendPoseReferenceFrame frame, const BackendDataManager* mgr = nullptr) const;
	BackendVec3 rotationInFrame(BackendPoseReferenceFrame frame, const BackendDataManager* mgr = nullptr) const;
	void setPoseInFrame(const BackendVec3& value, BackendPoseReferenceFrame frame,
						const BackendDataManager* mgr = nullptr);
	void setRotationInFrame(const BackendVec3& value, BackendPoseReferenceFrame frame,
							const BackendDataManager* mgr = nullptr);
	BackendPoseValue poseValue(BackendPoseReferenceFrame frame, const BackendDataManager* mgr = nullptr) const;
	void setPoseValue(const BackendPoseValue& value, BackendPoseReferenceFrame frame,
					  const BackendDataManager* mgr = nullptr);
	BackendMat4 worldMatrix(const BackendDataManager* mgr = nullptr) const;
	void setWorldMatrix(const BackendMat4& world, const BackendDataManager* mgr = nullptr);
	/// 用户拖动/Gizmo：后乘增量，geometry 不变
	void applyWorldMatrixIncrement(const BackendMat4& incrementLocal, const BackendDataManager* mgr = nullptr);
	bool validatePoseFrameRoundTrip(const BackendDataManager* mgr, double epsilon = 1e-6) const;

	PropertyBag& propertyBag() { return m_propertyBag; }
	const PropertyBag& propertyBag() const { return m_propertyBag; }
	nlohmann::json saveToJson() const;
	bool loadFromJson(const nlohmann::json& in, std::string* errMsg = nullptr);

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

	static std::string generateId();

protected:
	virtual void saveDerivedJson(nlohmann::json& out) const;
	virtual bool loadDerivedJson(const nlohmann::json& in, std::string* errMsg);

	std::vector<std::shared_ptr<BackendAttributeBase>> m_attributes;

private:
	std::string m_id;
	std::string m_name;
	bool m_visible = true;
	BackendPoseReferenceFrame m_poseReferenceFrame = BackendPoseReferenceFrame::World;
	BackendMat4 m_worldMatrix = BackendMat4::identity();
	mutable PropertyBag m_propertyBag;
	mutable std::mutex m_componentMutex;
	std::unordered_map<std::string, BackendComponentPtr> m_components;
	std::unordered_map<std::type_index, BackendComponentPtr> m_componentsByType;
};

#endif // DATA_BACKENDDATABASE_H
