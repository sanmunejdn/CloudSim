#include "BackendDataBase.h"

#include "BackendDataManager.h"
#include "BackendPropertyRow.h"
#include "FollowAttachmentComponent.h"

#include <atomic>
#include <mutex>

namespace
{
std::atomic<unsigned long long> g_backendDataIdCounter{ 1ULL };

std::string trimUtf8Whitespace(const std::string& s)
{
	std::size_t a = 0;
	std::size_t b = s.size();
	while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n'))
	{
		++a;
	}
	while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n'))
	{
		--b;
	}
	return s.substr(a, b - a);
}
} // namespace

BackendDataBase::BackendDataBase()
	: m_id(generateId())
	, m_name("UnnamedData")
{
}

const std::string& BackendDataBase::id() const
{
	return m_id;
}

void BackendDataBase::setId(const std::string& id)
{
	if (!id.empty())
	{
		m_id = id;
	}
}

const std::string& BackendDataBase::name() const
{
	return m_name;
}

void BackendDataBase::setName(const std::string& name)
{
	if (!name.empty())
	{
		m_name = name;
	}
}

void BackendDataBase::applyBackendWorldPose(const BackendVec3& centerWorld, const BackendVec3& eulerDegWorld)
{
	setPose(centerWorld);
	setRotation(eulerDegWorld);
}

std::string BackendDataBase::generateId()
{
	const auto id = g_backendDataIdCounter.fetch_add(1ULL);
	return "backend_data_" + std::to_string(id);
}

nlohmann::json BackendDataBase::snapshotPropertyRows(const BackendDataManager* mgr) const
{
	nlohmann::json rows = nlohmann::json::array();
	backend_property_json::appendRow(rows, "core.id", "ID", false, m_id);
	backend_property_json::appendRow(rows, "core.name", "Name", false, m_name);
	backend_property_json::appendRow(rows, "core.class", "Class", false, className());
	if (const auto f = std::dynamic_pointer_cast<FollowAttachmentComponent>(getComponent(FollowAttachmentComponent::typeKeyStatic())))
	{
		f->appendPropertyRows(rows, mgr);
	}
	else if (hasPoseProperty())
	{
		FollowAttachmentComponent defaults;
		defaults.appendPropertyRows(rows, mgr);
	}
	return rows;
}

bool BackendDataBase::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
	const BackendDataManager* mgr)
{
	const auto ensureFollow = [&]() {
		if (!getComponent(FollowAttachmentComponent::typeKeyStatic()))
		{
			addComponent(std::make_shared<FollowAttachmentComponent>());
		}
	};
	if (key == "follow.targetName")
	{
		const std::string trimmed = trimUtf8Whitespace(value);
		if (trimmed.empty())
		{
			removeComponent(FollowAttachmentComponent::typeKeyStatic());
			return true;
		}
	}
	if (key.rfind("follow.", 0) == 0)
	{
		ensureFollow();
		if (const auto f = std::dynamic_pointer_cast<FollowAttachmentComponent>(getComponent(FollowAttachmentComponent::typeKeyStatic())))
		{
			return f->applyPropertyChange(*this, key, value, errMsg, mgr);
		}
	}
	if (errMsg)
	{
		*errMsg = "Property is read-only for this object type.";
	}
	return false;
}

bool BackendDataBase::addComponent(const BackendComponentPtr& component)
{
	if (!component)
	{
		return false;
	}
	const std::string type = component->componentType();
	if (type.empty())
	{
		return false;
	}
	std::lock_guard<std::mutex> lock(m_componentMutex);
	m_components[type] = component;
	return true;
}

bool BackendDataBase::removeComponent(const std::string& componentType)
{
	if (componentType.empty())
	{
		return false;
	}
	std::lock_guard<std::mutex> lock(m_componentMutex);
	return m_components.erase(componentType) > 0;
}

BackendComponentPtr BackendDataBase::getComponent(const std::string& componentType) const
{
	if (componentType.empty())
	{
		return nullptr;
	}
	std::lock_guard<std::mutex> lock(m_componentMutex);
	const auto it = m_components.find(componentType);
	if (it == m_components.end())
	{
		return nullptr;
	}
	return it->second;
}

std::vector<BackendComponentPtr> BackendDataBase::listComponents() const
{
	std::lock_guard<std::mutex> lock(m_componentMutex);
	std::vector<BackendComponentPtr> components;
	components.reserve(m_components.size());
	for (const auto& item : m_components)
	{
		components.push_back(item.second);
	}
	return components;
}

bool BackendDataBase::hasComponent(const std::string& componentType) const
{
	if (componentType.empty())
	{
		return false;
	}
	std::lock_guard<std::mutex> lock(m_componentMutex);
	return m_components.find(componentType) != m_components.end();
}

std::vector<std::shared_ptr<BackendDataBase>> BackendDataBase::parentObjects(const BackendDataManager& manager) const
{
	std::vector<std::shared_ptr<BackendDataBase>> out;
	const std::vector<std::string> ids = manager.parentsOf(id());
	out.reserve(ids.size());
	for (const std::string& parentId : ids)
	{
		std::shared_ptr<BackendDataBase> obj = manager.getData(parentId);
		if (obj)
		{
			out.push_back(std::move(obj));
		}
	}
	return out;
}

std::vector<std::shared_ptr<BackendDataBase>> BackendDataBase::childObjects(const BackendDataManager& manager) const
{
	std::vector<std::shared_ptr<BackendDataBase>> out;
	const std::vector<std::string> ids = manager.childrenOf(id());
	out.reserve(ids.size());
	for (const std::string& childId : ids)
	{
		std::shared_ptr<BackendDataBase> obj = manager.getData(childId);
		if (obj)
		{
			out.push_back(std::move(obj));
		}
	}
	return out;
}

std::vector<std::shared_ptr<BackendDataBase>> BackendDataBase::descendantObjects(const BackendDataManager& manager) const
{
	std::vector<std::shared_ptr<BackendDataBase>> out;
	const std::vector<std::string> ids = manager.descendantsOf(id());
	out.reserve(ids.size());
	for (const std::string& childId : ids)
	{
		std::shared_ptr<BackendDataBase> obj = manager.getData(childId);
		if (obj)
		{
			out.push_back(std::move(obj));
		}
	}
	return out;
}

