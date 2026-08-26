/// @file FollowAttachmentComponent.cpp
/// @brief Follow 附着组件

#include "FollowAttachmentComponent.h"

#include "BackendDataManager.h"
#include "BackendPropertyRow.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace
{
/// 布尔解析统一入口，follow.* 各键保持一致
bool parseFollowBool(const std::string& value)
{
	return value == "1" || value == "true" || value == "True" || value == "TRUE" || value == "yes" ||
		   value == "Yes" || value == "on";
}

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

bool tryWorldMatForData(const BackendDataBase& data,
						const std::function<bool(const std::string&, BackendMat4& out)>& worldQuery,
						BackendMat4& outWorld)
{
	if (worldQuery && worldQuery(data.id(), outWorld))
	{
		return true;
	}
	if (!data.hasPoseProperty())
	{
		return false;
	}
	outWorld = data.worldMatrix();
	return true;
}
} // namespace

bool FollowAttachmentComponent::enabled() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_enabled;
}

void FollowAttachmentComponent::setEnabled(bool on)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_enabled = on;
}

FollowAttachmentComponent::Snapshot FollowAttachmentComponent::snapshot() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return Snapshot{m_enabled, m_targetId, m_localPos, m_localEuler, m_solverPaused, m_hierarchyDriven};
}

std::string FollowAttachmentComponent::targetBackendId() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_targetId;
}

void FollowAttachmentComponent::collectReferencedBackendIds(std::vector<std::string>& out) const
{
	std::string t = targetBackendId();
	if (!t.empty())
	{
		out.push_back(std::move(t));
	}
}

void FollowAttachmentComponent::setTargetBackendId(std::string id)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_targetId = std::move(id);
}

BackendVec3 FollowAttachmentComponent::localPosition() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_localPos;
}

void FollowAttachmentComponent::setLocalPosition(const BackendVec3& p)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_localPos = p;
}

BackendVec3 FollowAttachmentComponent::localEulerDeg() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_localEuler;
}

void FollowAttachmentComponent::setLocalEulerDeg(const BackendVec3& e)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_localEuler = e;
}

void FollowAttachmentComponent::setLocalPose(const BackendVec3& p, const BackendVec3& e)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_localPos = p;
	m_localEuler = e;
}

bool FollowAttachmentComponent::solverPaused() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_solverPaused;
}

void FollowAttachmentComponent::setSolverPaused(bool on)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_solverPaused = on;
}

bool FollowAttachmentComponent::hierarchyDriven() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_hierarchyDriven;
}

void FollowAttachmentComponent::setHierarchyDriven(bool on)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_hierarchyDriven = on;
}

void FollowAttachmentComponent::appendPropertyRows(nlohmann::json& rows, const BackendDataManager* mgr) const
{
	std::string display;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (!m_targetId.empty())
		{
			if (mgr)
			{
				if (const auto t = mgr->getData(m_targetId))
				{
					display = t->name();
				}
				else
				{
					display = m_targetId;
				}
			}
			else
			{
				display = m_targetId;
			}
		}
	}
	backend_property_json::appendRow(rows, "follow.targetName", "Follow: target object name", true, display);
}

bool FollowAttachmentComponent::appendDefaultPropertyRowsWhenAbsent(nlohmann::json& rows,
																	const BackendDataManager* mgr) const
{
	appendPropertyRows(rows, mgr);
	return true;
}

void FollowAttachmentComponent::syncTargetNameInOwnerPropertyBag(BackendDataBase& owner,
																const BackendDataManager* mgr)
{
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		owner.getComponent(typeKeyStatic()));
	if (!follow || !mgr)
	{
		return;
	}
	std::string targetId;
	{
		std::lock_guard<std::mutex> lock(follow->m_mutex);
		targetId = follow->m_targetId;
	}
	if (targetId.empty())
	{
		owner.propertyBag().set<std::string>("follow.targetName", std::string());
		return;
	}
	if (const auto target = mgr->getData(targetId))
	{
		owner.propertyBag().set<std::string>("follow.targetName", target->name());
	}
}

bool FollowAttachmentComponent::applyPropertyChange(BackendDataBase& owner, const std::string& key,
													const std::string& value, std::string* errMsg,
													const BackendDataManager* mgr)
{
	auto parseVec3 = [&](const std::string& s, BackendVec3& out) -> bool
	{
		std::istringstream iss(s);
		if (!(iss >> out.x >> out.y >> out.z))
		{
			return false;
		}
		return true;
	};

	if (key == "follow.targetName")
	{
		const std::string name = trimUtf8Whitespace(value);
		if (name.empty())
		{
			owner.removeComponent(typeKeyStatic());
			owner.propertyBag().set<std::string>("follow.targetName", std::string());
			return true;
		}
		if (!mgr)
		{
			if (errMsg)
			{
				*errMsg = "Document backend registry required to resolve object name.";
			}
			return false;
		}
		const std::vector<std::shared_ptr<BackendDataBase>> matches = mgr->findByName(name);
		if (matches.empty())
		{
			if (errMsg)
			{
				*errMsg = "No object with name \"" + name + "\".";
			}
			return false;
		}
		if (matches.size() > 1U)
		{
			if (errMsg)
			{
				*errMsg = "Multiple objects named \"" + name + "\"; use a unique name.";
			}
			return false;
		}
		const std::shared_ptr<BackendDataBase>& target = matches.front();
		if (!target)
		{
			return false;
		}
		if (target->id() == owner.id())
		{
			if (errMsg)
			{
				*errMsg = "Cannot follow self.";
			}
			return false;
		}
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_targetId = target->id();
			m_enabled = true;
			m_hierarchyDriven = false;
		}
		// 已解析到 target，直接写 bag；勿在持锁时调 sync（会再抢 m_mutex）
		owner.propertyBag().set<std::string>("follow.targetName", target->name());
		return true;
	}

	std::lock_guard<std::mutex> lock(m_mutex);
	if (key == "follow.hierarchyDriven")
	{
		(void)value;
		if (errMsg)
		{
			*errMsg = "Read-only: set automatically from object hierarchy.";
		}
		return false;
	}
	if (key == "follow.enabled")
	{
		m_enabled = parseFollowBool(value);
		return true;
	}
	if (key == "follow.targetId")
	{
		m_hierarchyDriven = false;
		m_targetId = value;
		if (m_targetId == owner.id())
		{
			if (errMsg)
			{
				*errMsg = "Cannot follow self.";
			}
			m_targetId.clear();
			return false;
		}
		return true;
	}
	if (key == "follow.localPosition")
	{
		BackendVec3 p{};
		if (!parseVec3(value, p))
		{
			if (errMsg)
			{
				*errMsg = "Expected three numbers: x y z";
			}
			return false;
		}
		m_localPos = p;
		return true;
	}
	if (key == "follow.localEulerDeg")
	{
		BackendVec3 e{};
		if (!parseVec3(value, e))
		{
			if (errMsg)
			{
				*errMsg = "Expected three numbers: rx ry rz (deg)";
			}
			return false;
		}
		m_localEuler = e;
		return true;
	}
	if (key == "follow.solverPaused")
	{
		m_solverPaused = parseFollowBool(value);
		return true;
	}
	if (key == "follow.snapLocal")
	{
		(void)owner;
		if (errMsg)
		{
			*errMsg = "Use FollowAttachmentComponent::recomputeLocalFromCurrentWorld from UI layer.";
		}
		return false;
	}
	return false;
}

void FollowAttachmentComponent::writeJson(nlohmann::json& out) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	out["enabled"] = m_enabled;
	out["targetId"] = m_targetId;
	out["localPosition"] = {{"x", m_localPos.x}, {"y", m_localPos.y}, {"z", m_localPos.z}};
	out["localEulerDeg"] = {{"x", m_localEuler.x}, {"y", m_localEuler.y}, {"z", m_localEuler.z}};
	out["solverPaused"] = m_solverPaused;
	out["hierarchyDriven"] = m_hierarchyDriven;
}

void FollowAttachmentComponent::readJson(const nlohmann::json& in)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_enabled = in.value("enabled", false);
	m_targetId = in.value("targetId", std::string());
	const auto p = in.value("localPosition", nlohmann::json::object());
	m_localPos.x = p.value("x", 0.0);
	m_localPos.y = p.value("y", 0.0);
	m_localPos.z = p.value("z", 0.0);
	const auto e = in.value("localEulerDeg", nlohmann::json::object());
	m_localEuler.x = e.value("x", 0.0);
	m_localEuler.y = e.value("y", 0.0);
	m_localEuler.z = e.value("z", 0.0);
	m_solverPaused = in.value("solverPaused", false);
	m_hierarchyDriven = in.value("hierarchyDriven", false);
}

bool FollowAttachmentComponent::recomputeLocalFromCurrentWorld(
	const BackendDataManager& mgr, const std::function<bool(const std::string&, BackendMat4& out)>& worldQuery,
	BackendDataBase& follower, std::string* errMsg)
{
	auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(follower.getComponent(typeKeyStatic()));
	if (!comp)
	{
		if (errMsg)
		{
			*errMsg = "No FollowAttachment component.";
		}
		return false;
	}
	const std::string tid = comp->targetBackendId();
	if (tid.empty())
	{
		if (errMsg)
		{
			*errMsg = "Target id empty.";
		}
		return false;
	}
	const auto targetObj = mgr.getData(tid);
	if (!targetObj)
	{
		if (errMsg)
		{
			*errMsg = "Target backend not found.";
		}
		return false;
	}
	BackendMat4 wT{};
	BackendMat4 wF{};
	if (!tryWorldMatForData(*targetObj, worldQuery, wT) || !tryWorldMatForData(follower, worldQuery, wF))
	{
		if (errMsg)
		{
			*errMsg = "Could not resolve world matrices.";
		}
		return false;
	}
	BackendMat4 invT{};
	(void)backend_mat4_invert_rigid(wT, invT);
	BackendMat4 localRel{};
	(void)backend_mat4_multiply(invT, wF, localRel);
	BackendVec3 lp{};
	BackendVec3 le{};
	backend_trans_euler_from_rigid_mat(localRel, lp, le);
	comp->setLocalPosition(lp);
	comp->setLocalEulerDeg(le);
	return true;
}
