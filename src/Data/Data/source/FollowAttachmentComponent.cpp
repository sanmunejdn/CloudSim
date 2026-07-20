/// @file FollowAttachmentComponent.cpp
/// @brief FollowAttachmentComponent 实现

#include "FollowAttachmentComponent.h"

#include "BackendDataManager.h"
#include "BackendPropertyRow.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace
{
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

BackendVec3 modelCenterForData(const BackendDataBase& data)
{
	if (const auto* pc = dynamic_cast<const PointCloudBackendData*>(&data))
	{
		const auto& xyz = pc->pointPositionsXyz();
		if (xyz.size() < 3U || (xyz.size() % 3U) != 0U)
		{
			return BackendVec3{};
		}
		float minx = xyz[0], maxx = xyz[0], miny = xyz[1], maxy = xyz[1], minz = xyz[2], maxz = xyz[2];
		for (std::size_t i = 0; i + 2 < xyz.size(); i += 3U)
		{
			const float x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
			minx = std::min(minx, x);
			maxx = std::max(maxx, x);
			miny = std::min(miny, y);
			maxy = std::max(maxy, y);
			minz = std::min(minz, z);
			maxz = std::max(maxz, z);
		}
		return BackendVec3{0.5 * (static_cast<double>(minx) + static_cast<double>(maxx)),
						   0.5 * (static_cast<double>(miny) + static_cast<double>(maxy)),
						   0.5 * (static_cast<double>(minz) + static_cast<double>(maxz))};
	}
	if (const auto* mesh = dynamic_cast<const MeshBackendData*>(&data))
	{
		const auto& soup = mesh->triangleSoup();
		if (soup.size() < 3U || (soup.size() % 3U) != 0U)
		{
			return BackendVec3{};
		}
		float minx = soup[0], maxx = soup[0], miny = soup[1], maxy = soup[1], minz = soup[2], maxz = soup[2];
		for (std::size_t i = 0; i + 2 < soup.size(); i += 3U)
		{
			const float x = soup[i], y = soup[i + 1], z = soup[i + 2];
			minx = std::min(minx, x);
			maxx = std::max(maxx, x);
			miny = std::min(miny, y);
			maxy = std::max(maxy, y);
			minz = std::min(minz, z);
			maxz = std::max(maxz, z);
		}
		return BackendVec3{0.5 * (static_cast<double>(minx) + static_cast<double>(maxx)),
						   0.5 * (static_cast<double>(miny) + static_cast<double>(maxy)),
						   0.5 * (static_cast<double>(minz) + static_cast<double>(maxz))};
	}
	return BackendVec3{};
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
	const BackendMat4 w = backend_world_mat_from_pose(data.pose(), data.rotation());
	outWorld = w;
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

const std::string& FollowAttachmentComponent::targetBackendId() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_targetId;
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
			std::lock_guard<std::mutex> lock(m_mutex);
			m_enabled = false;
			m_targetId.clear();
			m_hierarchyDriven = false;
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
		std::lock_guard<std::mutex> lock(m_mutex);
		m_targetId = target->id();
		m_enabled = true;
		m_hierarchyDriven = false;
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
		m_enabled = (value == "1" || value == "true" || value == "True" || value == "yes");
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
		m_solverPaused = (value == "1" || value == "true");
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
	if (!backend_mat4_invert_rigid(wT, invT))
	{
		if (errMsg)
		{
			*errMsg = "invert target failed.";
		}
		return false;
	}
	BackendMat4 localRel{};
	backend_mat4_multiply(invT, wF, localRel);
	BackendVec3 lp{};
	BackendVec3 le{};
	backend_trans_euler_from_rigid_mat(localRel, lp, le);
	comp->setLocalPosition(lp);
	comp->setLocalEulerDeg(le);
	return true;
}
