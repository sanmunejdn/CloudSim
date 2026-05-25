#include "BackendDataBase.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "BackendComponentCodecBuiltins.h"
#include "BackendComponentCodecRegistry.h"
#include "BackendPropertyRow.h"
#include "FollowAttachmentComponent.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "PropertyRowsCompatAdapter.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <limits>
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

std::string toLowerAscii(std::string s)
{
	for (char& ch : s)
	{
		ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
	}
	return s;
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
		return BackendVec3{ 0.5 * (static_cast<double>(minx) + static_cast<double>(maxx)),
			0.5 * (static_cast<double>(miny) + static_cast<double>(maxy)),
			0.5 * (static_cast<double>(minz) + static_cast<double>(maxz)) };
	}
	if (const auto* mesh = dynamic_cast<const MeshBackendData*>(&data))
	{
		if (mesh->transformPivotAtOrigin())  // URDF 枢轴在原点
		{
			return BackendVec3{};
		}
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
		return BackendVec3{ 0.5 * (static_cast<double>(minx) + static_cast<double>(maxx)),
			0.5 * (static_cast<double>(miny) + static_cast<double>(maxy)),
			0.5 * (static_cast<double>(minz) + static_cast<double>(maxz)) };
	}
	return BackendVec3{};
}

bool buildWorldPoseInFrame(
	const BackendDataBase& owner,
	const BackendVec3& poseFrame,
	const BackendVec3& rotFrame,
	BackendPoseReferenceFrame frame,
	const BackendDataManager* mgr,
	BackendVec3& outWorldPose,
	BackendVec3& outWorldEuler)
{
	const BackendVec3 center = modelCenterForData(owner);
	if (frame == BackendPoseReferenceFrame::World || mgr == nullptr)
	{
		outWorldPose = poseFrame;
		outWorldEuler = rotFrame;
		return true;
	}

	const std::vector<std::string> parents = mgr->parentsOf(owner.id());
	if (parents.empty())
	{
		outWorldPose = poseFrame;
		outWorldEuler = rotFrame;
		return true;
	}
	const auto parent = mgr->getData(parents.front());
	if (!parent || !parent->hasPoseProperty())
	{
		outWorldPose = poseFrame;
		outWorldEuler = rotFrame;
		return true;
	}

	const BackendVec3 parentCenter = modelCenterForData(*parent);
	const BackendMat4 parentWorld = backend_world_mat_from_pose(parentCenter, parent->pose(), parent->rotation());
	const BackendMat4 local = backend_world_mat_from_pose(center, poseFrame, rotFrame);
	BackendMat4 world{};
	backend_mat4_multiply(parentWorld, local, world);
	backend_pose_euler_from_world_mat(world, center, outWorldPose, outWorldEuler);
	return true;
}

double maxAbsVec3Diff(const BackendVec3& a, const BackendVec3& b)
{
	return std::max({ std::abs(a.x - b.x), std::abs(a.y - b.y), std::abs(a.z - b.z) });
}

BackendMat4 worldMatrixFromData(const BackendDataBase& data)
{
	const BackendVec3 center = modelCenterForData(data);
	return backend_world_mat_from_pose(center, data.pose(), data.rotation());
}

bool jsonToVec3(const nlohmann::json& in, BackendVec3& out)
{
	if (!in.is_object())
	{
		return false;
	}
	out = BackendVec3{ in.value("x", 0.0), in.value("y", 0.0), in.value("z", 0.0) };
	return true;
}

bool jsonToColor(const nlohmann::json& in, BackendColor& out)
{
	if (!in.is_object())
	{
		return false;
	}
	out = BackendColor{
		static_cast<float>(in.value("r", 1.0)),
		static_cast<float>(in.value("g", 1.0)),
		static_cast<float>(in.value("b", 1.0)),
		static_cast<float>(in.value("a", 1.0)) };
	return true;
}

void loadPropertyBagFromJson(const nlohmann::json& in, PropertyBag& outBag)
{
	PropertyBag restored;
	if (!in.is_object())
	{
		outBag = std::move(restored);
		return;
	}
	for (auto it = in.begin(); it != in.end(); ++it)
	{
		const std::string& key = it.key();
		const nlohmann::json& value = it.value();
		if (value.is_boolean())
		{
			restored.set<bool>(key, value.get<bool>());
			continue;
		}
		if (value.is_number_integer())
		{
			const long long v = value.get<long long>();
			if (v >= static_cast<long long>(std::numeric_limits<int>::min())
				&& v <= static_cast<long long>(std::numeric_limits<int>::max()))
			{
				restored.set<int>(key, static_cast<int>(v));
			}
			else
			{
				restored.set<double>(key, static_cast<double>(v));
			}
			continue;
		}
		if (value.is_number_float())
		{
			restored.set<double>(key, value.get<double>());
			continue;
		}
		if (value.is_string())
		{
			restored.set<std::string>(key, value.get<std::string>());
			continue;
		}
		if (value.is_array())
		{
			if (value.size() == 3 && value[0].is_number() && value[1].is_number() && value[2].is_number())
			{
				restored.set<std::array<double, 3>>(key, std::array<double, 3>{
														 value[0].get<double>(), value[1].get<double>(), value[2].get<double>() });
				continue;
			}
			if (value.size() == 4 && value[0].is_number() && value[1].is_number() && value[2].is_number()
				&& value[3].is_number())
			{
				restored.set<std::array<float, 4>>(key,
					std::array<float, 4>{ static_cast<float>(value[0].get<double>()),
						static_cast<float>(value[1].get<double>()),
						static_cast<float>(value[2].get<double>()),
						static_cast<float>(value[3].get<double>()) });
			}
		}
	}
	outBag = std::move(restored);
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

nlohmann::json BackendDataBase::saveToJson() const
{
	ensureBackendComponentCodecBuiltinsRegistered();
	nlohmann::json out = nlohmann::json::object();
	out["id"] = m_id;
	out["name"] = m_name;
	out["className"] = className();
	out["propertyBag"] = m_propertyBag.toJson();
	out["poseReferenceFrame"] = (m_poseReferenceFrame == BackendPoseReferenceFrame::Parent) ? "parent" : "world";
	nlohmann::json components = nlohmann::json::array();
	for (const auto& component : listComponents())
	{
		nlohmann::json item = BackendComponentCodecRegistry::instance().encodeComponent(component);
		if (!item.is_null())
		{
			components.push_back(std::move(item));
		}
	}
	if (!components.empty())
	{
		out["components"] = std::move(components);
	}

	if (hasPoseProperty())
	{
		const BackendVec3 p = pose();
		out["pose"] = nlohmann::json{ { "x", p.x }, { "y", p.y }, { "z", p.z } };
	}
	if (hasRotationProperty())
	{
		const BackendVec3 r = rotation();
		out["rotation"] = nlohmann::json{ { "x", r.x }, { "y", r.y }, { "z", r.z } };
	}
	if (hasColorProperty())
	{
		const BackendColor c = color();
		out["color"] = nlohmann::json{ { "r", c.r }, { "g", c.g }, { "b", c.b }, { "a", c.a } };
	}
	const BackendMat4 wm = worldMatrix();
	nlohmann::json wmArr = nlohmann::json::array();
	for (double v : wm.v)
	{
		wmArr.push_back(v);
	}
	out["worldMatrix"] = wmArr;

	saveDerivedJson(out);
	return out;
}

bool BackendDataBase::loadFromJson(const nlohmann::json& in, std::string* errMsg)
{
	ensureBackendComponentCodecBuiltinsRegistered();
	if (!in.is_object())
	{
		if (errMsg)
		{
			*errMsg = "Backend json must be object.";
		}
		return false;
	}

	const std::string newId = in.value("id", std::string());
	if (!newId.empty())
	{
		setId(newId);
	}
	const std::string newName = in.value("name", std::string());
	if (!newName.empty())
	{
		setName(newName);
	}
	const std::string frame = toLowerAscii(in.value("poseReferenceFrame", std::string("world")));
	setPoseReferenceFrame(frame == "parent" ? BackendPoseReferenceFrame::Parent : BackendPoseReferenceFrame::World);

	if (hasPoseProperty())
	{
		BackendVec3 p{};
		if (jsonToVec3(in.value("pose", nlohmann::json::object()), p))
		{
			setPose(p);
		}
	}
	if (hasRotationProperty())
	{
		BackendVec3 r{};
		if (jsonToVec3(in.value("rotation", nlohmann::json::object()), r))
		{
			setRotation(r);
		}
	}
	if (hasColorProperty())
	{
		BackendColor c{};
		if (jsonToColor(in.value("color", nlohmann::json::object()), c))
		{
			setColor(c);
		}
	}

	loadPropertyBagFromJson(in.value("propertyBag", nlohmann::json::object()), m_propertyBag);
	removeComponent(FollowAttachmentComponent::typeKeyStatic());
	const nlohmann::json components = in.value("components", nlohmann::json::array());
	if (components.is_array())
	{
		for (const auto& item : components)
		{
			const BackendComponentPtr component = BackendComponentCodecRegistry::instance().decodeComponent(item);
			if (component)
			{
				addComponent(component);
			}
		}
	}
	if (!hasComponent(FollowAttachmentComponent::typeKeyStatic()) && in.contains("followAttachment")
		&& in["followAttachment"].is_object())
	{
		auto follow = std::make_shared<FollowAttachmentComponent>();
		follow->readJson(in["followAttachment"]);
		addComponent(follow);
	}
	if (in.contains("worldMatrix"))
	{
		const nlohmann::json wm = in["worldMatrix"];
		if (wm.is_array() && wm.size() == 16)
		{
			BackendMat4 world{};
			bool ok = true;
			for (std::size_t i = 0; i < 16U; ++i)
			{
				if (!wm[i].is_number())
				{
					ok = false;
					break;
				}
				world.v[i] = wm[i].get<double>();
			}
			if (ok)
			{
				setWorldMatrix(world);
			}
		}
	}
	return loadDerivedJson(in, errMsg);
}

void BackendDataBase::applyBackendWorldPose(const BackendVec3& centerWorld, const BackendVec3& eulerDegWorld)
{
	setPose(centerWorld);
	setRotation(eulerDegWorld);
	std::unique_lock<std::shared_mutex> lock(m_worldMatrixMutex);
	m_worldMatrixDirty = true;
}

BackendPoseReferenceFrame BackendDataBase::poseReferenceFrame() const
{
	return m_poseReferenceFrame;
}

void BackendDataBase::setPoseReferenceFrame(BackendPoseReferenceFrame frame)
{
	m_poseReferenceFrame = frame;
}

BackendVec3 BackendDataBase::poseInFrame(BackendPoseReferenceFrame frame, const BackendDataManager* mgr) const
{
	if (!hasPoseProperty())
	{
		return BackendVec3{};
	}
	if (frame == BackendPoseReferenceFrame::World || mgr == nullptr)
	{
		return pose();
	}
	const std::vector<std::string> parents = mgr->parentsOf(id());
	if (parents.empty())
	{
		return pose();
	}
	const auto parent = mgr->getData(parents.front());
	if (!parent || !parent->hasPoseProperty())
	{
		return pose();
	}

	const BackendVec3 selfCenter = modelCenterForData(*this);
	const BackendVec3 parentCenter = modelCenterForData(*parent);
	const BackendMat4 selfWorld = backend_world_mat_from_pose(selfCenter, pose(), rotation());
	const BackendMat4 parentWorld = backend_world_mat_from_pose(parentCenter, parent->pose(), parent->rotation());
	BackendMat4 invParent{};
	backend_mat4_invert_rigid(parentWorld, invParent);
	BackendMat4 selfLocal{};
	backend_mat4_multiply(invParent, selfWorld, selfLocal);
	BackendVec3 localPose{};
	BackendVec3 localEuler{};
	backend_pose_euler_from_world_mat(selfLocal, selfCenter, localPose, localEuler);
	return localPose;
}

BackendVec3 BackendDataBase::rotationInFrame(BackendPoseReferenceFrame frame, const BackendDataManager* mgr) const
{
	if (!hasRotationProperty())
	{
		return BackendVec3{};
	}
	if (frame == BackendPoseReferenceFrame::World || mgr == nullptr)
	{
		return rotation();
	}
	const std::vector<std::string> parents = mgr->parentsOf(id());
	if (parents.empty())
	{
		return rotation();
	}
	const auto parent = mgr->getData(parents.front());
	if (!parent || !parent->hasPoseProperty())
	{
		return rotation();
	}

	const BackendVec3 selfCenter = modelCenterForData(*this);
	const BackendVec3 parentCenter = modelCenterForData(*parent);
	const BackendMat4 selfWorld = backend_world_mat_from_pose(selfCenter, pose(), rotation());
	const BackendMat4 parentWorld = backend_world_mat_from_pose(parentCenter, parent->pose(), parent->rotation());
	BackendMat4 invParent{};
	backend_mat4_invert_rigid(parentWorld, invParent);
	BackendMat4 selfLocal{};
	backend_mat4_multiply(invParent, selfWorld, selfLocal);
	BackendVec3 localPose{};
	BackendVec3 localEuler{};
	backend_pose_euler_from_world_mat(selfLocal, selfCenter, localPose, localEuler);
	return localEuler;
}

void BackendDataBase::setPoseInFrame(const BackendVec3& value, BackendPoseReferenceFrame frame, const BackendDataManager* mgr)
{
	if (!hasPoseProperty())
	{
		return;
	}
	BackendVec3 worldPose{};
	BackendVec3 worldEuler{};
	buildWorldPoseInFrame(*this, value, rotationInFrame(frame, mgr), frame, mgr, worldPose, worldEuler);
	setPose(worldPose);
	if (hasRotationProperty())
	{
		setRotation(worldEuler);
	}
	{
		std::unique_lock<std::shared_mutex> lock(m_worldMatrixMutex);
		m_worldMatrixDirty = true;
	}
}

void BackendDataBase::setRotationInFrame(const BackendVec3& value, BackendPoseReferenceFrame frame, const BackendDataManager* mgr)
{
	if (!hasRotationProperty())
	{
		return;
	}
	BackendVec3 worldPose{};
	BackendVec3 worldEuler{};
	buildWorldPoseInFrame(*this, poseInFrame(frame, mgr), value, frame, mgr, worldPose, worldEuler);
	if (hasPoseProperty())
	{
		setPose(worldPose);
	}
	setRotation(worldEuler);
	{
		std::unique_lock<std::shared_mutex> lock(m_worldMatrixMutex);
		m_worldMatrixDirty = true;
	}
}

BackendPoseValue BackendDataBase::poseValue(BackendPoseReferenceFrame frame, const BackendDataManager* mgr) const
{
	BackendPoseValue out;
	out.position = poseInFrame(frame, mgr);
	out.eulerDeg = rotationInFrame(frame, mgr);
	return out;
}

void BackendDataBase::setPoseValue(const BackendPoseValue& value, BackendPoseReferenceFrame frame, const BackendDataManager* mgr)
{
	BackendVec3 worldPose{};
	BackendVec3 worldEuler{};
	buildWorldPoseInFrame(*this, value.position, value.eulerDeg, frame, mgr, worldPose, worldEuler);
	if (hasPoseProperty())
	{
		setPose(worldPose);
	}
	if (hasRotationProperty())
	{
		setRotation(worldEuler);
	}
	std::unique_lock<std::shared_mutex> lock(m_worldMatrixMutex);
	m_worldMatrixDirty = true;
}

BackendMat4 BackendDataBase::worldMatrix(const BackendDataManager* mgr) const
{
	(void)mgr;
	{
		std::shared_lock<std::shared_mutex> lock(m_worldMatrixMutex);
		if (!m_worldMatrixDirty)
		{
			return m_worldMatrixCache;
		}
	}
	const BackendMat4 computed = worldMatrixFromData(*this);
	{
		std::unique_lock<std::shared_mutex> lock(m_worldMatrixMutex);
		m_worldMatrixCache = computed;
		m_worldMatrixDirty = false;
		return m_worldMatrixCache;
	}
}

void BackendDataBase::setWorldMatrix(const BackendMat4& world, const BackendDataManager* mgr)
{
	(void)mgr;
	const BackendVec3 center = modelCenterForData(*this);
	BackendVec3 poseWorld{};
	BackendVec3 rotWorld{};
	backend_pose_euler_from_world_mat(world, center, poseWorld, rotWorld);
	if (hasPoseProperty())
	{
		setPose(poseWorld);
	}
	if (hasRotationProperty())
	{
		setRotation(rotWorld);
	}
	std::unique_lock<std::shared_mutex> lock(m_worldMatrixMutex);
	m_worldMatrixCache = world;
	m_worldMatrixDirty = false;
}

bool BackendDataBase::validatePoseFrameRoundTrip(const BackendDataManager* mgr, double epsilon) const
{
	if (!hasPoseProperty() || !hasRotationProperty())
	{
		return true;
	}
	const BackendPoseValue world0 = poseValue(BackendPoseReferenceFrame::World, mgr);
	const BackendPoseValue local = poseValue(BackendPoseReferenceFrame::Parent, mgr);
	BackendVec3 worldPoseRebuilt{};
	BackendVec3 worldEulerRebuilt{};
	buildWorldPoseInFrame(*this, local.position, local.eulerDeg, BackendPoseReferenceFrame::Parent, mgr, worldPoseRebuilt,
		worldEulerRebuilt);
	return maxAbsVec3Diff(world0.position, worldPoseRebuilt) <= epsilon
		&& maxAbsVec3Diff(world0.eulerDeg, worldEulerRebuilt) <= epsilon;
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
	if (hasPoseProperty())
	{
		const std::string frameText =
			(m_poseReferenceFrame == BackendPoseReferenceFrame::Parent) ? "parent" : "world";
		property_rows_compat::syncTransformColorToBag(m_propertyBag, *this);
		backend_property_json::appendRow(rows, "pose.frame", "Pose frame (world|parent)", true, frameText);
	}
	if (const auto f = std::dynamic_pointer_cast<FollowAttachmentComponent>(getComponent(FollowAttachmentComponent::typeKeyStatic())))
	{
		if (mgr)
		{
			if (const auto target = mgr->getData(f->targetBackendId()))
			{
				m_propertyBag.set<std::string>("follow.targetName", target->name());
			}
		}
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
	if (key == "pose.frame")
	{
		const std::string frame = toLowerAscii(trimUtf8Whitespace(value));
		if (frame == "world")
		{
			m_poseReferenceFrame = BackendPoseReferenceFrame::World;
			m_propertyBag.set<std::string>("pose.frame", "world");
			return true;
		}
		if (frame == "parent")
		{
			m_poseReferenceFrame = BackendPoseReferenceFrame::Parent;
			m_propertyBag.set<std::string>("pose.frame", "parent");
			return true;
		}
		if (errMsg)
		{
			*errMsg = "pose.frame only supports 'world' or 'parent'.";
		}
		return false;
	}
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
			m_propertyBag.set<std::string>("follow.targetName", std::string());
			return true;
		}
		m_propertyBag.set<std::string>("follow.targetName", trimmed);
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
	m_componentsByType[std::type_index(typeid(*component))] = component;
	return true;
}

bool BackendDataBase::removeComponent(const std::string& componentType)
{
	if (componentType.empty())
	{
		return false;
	}
	std::lock_guard<std::mutex> lock(m_componentMutex);
	const auto it = m_components.find(componentType);
	if (it == m_components.end())
	{
		return false;
	}
	if (it->second)
	{
		m_componentsByType.erase(std::type_index(typeid(*it->second)));
	}
	m_components.erase(it);
	return true;
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
	components.reserve(m_componentsByType.size());
	for (const auto& item : m_componentsByType)
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

void BackendDataBase::saveDerivedJson(nlohmann::json& out) const
{
	(void)out;
}

bool BackendDataBase::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	(void)in;
	(void)errMsg;
	return true;
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

