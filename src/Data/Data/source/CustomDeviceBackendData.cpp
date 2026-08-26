/// @file CustomDeviceBackendData.cpp
/// @brief 自定义设备后端

#include "CustomDeviceBackendData.h"

#include "../../PropertyCore/inc/PropertyAttribute.h"
#include "BackendObjectAttribute.h"
#include "BackendTypeIdentity.h"
#include "CustomDeviceRobotMountComponent.h"
#include "RunLogger.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

namespace
{
void writeVec3(nlohmann::json& out, const double v[3])
{
	out = nlohmann::json::array({v[0], v[1], v[2]});
}

bool readVec3(const nlohmann::json& in, double out[3])
{
	if (!in.is_array() || in.size() < 3)
	{
		return false;
	}
	for (int i = 0; i < 3; ++i)
	{
		if (!in[i].is_number())
		{
			return false;
		}
		out[i] = in[i].get<double>();
	}
	return true;
}

void writeMat16(nlohmann::json& out, const double m[16])
{
	out = nlohmann::json::array();
	for (int i = 0; i < 16; ++i)
	{
		out.push_back(m[i]);
	}
}

bool readMat16(const nlohmann::json& in, double out[16])
{
	if (!in.is_array() || in.size() < 16)
	{
		return false;
	}
	for (int i = 0; i < 16; ++i)
	{
		if (!in[i].is_number())
		{
			return false;
		}
		out[i] = in[i].get<double>();
	}
	return true;
}

void setIdentity16(double m[16])
{
	std::memset(m, 0, sizeof(double) * 16);
	m[0] = m[5] = m[10] = m[15] = 1.0;
}

/// 读取并校验刚体（与 worldMatrix 的 fail-fast 哲学一致：脏矩阵告警后回退 identity）
void readRigidMat16(const nlohmann::json& in, double out[16], const char* label)
{
	if (!readMat16(in, out))
	{
		RunLogger::warn(std::string("[CustomDevice] ") + label + ": invalid 16-element matrix, keep identity.");
		setIdentity16(out);
		return;
	}
	BackendMat4 m{};
	for (int i = 0; i < 16; ++i)
	{
		m.v[i] = out[i];
	}
	if (!backend_mat4_is_nearly_rigid(m))
	{
		RunLogger::warn(std::string("[CustomDevice] ") + label + ": matrix not nearly rigid, keep identity.");
		setIdentity16(out);
	}
}

const char* motionToString(const CustomDeviceMotionType t)
{
	return t == CustomDeviceMotionType::Rotate ? "Rotate" : "Translate";
}

CustomDeviceMotionType motionFromString(const std::string& s)
{
	if (s == "Rotate" || s == "rotate")
	{
		return CustomDeviceMotionType::Rotate;
	}
	if (s != "Translate" && s != "translate")
	{
		RunLogger::warn("[CustomDevice] motionFromString: unknown \"" + s + "\", fallback to Translate.");
	}
	return CustomDeviceMotionType::Translate;
}

void normalizeAxis3(double a[3])
{
	const double n = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	if (n < 1e-12)
	{
		a[0] = 1.0;
		a[1] = 0.0;
		a[2] = 0.0;
		return;
	}
	a[0] /= n;
	a[1] /= n;
	a[2] /= n;
}

void writeAxisConfigObject(const CustomDeviceAxisConfig& a, nlohmann::json& item)
{
	item["enabled"] = a.enabled;
	item["displayName"] = a.displayName;
	item["jointName"] = a.jointName;
	item["motionType"] = motionToString(a.motionType);
	item["lower"] = a.lower;
	item["upper"] = a.upper;
	item["home"] = a.home;
	writeVec3(item["axis"], a.axis);
	writeVec3(item["originMm"], a.originMm);
	if (!a.motionCenterFrameBackendId.empty())
	{
		item["motionCenterFrameBackendId"] = a.motionCenterFrameBackendId;
	}
}

bool readAxisConfigObject(const nlohmann::json& item, CustomDeviceAxisConfig& cfg)
{
	if (!item.is_object())
	{
		return false;
	}
	cfg = makeDefaultCustomDeviceTranslateAxis();
	if (item.contains("enabled") && item["enabled"].is_boolean())
	{
		cfg.enabled = item["enabled"].get<bool>();
	}
	if (item.contains("displayName") && item["displayName"].is_string())
	{
		cfg.displayName = item["displayName"].get<std::string>();
	}
	if (item.contains("jointName") && item["jointName"].is_string())
	{
		cfg.jointName = item["jointName"].get<std::string>();
	}
	if (item.contains("motionType") && item["motionType"].is_string())
	{
		cfg.motionType = motionFromString(item["motionType"].get<std::string>());
	}
	if (item.contains("lower") && item["lower"].is_number())
	{
		cfg.lower = item["lower"].get<double>();
	}
	if (item.contains("upper") && item["upper"].is_number())
	{
		cfg.upper = item["upper"].get<double>();
	}
	if (item.contains("home") && item["home"].is_number())
	{
		cfg.home = item["home"].get<double>();
	}
	if (item.contains("axis"))
	{
		readVec3(item["axis"], cfg.axis);
	}
	if (item.contains("originMm"))
	{
		readVec3(item["originMm"], cfg.originMm);
	}
	if (item.contains("motionCenterFrameBackendId") && item["motionCenterFrameBackendId"].is_string())
	{
		cfg.motionCenterFrameBackendId = item["motionCenterFrameBackendId"].get<std::string>();
	}
	normalizeCustomDeviceAxisConfig(cfg);
	return true;
}
} // namespace

CustomDeviceAxisConfig makeDefaultCustomDeviceTranslateAxis()
{
	CustomDeviceAxisConfig cfg;
	cfg.displayName = "Translate";
	cfg.jointName = "device_translate";
	cfg.motionType = CustomDeviceMotionType::Translate;
	cfg.lower = 0.0;
	cfg.upper = 1000.0;
	cfg.home = 0.0;
	cfg.axis[0] = 1.0;
	cfg.axis[1] = 0.0;
	cfg.axis[2] = 0.0;
	return cfg;
}

CustomDeviceAxisConfig makeDefaultCustomDeviceRotateAxis()
{
	CustomDeviceAxisConfig cfg;
	cfg.displayName = "Rotate";
	cfg.jointName = "device_rotate";
	cfg.motionType = CustomDeviceMotionType::Rotate;
	cfg.lower = -3.14159265358979323846;
	cfg.upper = 3.14159265358979323846;
	cfg.home = 0.0;
	cfg.axis[0] = 0.0;
	cfg.axis[1] = 0.0;
	cfg.axis[2] = 1.0;
	return cfg;
}

void normalizeCustomDeviceAxisConfig(CustomDeviceAxisConfig& cfg)
{
	normalizeAxis3(cfg.axis);
	if (cfg.upper < cfg.lower)
	{
		std::swap(cfg.lower, cfg.upper);
	}
	cfg.home = std::clamp(cfg.home, cfg.lower, cfg.upper);
	if (cfg.displayName.empty())
	{
		cfg.displayName = cfg.motionType == CustomDeviceMotionType::Rotate ? "Rotate" : "Translate";
	}
	if (cfg.jointName.empty())
	{
		cfg.jointName = "device_joint";
	}
}

void writeCustomDeviceAxisConfigSetToJson(const CustomDeviceAxisConfigSet& set, nlohmann::json& out)
{
	out = nlohmann::json::object();
	nlohmann::json arr = nlohmann::json::array();
	for (const CustomDeviceAxisConfig& a : set.axes)
	{
		nlohmann::json item = nlohmann::json::object();
		writeAxisConfigObject(a, item);
		arr.push_back(std::move(item));
	}
	out["axes"] = std::move(arr);
}

bool readCustomDeviceAxisConfigSetFromJson(const nlohmann::json& in, CustomDeviceAxisConfigSet& out)
{
	out = CustomDeviceAxisConfigSet{};
	if (!in.is_object())
	{
		return false;
	}
	if (!in.contains("axes") || !in["axes"].is_array())
	{
		return true;
	}
	for (const auto& item : in["axes"])
	{
		CustomDeviceAxisConfig cfg;
		if (readAxisConfigObject(item, cfg))
		{
			out.axes.push_back(std::move(cfg));
		}
	}
	return true;
}

void writeCustomDeviceLinksToJson(const std::vector<CustomDeviceLink>& links, nlohmann::json& out)
{
	out = nlohmann::json::array();
	for (const CustomDeviceLink& L : links)
	{
		nlohmann::json item = nlohmann::json::object();
		item["id"] = L.id;
		item["displayName"] = L.displayName;
		item["geometryBackendId"] = L.geometryBackendId;
		item["fixed"] = L.fixed;
		item["canvasX"] = L.canvasX;
		item["canvasY"] = L.canvasY;
		writeMat16(item["restInDeviceW0"], L.restInDeviceW0);
		out.push_back(std::move(item));
	}
}

bool readCustomDeviceLinksFromJson(const nlohmann::json& in, std::vector<CustomDeviceLink>& out)
{
	out.clear();
	if (!in.is_array())
	{
		return false;
	}
	for (const auto& item : in)
	{
		if (!item.is_object())
		{
			continue;
		}
		CustomDeviceLink L;
		setIdentity16(L.restInDeviceW0);
		if (item.contains("id") && item["id"].is_string())
		{
			L.id = item["id"].get<std::string>();
		}
		if (item.contains("displayName") && item["displayName"].is_string())
		{
			L.displayName = item["displayName"].get<std::string>();
		}
		if (item.contains("geometryBackendId") && item["geometryBackendId"].is_string())
		{
			L.geometryBackendId = item["geometryBackendId"].get<std::string>();
		}
		if (item.contains("fixed") && item["fixed"].is_boolean())
		{
			L.fixed = item["fixed"].get<bool>();
		}
		if (item.contains("canvasX") && item["canvasX"].is_number())
		{
			L.canvasX = item["canvasX"].get<double>();
		}
		if (item.contains("canvasY") && item["canvasY"].is_number())
		{
			L.canvasY = item["canvasY"].get<double>();
		}
		if (item.contains("restInDeviceW0"))
		{
			readRigidMat16(item["restInDeviceW0"], L.restInDeviceW0, "link.restInDeviceW0");
		}
		if (L.id.empty())
		{
			continue;
		}
		out.push_back(std::move(L));
	}
	return true;
}

void writeCustomDeviceJointsToJson(const std::vector<CustomDeviceJoint>& joints, nlohmann::json& out)
{
	out = nlohmann::json::array();
	for (const CustomDeviceJoint& J : joints)
	{
		nlohmann::json item = nlohmann::json::object();
		item["id"] = J.id;
		item["parentLinkId"] = J.parentLinkId;
		item["childLinkId"] = J.childLinkId;
		nlohmann::json motion = nlohmann::json::object();
		writeAxisConfigObject(J.motion, motion);
		item["motion"] = std::move(motion);
		writeMat16(item["parentToChildRest"], J.parentToChildRest);
		out.push_back(std::move(item));
	}
}

bool readCustomDeviceJointsFromJson(const nlohmann::json& in, std::vector<CustomDeviceJoint>& out)
{
	out.clear();
	if (!in.is_array())
	{
		return false;
	}
	for (const auto& item : in)
	{
		if (!item.is_object())
		{
			continue;
		}
		CustomDeviceJoint J;
		setIdentity16(J.parentToChildRest);
		if (item.contains("id") && item["id"].is_string())
		{
			J.id = item["id"].get<std::string>();
		}
		if (item.contains("parentLinkId") && item["parentLinkId"].is_string())
		{
			J.parentLinkId = item["parentLinkId"].get<std::string>();
		}
		if (item.contains("childLinkId") && item["childLinkId"].is_string())
		{
			J.childLinkId = item["childLinkId"].get<std::string>();
		}
		if (item.contains("motion"))
		{
			readAxisConfigObject(item["motion"], J.motion);
		}
		if (item.contains("parentToChildRest"))
		{
			readRigidMat16(item["parentToChildRest"], J.parentToChildRest, "joint.parentToChildRest");
		}
		if (J.id.empty() || J.parentLinkId.empty() || J.childLinkId.empty())
		{
			continue;
		}
		out.push_back(std::move(J));
	}
	return true;
}

namespace
{
std::atomic<unsigned long long> g_customDevicePoseIdCounter{1ULL};
std::atomic<unsigned long long> g_customDevicePoseBindingIdCounter{1ULL};

/// 加载期回推计数器，避免生成 id 与已加载 id 碰撞（同 BackendDataBase::setId 哲学）
void advanceIdCounterFromLoaded(std::atomic<unsigned long long>& counter, const std::string& id, const char* prefix)
{
	const std::size_t prefixLen = std::strlen(prefix);
	if (id.size() <= prefixLen || id.compare(0, prefixLen, prefix) != 0)
	{
		return;
	}
	try
	{
		const unsigned long long parsed = std::stoull(id.substr(prefixLen));
		unsigned long long expected = counter.load(std::memory_order_relaxed);
		while (parsed >= expected &&
			   !counter.compare_exchange_weak(expected, parsed + 1ULL, std::memory_order_relaxed,
											  std::memory_order_relaxed))
		{
		}
	}
	catch (...)
	{
	}
}
} // namespace

std::string makeCustomDevicePoseId()
{
	return std::string("POSE_") + std::to_string(g_customDevicePoseIdCounter.fetch_add(1ULL));
}

std::string makeCustomDevicePoseBindingId()
{
	return std::string("PSB_") + std::to_string(g_customDevicePoseBindingIdCounter.fetch_add(1ULL));
}

void writeCustomDeviceNamedPosesToJson(const std::vector<CustomDeviceNamedPose>& poses, nlohmann::json& out)
{
	out = nlohmann::json::array();
	for (const CustomDeviceNamedPose& p : poses)
	{
		nlohmann::json item = nlohmann::json::object();
		item["id"] = p.id;
		item["name"] = p.name;
		item["q"] = p.q;
		out.push_back(std::move(item));
	}
}

bool readCustomDeviceNamedPosesFromJson(const nlohmann::json& in, std::vector<CustomDeviceNamedPose>& out)
{
	out.clear();
	if (!in.is_array())
	{
		return false;
	}
	for (const auto& item : in)
	{
		if (!item.is_object())
		{
			continue;
		}
		CustomDeviceNamedPose p;
		if (item.contains("id") && item["id"].is_string())
		{
			p.id = item["id"].get<std::string>();
		}
		if (item.contains("name") && item["name"].is_string())
		{
			p.name = item["name"].get<std::string>();
		}
		if (item.contains("q") && item["q"].is_array())
		{
			for (const auto& v : item["q"])
			{
				if (v.is_number())
				{
					p.q.push_back(v.get<double>());
				}
			}
		}
		if (p.id.empty())
		{
			p.id = makeCustomDevicePoseId();
		}
		else
		{
			advanceIdCounterFromLoaded(g_customDevicePoseIdCounter, p.id, "POSE_");
		}
		if (p.name.empty())
		{
			p.name = p.id;
		}
		out.push_back(std::move(p));
	}
	return true;
}

void writeCustomDevicePoseSignalBindingsToJson(const std::vector<CustomDevicePoseSignalBinding>& bindings,
											  nlohmann::json& out)
{
	out = nlohmann::json::array();
	for (const CustomDevicePoseSignalBinding& b : bindings)
	{
		nlohmann::json item = nlohmann::json::object();
		item["id"] = b.id;
		item["signalName"] = b.signalName;
		item["poseId"] = b.poseId;
		item["durationSec"] = b.durationSec;
		item["enabled"] = b.enabled;
		out.push_back(std::move(item));
	}
}

bool readCustomDevicePoseSignalBindingsFromJson(const nlohmann::json& in,
											   std::vector<CustomDevicePoseSignalBinding>& out)
{
	out.clear();
	if (!in.is_array())
	{
		return false;
	}
	for (const auto& item : in)
	{
		if (!item.is_object())
		{
			continue;
		}
		CustomDevicePoseSignalBinding b;
		if (item.contains("id") && item["id"].is_string())
		{
			b.id = item["id"].get<std::string>();
		}
		if (item.contains("signalName") && item["signalName"].is_string())
		{
			b.signalName = item["signalName"].get<std::string>();
		}
		if (item.contains("poseId") && item["poseId"].is_string())
		{
			b.poseId = item["poseId"].get<std::string>();
		}
		if (item.contains("durationSec") && item["durationSec"].is_number())
		{
			b.durationSec = item["durationSec"].get<double>();
		}
		if (item.contains("enabled") && item["enabled"].is_boolean())
		{
			b.enabled = item["enabled"].get<bool>();
		}
		if (b.id.empty())
		{
			b.id = makeCustomDevicePoseBindingId();
		}
		else
		{
			advanceIdCounterFromLoaded(g_customDevicePoseBindingIdCounter, b.id, "PSB_");
		}
		out.push_back(std::move(b));
	}
	return true;
}

CustomDeviceBackendData::CustomDeviceBackendData()
{
	setName(backend_type::kCatalogCustomDevice);
	appendStandardAttributesForCapabilities(*this, m_attributes);
	m_baseWorldW0 = BackendMat4::identity();
	m_baseWorldW0Valid = true;
}

std::string CustomDeviceBackendData::className() const
{
	return backend_type::kClassCustomDevice;
}

bool CustomDeviceBackendData::isPoseExternallyDriven() const
{
	const auto mount = std::dynamic_pointer_cast<CustomDeviceRobotMountComponent>(
		getComponent(CustomDeviceRobotMountComponent::typeKeyStatic()));
	return mount && mount->enabled();
}

bool CustomDeviceBackendData::hasGeometry() const
{
	// 设备根自身无几何：真实几何在 Link 引用的 geometryBackendId 上；此处仅示意轴（visual 自画）
	return false;
}

BackendBoundingBox CustomDeviceBackendData::geometryBounds() const
{
	// 谎报 ±axisLengthMm 立方会污染 fit-to-view/场景统计；报无效，由子件几何承担包络
	BackendBoundingBox box{};
	box.valid = false;
	return box;
}

std::size_t CustomDeviceBackendData::geometryElementCount() const
{
	return 0U;
}

void CustomDeviceBackendData::clearGeometry()
{
}

void CustomDeviceBackendData::collectReferencedBackendIds(std::vector<std::string>& out) const
{
	BackendDataBase::collectReferencedBackendIds(out);
	for (const CustomDeviceLink& L : m_links)
	{
		if (!L.geometryBackendId.empty())
		{
			out.push_back(L.geometryBackendId);
		}
	}
	for (const CustomDeviceAxisConfig& a : m_kinematic.axes.axes)
	{
		if (!a.motionCenterFrameBackendId.empty())
		{
			out.push_back(a.motionCenterFrameBackendId);
		}
	}
}

void CustomDeviceKinematicState::setAxes(const CustomDeviceAxisConfigSet& nextAxes)
{
	axes = nextAxes;
	for (CustomDeviceAxisConfig& a : axes.axes)
	{
		normalizeCustomDeviceAxisConfig(a);
	}
	ensureQSize();
}

void CustomDeviceKinematicState::setQValues(const std::vector<double>& nextQ)
{
	q = nextQ;
	ensureQSize();
	for (size_t i = 0; i < axes.axes.size() && i < q.size(); ++i)
	{
		q[i] = std::clamp(q[i], axes.axes[i].lower, axes.axes[i].upper);
	}
}

void CustomDeviceKinematicState::ensureQSize()
{
	if (q.size() < axes.axes.size())
	{
		const size_t old = q.size();
		q.resize(axes.axes.size(), 0.0);
		for (size_t i = old; i < axes.axes.size(); ++i)
		{
			q[i] = axes.axes[i].home;
		}
	}
	else if (q.size() > axes.axes.size())
	{
		q.resize(axes.axes.size());
	}
}

void CustomDeviceKinematicState::syncAxesFromJoints(const std::vector<CustomDeviceJoint>& joints)
{
	if (joints.empty())
	{
		return;
	}
	CustomDeviceAxisConfigSet set;
	set.axes.reserve(joints.size());
	for (const CustomDeviceJoint& j : joints)
	{
		CustomDeviceAxisConfig a = j.motion;
		if (a.displayName.empty())
		{
			a.displayName = j.id;
		}
		if (a.jointName.empty())
		{
			a.jointName = j.id;
		}
		normalizeCustomDeviceAxisConfig(a);
		set.axes.push_back(std::move(a));
	}
	setAxes(set);
}

void CustomDeviceBackendData::setAxisLengthMm(const float mm)
{
	if (mm > 0.0f)
	{
		m_axisLengthMm = mm;
		bumpGeometryRevision();
		return;
	}
	RunLogger::warn("[CustomDeviceBackendData] setAxisLengthMm: ignore non-positive value.");
}

void CustomDeviceBackendData::setAxes(const CustomDeviceAxisConfigSet& axes)
{
	m_kinematic.setAxes(axes);
}

void CustomDeviceBackendData::setLinks(const std::vector<CustomDeviceLink>& links)
{
	m_links = links;
}

void CustomDeviceBackendData::setJoints(const std::vector<CustomDeviceJoint>& joints)
{
	m_joints = joints;
	for (CustomDeviceJoint& j : m_joints)
	{
		normalizeCustomDeviceAxisConfig(j.motion);
	}
	syncAxesFromJoints();
}

void CustomDeviceBackendData::syncAxesFromJoints()
{
	m_kinematic.syncAxesFromJoints(m_joints);
}

void CustomDeviceBackendData::setQValues(const std::vector<double>& q)
{
	m_kinematic.setQValues(q);
}

void CustomDeviceBackendData::ensureQSize()
{
	m_kinematic.ensureQSize();
}

void CustomDeviceBackendData::setBaseWorldW0(const BackendMat4& w0)
{
	m_baseWorldW0 = w0;
	m_baseWorldW0Valid = true;
}

void CustomDeviceBackendData::captureBaseWorldW0FromCurrentWorld()
{
	m_baseWorldW0 = worldMatrix();
	m_baseWorldW0Valid = true;
}

void CustomDeviceBackendData::setNamedPoses(const std::vector<CustomDeviceNamedPose>& poses)
{
	m_namedPoses = poses;
}

const CustomDeviceNamedPose* CustomDeviceBackendData::findNamedPose(const std::string& poseId) const
{
	if (poseId.empty())
	{
		return nullptr;
	}
	for (const CustomDeviceNamedPose& p : m_namedPoses)
	{
		if (p.id == poseId)
		{
			return &p;
		}
	}
	return nullptr;
}

void CustomDeviceBackendData::setPoseSignalBindings(const std::vector<CustomDevicePoseSignalBinding>& bindings)
{
	m_poseSignalBindings = bindings;
}

void CustomDeviceBackendData::setIoSignalsJson(nlohmann::json signalsJson)
{
	m_ioSignalsJson = std::move(signalsJson);
}

nlohmann::json CustomDeviceBackendData::snapshotPropertyRows(const BackendDataManager* mgr) const
{
	nlohmann::json rows = BackendDataBase::snapshotPropertyRows(mgr);
	property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::appendRows(m_attributes, *this, rows);
	return rows;
}

bool CustomDeviceBackendData::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
												  const BackendDataManager* mgr)
{
	if (property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::apply(m_attributes, *this, key, value,
																					  errMsg))
	{
		return true;
	}
	return BackendDataBase::applyPropertyChange(key, value, errMsg, mgr);
}

void CustomDeviceBackendData::saveDerivedJson(nlohmann::json& out) const
{
	out["geometry"] = nlohmann::json{{"kind", "customDevice"}};
	out["axisLengthMm"] = m_axisLengthMm;
	nlohmann::json linksJson;
	writeCustomDeviceLinksToJson(m_links, linksJson);
	out["links"] = std::move(linksJson);
	nlohmann::json jointsJson;
	writeCustomDeviceJointsToJson(m_joints, jointsJson);
	out["joints"] = std::move(jointsJson);
	out["q"] = m_kinematic.q;
	nlohmann::json w0 = nlohmann::json::array();
	for (int i = 0; i < 16; ++i)
	{
		w0.push_back(m_baseWorldW0.v[i]);
	}
	out["baseWorldW0"] = std::move(w0);
	out["baseWorldW0Valid"] = m_baseWorldW0Valid;
	nlohmann::json posesJson;
	writeCustomDeviceNamedPosesToJson(m_namedPoses, posesJson);
	out["namedPoses"] = std::move(posesJson);
	nlohmann::json bindingsJson;
	writeCustomDevicePoseSignalBindingsToJson(m_poseSignalBindings, bindingsJson);
	out["poseSignalBindings"] = std::move(bindingsJson);
	out["signals"] = m_ioSignalsJson;
}

bool CustomDeviceBackendData::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	// 逐段失败逐段告警，不再整体吞错；错误不阻断其余段落加载
	std::string errs;
	auto noteErr = [&](const char* section)
	{
		if (!errs.empty())
		{
			errs += "; ";
		}
		errs += section;
	};
	if (in.contains("axisLengthMm") && in["axisLengthMm"].is_number())
	{
		setAxisLengthMm(static_cast<float>(in["axisLengthMm"].get<double>()));
	}
	if (in.contains("links"))
	{
		std::vector<CustomDeviceLink> links;
		if (readCustomDeviceLinksFromJson(in["links"], links))
		{
			setLinks(links);
		}
		else
		{
			noteErr("links");
		}
	}
	if (in.contains("joints"))
	{
		std::vector<CustomDeviceJoint> joints;
		if (readCustomDeviceJointsFromJson(in["joints"], joints))
		{
			setJoints(joints);
		}
		else
		{
			noteErr("joints");
		}
	}
	if (in.contains("q") && in["q"].is_array())
	{
		std::vector<double> q;
		for (const auto& item : in["q"])
		{
			if (item.is_number())
			{
				q.push_back(item.get<double>());
			}
		}
		setQValues(q);
	}
	if (in.contains("baseWorldW0") && in["baseWorldW0"].is_array() && in["baseWorldW0"].size() >= 16)
	{
		double w0raw[16];
		setIdentity16(w0raw);
		readRigidMat16(in["baseWorldW0"], w0raw, "baseWorldW0");
		BackendMat4 w0 = BackendMat4::identity();
		for (int i = 0; i < 16; ++i)
		{
			w0.v[i] = w0raw[i];
		}
		setBaseWorldW0(w0);
	}
	if (in.contains("baseWorldW0Valid") && in["baseWorldW0Valid"].is_boolean())
	{
		m_baseWorldW0Valid = in["baseWorldW0Valid"].get<bool>();
	}
	if (in.contains("namedPoses"))
	{
		std::vector<CustomDeviceNamedPose> poses;
		if (readCustomDeviceNamedPosesFromJson(in["namedPoses"], poses))
		{
			setNamedPoses(poses);
		}
		else
		{
			noteErr("namedPoses");
		}
	}
	if (in.contains("poseSignalBindings"))
	{
		std::vector<CustomDevicePoseSignalBinding> bindings;
		if (readCustomDevicePoseSignalBindingsFromJson(in["poseSignalBindings"], bindings))
		{
			setPoseSignalBindings(bindings);
		}
		else
		{
			noteErr("poseSignalBindings");
		}
	}
	if (in.contains("signals"))
	{
		setIoSignalsJson(in["signals"]);
	}
	ensureQSize();
	if (!errs.empty() && errMsg)
	{
		*errMsg = "CustomDevice partially loaded, failed sections: " + errs;
	}
	return true;
}
