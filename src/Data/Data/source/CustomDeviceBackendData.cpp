/// @file CustomDeviceBackendData.cpp
/// @brief CustomDeviceBackendData 实现

#include "CustomDeviceBackendData.h"

#include "BackendObjectAttribute.h"
#include "BackendTypeIdentity.h"

#include <algorithm>
#include <cmath>

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
		item["enabled"] = a.enabled;
		item["displayName"] = a.displayName;
		item["jointName"] = a.jointName;
		item["motionType"] = motionToString(a.motionType);
		item["lower"] = a.lower;
		item["upper"] = a.upper;
		item["home"] = a.home;
		writeVec3(item["axis"], a.axis);
		writeVec3(item["originMm"], a.originMm);
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
		if (!item.is_object())
		{
			continue;
		}
		CustomDeviceAxisConfig cfg = makeDefaultCustomDeviceTranslateAxis();
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
		normalizeCustomDeviceAxisConfig(cfg);
		out.axes.push_back(std::move(cfg));
	}
	return true;
}

CustomDeviceBackendData::CustomDeviceBackendData()
{
	setName(backend_type::kCatalogCustomDevice);
	m_attributes.push_back(makeBackendPoseAttribute());
	m_attributes.push_back(makeBackendRotationAttribute());
	m_baseWorldW0 = BackendMat4::identity();
	m_baseWorldW0Valid = true;
}

std::string CustomDeviceBackendData::className() const
{
	return backend_type::kClassCustomDevice;
}

bool CustomDeviceBackendData::hasGeometry() const
{
	return true;
}

BackendBoundingBox CustomDeviceBackendData::geometryBounds() const
{
	const double half = static_cast<double>(m_axisLengthMm);
	BackendBoundingBox box{};
	box.min = {-half, -half, -half};
	box.max = {half, half, half};
	box.valid = true;
	return box;
}

std::size_t CustomDeviceBackendData::geometryElementCount() const
{
	return 1U;
}

void CustomDeviceBackendData::clearGeometry()
{
}

void CustomDeviceBackendData::setAxisLengthMm(const float mm)
{
	if (mm > 1.0f)
	{
		m_axisLengthMm = mm;
	}
}

void CustomDeviceBackendData::setAxes(const CustomDeviceAxisConfigSet& axes)
{
	m_axes = axes;
	for (CustomDeviceAxisConfig& a : m_axes.axes)
	{
		normalizeCustomDeviceAxisConfig(a);
	}
	ensureQSize();
}

void CustomDeviceBackendData::setQValues(const std::vector<double>& q)
{
	m_q = q;
	ensureQSize();
	for (size_t i = 0; i < m_axes.axes.size() && i < m_q.size(); ++i)
	{
		m_q[i] = std::clamp(m_q[i], m_axes.axes[i].lower, m_axes.axes[i].upper);
	}
}

void CustomDeviceBackendData::ensureQSize()
{
	if (m_q.size() < m_axes.axes.size())
	{
		const size_t old = m_q.size();
		m_q.resize(m_axes.axes.size(), 0.0);
		for (size_t i = old; i < m_axes.axes.size(); ++i)
		{
			m_q[i] = m_axes.axes[i].home;
		}
	}
	else if (m_q.size() > m_axes.axes.size())
	{
		m_q.resize(m_axes.axes.size());
	}
}

void CustomDeviceBackendData::setBaseWorldW0(const BackendMat4& w0)
{
	m_baseWorldW0 = w0;
	m_baseWorldW0Valid = true;
}

void CustomDeviceBackendData::captureBaseWorldW0FromCurrentWorld()
{
	m_baseWorldW0 = worldMatrix(nullptr);
	m_baseWorldW0Valid = true;
}

void CustomDeviceBackendData::saveDerivedJson(nlohmann::json& out) const
{
	out["geometry"] = nlohmann::json{{"kind", "customDevice"}};
	out["axisLengthMm"] = m_axisLengthMm;
	nlohmann::json axesJson;
	writeCustomDeviceAxisConfigSetToJson(m_axes, axesJson);
	out["deviceAxes"] = std::move(axesJson);
	out["q"] = m_q;
	nlohmann::json w0 = nlohmann::json::array();
	for (int i = 0; i < 16; ++i)
	{
		w0.push_back(m_baseWorldW0.v[i]);
	}
	out["baseWorldW0"] = std::move(w0);
	out["baseWorldW0Valid"] = m_baseWorldW0Valid;
}

bool CustomDeviceBackendData::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	(void)errMsg;
	if (in.contains("axisLengthMm") && in["axisLengthMm"].is_number())
	{
		setAxisLengthMm(static_cast<float>(in["axisLengthMm"].get<double>()));
	}
	if (in.contains("deviceAxes"))
	{
		CustomDeviceAxisConfigSet set;
		if (readCustomDeviceAxisConfigSetFromJson(in["deviceAxes"], set))
		{
			setAxes(set);
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
		BackendMat4 w0 = BackendMat4::identity();
		bool ok = true;
		for (int i = 0; i < 16; ++i)
		{
			if (!in["baseWorldW0"][i].is_number())
			{
				ok = false;
				break;
			}
			w0.v[i] = in["baseWorldW0"][i].get<double>();
		}
		if (ok)
		{
			setBaseWorldW0(w0);
		}
	}
	if (in.contains("baseWorldW0Valid") && in["baseWorldW0Valid"].is_boolean())
	{
		m_baseWorldW0Valid = in["baseWorldW0Valid"].get<bool>();
	}
	ensureQSize();
	return true;
}
