/// @file RobotExternalAxes.cpp
/// @brief 外部轴配置 JSON、校验与多轴 FK 合成

#include "RobotExternalAxes.h"

#include "JointMotionAdapters.h"
#include "BackendFollowMath.h"

#include "JointMotionEval.h"
#include "Mat4Ops.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace RobotExternal
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

void readVec3(const nlohmann::json& j, double out[3])
{
	if (!j.is_array() || j.size() < 3)
	{
		return;
	}
	out[0] = j[0].get<double>();
	out[1] = j[1].get<double>();
	out[2] = j[2].get<double>();
}

void writeVec3(nlohmann::json& j, const double v[3])
{
	j = nlohmann::json::array({v[0], v[1], v[2]});
}

const char* kindToString(const RobotExternalAxisKind kind)
{
	return kind == RobotExternalAxisKind::Turntable ? "Turntable" : "LinearRail";
}

RobotExternalAxisKind kindFromString(const std::string& s)
{
	if (s == "Turntable" || s == "turntable")
	{
		return RobotExternalAxisKind::Turntable;
	}
	return RobotExternalAxisKind::LinearRail;
}

const char* motionToString(const RobotExternalMotionType t)
{
	return t == RobotExternalMotionType::Rotate ? "Rotate" : "Translate";
}

RobotExternalMotionType motionFromString(const std::string& s)
{
	if (s == "Rotate" || s == "rotate" || s == "Revolute" || s == "revolute")
	{
		return RobotExternalMotionType::Rotate;
	}
	return RobotExternalMotionType::Translate;
}

const char* attachmentToString(const RobotExternalAttachment a)
{
	return a == RobotExternalAttachment::Workpiece ? "Workpiece" : "RobotBase";
}

RobotExternalAttachment attachmentFromString(const std::string& s)
{
	if (s == "Workpiece" || s == "workpiece" || s == "Part" || s == "part")
	{
		return RobotExternalAttachment::Workpiece;
	}
	return RobotExternalAttachment::RobotBase;
}

void mat4Identity(double out[16])
{
	for (int i = 0; i < 16; ++i)
	{
		out[i] = 0.0;
	}
	out[0] = out[5] = out[10] = out[15] = 1.0;
}

void mat4MulColumnMajor(const double a[16], const double b[16], double out[16])
{
	double tmp[16];
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			tmp[c * 4 + r] = a[0 * 4 + r] * b[c * 4 + 0] + a[1 * 4 + r] * b[c * 4 + 1] + a[2 * 4 + r] * b[c * 4 + 2] +
							 a[3 * 4 + r] * b[c * 4 + 3];
		}
	}
	for (int i = 0; i < 16; ++i)
	{
		out[i] = tmp[i];
	}
}

void mat4Copy(const double in[16], double out[16])
{
	for (int i = 0; i < 16; ++i)
	{
		out[i] = in[i];
	}
}

void makeTranslateColumnMajor(const double tx, const double ty, const double tz, double out[16])
{
	mat4Identity(out);
	// 与 BackendMat4/OSG 同序：平移在 index 3/7/11
	out[3] = tx;
	out[7] = ty;
	out[11] = tz;
}

void makeRotateAboutAxisColumnMajor(const double ox, const double oy, const double oz, const double ax, const double ay,
									const double az, const double angleRad, double out[16])
{
	const double c = std::cos(angleRad);
	const double s = std::sin(angleRad);
	const double t = 1.0 - c;
	const double r00 = t * ax * ax + c;
	const double r01 = t * ax * ay - s * az;
	const double r02 = t * ax * az + s * ay;
	const double r10 = t * ax * ay + s * az;
	const double r11 = t * ay * ay + c;
	const double r12 = t * ay * az - s * ax;
	const double r20 = t * ax * az - s * ay;
	const double r21 = t * ay * az + s * ax;
	const double r22 = t * az * az + c;

	double toOrigin[16];
	double rot[16];
	double fromOrigin[16];
	makeTranslateColumnMajor(-ox, -oy, -oz, toOrigin);
	mat4Identity(rot);
	// OSG/Backend 底行平移序：旋转按列主序 (r,c)→c*4+r
	rot[0] = r00;
	rot[1] = r10;
	rot[2] = r20;
	rot[4] = r01;
	rot[5] = r11;
	rot[6] = r21;
	rot[8] = r02;
	rot[9] = r12;
	rot[10] = r22;
	makeTranslateColumnMajor(ox, oy, oz, fromOrigin);
	// 同布局下绕 +origin 的共轭是 T(-o)*R*T(+o)（T(+o)*R*T(-o) 会绕到 -origin）
	double tmp[16];
	mat4MulColumnMajor(toOrigin, rot, tmp);
	mat4MulColumnMajor(tmp, fromOrigin, out);
}

double qForAxisIndex(const RobotExternalAxisConfigSet& set, const std::vector<double>& qValues, const int axisIndex)
{
	if (axisIndex < 0 || axisIndex >= static_cast<int>(set.axes.size()))
	{
		return 0.0;
	}
	if (axisIndex < static_cast<int>(qValues.size()))
	{
		return qValues[static_cast<size_t>(axisIndex)];
	}
	return set.axes[static_cast<size_t>(axisIndex)].home;
}

void composeAxesChain(const double base[16], const RobotExternalAxisConfigSet& set, const std::vector<double>& qValues,
					  const RobotExternalAttachment attachment, const std::string* filterBackendId, double out[16])
{
	mat4Copy(base, out);
	for (int i = 0; i < static_cast<int>(set.axes.size()); ++i)
	{
		const RobotExternalAxisConfig& cfg = set.axes[static_cast<size_t>(i)];
		if (!cfg.enabled || cfg.attachment != attachment)
		{
			continue;
		}
		if (filterBackendId && cfg.boundBackendId != *filterBackendId)
		{
			continue;
		}
		double motion[16];
		makeAxisMotionColumnMajor(cfg, qForAxisIndex(set, qValues, i), motion);
		double next[16];
		mat4MulColumnMajor(out, motion, next);
		mat4Copy(next, out);
	}
}

void unbakeAxesChain(const double eff[16], const RobotExternalAxisConfigSet& set, const std::vector<double>& qValues,
					 const RobotExternalAttachment attachment, const std::string* filterBackendId, double out[16])
{
	mat4Copy(eff, out);
	for (int i = static_cast<int>(set.axes.size()) - 1; i >= 0; --i)
	{
		const RobotExternalAxisConfig& cfg = set.axes[static_cast<size_t>(i)];
		if (!cfg.enabled || cfg.attachment != attachment)
		{
			continue;
		}
		if (filterBackendId && cfg.boundBackendId != *filterBackendId)
		{
			continue;
		}
		const double q = qForAxisIndex(set, qValues, i);
		double motionInv[16];
		if (cfg.motionType == RobotExternalMotionType::Translate)
		{
			makeTranslateColumnMajor(-cfg.axis[0] * q, -cfg.axis[1] * q, -cfg.axis[2] * q, motionInv);
		}
		else
		{
			makeRotateAboutAxisColumnMajor(cfg.originMm[0], cfg.originMm[1], cfg.originMm[2], cfg.axis[0], cfg.axis[1],
										   cfg.axis[2], -q, motionInv);
		}
		double next[16];
		mat4MulColumnMajor(out, motionInv, next);
		mat4Copy(next, out);
	}
}
} // namespace

bool hasEnabledExternalAxes(const RobotExternalAxisConfigSet& set)
{
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		if (a.enabled)
		{
			return true;
		}
	}
	return false;
}

bool hasEnabledWorkpieceExternalAxes(const RobotExternalAxisConfigSet& set)
{
	return !enabledExternalAxesForAttachment(set, RobotExternalAttachment::Workpiece).empty();
}

const RobotExternalAxisConfig* firstEnabledExternalAxis(const RobotExternalAxisConfigSet& set)
{
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		if (a.enabled)
		{
			return &a;
		}
	}
	return nullptr;
}

const RobotExternalAxisConfig* firstEnabledWorkpieceAxis(const RobotExternalAxisConfigSet& set)
{
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		if (a.enabled && a.attachment == RobotExternalAttachment::Workpiece)
		{
			return &a;
		}
	}
	return nullptr;
}

std::vector<const RobotExternalAxisConfig*> enabledExternalAxes(const RobotExternalAxisConfigSet& set)
{
	std::vector<const RobotExternalAxisConfig*> out;
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		if (a.enabled)
		{
			out.push_back(&a);
		}
	}
	return out;
}

std::vector<const RobotExternalAxisConfig*>
enabledExternalAxesForAttachment(const RobotExternalAxisConfigSet& set, const RobotExternalAttachment attachment)
{
	std::vector<const RobotExternalAxisConfig*> out;
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		if (a.enabled && a.attachment == attachment)
		{
			out.push_back(&a);
		}
	}
	return out;
}

std::vector<int> enabledExternalAxisIndices(const RobotExternalAxisConfigSet& set)
{
	std::vector<int> out;
	for (int i = 0; i < static_cast<int>(set.axes.size()); ++i)
	{
		if (set.axes[static_cast<size_t>(i)].enabled)
		{
			out.push_back(i);
		}
	}
	return out;
}

std::vector<int> enabledExternalAxisIndicesForAttachment(const RobotExternalAxisConfigSet& set,
														 const RobotExternalAttachment attachment)
{
	std::vector<int> out;
	for (int i = 0; i < static_cast<int>(set.axes.size()); ++i)
	{
		const RobotExternalAxisConfig& a = set.axes[static_cast<size_t>(i)];
		if (a.enabled && a.attachment == attachment)
		{
			out.push_back(i);
		}
	}
	return out;
}

std::string primaryWorkpieceBackendId(const RobotExternalAxisConfigSet& set)
{
	if (const RobotExternalAxisConfig* a = firstEnabledWorkpieceAxis(set))
	{
		return a->boundBackendId;
	}
	return {};
}

std::string resolveWorkingFrameId(const RobotExternalAxisConfigSet& set)
{
	const RobotExternalAxisConfig* a = firstEnabledWorkpieceAxis(set);
	if (!a)
	{
		return {};
	}
	if (!a->workingFrameId.empty())
	{
		return a->workingFrameId;
	}
	return a->boundBackendId;
}

void mat4IdentityColumnMajor(double out[16])
{
	kinematic_core::mat4IdentityColumnMajor(out);
}

void mat4MulColumnMajor16(const double a[16], const double b[16], double out[16])
{
	kinematic_core::mat4MulColumnMajor16(a, b, out);
}

bool mat4InvertRigidColumnMajor(const double in[16], double out[16])
{
	BackendMat4 m = BackendMat4::identity();
	BackendMat4 inv = BackendMat4::identity();
	for (int i = 0; i < 16; ++i)
	{
		m.v[i] = in[i];
	}
	if (!backend_mat4_invert_rigid(m, inv))
	{
		mat4Identity(out);
		return false;
	}
	for (int i = 0; i < 16; ++i)
	{
		out[i] = inv.v[i];
	}
	return true;
}

void composeWorkpieceWorkingFrameWorld(const double w0ColumnMajor[16], const RobotExternalAxisConfigSet& set,
									   const std::string& boundBackendId, const std::vector<double>& qValues,
									   const double offsetW0Local[16], double outWorkWorld[16])
{
	double wEff[16];
	composeWorkpiecePlacementWithExternalAxis(w0ColumnMajor, set, boundBackendId, qValues, wEff);
	if (!offsetW0Local)
	{
		mat4Copy(wEff, outWorkWorld);
		return;
	}
	mat4MulColumnMajor(wEff, offsetW0Local, outWorkWorld);
}

bool composeWorkpieceWorkingFrameInRobotP0(const double p0WorldColumnMajor[16], const double w0ColumnMajor[16],
										   const RobotExternalAxisConfigSet& set, const std::string& boundBackendId,
										   const std::vector<double>& qValues, const double offsetW0Local[16],
										   double outTp0Work[16])
{
	double workWorld[16];
	composeWorkpieceWorkingFrameWorld(w0ColumnMajor, set, boundBackendId, qValues, offsetW0Local, workWorld);
	double invP0[16];
	if (!mat4InvertRigidColumnMajor(p0WorldColumnMajor, invP0))
	{
		mat4Identity(outTp0Work);
		return false;
	}
	mat4MulColumnMajor(invP0, workWorld, outTp0Work);
	return true;
}

bool validateExternalAxisConfig(const RobotExternalAxisConfig& cfg, std::string* errMsg)
{
	if (cfg.attachment == RobotExternalAttachment::Workpiece && cfg.boundBackendId.empty())
	{
		if (errMsg)
		{
			*errMsg = "Workpiece 外轴必须绑定 boundBackendId";
		}
		return false;
	}
	return true;
}

bool validateExternalAxisConfigSet(const RobotExternalAxisConfigSet& set, std::string* errMsg)
{
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		if (!validateExternalAxisConfig(a, errMsg))
		{
			return false;
		}
	}
	return true;
}

void normalizeExternalAxisConfig(RobotExternalAxisConfig& cfg)
{
	if (cfg.displayName.empty())
	{
		cfg.displayName = cfg.motionType == RobotExternalMotionType::Rotate ? "Turntable" : "Rail";
	}
	if (cfg.jointName.empty())
	{
		cfg.jointName = cfg.motionType == RobotExternalMotionType::Rotate ? "turn_joint" : "rail_joint";
	}
	// 旧 kind 回填 motionType
	if (cfg.kind == RobotExternalAxisKind::Turntable && cfg.motionType == RobotExternalMotionType::Translate &&
		cfg.isPrismatic)
	{
		cfg.motionType = RobotExternalMotionType::Rotate;
	}
	cfg.isPrismatic = (cfg.motionType == RobotExternalMotionType::Translate);
	cfg.kind = cfg.isPrismatic ? RobotExternalAxisKind::LinearRail : RobotExternalAxisKind::Turntable;
	if (cfg.upper < cfg.lower)
	{
		std::swap(cfg.lower, cfg.upper);
	}
	cfg.home = std::clamp(cfg.home, cfg.lower, cfg.upper);
	const double len = std::sqrt(cfg.axis[0] * cfg.axis[0] + cfg.axis[1] * cfg.axis[1] + cfg.axis[2] * cfg.axis[2]);
	if (len < 1e-9)
	{
		cfg.axis[0] = 1.0;
		cfg.axis[1] = 0.0;
		cfg.axis[2] = 0.0;
	}
	else
	{
		cfg.axis[0] /= len;
		cfg.axis[1] /= len;
		cfg.axis[2] /= len;
	}
}

RobotExternalAxisConfig makeDefaultLinearRailConfig()
{
	RobotExternalAxisConfig cfg;
	cfg.motionType = RobotExternalMotionType::Translate;
	cfg.attachment = RobotExternalAttachment::RobotBase;
	normalizeExternalAxisConfig(cfg);
	return cfg;
}

RobotExternalAxisConfig makeDefaultRotateAxisConfig(const RobotExternalAttachment attachment)
{
	RobotExternalAxisConfig cfg;
	cfg.displayName = "Turntable";
	cfg.jointName = "turn_joint";
	cfg.motionType = RobotExternalMotionType::Rotate;
	cfg.attachment = attachment;
	cfg.isPrismatic = false;
	cfg.kind = RobotExternalAxisKind::Turntable;
	cfg.lower = -kPi;
	cfg.upper = kPi;
	cfg.home = 0.0;
	cfg.axis[0] = 0.0;
	cfg.axis[1] = 0.0;
	cfg.axis[2] = 1.0;
	normalizeExternalAxisConfig(cfg);
	return cfg;
}

void writeExternalAxisConfigSetToJson(const RobotExternalAxisConfigSet& set, nlohmann::json& out)
{
	out = nlohmann::json::object();
	nlohmann::json arr = nlohmann::json::array();
	for (const RobotExternalAxisConfig& a : set.axes)
	{
		nlohmann::json item = nlohmann::json::object();
		item["enabled"] = a.enabled;
		item["displayName"] = a.displayName;
		item["jointName"] = a.jointName;
		item["kind"] = kindToString(a.kind);
		item["motionType"] = motionToString(a.motionType);
		item["attachment"] = attachmentToString(a.attachment);
		item["isPrismatic"] = a.isPrismatic;
		item["lower"] = a.lower;
		item["upper"] = a.upper;
		item["home"] = a.home;
		writeVec3(item["axis"], a.axis);
		writeVec3(item["originMm"], a.originMm);
		item["boundBackendId"] = a.boundBackendId;
		item["workingFrameId"] = a.workingFrameId;
		arr.push_back(std::move(item));
	}
	out["axes"] = std::move(arr);
}

bool readExternalAxisConfigSetFromJson(const nlohmann::json& in, RobotExternalAxisConfigSet& out)
{
	out = RobotExternalAxisConfigSet{};
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
		RobotExternalAxisConfig cfg = makeDefaultLinearRailConfig();
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
		if (item.contains("kind") && item["kind"].is_string())
		{
			cfg.kind = kindFromString(item["kind"].get<std::string>());
		}
		if (item.contains("isPrismatic") && item["isPrismatic"].is_boolean())
		{
			cfg.isPrismatic = item["isPrismatic"].get<bool>();
			cfg.motionType = cfg.isPrismatic ? RobotExternalMotionType::Translate : RobotExternalMotionType::Rotate;
		}
		if (item.contains("motionType") && item["motionType"].is_string())
		{
			cfg.motionType = motionFromString(item["motionType"].get<std::string>());
		}
		else if (cfg.kind == RobotExternalAxisKind::Turntable)
		{
			cfg.motionType = RobotExternalMotionType::Rotate;
		}
		if (item.contains("attachment") && item["attachment"].is_string())
		{
			cfg.attachment = attachmentFromString(item["attachment"].get<std::string>());
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
		if (item.contains("boundBackendId") && item["boundBackendId"].is_string())
		{
			cfg.boundBackendId = item["boundBackendId"].get<std::string>();
		}
		if (item.contains("workingFrameId") && item["workingFrameId"].is_string())
		{
			cfg.workingFrameId = item["workingFrameId"].get<std::string>();
		}
		normalizeExternalAxisConfig(cfg);
		out.axes.push_back(std::move(cfg));
	}
	return true;
}

std::string encodeExternalAxisQCsv(const std::vector<double>& qs)
{
	std::ostringstream os;
	for (size_t i = 0; i < qs.size(); ++i)
	{
		if (i > 0)
		{
			os << ',';
		}
		os.precision(12);
		os << std::defaultfloat << qs[i];
	}
	return os.str();
}

std::vector<double> parseExternalAxisQCsv(const std::string& csv)
{
	std::vector<double> out;
	if (csv.empty())
	{
		return out;
	}
	std::stringstream ss(csv);
	std::string part;
	while (std::getline(ss, part, ','))
	{
		try
		{
			out.push_back(std::stod(part));
		}
		catch (...)
		{
			out.push_back(0.0);
		}
	}
	return out;
}

void makeAxisMotionColumnMajor(const RobotExternalAxisConfig& cfg, const double q, double out[16])
{
	const kinematic_core::JointMotion1D motion = JointMotionAdapters::fromRobotExternalAxisConfig(cfg);
	kinematic_core::evaluateJointMotion1D(motion, q, out);
}

void composeBasePlacementWithExternalAxis(const double p0ColumnMajor[16], const RobotExternalAxisConfigSet& set,
										  const std::vector<double>& qValues, double outColumnMajor[16])
{
	composeAxesChain(p0ColumnMajor, set, qValues, RobotExternalAttachment::RobotBase, nullptr, outColumnMajor);
}

void composeBasePlacementWithExternalAxis(const double p0ColumnMajor[16], const RobotExternalAxisConfigSet& set,
										  const double qMm, double outColumnMajor[16])
{
	std::vector<double> qs(set.axes.size(), 0.0);
	for (size_t i = 0; i < set.axes.size(); ++i)
	{
		qs[i] = set.axes[i].home;
		if (set.axes[i].enabled && set.axes[i].attachment == RobotExternalAttachment::RobotBase)
		{
			qs[i] = qMm;
			break;
		}
	}
	composeBasePlacementWithExternalAxis(p0ColumnMajor, set, qs, outColumnMajor);
}

void unbakeBasePlacementExternalAxis(const double pEffColumnMajor[16], const RobotExternalAxisConfigSet& set,
									 const std::vector<double>& qValues, double outP0ColumnMajor[16])
{
	unbakeAxesChain(pEffColumnMajor, set, qValues, RobotExternalAttachment::RobotBase, nullptr, outP0ColumnMajor);
}

void unbakeBasePlacementExternalAxis(const double pEffColumnMajor[16], const RobotExternalAxisConfigSet& set,
									 const double qMm, double outP0ColumnMajor[16])
{
	std::vector<double> qs(set.axes.size(), 0.0);
	for (size_t i = 0; i < set.axes.size(); ++i)
	{
		qs[i] = set.axes[i].home;
		if (set.axes[i].enabled && set.axes[i].attachment == RobotExternalAttachment::RobotBase)
		{
			qs[i] = qMm;
			break;
		}
	}
	unbakeBasePlacementExternalAxis(pEffColumnMajor, set, qs, outP0ColumnMajor);
}

void composeWorkpiecePlacementWithExternalAxis(const double w0ColumnMajor[16], const RobotExternalAxisConfigSet& set,
											   const std::string& boundBackendId, const std::vector<double>& qValues,
											   double outColumnMajor[16])
{
	composeAxesChain(w0ColumnMajor, set, qValues, RobotExternalAttachment::Workpiece, &boundBackendId, outColumnMajor);
}

void unbakeWorkpiecePlacementExternalAxis(const double wEffColumnMajor[16], const RobotExternalAxisConfigSet& set,
										  const std::string& boundBackendId, const std::vector<double>& qValues,
										  double outW0ColumnMajor[16])
{
	unbakeAxesChain(wEffColumnMajor, set, qValues, RobotExternalAttachment::Workpiece, &boundBackendId,
					outW0ColumnMajor);
}

} // namespace RobotExternal
