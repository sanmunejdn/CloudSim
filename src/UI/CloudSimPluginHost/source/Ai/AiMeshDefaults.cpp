#include "Ai/AiMeshDefaults.h"

#include "AiCommandSchema.h"

#include <cmath>

namespace AiMeshDefaults
{
namespace
{
MeshCreateDefaults gOverrides;
bool gHasOverrides = false;

bool isPositive(double v)
{
	return std::isfinite(v) && v > 0.0;
}

bool readDimIfPositive(const nlohmann::json& dims, const char* key, double& out)
{
	if (!dims.contains(key) || !dims[key].is_number())
		return false;
	const double v = dims[key].get<double>();
	if (!isPositive(v) || v < AiCommandSchema::kMinDimMm || v > AiCommandSchema::kMaxDimMm)
		return false;
	out = v;
	return true;
}

void setDim(nlohmann::json& dims, const char* key, double value, bool& usedDefaults)
{
	dims[key] = value;
	usedDefaults = true;
}
}

MeshCreateDefaults builtinDefaults()
{
	return MeshCreateDefaults{};
}

MeshCreateDefaults activeDefaults()
{
	if (!gHasOverrides)
		return builtinDefaults();
	return gOverrides;
}

void setConfigOverrides(const MeshCreateDefaults& overrides)
{
	gOverrides = overrides;
	gHasOverrides = true;
}

void loadFromConfigJson(const nlohmann::json& root)
{
	if (!root.contains("mesh_create_defaults") || !root["mesh_create_defaults"].is_object())
		return;
	const auto& d = root["mesh_create_defaults"];
	MeshCreateDefaults m = builtinDefaults();
	if (d.contains("box") && d["box"].is_object())
	{
		const auto& b = d["box"];
		if (b.contains("length"))
			m.boxLengthMm = b["length"].get<double>();
		if (b.contains("width"))
			m.boxWidthMm = b["width"].get<double>();
		if (b.contains("height"))
			m.boxHeightMm = b["height"].get<double>();
	}
	if (d.contains("cylinder") && d["cylinder"].is_object())
	{
		const auto& c = d["cylinder"];
		if (c.contains("radius"))
			m.cylinderRadiusMm = c["radius"].get<double>();
		if (c.contains("height"))
			m.cylinderHeightMm = c["height"].get<double>();
	}
	if (d.contains("cone") && d["cone"].is_object())
	{
		const auto& c = d["cone"];
		if (c.contains("radius"))
			m.coneRadiusMm = c["radius"].get<double>();
		if (c.contains("height"))
			m.coneHeightMm = c["height"].get<double>();
	}
	if (d.contains("sphere") && d["sphere"].is_object())
	{
		const auto& s = d["sphere"];
		if (s.contains("radius"))
			m.sphereRadiusMm = s["radius"].get<double>();
	}
	setConfigOverrides(m);
}

bool applyMissingDimensions(nlohmann::json& cmd, bool* usedDefaults)
{
	bool anyDefault = false;
	if (usedDefaults)
		*usedDefaults = false;

	if (!cmd.is_object() || cmd.value("action", std::string()) != "create_mesh")
		return false;

	const std::string prim = cmd.value("primitive", "");
	BackendPrimitiveGeometry::PrimitiveKind kind{};
	if (!AiCommandSchema::primitiveKindFromString(prim, kind))
		return false;

	const MeshCreateDefaults defs = activeDefaults();
	if (!cmd.contains("dimensions_mm") || !cmd["dimensions_mm"].is_object())
		cmd["dimensions_mm"] = nlohmann::json::object();
	nlohmann::json& dims = cmd["dimensions_mm"];

	switch (kind)
	{
	case BackendPrimitiveGeometry::PrimitiveKind::Box:
	{
		double L = 0, W = 0, H = 0;
		const bool hasL = readDimIfPositive(dims, "length", L);
		const bool hasW = readDimIfPositive(dims, "width", W);
		const bool hasH = readDimIfPositive(dims, "height", H);
		if (!hasL)
			setDim(dims, "length", defs.boxLengthMm, anyDefault);
		if (!hasW)
			setDim(dims, "width", defs.boxWidthMm, anyDefault);
		if (!hasH)
			setDim(dims, "height", defs.boxHeightMm, anyDefault);
		break;
	}
	case BackendPrimitiveGeometry::PrimitiveKind::Cylinder:
	{
		double R = 0, D = 0, H = 0;
		const bool hasR = readDimIfPositive(dims, "radius", R);
		const bool hasD = readDimIfPositive(dims, "diameter", D);
		const bool hasH = readDimIfPositive(dims, "height", H);
		if (!hasR && hasD)
			setDim(dims, "radius", D * 0.5, anyDefault);
		else if (!hasR)
			setDim(dims, "radius", defs.cylinderRadiusMm, anyDefault);
		if (!hasH)
			setDim(dims, "height", defs.cylinderHeightMm, anyDefault);
		break;
	}
	case BackendPrimitiveGeometry::PrimitiveKind::Cone:
	{
		double R = 0, H = 0;
		const bool hasR = readDimIfPositive(dims, "radius", R);
		const bool hasH = readDimIfPositive(dims, "height", H);
		if (!hasR)
			setDim(dims, "radius", defs.coneRadiusMm, anyDefault);
		if (!hasH)
			setDim(dims, "height", defs.coneHeightMm, anyDefault);
		break;
	}
	case BackendPrimitiveGeometry::PrimitiveKind::Sphere:
	{
		double R = 0, D = 0;
		const bool hasR = readDimIfPositive(dims, "radius", R);
		const bool hasD = readDimIfPositive(dims, "diameter", D);
		if (!hasR && !hasD)
			setDim(dims, "radius", defs.sphereRadiusMm, anyDefault);
		break;
	}
	}

	if (usedDefaults)
		*usedDefaults = anyDefault;
	return anyDefault;
}

QString summarizeDimensionsMm(const nlohmann::json& cmd)
{
	if (!cmd.contains("dimensions_mm") || !cmd["dimensions_mm"].is_object())
		return {};
	const auto& dims = cmd["dimensions_mm"];
	const std::string prim = cmd.value("primitive", "");
	if (prim == "box")
	{
		return QStringLiteral("%1×%2×%3 mm")
			.arg(dims.value("length", 0.0))
			.arg(dims.value("width", 0.0))
			.arg(dims.value("height", 0.0));
	}
	if (prim == "cylinder" || prim == "cone")
	{
		return QStringLiteral("R%1 H%2 mm")
			.arg(dims.value("radius", 0.0))
			.arg(dims.value("height", 0.0));
	}
	if (prim == "sphere")
	{
		if (dims.contains("diameter"))
			return QStringLiteral("Ø%1 mm").arg(dims.value("diameter", 0.0));
		return QStringLiteral("R%1 mm").arg(dims.value("radius", 0.0));
	}
	return {};
}

QString defaultsAppliedNote(const nlohmann::json& cmd, bool usedDefaults)
{
	if (!usedDefaults)
		return {};
	const QString dims = summarizeDimensionsMm(cmd);
	if (dims.isEmpty())
		return QStringLiteral("未指定尺寸，已使用默认参数。");
	return QStringLiteral("未指定完整尺寸，已用默认值补全：%1。").arg(dims);
}

} // namespace AiMeshDefaults
