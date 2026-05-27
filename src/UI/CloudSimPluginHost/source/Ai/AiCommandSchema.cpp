#include "AiCommandSchema.h"

#include "Ai/AiMeshDefaults.h"

#include <cctype>
#include <cmath>
#include <cstddef>

namespace AiCommandSchema
{
namespace
{
std::string trimCopy(std::string s)
{
	const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
	while (!s.empty() && !notSpace(static_cast<unsigned char>(s.front())))
		s.erase(s.begin());
	while (!s.empty() && !notSpace(static_cast<unsigned char>(s.back())))
		s.pop_back();
	return s;
}

std::string extractJsonObjectText(const std::string& text)
{
	std::string s = trimCopy(text);
	const std::string fence = "```";
	const std::size_t fencePos = s.find(fence);
	if (fencePos != std::string::npos)
	{
		std::size_t start = fencePos + fence.size();
		if (start < s.size() && (s[start] == 'j' || s[start] == 'J'))
		{
			const std::size_t nl = s.find('\n', start);
			if (nl != std::string::npos)
				start = nl + 1;
		}
		const std::size_t endFence = s.find(fence, start);
		if (endFence != std::string::npos)
			s = trimCopy(s.substr(start, endFence - start));
	}
	const std::size_t b = s.find('{');
	const std::size_t e = s.rfind('}');
	if (b != std::string::npos && e != std::string::npos && e > b)
		return s.substr(b, e - b + 1);
	return s;
}

bool readDim(const nlohmann::json& j, const char* key, double& out)
{
	if (!j.contains(key) || !j[key].is_number())
		return false;
	out = j[key].get<double>();
	return out >= kMinDimMm && out <= kMaxDimMm;
}

int readIntClamped(const nlohmann::json& j, const char* key, int defaultVal, int lo, int hi)
{
	if (!j.contains(key) || !j[key].is_number_integer())
		return defaultVal;
	return std::max(lo, std::min(hi, j[key].get<int>()));
}
}

bool primitiveKindFromString(const std::string& s, BackendPrimitiveGeometry::PrimitiveKind& out)
{
	if (s == "box") { out = BackendPrimitiveGeometry::PrimitiveKind::Box; return true; }
	if (s == "cylinder") { out = BackendPrimitiveGeometry::PrimitiveKind::Cylinder; return true; }
	if (s == "cone") { out = BackendPrimitiveGeometry::PrimitiveKind::Cone; return true; }
	if (s == "sphere") { out = BackendPrimitiveGeometry::PrimitiveKind::Sphere; return true; }
	return false;
}

std::string primitiveKindToString(BackendPrimitiveGeometry::PrimitiveKind kind)
{
	switch (kind)
	{
	case BackendPrimitiveGeometry::PrimitiveKind::Box: return "box";
	case BackendPrimitiveGeometry::PrimitiveKind::Cylinder: return "cylinder";
	case BackendPrimitiveGeometry::PrimitiveKind::Cone: return "cone";
	case BackendPrimitiveGeometry::PrimitiveKind::Sphere: return "sphere";
	}
	return "box";
}

bool tryParseCreateMeshCommandJson(const std::string& llmText, nlohmann::json& outCommand, std::string& errorMessage)
{
	errorMessage.clear();
	const std::string jsonText = extractJsonObjectText(llmText);
	try
	{
		outCommand = nlohmann::json::parse(jsonText, nullptr, true);
	}
	catch (const std::exception& e)
	{
		errorMessage = std::string("LLM JSON parse failed: ") + e.what();
		return false;
	}
	BackendPrimitiveGeometry::PrimitiveMeshParams params;
	BackendPrimitiveGeometry::PrimitiveMeshQuality quality;
	std::string name;
	std::string source;
	return parseCreateMeshCommand(outCommand, params, quality, name, source, errorMessage);
}

std::string defaultDisplayNameFor(const BackendPrimitiveGeometry::PrimitiveMeshParams& p)
{
	switch (p.kind)
	{
	case BackendPrimitiveGeometry::PrimitiveKind::Box:
		return "Box_" + std::to_string(static_cast<int>(p.lengthMm)) + "x"
			+ std::to_string(static_cast<int>(p.widthMm)) + "x" + std::to_string(static_cast<int>(p.heightMm));
	case BackendPrimitiveGeometry::PrimitiveKind::Cylinder:
		return "Cylinder_R" + std::to_string(static_cast<int>(p.radiusMm)) + "_H"
			+ std::to_string(static_cast<int>(p.heightMm));
	case BackendPrimitiveGeometry::PrimitiveKind::Cone:
		return "Cone_R" + std::to_string(static_cast<int>(p.radiusMm)) + "_H"
			+ std::to_string(static_cast<int>(p.heightMm));
	case BackendPrimitiveGeometry::PrimitiveKind::Sphere:
		return "Sphere_R" + std::to_string(static_cast<int>(p.radiusMm));
	}
	return "Mesh";
}

bool parseCreateMeshCommand(
	const nlohmann::json& cmd,
	BackendPrimitiveGeometry::PrimitiveMeshParams& outParams,
	BackendPrimitiveGeometry::PrimitiveMeshQuality& outQuality,
	std::string& outDisplayName,
	std::string& outSourcePath,
	std::string& errorMessage)
{
	errorMessage.clear();
	if (!cmd.is_object())
	{
		errorMessage = "Command must be a JSON object.";
		return false;
	}
	const int ver = cmd.value("version", 1);
	if (ver != kSchemaVersion)
	{
		errorMessage = "Unsupported command version.";
		return false;
	}
	const std::string action = cmd.value("action", "");
	if (action != "create_mesh")
	{
		errorMessage = "Unknown action (expected create_mesh).";
		return false;
	}
	const std::string prim = cmd.value("primitive", "");
	if (!primitiveKindFromString(prim, outParams.kind))
	{
		errorMessage = "Unknown primitive (box|cylinder|cone|sphere).";
		return false;
	}
	nlohmann::json cmdMut = cmd;
	AiMeshDefaults::applyMissingDimensions(cmdMut);
	const nlohmann::json& dims = cmdMut.contains("dimensions_mm") ? cmdMut["dimensions_mm"] : nlohmann::json::object();
	switch (outParams.kind)
	{
	case BackendPrimitiveGeometry::PrimitiveKind::Box:
		if (!readDim(dims, "length", outParams.lengthMm) || !readDim(dims, "width", outParams.widthMm)
			|| !readDim(dims, "height", outParams.heightMm))
		{
			errorMessage = "box requires length, width, height (mm).";
			return false;
		}
		break;
	case BackendPrimitiveGeometry::PrimitiveKind::Cylinder:
		if (!readDim(dims, "radius", outParams.radiusMm) || !readDim(dims, "height", outParams.heightMm))
		{
			errorMessage = "cylinder requires radius, height (mm).";
			return false;
		}
		break;
	case BackendPrimitiveGeometry::PrimitiveKind::Cone:
		if (!readDim(dims, "radius", outParams.radiusMm) || !readDim(dims, "height", outParams.heightMm))
		{
			errorMessage = "cone requires radius, height (mm).";
			return false;
		}
		outParams.radiusTopMm = 0.0;
		if (dims.contains("radius_top") && dims["radius_top"].is_number())
			outParams.radiusTopMm = std::max(0.0, dims["radius_top"].get<double>());
		break;
	case BackendPrimitiveGeometry::PrimitiveKind::Sphere:
		if (dims.contains("diameter") && dims["diameter"].is_number())
		{
			const double d = dims["diameter"].get<double>();
			if (d < kMinDimMm || d > kMaxDimMm)
			{
				errorMessage = "diameter out of range.";
				return false;
			}
			outParams.radiusMm = d * 0.5;
		}
		else if (!readDim(dims, "radius", outParams.radiusMm))
		{
			errorMessage = "sphere requires radius or diameter (mm).";
			return false;
		}
		break;
	}
	if (cmd.contains("mesh_quality") && cmd["mesh_quality"].is_object())
	{
		const auto& mq = cmd["mesh_quality"];
		outQuality.segments = readIntClamped(mq, "segments", outQuality.segments, 8, 128);
		outQuality.rings = readIntClamped(mq, "rings", outQuality.rings, 4, 64);
	}
	outDisplayName = cmd.value("name", defaultDisplayNameFor(outParams));
	outSourcePath = std::string("ai://primitive/") + prim;
	return true;
}

}