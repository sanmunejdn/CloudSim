#include "AiCommandSchema.h"

#include "Ai/AiMeshDefaults.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <string>

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

std::string extractJsonObjectTextImpl(const std::string& text)
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
	const std::string jsonText = extractJsonObjectTextImpl(llmText);
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
			errorMessage = "cylinder requires radius or diameter (mm).";
			return false;
		}
		if (!readDim(dims, "height", outParams.heightMm))
		{
			errorMessage = "cylinder requires height (mm).";
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

std::string extractJsonObjectText(const std::string& text)
{
	return extractJsonObjectTextImpl(text);
}

bool isReservedComposeStepKey(const std::string& key)
{
	static const char* keys[] = {
		"args", "api", "id", "version", "domain", "steps", "primitive",
		"dimensions_mm", "name", "pose_mm", "rotation_deg", "op",
		"target", "tool", "result_name", "hide_operands", "mesh_quality",
	};
	for (const char* k : keys)
	{
		if (key == k)
			return true;
	}
	return false;
}

std::string repairComposePlanJsonText(const std::string& text)
{
	std::string s = text;
	const std::string stepsMarker = "\"steps\":[";
	const std::size_t stepsPos = s.find(stepsMarker);
	if (stepsPos == std::string::npos)
		return s;

	std::size_t pos = stepsPos + stepsMarker.size();
	for (;;)
	{
		const std::size_t closeElem = s.find("},", pos);
		if (closeElem == std::string::npos)
			break;
		std::size_t j = closeElem + 2;
		while (j < s.size() && std::isspace(static_cast<unsigned char>(s[j])))
			++j;
		if (j >= s.size() || s[j] != '"')
		{
			pos = closeElem + 2;
			continue;
		}
		const std::size_t idStart = j + 1;
		const std::size_t idEnd = s.find('"', idStart);
		if (idEnd == std::string::npos)
			break;
		const std::string key = s.substr(idStart, idEnd - idStart);
		if (isReservedComposeStepKey(key))
		{
			pos = idEnd + 1;
			continue;
		}
		std::size_t afterId = idEnd + 1;
		while (afterId < s.size() && std::isspace(static_cast<unsigned char>(s[afterId])))
			++afterId;
		if (afterId >= s.size() || s[afterId] != ':')
		{
			pos = idEnd + 1;
			continue;
		}
		++afterId;
		while (afterId < s.size() && std::isspace(static_cast<unsigned char>(s[afterId])))
			++afterId;
		if (afterId >= s.size() || s[afterId] != '{')
		{
			pos = idEnd + 1;
			continue;
		}
		const std::string replacement = "{\"id\":\"" + key + "\",";
		s.replace(j, afterId - j, replacement);
		pos = j + replacement.size();
	}

	// 首步之后误写为 }},"stepId":{"api"（缺 step 闭合括号）
	for (std::size_t scan = stepsPos; scan + 8 < s.size();)
	{
		const std::size_t comma = s.find(",\"", scan);
		if (comma == std::string::npos || comma < stepsPos)
			break;
		const std::size_t keyStart = comma + 2;
		const std::size_t keyEnd = s.find('"', keyStart);
		if (keyEnd == std::string::npos)
			break;
		const std::string key = s.substr(keyStart, keyEnd - keyStart);
		if (isReservedComposeStepKey(key))
		{
			scan = keyEnd + 1;
			continue;
		}
		std::size_t p = keyEnd + 1;
		while (p < s.size() && std::isspace(static_cast<unsigned char>(s[p])))
			++p;
		if (p >= s.size() || s[p] != ':')
		{
			scan = keyEnd + 1;
			continue;
		}
		++p;
		while (p < s.size() && std::isspace(static_cast<unsigned char>(s[p])))
			++p;
		if (p >= s.size() || s[p] != '{')
		{
			scan = keyEnd + 1;
			continue;
		}
		std::size_t afterBrace = p + 1;
		while (afterBrace < s.size() && std::isspace(static_cast<unsigned char>(s[afterBrace])))
			++afterBrace;
		if (afterBrace + 5 > s.size() || s.compare(afterBrace, 5, "\"api\"") != 0)
		{
			scan = keyEnd + 1;
			continue;
		}
		const std::string replacement = "}},{\"id\":\"" + key + "\",";
		s.replace(comma, p + 1 - comma, replacement);
		scan = comma + replacement.size();
	}

	int bracketDepth = 0;
	for (std::size_t i = stepsPos + stepsMarker.size() - 1; i < s.size(); ++i)
	{
		if (s[i] == '[')
			++bracketDepth;
		else if (s[i] == ']')
			--bracketDepth;
	}
	if (bracketDepth > 0)
	{
		const std::size_t lastBrace = s.rfind('}');
		if (lastBrace != std::string::npos)
			s.insert(lastBrace, static_cast<std::size_t>(bracketDepth), ']');
	}
	return s;
}

std::string toLowerAscii(std::string s)
{
	for (char& c : s)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return s;
}

std::string inferStepIdFromLabel(const std::string& label)
{
	const std::string lower = toLowerAscii(label);
	if (lower.find("hole") != std::string::npos && lower.find("tool") != std::string::npos)
		return "hole_tool";
	if (lower.find("body") != std::string::npos || lower.find("box") != std::string::npos)
		return "body";
	if (lower.find("boolean") != std::string::npos || lower.find("result") != std::string::npos)
		return "result";
	return lower;
}

void hoistJsonField(nlohmann::json& step, nlohmann::json& args, const char* key)
{
	if (!step.contains(key))
		return;
	if (std::string(key) != "name" || !args.contains("name"))
		args[key] = step[key];
	step.erase(key);
}

void canonicalizeComposeStepShape(nlohmann::json& step)
{
	if (!step.is_object())
		return;

	if ((!step.contains("api") || !step["api"].is_string() || step["api"].get<std::string>().empty()) && step.contains("type"))
		step["api"] = step["type"];
	if (step.contains("type"))
		step.erase("type");

	if (step.contains("name") && (!step.contains("id") || !step["id"].is_string() || step.value("id", "").empty()))
	{
		const std::string label = step["name"].get<std::string>();
		step["id"] = inferStepIdFromLabel(label);
	}

	if (!step.contains("args") || !step["args"].is_object())
		step["args"] = nlohmann::json::object();
	nlohmann::json& args = step["args"];

	static const char* kFlatKeys[] = {
		"primitive", "dimensions_mm", "pose_mm", "rotation_deg", "mesh_quality",
		"op", "target", "tool", "result_name", "hide_operands", "path", "is_point_cloud",
	};
	for (const char* key : kFlatKeys)
		hoistJsonField(step, args, key);

	if (step.contains("name"))
	{
		if (!args.contains("name"))
			args["name"] = step["name"];
		step.erase("name");
	}

	if (args.contains("pose_mm") && args["pose_mm"].is_array() && args["pose_mm"].size() >= 3U)
	{
		args["pose_mm"] = nlohmann::json::object({ { "x", args["pose_mm"][0] }, { "y", args["pose_mm"][1] },
			{ "z", args["pose_mm"][2] } });
	}
	if (args.contains("rotation_deg") && args["rotation_deg"].is_number())
	{
		const double z = args["rotation_deg"].get<double>();
		args["rotation_deg"] = nlohmann::json::object({ { "x", 0.0 }, { "y", 0.0 }, { "z", z } });
	}

	const std::string api = step.value("api", "");
	if (api == "booleanMesh")
	{
		for (const char* refKey : { "target", "tool" })
		{
			if (!args.contains(refKey) || !args[refKey].is_string())
				continue;
			std::string ref = args[refKey].get<std::string>();
			if (!ref.empty() && ref.front() != '$')
				args[refKey] = std::string("$") + ref;
		}
		if (!args.contains("op") && step.contains("op"))
			args["op"] = step["op"];
	}
}

void normalizeComposePlanJson(nlohmann::json& root)
{
	if (!root.is_object())
		return;

	auto appendToSteps = [&](const nlohmann::json& extra) {
		if (!root.contains("steps") || !root["steps"].is_array())
			root["steps"] = nlohmann::json::array();
		if (extra.is_array())
		{
			for (const auto& s : extra)
			{
				if (s.is_object())
					root["steps"].push_back(s);
			}
		}
		else if (extra.is_object())
			root["steps"].push_back(extra);
	};

	// 小模型常把 booleanMesh 写在顶层 result[]，执行器只遍历 steps[]
	if (root.contains("result"))
	{
		appendToSteps(root["result"]);
		root.erase("result");
	}
	if (root.contains("results"))
	{
		appendToSteps(root["results"]);
		root.erase("results");
	}

	if (!root.contains("steps") || !root["steps"].is_array())
		return;

	for (auto& step : root["steps"])
		canonicalizeComposeStepShape(step);

	for (auto& step : root["steps"])
	{
		if (!step.is_object())
			continue;
		if (step.value("api", "") == "booleanMesh" && !step.contains("id"))
			step["id"] = "result";
	}

	for (auto& step : root["steps"])
	{
		if (!step.is_object())
			continue;
		std::string id = step.value("id", "");
		if (id == "optional" || id.empty())
		{
			if (step.contains("args") && step["args"].is_object())
			{
				const std::string name = step["args"].value("name", "");
				if (!name.empty())
					step["id"] = inferStepIdFromLabel(name);
			}
		}

		if (!step.contains("args") || !step["args"].is_object())
			continue;
		nlohmann::json& args = step["args"];
		if (!args.contains("dimensions_mm") || !args["dimensions_mm"].is_array())
			continue;

		const auto& arr = args["dimensions_mm"];
		const std::string prim = args.value("primitive", "");
		nlohmann::json dims = nlohmann::json::object();
		if (prim == "box" && arr.size() >= 3)
		{
			dims["length"] = arr[0];
			dims["width"] = arr[1];
			dims["height"] = arr[2];
		}
		else if ((prim == "cylinder" || prim == "cone") && arr.size() >= 2)
		{
			if (arr.size() >= 3 && arr[0].is_number() && arr[1].is_number())
			{
				const double a0 = arr[0].get<double>();
				const double a1 = arr[1].get<double>();
				dims["radius"] = (a1 > 0.0 && std::abs(a0 - 2.0 * a1) < 0.01 * a0) ? a1 : a0;
				dims["height"] = arr[2];
			}
			else
			{
				dims["radius"] = arr[0];
				if (arr.size() >= 2)
					dims["height"] = arr[1];
			}
		}
		else if (prim == "sphere" && arr.size() >= 1)
		{
			dims["radius"] = arr[0];
		}
		if (!dims.empty())
			args["dimensions_mm"] = dims;
	}

	for (auto& step : root["steps"])
	{
		if (!step.is_object() || !step.contains("args") || !step["args"].is_object())
			continue;
		nlohmann::json& args = step["args"];
		if (args.value("primitive", "") != "cylinder" || !args.contains("dimensions_mm")
			|| !args["dimensions_mm"].is_object())
			continue;
		nlohmann::json& cdims = args["dimensions_mm"];
		if (cdims.contains("diameter") && cdims["diameter"].is_number())
		{
			const double d = cdims["diameter"].get<double>();
			if (!cdims.contains("radius") || !cdims["radius"].is_number())
				cdims["radius"] = d * 0.5;
		}
	}

	// 通孔差集：刀具圆柱高度须大于坯料，避免顶底与盒体共面触发 CGAL corefinement 断言
	bool hasDifference = false;
	double boxHeightMm = 0.0;
	nlohmann::json* cylinderArgs = nullptr;
	for (auto& step : root["steps"])
	{
		if (!step.is_object())
			continue;
		const std::string api = step.value("api", "");
		if (api == "booleanMesh")
		{
			const nlohmann::json& bargs = step.contains("args") && step["args"].is_object() ? step["args"] : nlohmann::json::object();
			if (bargs.value("op", "difference") == "difference")
				hasDifference = true;
		}
		if (api != "createPrimitiveMesh" || !step.contains("args") || !step["args"].is_object())
			continue;
		nlohmann::json& args = step["args"];
		const std::string prim = args.value("primitive", "");
		if (prim == "box" && args.contains("dimensions_mm") && args["dimensions_mm"].is_object())
			boxHeightMm = std::max(boxHeightMm, args["dimensions_mm"].value("height", 0.0));
		if (prim == "cylinder")
			cylinderArgs = &args;
	}
	if (hasDifference && cylinderArgs != nullptr && boxHeightMm > 0.0)
	{
		if (!cylinderArgs->contains("dimensions_mm") || !(*cylinderArgs)["dimensions_mm"].is_object())
			(*cylinderArgs)["dimensions_mm"] = nlohmann::json::object();
		nlohmann::json& cdims = (*cylinderArgs)["dimensions_mm"];
		const double minToolHeightMm = boxHeightMm * 1.2 + 20.0;
		const double curH = cdims.value("height", 0.0);
		if (curH < minToolHeightMm)
			cdims["height"] = minToolHeightMm;
	}

	// 小模型常把坯料建成圆柱，或把直径写进 radius；按 step id / 尺寸字段纠正
	double boxMinSideMm = 0.0;
	for (auto& step : root["steps"])
	{
		if (!step.is_object() || step.value("api", "") != "createPrimitiveMesh" || !step.contains("args")
			|| !step["args"].is_object())
			continue;
		nlohmann::json& args = step["args"];
		const std::string stepId = step.value("id", "");
		nlohmann::json& dims = args["dimensions_mm"];
		if (!dims.is_object())
			continue;
		const bool hasBoxDims =
			dims.contains("length") && dims.contains("width") && dims.contains("height");
		if (hasBoxDims)
		{
			const double L = dims.value("length", 0.0);
			const double W = dims.value("width", 0.0);
			boxMinSideMm = std::max(boxMinSideMm, std::min(L, W));
			if (stepId == "body" || args.value("primitive", "") == "cylinder")
				args["primitive"] = "box";
		}
		if (stepId == "hole_tool" || stepId == "tool")
			args["primitive"] = "cylinder";
		if (args.contains("mesh_quality") && args["mesh_quality"].is_object())
		{
			auto& mq = args["mesh_quality"];
			if (mq.contains("segments") && mq["segments"].is_number_integer())
				mq["segments"] = std::min(48, mq["segments"].get<int>());
		}
	}
	for (auto& step : root["steps"])
	{
		if (!step.is_object() || step.value("api", "") != "createPrimitiveMesh" || !step.contains("args")
			|| !step["args"].is_object())
			continue;
		nlohmann::json& args = step["args"];
		if (args.value("primitive", "") != "cylinder" || !args.contains("dimensions_mm")
			|| !args["dimensions_mm"].is_object())
			continue;
		const std::string stepId = step.value("id", "");
		if (stepId != "hole_tool" && stepId != "tool")
			continue;
		nlohmann::json& cdims = args["dimensions_mm"];
		if (cdims.contains("radius") && cdims["radius"].is_number() && boxMinSideMm > 0.0
			&& !cdims.contains("diameter"))
		{
			double R = cdims["radius"].get<double>();
			// 直径 50 被写成 radius=50 时缩小一半
			if (R > boxMinSideMm * 0.35 && R > 30.0)
			{
				cdims["radius"] = R * 0.5;
				R = cdims["radius"].get<double>();
			}
			(void)R;
		}
	}
}

bool parseModifyObjectCommand(
	const nlohmann::json& cmd,
	std::string& outBackendId,
	nlohmann::json& outPropertyPatch,
	std::string& errorMessage)
{
	errorMessage.clear();
	outBackendId.clear();
	outPropertyPatch = nlohmann::json::object();
	if (!cmd.is_object())
	{
		errorMessage = "Command must be a JSON object.";
		return false;
	}
	if (cmd.value("version", 1) != kSchemaVersion)
	{
		errorMessage = "Unsupported command version.";
		return false;
	}
	if (cmd.value("action", "") != "modify_object")
	{
		errorMessage = "action must be modify_object.";
		return false;
	}
	outBackendId = cmd.value("backend_id", std::string());
	if (outBackendId.empty())
	{
		errorMessage = "modify_object requires backend_id.";
		return false;
	}
	if (cmd.contains("propertyBag") && cmd["propertyBag"].is_object())
	{
		outPropertyPatch = cmd["propertyBag"];
	}
	else if (cmd.contains("pose") && cmd["pose"].is_object())
	{
		outPropertyPatch["pose"] = cmd["pose"];
	}
	else
	{
		errorMessage = "modify_object requires propertyBag or pose.";
		return false;
	}
	return true;
}

bool parseImportAssetCommand(const nlohmann::json& cmd, std::string& outFilePath, std::string& errorMessage)
{
	errorMessage.clear();
	outFilePath.clear();
	if (!cmd.is_object())
	{
		errorMessage = "Command must be a JSON object.";
		return false;
	}
	if (cmd.value("version", 1) != kSchemaVersion)
	{
		errorMessage = "Unsupported command version.";
		return false;
	}
	if (cmd.value("action", "") != "import_asset")
	{
		errorMessage = "action must be import_asset.";
		return false;
	}
	outFilePath = cmd.value("file_path", std::string());
	if (outFilePath.empty())
	{
		errorMessage = "import_asset requires file_path.";
		return false;
	}
	return true;
}

}