#include "ProcessFlowPresetLoader.h"

#include "TrajectoryOpRegistry.h"
#include "TrajectoryOpConfigRegistry.h"
#include "TrajectoryOpDescriptorCodec.h"

#include <json.hpp>

#include <fstream>
#include <optional>

namespace RobotInstruction
{
namespace
{

std::optional<std::string> readTextFile(const std::string& path)
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
	{
		return std::nullopt;
	}
	std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	return content;
}

std::vector<TrajectoryOpKind> hardcodedPresetOpKinds(const RecipeKind kind)
{
	switch (kind)
	{
	case RecipeKind::Weld:
		return {
			TrajectoryOpKind::Resample,
			TrajectoryOpKind::OffsetAlongNormal,
			TrajectoryOpKind::SmoothPose,
			TrajectoryOpKind::AssignBlend,
			TrajectoryOpKind::Approach,
			TrajectoryOpKind::Retract};
	case RecipeKind::Glue:
		return {
			TrajectoryOpKind::Resample,
			TrajectoryOpKind::OffsetAlongNormal,
			TrajectoryOpKind::SmoothPose,
			TrajectoryOpKind::AssignSpeedZone};
	case RecipeKind::Grind:
		return {
			TrajectoryOpKind::Resample,
			TrajectoryOpKind::OffsetAlongNormal,
			TrajectoryOpKind::SmoothPose,
			TrajectoryOpKind::Weave,
			TrajectoryOpKind::Approach,
			TrajectoryOpKind::Retract};
	}
	return {};
}

bool recipeKindFromPresetId(const std::string& presetId, RecipeKind& out)
{
	if (presetId == "weld")
	{
		out = RecipeKind::Weld;
		return true;
	}
	if (presetId == "glue")
	{
		out = RecipeKind::Glue;
		return true;
	}
	if (presetId == "grind")
	{
		out = RecipeKind::Grind;
		return true;
	}
	return false;
}

std::vector<TrajectoryOpDescriptor> descriptorsFromOpKinds(const std::vector<TrajectoryOpKind>& kinds)
{
	std::vector<TrajectoryOpDescriptor> out;
	out.reserve(kinds.size());
	for (const TrajectoryOpKind kind : kinds)
	{
		TrajectoryOpDescriptor op{};
		op.kind = kind;
		op.scope.kind = OpScope::Kind::Group;
		out.push_back(op);
	}
	return out;
}

std::vector<ProcessFlowPresetEntry> fallbackPresets()
{
	std::vector<ProcessFlowPresetEntry> out;
	auto push = [&out](
		const char* id,
		const char* zh,
		const char* en,
		const std::vector<TrajectoryOpKind>& kinds) {
		ProcessFlowPresetEntry entry{};
		entry.id = id;
		entry.labelZh = zh;
		entry.labelEn = en;
		entry.ops = kinds;
		out.push_back(std::move(entry));
	};
	push("weld", "焊缝", "Weld", hardcodedPresetOpKinds(RecipeKind::Weld));
	push("glue", "涂胶", "Glue", hardcodedPresetOpKinds(RecipeKind::Glue));
	push("grind", "打磨", "Grind", hardcodedPresetOpKinds(RecipeKind::Grind));
	return out;
}

} // namespace

TrajectoryOpKind trajectoryOpKindFromPresetToken(const std::string& token, bool* ok)
{
	auto setOk = [&](const bool value) {
		if (ok)
		{
			*ok = value;
		}
	};
	if (token == "Translate")
	{
		setOk(true);
		return TrajectoryOpKind::Translate;
	}
	if (token == "Rotate")
	{
		setOk(true);
		return TrajectoryOpKind::Rotate;
	}
	if (token == "Mirror")
	{
		setOk(true);
		return TrajectoryOpKind::Mirror;
	}
	if (token == "Delete")
	{
		setOk(true);
		return TrajectoryOpKind::Delete;
	}
	if (token == "Duplicate")
	{
		setOk(true);
		return TrajectoryOpKind::Duplicate;
	}
	if (token == "Reorder")
	{
		setOk(true);
		return TrajectoryOpKind::Reorder;
	}
	if (token == "Approach")
	{
		setOk(true);
		return TrajectoryOpKind::Approach;
	}
	if (token == "Retract")
	{
		setOk(true);
		return TrajectoryOpKind::Retract;
	}
	if (token == "Resample")
	{
		setOk(true);
		return TrajectoryOpKind::Resample;
	}
	if (token == "OffsetAlongNormal")
	{
		setOk(true);
		return TrajectoryOpKind::OffsetAlongNormal;
	}
	if (token == "OffsetLateral")
	{
		setOk(true);
		return TrajectoryOpKind::OffsetLateral;
	}
	if (token == "SmoothPose")
	{
		setOk(true);
		return TrajectoryOpKind::SmoothPose;
	}
	if (token == "AssignBlend")
	{
		setOk(true);
		return TrajectoryOpKind::AssignBlend;
	}
	if (token == "AssignSpeedZone")
	{
		setOk(true);
		return TrajectoryOpKind::AssignSpeedZone;
	}
	if (token == "Weave")
	{
		setOk(true);
		return TrajectoryOpKind::Weave;
	}
	if (token == "ReachabilityFilter")
	{
		setOk(true);
		return TrajectoryOpKind::ReachabilityFilter;
	}
	if (token == "ExternalAxisSearch")
	{
		setOk(true);
		return TrajectoryOpKind::ExternalAxisSearch;
	}
	if (token == "RecipeWeld" || token == "RecipeGlue" || token == "RecipeGrind")
	{
		setOk(false);
		return TrajectoryOpKind::Translate;
	}
	setOk(false);
	return TrajectoryOpKind::Translate;
}

std::vector<ProcessFlowPresetEntry> loadProcessFlowPresets(
	const std::string& resourceBaseDir,
	std::string* errMsg)
{
	std::vector<std::string> candidates;
	if (!resourceBaseDir.empty())
	{
		candidates.push_back(resourceBaseDir + "/resource/trajectory/ProcessFlowPresets.json");
		candidates.push_back(resourceBaseDir + "/ProcessFlowPresets.defaults.json");
	}
	candidates.push_back("resource/trajectory/ProcessFlowPresets.json");
	candidates.push_back("ProcessFlowPresets.defaults.json");
	for (const std::string& rel : candidates)
	{
		const std::optional<std::string> text = readTextFile(rel);
		if (!text.has_value() || text->empty())
		{
			continue;
		}
		try
		{
			const nlohmann::json root = nlohmann::json::parse(*text);
			if (!root.contains("presets") || !root["presets"].is_array())
			{
				continue;
			}
			std::vector<ProcessFlowPresetEntry> out;
			for (const nlohmann::json& item : root["presets"])
			{
				ProcessFlowPresetEntry entry{};
				entry.id = item.value("id", "");
				entry.labelZh = item.value("labelZh", entry.id);
				entry.labelEn = item.value("labelEn", entry.id);
				if (item.contains("ops") && item["ops"].is_array())
				{
					for (const nlohmann::json& token : item["ops"])
					{
						if (!token.is_string())
						{
							continue;
						}
						bool tokenOk = false;
						const TrajectoryOpKind kind = trajectoryOpKindFromPresetToken(token.get<std::string>(), &tokenOk);
						if (tokenOk)
						{
							entry.ops.push_back(kind);
						}
					}
				}
				if (item.contains("pipeline") && item["pipeline"].is_array())
				{
					std::string pipeErr;
					trajectory_algo::pipelineFromJson(item["pipeline"], entry.pipeline, &pipeErr);
				}
				if (!entry.id.empty() && (!entry.ops.empty() || !entry.pipeline.empty()))
				{
					out.push_back(std::move(entry));
				}
			}
			if (!out.empty())
			{
				return out;
			}
		}
		catch (const std::exception& ex)
		{
			if (errMsg)
			{
				*errMsg = ex.what();
			}
		}
	}
	return fallbackPresets();
}

std::vector<TrajectoryOpDescriptor> buildRecipePresetFromId(
	const std::string& presetId,
	const std::string& resourceBaseDir,
	std::string* errMsg)
{
	trajectory_algo::ensureTrajectoryOpBuiltinsRegistered();
	trajectory_algo::TrajectoryOpConfigRegistry::instance().ensureLoaded(resourceBaseDir, errMsg);

	const std::vector<ProcessFlowPresetEntry> presets = loadProcessFlowPresets(resourceBaseDir, errMsg);
	for (const ProcessFlowPresetEntry& entry : presets)
	{
		if (entry.id != presetId)
		{
			continue;
		}
		if (!entry.pipeline.empty())
		{
			return entry.pipeline;
		}
		std::vector<TrajectoryOpDescriptor> out;
		out.reserve(entry.ops.size());
		OpScope scope{};
		scope.kind = OpScope::Kind::Group;
		for (const TrajectoryOpKind kind : entry.ops)
		{
			out.push_back(trajectory_algo::TrajectoryOpConfigRegistry::instance().defaultUnifiedOp(kind, scope));
		}
		return out;
	}
	RecipeKind kind{};
	if (recipeKindFromPresetId(presetId, kind))
	{
		return descriptorsFromOpKinds(hardcodedPresetOpKinds(kind));
	}
	if (errMsg)
	{
		*errMsg = "unknown preset id";
	}
	return {};
}

} // namespace RobotInstruction
