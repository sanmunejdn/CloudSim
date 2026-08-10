/// @file TrajectoryOpConfigImpl.cpp
/// @brief TrajectoryOpConfigImpl 实现

// TrajectoryOpConfigImpl 实现
#include "TrajectoryOpConfigImpl.h"

#include "ITrajectoryOp.h"
#include "TrajectoryOpConfigRegistry.h"
#include "TrajectoryOpDescriptorCodec.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpRegistry.h"
#include "TrajectoryParamJsonIo.h"

namespace trajectory_algo
{
TrajectoryOpConfigImpl::TrajectoryOpConfigImpl(const RobotInstruction::TrajectoryOpKind kind,
											   std::string jsonRelativePath)
	: m_kind(kind), m_jsonRelativePath(std::move(jsonRelativePath))
{
}

std::vector<TrajectoryOpParamField> TrajectoryOpConfigImpl::paramFields() const
{
	const ITrajectoryOp* algo = TrajectoryOpRegistry::instance().get(m_kind);
	if (!algo)
	{
		return {};
	}
	const std::vector<TrajectoryOpParamField> fallback = algo->paramFields();
	const std::string& baseDir = TrajectoryOpConfigRegistry::instance().resourceBaseDir();
	const std::optional<nlohmann::json> root = loadTrajectoryJsonFile(baseDir, m_jsonRelativePath);
	if (!root.has_value() || !root->contains("schema"))
	{
		return fallback;
	}
	const std::vector<TrajectoryOpParamField> fromJson = parseSchemaFieldsFromJson((*root)["schema"]);
	if (fromJson.empty())
	{
		return fallback;
	}
	std::vector<TrajectoryOpParamField> merged = fallback;
	for (const TrajectoryOpParamField& o : fromJson)
	{
		bool replaced = false;
		for (TrajectoryOpParamField& b : merged)
		{
			if (b.key == o.key)
			{
				if (!o.labelEn.empty())
				{
					b.labelEn = o.labelEn;
				}
				if (!o.labelZh.empty())
				{
					b.labelZh = o.labelZh;
				}
				if (!o.unit.empty())
				{
					b.unit = o.unit;
				}
				if (!o.group.empty())
				{
					b.group = o.group;
				}
				b.order = o.order;
				b.minValue = o.minValue;
				b.maxValue = o.maxValue;
				b.step = o.step;
				b.minInt = o.minInt;
				b.maxInt = o.maxInt;
				b.defaultDouble = o.defaultDouble;
				b.defaultInt = o.defaultInt;
				b.defaultBool = o.defaultBool;
				// JSON 未给枚举项时保留 C++ fallback，避免轴反向等下拉被清空
				if (!o.enumValues.empty())
				{
					b.type = TrajectoryParamType::Enum;
					b.enumValues = o.enumValues;
					b.enumLabelsZh = o.enumLabelsZh;
					b.enumLabelsEn = o.enumLabelsEn;
				}
				if (!o.visibleWhenScopeKind.empty())
				{
					b.visibleWhenScopeKind = o.visibleWhenScopeKind;
				}
				replaced = true;
				break;
			}
		}
		if (!replaced)
		{
			merged.push_back(o);
		}
	}
	return merged;
}

RobotInstruction::TrajectoryOpDescriptor
TrajectoryOpConfigImpl::defaultDescriptor(const RobotInstruction::OpScope& scope) const
{
	const ITrajectoryOp* algo = TrajectoryOpRegistry::instance().get(m_kind);
	if (!algo)
	{
		RobotInstruction::TrajectoryOpDescriptor op{};
		op.kind = m_kind;
		op.scope = scope;
		return op;
	}
	RobotInstruction::TrajectoryOpDescriptor op = algo->makeDefaultDescriptor(scope);
	const std::string& baseDir = TrajectoryOpConfigRegistry::instance().resourceBaseDir();
	const std::optional<nlohmann::json> root = loadTrajectoryJsonFile(baseDir, m_jsonRelativePath);
	if (!root.has_value() || !root->contains("defaults"))
	{
		return op;
	}
	nlohmann::json patch = nlohmann::json::object();
	patch["kind"] = TrajectoryOpRegistry::instance().kindToString(m_kind);
	patch["scope"] = (*root)["defaults"].value("scope", nlohmann::json::object());
	if ((*root)["defaults"].contains("params"))
	{
		patch["params"] = (*root)["defaults"]["params"];
	}
	std::string err;
	fromJson(patch, op, &err);
	op.scope = scope;
	return op;
}

std::unique_ptr<IOpParamConfig> makeTrajectoryOpConfig(const RobotInstruction::TrajectoryOpKind kind,
													   const char* jsonRelativePath)
{
	return std::make_unique<TrajectoryOpConfigImpl>(kind, jsonRelativePath);
}

} // namespace trajectory_algo
