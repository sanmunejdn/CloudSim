#include "TrajectoryOpParamSchema.h"

namespace trajectory_algo
{

TrajectoryOpParamField doubleParamField(
	const std::string& key,
	const std::string& labelEn,
	const std::string& labelZh,
	const std::string& unit,
	const double minValue,
	const double maxValue,
	const double step,
	const double defaultValue,
	const int order,
	const std::string& group)
{
	TrajectoryOpParamField field{};
	field.key = key;
	field.type = TrajectoryParamType::Double;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.unit = unit;
	field.minValue = minValue;
	field.maxValue = maxValue;
	field.step = step;
	field.defaultDouble = defaultValue;
	field.order = order;
	field.group = group;
	return field;
}

TrajectoryOpParamField intParamField(
	const std::string& key,
	const std::string& labelEn,
	const std::string& labelZh,
	const int minValue,
	const int maxValue,
	const int defaultValue,
	const int order,
	const std::string& group)
{
	TrajectoryOpParamField field{};
	field.key = key;
	field.type = TrajectoryParamType::Int;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.minInt = minValue;
	field.maxInt = maxValue;
	field.defaultInt = defaultValue;
	field.order = order;
	field.group = group;
	return field;
}

TrajectoryOpParamField enumParamField(
	const std::string& key,
	const std::string& labelEn,
	const std::string& labelZh,
	const std::vector<std::string>& values,
	const std::vector<std::string>& labelsZh,
	const std::vector<std::string>& labelsEn,
	const int defaultIndex,
	const int order,
	const std::string& group)
{
	TrajectoryOpParamField field{};
	field.key = key;
	field.type = TrajectoryParamType::Enum;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.enumValues = values;
	field.enumLabelsZh = labelsZh;
	field.enumLabelsEn = labelsEn;
	field.defaultInt = defaultIndex;
	field.order = order;
	field.group = group;
	return field;
}

TrajectoryOpParamField messageParamField(
	const std::string& key,
	const std::string& messageEn,
	const std::string& messageZh,
	const int order)
{
	TrajectoryOpParamField field{};
	field.key = key;
	field.type = TrajectoryParamType::Message;
	field.messageEn = messageEn;
	field.messageZh = messageZh;
	field.order = order;
	field.group = "hint";
	return field;
}

namespace
{
TrajectoryOpParamField scopePointFromField()
{
	TrajectoryOpParamField field = intParamField("scope.pointFrom", "P from", "P 起", 1, 9999, 1, 2, "scope");
	field.visibleWhenScopeKind = "PointIndexRange";
	return field;
}

TrajectoryOpParamField scopePointToField()
{
	TrajectoryOpParamField field = intParamField("scope.pointTo", "P to", "P 止", 1, 9999, 1, 3, "scope");
	field.visibleWhenScopeKind = "PointIndexRange";
	return field;
}

TrajectoryOpParamField scopeGroupIdField()
{
	TrajectoryOpParamField field = enumParamField(
		"scope.groupId",
		"Group",
		"分组",
		{},
		{},
		{},
		0,
		1,
		"scope");
	field.visibleWhenScopeKind = "Group";
	return field;
}
} // namespace

std::vector<TrajectoryOpParamField> trajectoryOpCommonScopeFields()
{
	return {
		enumParamField(
			"scope.kind",
			"Scope",
			"作用域",
			{ "0", "1", "2" },
			{ "全程序", "分组", "P 范围" },
			{ "Entire program", "Group", "Point range" },
			1,
			0,
			"scope"),
		scopeGroupIdField(),
		scopePointFromField(),
		scopePointToField(),
	};
}

} // namespace trajectory_algo
