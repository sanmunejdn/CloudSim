#pragma once

#include "trajectory_algorithm_global.h"

#include <string>
#include <vector>

namespace trajectory_algo
{

enum class TrajectoryParamType
{
	Double = 0,
	Int,
	Bool,
	Enum,
	Vec3,
	Message
};

enum class TrajectoryParamStorage
{
	DescriptorMember = 0,
	ExtensionBag
};

struct TRAJECTORY_ALGORITHM_API TrajectoryOpParamField
{
	std::string key;
	TrajectoryParamType type = TrajectoryParamType::Double;
	std::string labelEn;
	std::string labelZh;
	std::string unit;
	std::string group;
	int order = 0;

	double minValue = -1e6;
	double maxValue = 1e6;
	double step = 1.0;
	int minInt = 0;
	int maxInt = 9999;
	double defaultDouble = 0.0;
	int defaultInt = 1;
	bool defaultBool = false;

	std::vector<std::string> enumValues;
	std::vector<std::string> enumLabelsZh;
	std::vector<std::string> enumLabelsEn;

	std::string vec3SuffixX = ".x";
	std::string vec3SuffixY = ".y";
	std::string vec3SuffixZ = ".z";

	std::string visibleWhenScopeKind;
	/// 当另一参数字段等于 visibleWhenIntValue 时显示（如 directionMode==Custom）
	std::string visibleWhenFieldKey;
	int visibleWhenIntValue = -1;
	std::string messageEn;
	std::string messageZh;

	TrajectoryParamStorage storage = TrajectoryParamStorage::DescriptorMember;
};

struct TRAJECTORY_ALGORITHM_API TrajectoryParamValue
{
	enum class Kind
	{
		Double,
		Int,
		Bool,
		String
	};

	Kind kind = Kind::Double;
	double asDouble = 0.0;
	int asInt = 0;
	bool asBool = false;
	std::string asString;
};

TRAJECTORY_ALGORITHM_API TrajectoryOpParamField doubleParamField(
	const std::string& key,
	const std::string& labelEn,
	const std::string& labelZh,
	const std::string& unit,
	double minValue,
	double maxValue,
	double step,
	double defaultValue,
	int order = 0,
	const std::string& group = "transform");

TRAJECTORY_ALGORITHM_API TrajectoryOpParamField intParamField(
	const std::string& key,
	const std::string& labelEn,
	const std::string& labelZh,
	int minValue,
	int maxValue,
	int defaultValue,
	int order = 0,
	const std::string& group = "transform");

TRAJECTORY_ALGORITHM_API TrajectoryOpParamField enumParamField(
	const std::string& key,
	const std::string& labelEn,
	const std::string& labelZh,
	const std::vector<std::string>& values,
	const std::vector<std::string>& labelsZh,
	const std::vector<std::string>& labelsEn,
	int defaultIndex,
	int order = 0,
	const std::string& group = "transform");

TRAJECTORY_ALGORITHM_API TrajectoryOpParamField messageParamField(
	const std::string& key,
	const std::string& messageEn,
	const std::string& messageZh,
	int order = 0);

TRAJECTORY_ALGORITHM_API std::vector<TrajectoryOpParamField> trajectoryOpCommonScopeFields();

} // namespace trajectory_algo
