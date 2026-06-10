#pragma once

#include <TrajectoryOpParamSchema.h>

#include <functional>
#include <string>

class QLabel;
class QWidget;

namespace trajectory_algo
{

struct TrajectoryParamBinding
{
	QLabel* label = nullptr;
	QWidget* widget = nullptr;
	TrajectoryOpParamField field{};
	std::function<bool(const TrajectoryParamValue&)> write;
	std::function<bool(TrajectoryParamValue&)> read;
	std::function<bool(double&, double&, double&)> readVec3;
	std::function<bool(double, double, double)> writeVec3;
};

class TrajectoryParamWidgetFactory
{
public:
	static TrajectoryParamBinding create(
		const TrajectoryOpParamField& field,
		bool useChinese);
};

} // namespace trajectory_algo
