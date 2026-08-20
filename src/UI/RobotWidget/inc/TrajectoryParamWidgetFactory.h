#ifndef ROBOTWIDGET_TRAJECTORYPARAMWIDGETFACTORY_H
#define ROBOTWIDGET_TRAJECTORYPARAMWIDGETFACTORY_H

/// @file TrajectoryParamWidgetFactory.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief TrajectoryParamWidgetFactory 接口

#include <functional>
#include <string>

#include <TrajectoryOpParamSchema.h>

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
	static TrajectoryParamBinding create(const TrajectoryOpParamField& field, bool useChinese);
};

} // namespace trajectory_algo

#endif // ROBOTWIDGET_TRAJECTORYPARAMWIDGETFACTORY_H
