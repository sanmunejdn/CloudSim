#ifndef ROBOTWIDGET_FEATUREDISCRETIZERPARAMPANEL_H
#define ROBOTWIDGET_FEATUREDISCRETIZERPARAMPANEL_H

/// @file FeatureDiscretizerParamPanel.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief FeatureDiscretizerParamPanel 接口

#include "robotwidget_global.h"

#include <QWidget>
#include <functional>
#include <string>
#include <vector>

#include <FeatureListDocument.h>
#include <TrajectoryOpParamSchema.h>
#include <json.hpp>

class QLabel;
class QFormLayout;
class QWidget;

class ROBOTWIDGET_EXPORT FeatureDiscretizerParamPanel : public QWidget
{
	Q_OBJECT

public:
	explicit FeatureDiscretizerParamPanel(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setLoading(bool loading);
	void rebuildForStrategy(const std::string& strategyId);
	void loadParams(const nlohmann::json& params);
	bool applyParams(nlohmann::json& outParams) const;
	void clear();

	bool isRebuilding() const { return m_rebuilding; }

signals:
	void paramsChanged();

private:
	struct ParamBinding
	{
		QLabel* label = nullptr;
		QWidget* widget = nullptr;
		geoalgo::FeatureDiscretizerParamField field{};
		std::function<bool(trajectory_algo::TrajectoryParamValue&)> read;
		std::function<bool(const trajectory_algo::TrajectoryParamValue&)> write;
		std::function<bool(double&, double&, double&)> readVec3;
		std::function<bool(double, double, double)> writeVec3;
	};

	void clearRows();
	static trajectory_algo::TrajectoryOpParamField
	toTrajectoryField(const geoalgo::FeatureDiscretizerParamField& field);
	static nlohmann::json defaultParamsFromFields(const std::vector<geoalgo::FeatureDiscretizerParamField>& fields);
	static void writeJsonValue(nlohmann::json& params, const geoalgo::FeatureDiscretizerParamField& field,
							   const trajectory_algo::TrajectoryParamValue& value);
	static bool readJsonValue(const nlohmann::json& params, const geoalgo::FeatureDiscretizerParamField& field,
							  trajectory_algo::TrajectoryParamValue& out);

	bool m_useChinese = true;
	bool m_loading = false;
	bool m_clearingRows = false;
	bool m_rebuilding = false;
	std::string m_strategyId;
	QFormLayout* m_form = nullptr;
	std::vector<ParamBinding> m_rows;
};

#endif // ROBOTWIDGET_FEATUREDISCRETIZERPARAMPANEL_H
