#pragma once

#include "robotwidget_global.h"

#include <FeatureListDocument.h>
#include <TrajectoryOpParamSchema.h>

#include <json.hpp>

#include <QWidget>

#include <functional>
#include <string>
#include <vector>

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
	static trajectory_algo::TrajectoryOpParamField toTrajectoryField(
		const geoalgo::FeatureDiscretizerParamField& field);
	static nlohmann::json defaultParamsFromFields(
		const std::vector<geoalgo::FeatureDiscretizerParamField>& fields);
	static void writeJsonValue(
		nlohmann::json& params,
		const geoalgo::FeatureDiscretizerParamField& field,
		const trajectory_algo::TrajectoryParamValue& value);
	static bool readJsonValue(
		const nlohmann::json& params,
		const geoalgo::FeatureDiscretizerParamField& field,
		trajectory_algo::TrajectoryParamValue& out);

	bool m_useChinese = true;
	bool m_loading = false;
	bool m_clearingRows = false;
	bool m_rebuilding = false;
	std::string m_strategyId;
	QFormLayout* m_form = nullptr;
	std::vector<ParamBinding> m_rows;
};
