#ifndef ROBOTWIDGET_TRAJECTORYPLANCONFIRMDIALOG_H
#define ROBOTWIDGET_TRAJECTORYPLANCONFIRMDIALOG_H

/// @file TrajectoryPlanConfirmDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AI 轨迹离散：策略/参数/管线算子模态确认

#include "robotwidget_global.h"

#include <QByteArray>
#include <QDialog>

#include <FeatureListDocument.h>
#include <json.hpp>

class FeatureDiscretizerParamPanel;
class TrajectoryOpParamPanel;
class TrajectoryPipelineListWidget;
class QComboBox;
class QListWidget;
class QPushButton;

class ROBOTWIDGET_EXPORT TrajectoryPlanConfirmDialog : public QDialog
{
	Q_OBJECT

public:
	enum class Outcome
	{
		Accepted = 1,
		Cancelled = 0,
		Retry = 2
	};

	explicit TrajectoryPlanConfirmDialog(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setShowRetry(bool show);
	/// plan JSON v2（features + pipeline）；失败返回 false
	bool loadPlan(const QByteArray& planJsonUtf8, QString* err = nullptr);
	QByteArray resultPlanJson() const;
	Outcome outcome() const { return m_outcome; }

private slots:
	void onFeatureRowChanged(int row);
	void onStrategyChanged(int index);
	void onDiscretizeParamsChanged();
	void onPipelineSelectionChanged(int index);
	void onPipelineOpsChanged();
	void onOpParamsChanged();
	void onAddOpClicked();
	void onRemoveOpClicked();
	void onMoveOpUp();
	void onMoveOpDown();
	void onAcceptClicked();
	void onCancelClicked();
	void onRetryClicked();

private:
	void rebuildFeatureList();
	void refreshStrategyComboForCurrent();
	void loadDiscretizePanelForCurrent();
	void flushCurrentFeatureParams();
	void flushSelectedOpParams();
	void rebuildOpKindCombo();
	geoalgo::GeometryAffinity affinityForCurrentFeature() const;

	bool m_useChinese = true;
	bool m_loading = false;
	int m_lastFeatureRow = -1;
	Outcome m_outcome = Outcome::Cancelled;
	nlohmann::json m_plan;

	QListWidget* m_featureList = nullptr;
	QComboBox* m_strategyCombo = nullptr;
	FeatureDiscretizerParamPanel* m_discretizePanel = nullptr;
	TrajectoryPipelineListWidget* m_pipeline = nullptr;
	QComboBox* m_opKindCombo = nullptr;
	TrajectoryOpParamPanel* m_opParamPanel = nullptr;
	QPushButton* m_retryBtn = nullptr;
};

/// 打开确认框前补全 strategy/params，并按模板展开 pipeline
ROBOTWIDGET_EXPORT bool enrichTrajectoryPlanJsonInPlace(nlohmann::json& plan, QString* err = nullptr);

#endif // ROBOTWIDGET_TRAJECTORYPLANCONFIRMDIALOG_H
