#ifndef LABELINGPLUGIN_LABELINGTRAINWIDGET_H
#define LABELINGPLUGIN_LABELINGTRAINWIDGET_H

/// @file LabelingTrainWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief LabelingTrainWidget 接口

#include "PointNetTrainingRunner.h"

#include <QWidget>

class IPluginHostContext;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;

class LabelingTrainWidget : public QWidget
{
	Q_OBJECT

public:
	explicit LabelingTrainWidget(IPluginHostContext* host, QWidget* parent = nullptr);

	void applyLanguage();
	void setDatasetRoot(const QString& path);

private slots:
	void onBrowseDataset();
	void onBrowseOutput();
	void onBrowsePython();
	void onBrowseResume();
	void onValidateClicked();
	void onStartClicked();
	void onStopClicked();
	void onExportOnnxClicked();
	void onDeployClicked();
	void onRunnerLog(const QString& line);
	void onMetricsReceived(const TrainingEpochMetrics& metrics);
	void onRunnerFinished(bool success, const QString& message);
	void onRunningChanged(bool running);

private:
	QString i18n(const QString& en, const QString& zh) const;
	void loadLabelingConfig();
	void saveLabelingConfig();
	void syncRunnerConfig();
	void refreshBestResultCard();
	void appendLog(const QString& line);
	void notifyError(const QString& title, const QString& message);

	IPluginHostContext* m_host = nullptr;
	PointNetTrainingRunner* m_runner = nullptr;

	QString m_configFilePath;
	QString m_defaultSegConfig;
	QString m_deployOnnxRel;

	QGroupBox* m_configGroup = nullptr;
	QLineEdit* m_datasetEdit = nullptr;
	QLineEdit* m_outputEdit = nullptr;
	QSpinBox* m_numClassesSpin = nullptr;
	QSpinBox* m_numPointsSpin = nullptr;
	QSpinBox* m_epochsSpin = nullptr;
	QSpinBox* m_batchSpin = nullptr;
	QDoubleSpinBox* m_lrSpin = nullptr;
	QLineEdit* m_pythonEdit = nullptr;
	QLineEdit* m_resumeEdit = nullptr;

	QPushButton* m_validateBtn = nullptr;
	QPushButton* m_startBtn = nullptr;
	QPushButton* m_stopBtn = nullptr;
	QPushButton* m_exportOnnxBtn = nullptr;
	QPushButton* m_deployBtn = nullptr;

	QLabel* m_statusLabel = nullptr;
	QLabel* m_bestResultLabel = nullptr;
	QPlainTextEdit* m_logEdit = nullptr;
	QTableWidget* m_metricsTable = nullptr;

	QString m_deployConfigRel;
};

#endif // LABELINGPLUGIN_LABELINGTRAINWIDGET_H
