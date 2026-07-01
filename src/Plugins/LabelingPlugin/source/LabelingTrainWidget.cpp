#include "LabelingTrainWidget.h"

#include "IPluginHostContext.h"
#include "LabelingConfigIO.h"

#include <QCoreApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>

#include <climits>

namespace
{

QString resolvePathFromExe(const QString& relative)
{
	const QString exeDir = QCoreApplication::applicationDirPath();
	return QDir::cleanPath(exeDir + QStringLiteral("/") + relative);
}

} // namespace

LabelingTrainWidget::LabelingTrainWidget(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent)
	, m_host(host)
	, m_runner(new PointNetTrainingRunner(this))
{
	auto* outer = new QVBoxLayout(this);
	outer->setContentsMargins(0, 0, 0, 0);
	auto* scroll = new QScrollArea(this);
	scroll->setWidgetResizable(true);
	auto* content = new QWidget(scroll);
	auto* layout = new QVBoxLayout(content);
	layout->setContentsMargins(12, 12, 12, 12);
	layout->setSpacing(8);
	scroll->setWidget(content);
	outer->addWidget(scroll);

	m_configGroup = new QGroupBox(content);
	auto* cfgLayout = new QVBoxLayout(m_configGroup);
	auto addRow = [cfgLayout](const QString& label, QWidget* field) {
		auto* row = new QHBoxLayout;
		row->addWidget(new QLabel(label));
		row->addWidget(field, 1);
		cfgLayout->addLayout(row);
	};
	m_datasetEdit = new QLineEdit(m_configGroup);
	m_outputEdit = new QLineEdit(m_configGroup);
	m_numClassesSpin = new QSpinBox(m_configGroup);
	m_numPointsSpin = new QSpinBox(m_configGroup);
	m_epochsSpin = new QSpinBox(m_configGroup);
	m_batchSpin = new QSpinBox(m_configGroup);
	m_lrSpin = new QDoubleSpinBox(m_configGroup);
	m_pythonEdit = new QLineEdit(m_configGroup);
	m_resumeEdit = new QLineEdit(m_configGroup);
	m_numClassesSpin->setRange(2, 64);
	m_numClassesSpin->setValue(4);
	m_numPointsSpin->setRange(256, INT_MAX);
	m_numPointsSpin->setValue(2048);
	m_epochsSpin->setRange(1, 10000);
	m_epochsSpin->setValue(100);
	m_batchSpin->setRange(1, 256);
	m_batchSpin->setValue(16);
	m_lrSpin->setDecimals(6);
	m_lrSpin->setRange(1e-6, 1.0);
	m_lrSpin->setValue(0.001);
	m_outputEdit->setText(QStringLiteral("output/seg"));
	addRow(QStringLiteral("Dataset"), m_datasetEdit);
	addRow(QStringLiteral("Output"), m_outputEdit);
	addRow(QStringLiteral("Classes"), m_numClassesSpin);
	addRow(QStringLiteral("Points"), m_numPointsSpin);
	addRow(QStringLiteral("Epochs"), m_epochsSpin);
	addRow(QStringLiteral("Batch"), m_batchSpin);
	addRow(QStringLiteral("LR"), m_lrSpin);
	addRow(QStringLiteral("Python"), m_pythonEdit);
	addRow(QStringLiteral("Resume"), m_resumeEdit);
	layout->addWidget(m_configGroup);

	auto* btnRow = new QHBoxLayout;
	m_validateBtn = new QPushButton(content);
	m_startBtn = new QPushButton(content);
	m_stopBtn = new QPushButton(content);
	m_exportOnnxBtn = new QPushButton(content);
	m_deployBtn = new QPushButton(content);
	btnRow->addWidget(m_validateBtn);
	btnRow->addWidget(m_startBtn);
	btnRow->addWidget(m_stopBtn);
	btnRow->addWidget(m_exportOnnxBtn);
	btnRow->addWidget(m_deployBtn);
	layout->addLayout(btnRow);

	m_statusLabel = new QLabel(content);
	m_bestResultLabel = new QLabel(content);
	m_bestResultLabel->setWordWrap(true);
	layout->addWidget(m_statusLabel);
	layout->addWidget(m_bestResultLabel);

	m_logEdit = new QPlainTextEdit(content);
	m_logEdit->setReadOnly(true);
	m_logEdit->setMaximumBlockCount(5000);
	layout->addWidget(m_logEdit);

	m_metricsTable = new QTableWidget(0, 7, content);
	m_metricsTable->setHorizontalHeaderLabels(
		{ QStringLiteral("Epoch"), QStringLiteral("TrainLoss"), QStringLiteral("TrainAcc"),
			QStringLiteral("ValLoss"), QStringLiteral("ValAcc"), QStringLiteral("LR"), QStringLiteral("Time(s)") });
	m_metricsTable->horizontalHeader()->setStretchLastSection(true);
	layout->addWidget(m_metricsTable);

	loadLabelingConfig();
	applyLanguage();
	syncRunnerConfig();

	connect(m_validateBtn, &QPushButton::clicked, this, &LabelingTrainWidget::onValidateClicked);
	connect(m_startBtn, &QPushButton::clicked, this, &LabelingTrainWidget::onStartClicked);
	connect(m_stopBtn, &QPushButton::clicked, this, &LabelingTrainWidget::onStopClicked);
	connect(m_exportOnnxBtn, &QPushButton::clicked, this, &LabelingTrainWidget::onExportOnnxClicked);
	connect(m_deployBtn, &QPushButton::clicked, this, &LabelingTrainWidget::onDeployClicked);
	connect(m_runner, &PointNetTrainingRunner::logLine, this, &LabelingTrainWidget::onRunnerLog);
	connect(m_runner, &PointNetTrainingRunner::metricsReceived, this, &LabelingTrainWidget::onMetricsReceived);
	connect(m_runner, &PointNetTrainingRunner::finished, this, &LabelingTrainWidget::onRunnerFinished);
	connect(m_runner, &PointNetTrainingRunner::runningChanged, this, &LabelingTrainWidget::onRunningChanged);
	connect(m_runner, &PointNetTrainingRunner::validationFinished, this, [this](bool ok, const QString& msg) {
		appendLog(msg);
		if (m_host)
		{
			(ok ? m_host->logInfo(msg) : m_host->logWarn(msg));
		}
		if (!ok)
		{
			notifyError(i18n(QStringLiteral("Validate Dataset"), QStringLiteral("校验数据集")), msg);
		}
	});
	connect(m_runner, &PointNetTrainingRunner::exportFinished, this, [this](bool ok, const QString& msg) {
		appendLog(msg);
		if (m_host)
		{
			(ok ? m_host->logInfo(msg) : m_host->logWarn(msg));
		}
		if (!ok)
		{
			notifyError(i18n(QStringLiteral("Export ONNX"), QStringLiteral("导出 ONNX")), msg);
		}
	});
	connect(m_runner, &PointNetTrainingRunner::deployFinished, this, [this](bool ok, const QString& msg) {
		appendLog(msg);
		if (m_host)
		{
			(ok ? m_host->logInfo(msg) : m_host->logWarn(msg));
		}
		if (!ok)
		{
			notifyError(i18n(QStringLiteral("Deploy"), QStringLiteral("部署")), msg);
		}
	});

	connect(m_pythonEdit, &QLineEdit::editingFinished, this, &LabelingTrainWidget::saveLabelingConfig);
	connect(m_datasetEdit, &QLineEdit::editingFinished, this, &LabelingTrainWidget::saveLabelingConfig);
}

QString LabelingTrainWidget::i18n(const QString& en, const QString& zh) const
{
	return m_host && m_host->useChinese() ? zh : en;
}

void LabelingTrainWidget::applyLanguage()
{
	m_configGroup->setTitle(i18n(QStringLiteral("Training Config"), QStringLiteral("训练配置")));
	m_validateBtn->setText(i18n(QStringLiteral("Validate Dataset"), QStringLiteral("校验数据集")));
	m_startBtn->setText(i18n(QStringLiteral("Start Training"), QStringLiteral("开始训练")));
	m_stopBtn->setText(i18n(QStringLiteral("Stop"), QStringLiteral("停止训练")));
	m_exportOnnxBtn->setText(i18n(QStringLiteral("Export ONNX"), QStringLiteral("导出 ONNX")));
	m_deployBtn->setText(i18n(QStringLiteral("Deploy to PointNet"), QStringLiteral("部署到 PointNet")));
	m_statusLabel->setText(i18n(QStringLiteral("Status: Idle"), QStringLiteral("状态：空闲")));
	m_metricsTable->setHorizontalHeaderLabels(
		{ i18n(QStringLiteral("Epoch"), QStringLiteral("轮次")),
			i18n(QStringLiteral("TrainLoss"), QStringLiteral("训练损失")),
			i18n(QStringLiteral("TrainAcc"), QStringLiteral("训练精度")),
			i18n(QStringLiteral("ValLoss"), QStringLiteral("验证损失")),
			i18n(QStringLiteral("ValAcc"), QStringLiteral("验证精度")),
			i18n(QStringLiteral("LR"), QStringLiteral("学习率")),
			i18n(QStringLiteral("Time(s)"), QStringLiteral("耗时(s)")) });
	refreshBestResultCard();
}

void LabelingTrainWidget::loadLabelingConfig()
{
	const LabelingPluginConfig cfg = loadLabelingPluginConfig();
	m_configFilePath = cfg.configFilePath;
	m_pythonEdit->setText(cfg.pythonExecutable);
	m_datasetEdit->setText(cfg.datasetRoot);
	m_runner->setTrainingRoot(cfg.trainingRoot);
	m_runner->setDefaultSegConfig(cfg.defaultSegConfig);
	m_defaultSegConfig = cfg.defaultSegConfig;
	m_deployOnnxRel = cfg.deployOnnxRel;
	m_deployConfigRel = cfg.deployConfigRel;
}

void LabelingTrainWidget::saveLabelingConfig()
{
	if (m_configFilePath.isEmpty())
	{
		m_configFilePath = resolveLabelingConfigFilePath();
	}
	LabelingPluginConfig cfg;
	cfg.configFilePath = m_configFilePath;
	cfg.pythonExecutable = m_pythonEdit->text().trimmed();
	cfg.datasetRoot = m_datasetEdit->text().trimmed();
	cfg.trainingRoot = m_runner->trainingRoot();
	cfg.defaultSegConfig = m_defaultSegConfig;
	cfg.deployOnnxRel = m_deployOnnxRel;
	cfg.deployConfigRel = m_deployConfigRel;
	(void)saveLabelingPluginConfig(cfg);
}

void LabelingTrainWidget::syncRunnerConfig()
{
	m_runner->setPythonExecutable(m_pythonEdit->text().trimmed());
	m_runner->setDatasetRoot(m_datasetEdit->text().trimmed());
	m_runner->setOutputDir(m_outputEdit->text().trimmed());
	m_runner->setNumClasses(m_numClassesSpin->value());
	m_runner->setNumPoints(m_numPointsSpin->value());
	m_runner->setEpochs(m_epochsSpin->value());
	m_runner->setBatchSize(m_batchSpin->value());
	m_runner->setLearningRate(m_lrSpin->value());
	m_runner->setResumeCheckpoint(m_resumeEdit->text().trimmed());
}

void LabelingTrainWidget::setDatasetRoot(const QString& path)
{
	m_datasetEdit->setText(path);
	syncRunnerConfig();
	saveLabelingConfig();
}

void LabelingTrainWidget::notifyError(const QString& title, const QString& message)
{
	if (message.isEmpty())
	{
		return;
	}
	QMessageBox::warning(this, title, message);
}

void LabelingTrainWidget::onBrowseDataset()
{
	const QString dir = QFileDialog::getExistingDirectory(this, i18n(QStringLiteral("Dataset"), QStringLiteral("数据集")));
	if (!dir.isEmpty())
	{
		m_datasetEdit->setText(dir);
		saveLabelingConfig();
	}
}

void LabelingTrainWidget::onBrowseOutput()
{
	const QString dir = QFileDialog::getExistingDirectory(this, i18n(QStringLiteral("Output"), QStringLiteral("输出目录")));
	if (!dir.isEmpty())
	{
		m_outputEdit->setText(dir);
	}
}

void LabelingTrainWidget::onBrowsePython()
{
	const QString path = QFileDialog::getOpenFileName(this, i18n(QStringLiteral("Python"), QStringLiteral("Python 解释器")));
	if (!path.isEmpty())
	{
		m_pythonEdit->setText(path);
		saveLabelingConfig();
	}
}

void LabelingTrainWidget::onBrowseResume()
{
	const QString path = QFileDialog::getOpenFileName(
		this,
		i18n(QStringLiteral("Checkpoint"), QStringLiteral("断点文件")),
		m_runner->trainingRoot(),
		QStringLiteral("PyTorch (*.pth)"));
	if (!path.isEmpty())
	{
		m_resumeEdit->setText(path);
	}
}

void LabelingTrainWidget::onValidateClicked()
{
	syncRunnerConfig();
	saveLabelingConfig();
	m_runner->validateDataset();
}

void LabelingTrainWidget::onStartClicked()
{
	syncRunnerConfig();
	saveLabelingConfig();
	m_metricsTable->setRowCount(0);
	m_statusLabel->setText(i18n(QStringLiteral("Status: Running"), QStringLiteral("状态：运行中")));
	m_runner->startTraining();
}

void LabelingTrainWidget::onStopClicked()
{
	m_runner->stopTraining();
}

void LabelingTrainWidget::onExportOnnxClicked()
{
	syncRunnerConfig();
	const TrainingJobResult result = m_runner->lastResult();
	QString checkpoint = result.bestCheckpoint;
	if (checkpoint.isEmpty())
	{
		checkpoint = QDir(m_runner->trainingRoot()).filePath(m_outputEdit->text().trimmed() + QStringLiteral("/best.pth"));
	}
	const QString outOnnx = QDir(m_runner->trainingRoot()).filePath(
		m_deployOnnxRel.isEmpty() ? QStringLiteral("models/pointnet_seg.onnx") : m_deployOnnxRel);
	m_runner->exportOnnx(checkpoint, outOnnx);
}

void LabelingTrainWidget::onDeployClicked()
{
	syncRunnerConfig();
	const QString onnxPath = QDir(m_runner->trainingRoot()).filePath(
		m_deployOnnxRel.isEmpty() ? QStringLiteral("models/pointnet_seg.onnx") : m_deployOnnxRel);
	const QString cfgPath = resolvePathFromExe(
		m_deployConfigRel.isEmpty() ? QStringLiteral("plugins/com.cloudsim.pointnet/pointnet_config.json") : m_deployConfigRel);
	m_runner->deployToPointNet(onnxPath, cfgPath, m_numClassesSpin->value(), m_numPointsSpin->value());
}

void LabelingTrainWidget::appendLog(const QString& line)
{
	if (line.isEmpty())
	{
		return;
	}
	m_logEdit->appendPlainText(line);
	if (m_host)
	{
		m_host->logInfo(line);
	}
}

void LabelingTrainWidget::onRunnerLog(const QString& line)
{
	appendLog(line);
}

void LabelingTrainWidget::onMetricsReceived(const TrainingEpochMetrics& metrics)
{
	for (int row = 0; row < m_metricsTable->rowCount(); ++row)
	{
		if (m_metricsTable->item(row, 0)->text().toInt() == metrics.epoch)
		{
			m_metricsTable->item(row, 1)->setText(QString::number(metrics.trainLoss, 'f', 4));
			m_metricsTable->item(row, 2)->setText(QString::number(metrics.trainAcc, 'f', 4));
			m_metricsTable->item(row, 3)->setText(QString::number(metrics.valLoss, 'f', 4));
			m_metricsTable->item(row, 4)->setText(QString::number(metrics.valAcc, 'f', 4));
			m_metricsTable->item(row, 5)->setText(QString::number(metrics.lr, 'g', 4));
			m_metricsTable->item(row, 6)->setText(QString::number(metrics.elapsedS, 'f', 1));
			if (metrics.isBest)
			{
				refreshBestResultCard();
			}
			return;
		}
	}
	const int row = m_metricsTable->rowCount();
	m_metricsTable->insertRow(row);
	m_metricsTable->setItem(row, 0, new QTableWidgetItem(QString::number(metrics.epoch)));
	m_metricsTable->setItem(row, 1, new QTableWidgetItem(QString::number(metrics.trainLoss, 'f', 4)));
	m_metricsTable->setItem(row, 2, new QTableWidgetItem(QString::number(metrics.trainAcc, 'f', 4)));
	m_metricsTable->setItem(row, 3, new QTableWidgetItem(QString::number(metrics.valLoss, 'f', 4)));
	m_metricsTable->setItem(row, 4, new QTableWidgetItem(QString::number(metrics.valAcc, 'f', 4)));
	m_metricsTable->setItem(row, 5, new QTableWidgetItem(QString::number(metrics.lr, 'g', 4)));
	m_metricsTable->setItem(row, 6, new QTableWidgetItem(QString::number(metrics.elapsedS, 'f', 1)));
	if (metrics.isBest)
	{
		refreshBestResultCard();
	}
}

void LabelingTrainWidget::refreshBestResultCard()
{
	const TrainingJobResult result = m_runner->lastResult();
	if (result.bestCheckpoint.isEmpty() && result.bestValAcc <= 0.0)
	{
		m_bestResultLabel->setText(i18n(QStringLiteral("Best result: —"), QStringLiteral("最佳结果：—")));
		return;
	}
	m_bestResultLabel->setText(
		i18n(QStringLiteral("Best val acc: %1 | Checkpoint: %2 | Device: %3"),
			 QStringLiteral("最佳验证精度: %1 | 权重: %2 | 设备: %3"))
			.arg(result.bestValAcc, 0, 'f', 4)
			.arg(result.bestCheckpoint.isEmpty() ? QStringLiteral("—") : result.bestCheckpoint)
			.arg(result.device.isEmpty() ? QStringLiteral("—") : result.device));
}

void LabelingTrainWidget::onRunnerFinished(bool success, const QString& message)
{
	m_statusLabel->setText(success ? i18n(QStringLiteral("Status: Completed"), QStringLiteral("状态：已完成"))
								   : i18n(QStringLiteral("Status: Failed"), QStringLiteral("状态：失败")));
	appendLog(message);
	if (!success)
	{
		notifyError(i18n(QStringLiteral("Training"), QStringLiteral("训练")), message);
	}
	refreshBestResultCard();
}

void LabelingTrainWidget::onRunningChanged(bool running)
{
	m_startBtn->setEnabled(!running);
	m_stopBtn->setEnabled(running);
	if (running)
	{
		m_statusLabel->setText(i18n(QStringLiteral("Status: Running"), QStringLiteral("状态：运行中")));
	}
}
