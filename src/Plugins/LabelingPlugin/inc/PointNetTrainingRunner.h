#ifndef LABELINGPLUGIN_POINTNETTRAININGRUNNER_H
#define LABELINGPLUGIN_POINTNETTRAININGRUNNER_H

/// @file PointNetTrainingRunner.h
/// @brief PointNetTrainingRunner 接口

#include <QObject>
#include <QProcess>
#include <QString>
#include <QTimer>

struct TrainingEpochMetrics
{
	int epoch = 0;
	int totalEpochs = 0;
	double trainLoss = 0.0;
	double trainAcc = 0.0;
	double valLoss = 0.0;
	double valAcc = 0.0;
	double lr = 0.0;
	double elapsedS = 0.0;
	bool isBest = false;
};

struct TrainingJobResult
{
	QString status;
	double bestValAcc = 0.0;
	QString bestCheckpoint;
	QString device;
};

class PointNetTrainingRunner : public QObject
{
	Q_OBJECT

public:
	explicit PointNetTrainingRunner(QObject* parent = nullptr);
	~PointNetTrainingRunner() override;

	bool isRunning() const;
	QString trainingRoot() const { return m_trainingRoot; }
	QString outputDir() const { return m_outputDir; }
	QString metricsFilePath() const { return m_metricsFilePath; }
	QString generatedConfigPath() const { return m_generatedConfigPath; }
	TrainingJobResult lastResult() const { return m_lastResult; }

	void setPythonExecutable(const QString& path);
	void setTrainingRoot(const QString& path);
	void setDatasetRoot(const QString& path);
	void setOutputDir(const QString& path);
	void setNumClasses(int n);
	void setNumPoints(int n);
	void setEpochs(int n);
	void setBatchSize(int n);
	void setLearningRate(double lr);
	void setResumeCheckpoint(const QString& path);
	void setDefaultSegConfig(const QString& path);

public slots:
	void validateDataset();
	void startTraining();
	void stopTraining();
	void exportOnnx(const QString& checkpointPath, const QString& outputOnnxPath);
	void deployToPointNet(const QString& onnxPath, const QString& pointNetConfigPath, int numClasses, int numPoints);

signals:
	void logLine(const QString& line);
	void metricsReceived(const TrainingEpochMetrics& metrics);
	void runningChanged(bool running);
	void finished(bool success, const QString& message);
	void validationFinished(bool ok, const QString& message);
	void exportFinished(bool ok, const QString& message);
	void deployFinished(bool ok, const QString& message);

private slots:
	void onProcessReadyRead();
	void onProcessFinished(int exitCode, QProcess::ExitStatus status);
	void pollMetricsFile();

private:
	QString resolvePython() const;
	QString absoluteTrainingPath(const QString& relative) const;
	bool writeGeneratedConfig(QString* err);
	void parseStdoutLine(const QString& line);
	void readNewMetricsLines();

	QProcess* m_process = nullptr;
	QTimer* m_metricsTimer = nullptr;
	QString m_pythonExecutable;
	QString m_trainingRoot;
	QString m_datasetRoot;
	QString m_outputDir;
	QString m_metricsFilePath;
	QString m_generatedConfigPath;
	QString m_defaultSegConfig;
	QString m_resumeCheckpoint;
	int m_numClasses = 4;
	int m_numPoints = 2048;
	int m_epochs = 100;
	int m_batchSize = 16;
	double m_learningRate = 0.001;
	qint64 m_metricsFilePos = 0;
	TrainingJobResult m_lastResult;
};

#endif // LABELINGPLUGIN_POINTNETTRAININGRUNNER_H
