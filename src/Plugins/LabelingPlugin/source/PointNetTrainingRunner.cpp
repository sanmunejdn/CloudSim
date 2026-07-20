/// @file PointNetTrainingRunner.cpp
/// @brief PointNetTrainingRunner 实现

#include "PointNetTrainingRunner.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QTimer>

#include <json.hpp>

namespace
{
QString findExecutableOnPath(const QString& name)
{
	const QString pathEnv = qEnvironmentVariable("PATH");
#if defined(Q_OS_WIN)
	const QChar sep = QLatin1Char(';');
#else
	const QChar sep = QLatin1Char(':');
#endif
	for (const QString& part : pathEnv.split(sep, Qt::SkipEmptyParts))
	{
		const QString candidate = QDir(part).filePath(name);
		if (QFileInfo::exists(candidate))
		{
			return candidate;
		}
	}
	return {};
}

QString decodeProcessText(const QByteArray& raw)
{
	if (raw.isEmpty())
	{
		return {};
	}
	const QString asUtf8 = QString::fromUtf8(raw);
	if (!asUtf8.contains(QChar::ReplacementCharacter))
	{
		return asUtf8;
	}
	return QString::fromLocal8Bit(raw);
}

void applyPythonUtf8Env(QProcess* proc)
{
	if (!proc)
	{
		return;
	}
	QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
	env.insert(QStringLiteral("PYTHONIOENCODING"), QStringLiteral("utf-8"));
	env.insert(QStringLiteral("PYTHONUTF8"), QStringLiteral("1"));
	proc->setProcessEnvironment(env);
}

QStringList pythonLaunchArgs(const QString& scriptPath, const QStringList& scriptArgs)
{
#if defined(Q_OS_WIN)
	QStringList args;
	args << QStringLiteral("-X") << QStringLiteral("utf8") << scriptPath;
	args.append(scriptArgs);
	return args;
#else
	QStringList args;
	args << scriptPath;
	args.append(scriptArgs);
	return args;
#endif
}

bool startPythonProcess(QProcess* proc, const QString& workingDir, const QString& python, const QStringList& args,
						QString* errMsg)
{
	if (!proc)
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("Invalid process.");
		}
		return false;
	}
	applyPythonUtf8Env(proc);
	proc->setWorkingDirectory(workingDir);
	proc->setProgram(python);
	proc->setArguments(args);
	proc->start();
	if (proc->waitForStarted(10000))
	{
		return true;
	}
	if (errMsg)
	{
		*errMsg = proc->errorString();
		if (errMsg->isEmpty())
		{
			*errMsg = QStringLiteral("Failed to start Python process.");
		}
	}
	return false;
}

} // namespace

PointNetTrainingRunner::PointNetTrainingRunner(QObject* parent)
	: QObject(parent), m_process(new QProcess(this)), m_metricsTimer(new QTimer(this))
{
	m_metricsTimer->setInterval(1000);
	connect(m_process, &QProcess::readyReadStandardOutput, this, &PointNetTrainingRunner::onProcessReadyRead);
	connect(m_process, &QProcess::readyReadStandardError, this, &PointNetTrainingRunner::onProcessReadyRead);
	connect(m_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
			&PointNetTrainingRunner::onProcessFinished);
	connect(m_metricsTimer, &QTimer::timeout, this, &PointNetTrainingRunner::pollMetricsFile);
}

PointNetTrainingRunner::~PointNetTrainingRunner()
{
	if (m_metricsTimer)
	{
		m_metricsTimer->stop();
	}
	if (m_process && m_process->state() != QProcess::NotRunning)
	{
		m_process->kill();
		m_process->waitForFinished(3000);
	}
	// 数据集校验 / ONNX 导出等旁路 QProcess
	const QList<QProcess*> children = findChildren<QProcess*>(QString(), Qt::FindDirectChildrenOnly);
	for (QProcess* proc : children)
	{
		if (!proc || proc == m_process)
		{
			continue;
		}
		if (proc->state() != QProcess::NotRunning)
		{
			proc->kill();
			proc->waitForFinished(3000);
		}
	}
}

bool PointNetTrainingRunner::isRunning() const
{
	return m_process->state() != QProcess::NotRunning;
}

void PointNetTrainingRunner::setPythonExecutable(const QString& path)
{
	m_pythonExecutable = path;
}

void PointNetTrainingRunner::setTrainingRoot(const QString& path)
{
	m_trainingRoot = QDir::cleanPath(path);
}

void PointNetTrainingRunner::setDatasetRoot(const QString& path)
{
	m_datasetRoot = QDir::cleanPath(path);
}

void PointNetTrainingRunner::setOutputDir(const QString& path)
{
	m_outputDir = path;
}

void PointNetTrainingRunner::setNumClasses(int n)
{
	m_numClasses = n;
}

void PointNetTrainingRunner::setNumPoints(int n)
{
	m_numPoints = n;
}

void PointNetTrainingRunner::setEpochs(int n)
{
	m_epochs = n;
}

void PointNetTrainingRunner::setBatchSize(int n)
{
	m_batchSize = n;
}

void PointNetTrainingRunner::setLearningRate(double lr)
{
	m_learningRate = lr;
}

void PointNetTrainingRunner::setResumeCheckpoint(const QString& path)
{
	m_resumeCheckpoint = path;
}

void PointNetTrainingRunner::setDefaultSegConfig(const QString& path)
{
	m_defaultSegConfig = path;
}

QString PointNetTrainingRunner::resolvePython() const
{
	if (!m_pythonExecutable.isEmpty())
	{
		return m_pythonExecutable;
	}
#if defined(Q_OS_WIN)
	const QString py = findExecutableOnPath(QStringLiteral("python.exe"));
	if (!py.isEmpty())
	{
		return py;
	}
	return findExecutableOnPath(QStringLiteral("python"));
#else
	return findExecutableOnPath(QStringLiteral("python3"));
#endif
}

QString PointNetTrainingRunner::absoluteTrainingPath(const QString& relative) const
{
	if (QDir(relative).isAbsolute())
	{
		return QDir::cleanPath(relative);
	}
	return QDir::cleanPath(m_trainingRoot + QStringLiteral("/") + relative);
}

bool PointNetTrainingRunner::writeGeneratedConfig(QString* err)
{
	const QString baseConfig = absoluteTrainingPath(
		m_defaultSegConfig.isEmpty() ? QStringLiteral("configs/seg_config.yaml") : m_defaultSegConfig);
	if (!QFile::exists(baseConfig))
	{
		if (err)
		{
			*err = QStringLiteral("Missing base config: %1").arg(baseConfig);
		}
		return false;
	}
	QFile in(baseConfig);
	if (!in.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		if (err)
		{
			*err = QStringLiteral("Cannot read: %1").arg(baseConfig);
		}
		return false;
	}
	QString yaml = QString::fromUtf8(in.readAll());
	in.close();

	auto replaceField = [&yaml](const QString& key, const QString& value)
	{
		const QRegularExpression re(QStringLiteral("(?m)^(%1:\\s*).*$").arg(QRegularExpression::escape(key)));
		yaml.replace(re, QStringLiteral("\\1%1").arg(value));
	};
	replaceField(QStringLiteral("num_points"), QString::number(m_numPoints));
	replaceField(QStringLiteral("batch_size"), QString::number(m_batchSize));
	replaceField(QStringLiteral("epochs"), QString::number(m_epochs));
	replaceField(QStringLiteral("learning_rate"), QString::number(m_learningRate, 'g', 8));
	replaceField(QStringLiteral("num_classes"), QString::number(m_numClasses));
	replaceField(QStringLiteral("output_dir"), m_outputDir);

	const QString datasetLine = QStringLiteral("  root: %1").arg(QDir::toNativeSeparators(m_datasetRoot));
	yaml.replace(QRegularExpression(QStringLiteral("(?m)^\\s*root:\\s*.*$")), datasetLine);

	const QString outPath = absoluteTrainingPath(QStringLiteral("configs/seg_config.generated.yaml"));
	QDir().mkpath(QFileInfo(outPath).absolutePath());
	QFile out(outPath);
	if (!out.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		if (err)
		{
			*err = QStringLiteral("Cannot write: %1").arg(outPath);
		}
		return false;
	}
	out.write(yaml.toUtf8());
	out.close();
	m_generatedConfigPath = outPath;
	return true;
}

void PointNetTrainingRunner::validateDataset()
{
	if (isRunning())
	{
		emit validationFinished(false, QStringLiteral("Training already running."));
		return;
	}
	if (m_datasetRoot.isEmpty())
	{
		emit validationFinished(false, QStringLiteral("Dataset path is empty."));
		return;
	}
	const QString python = resolvePython();
	if (python.isEmpty())
	{
		emit validationFinished(false, QStringLiteral("Python executable not found."));
		return;
	}
	const QString script = absoluteTrainingPath(QStringLiteral("scripts/build_dataset.py"));
	if (!QFile::exists(script))
	{
		emit validationFinished(false, QStringLiteral("Missing build_dataset.py"));
		return;
	}
	auto* proc = new QProcess(this);
	connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
			[this, proc](int code, QProcess::ExitStatus)
			{
				const QString output =
					decodeProcessText(proc->readAllStandardOutput()) + decodeProcessText(proc->readAllStandardError());
				emit logLine(output.trimmed());
				emit validationFinished(code == 0, code == 0 ? QStringLiteral("Dataset OK.")
															 : QStringLiteral("Dataset validation failed."));
				proc->deleteLater();
			});
	const QStringList args =
		pythonLaunchArgs(script, {QStringLiteral("segmentation"), QStringLiteral("--root"), m_datasetRoot});
	QString err;
	if (!startPythonProcess(proc, m_trainingRoot, python, args, &err))
	{
		emit validationFinished(false, err);
		proc->deleteLater();
	}
}

void PointNetTrainingRunner::startTraining()
{
	if (isRunning())
	{
		return;
	}
	const QString python = resolvePython();
	if (python.isEmpty())
	{
		emit finished(false, QStringLiteral("Python executable not found."));
		return;
	}
	if (m_datasetRoot.isEmpty())
	{
		emit finished(false, QStringLiteral("Dataset path is empty."));
		return;
	}
	QString err;
	if (!writeGeneratedConfig(&err))
	{
		emit finished(false, err);
		return;
	}

	const QString jobScript = absoluteTrainingPath(QStringLiteral("scripts/run_seg_training_job.py"));
	const QString trainScript = absoluteTrainingPath(QStringLiteral("scripts/train_seg.py"));
	const QString script = QFile::exists(jobScript) ? jobScript : trainScript;

	m_metricsFilePath = absoluteTrainingPath(m_outputDir + QStringLiteral("/metrics.jsonl"));
	m_metricsFilePos = 0;
	if (QFile::exists(m_metricsFilePath))
	{
		QFile::remove(m_metricsFilePath);
	}

	QStringList args;
	if (script.endsWith(QStringLiteral("run_seg_training_job.py")))
	{
		QStringList scriptArgs;
		scriptArgs << QStringLiteral("--config") << m_generatedConfigPath << QStringLiteral("--metrics-file")
				   << m_metricsFilePath;
		if (!m_resumeCheckpoint.isEmpty())
		{
			scriptArgs << QStringLiteral("--resume") << m_resumeCheckpoint;
		}
		args = pythonLaunchArgs(script, scriptArgs);
	}
	else
	{
		QStringList scriptArgs;
		scriptArgs << QStringLiteral("--config") << m_generatedConfigPath;
		if (!m_resumeCheckpoint.isEmpty())
		{
			scriptArgs << QStringLiteral("--resume") << m_resumeCheckpoint;
		}
		args = pythonLaunchArgs(trainScript, scriptArgs);
	}

	applyPythonUtf8Env(m_process);
	m_process->setWorkingDirectory(m_trainingRoot);
	m_process->setProgram(python);
	m_process->setArguments(args);
	emit logLine(QStringLiteral("> %1 %2").arg(python, args.join(QLatin1Char(' '))));
	emit runningChanged(true);
	m_metricsTimer->start();
	m_process->start();
	if (!m_process->waitForStarted(10000))
	{
		m_metricsTimer->stop();
		emit runningChanged(false);
		QString err = m_process->errorString();
		if (err.isEmpty())
		{
			err = QStringLiteral("Failed to start training process.");
		}
		emit finished(false, err);
	}
}

void PointNetTrainingRunner::stopTraining()
{
	if (!isRunning())
	{
		return;
	}
	emit logLine(QStringLiteral("Stopping training process..."));
	m_process->kill();
}

void PointNetTrainingRunner::exportOnnx(const QString& checkpointPath, const QString& outputOnnxPath)
{
	const QString python = resolvePython();
	if (python.isEmpty())
	{
		emit exportFinished(false, QStringLiteral("Python executable not found."));
		return;
	}
	const QString script = absoluteTrainingPath(QStringLiteral("scripts/export_onnx.py"));
	if (!QFile::exists(script))
	{
		emit exportFinished(false, QStringLiteral("Missing export_onnx.py"));
		return;
	}
	auto* proc = new QProcess(this);
	connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
			[this, proc, outputOnnxPath](int code, QProcess::ExitStatus)
			{
				const QString output =
					decodeProcessText(proc->readAllStandardOutput()) + decodeProcessText(proc->readAllStandardError());
				emit logLine(output.trimmed());
				emit exportFinished(code == 0 && QFile::exists(outputOnnxPath),
									code == 0 ? QStringLiteral("ONNX exported.")
											  : QStringLiteral("ONNX export failed."));
				proc->deleteLater();
			});
	const QStringList args = pythonLaunchArgs(
		script, {QStringLiteral("--task"), QStringLiteral("seg"), QStringLiteral("--config"),
				 m_generatedConfigPath.isEmpty() ? absoluteTrainingPath(QStringLiteral("configs/seg_config.yaml"))
												 : m_generatedConfigPath,
				 QStringLiteral("--checkpoint"), checkpointPath, QStringLiteral("--output"), outputOnnxPath});
	QString err;
	if (!startPythonProcess(proc, m_trainingRoot, python, args, &err))
	{
		emit exportFinished(false, err);
		proc->deleteLater();
	}
}

void PointNetTrainingRunner::deployToPointNet(const QString& onnxPath, const QString& pointNetConfigPath,
											  int numClasses, int numPoints)
{
	if (!QFile::exists(onnxPath))
	{
		emit deployFinished(false, QStringLiteral("ONNX file not found."));
		return;
	}
	QFileInfo cfgInfo(pointNetConfigPath);
	QDir().mkpath(cfgInfo.absolutePath());
	const QString targetOnnx = cfgInfo.dir().filePath(QStringLiteral("models/pointnet_seg.onnx"));
	QDir().mkpath(QFileInfo(targetOnnx).absolutePath());
	if (QFile::exists(targetOnnx))
	{
		QFile::remove(targetOnnx);
	}
	if (!QFile::copy(onnxPath, targetOnnx))
	{
		emit deployFinished(false, QStringLiteral("Failed to copy ONNX."));
		return;
	}

	QFile f(pointNetConfigPath);
	nlohmann::json cfg;
	if (f.open(QIODevice::ReadOnly))
	{
		try
		{
			cfg = nlohmann::json::parse(f.readAll().constData(), nullptr, true);
		}
		catch (...)
		{
			cfg = nlohmann::json::object();
		}
		f.close();
	}
	if (!cfg.contains("models"))
	{
		cfg["models"] = nlohmann::json::object();
	}
	cfg["models"]["segment"]["path"] = "models/pointnet_seg.onnx";
	cfg["models"]["segment"]["num_points"] = numPoints;
	cfg["models"]["segment"]["num_classes"] = numClasses;
	if (!cfg.contains("inference"))
	{
		cfg["inference"] = nlohmann::json::object();
		cfg["inference"]["provider"] = "cpu";
	}
	QFile out(pointNetConfigPath);
	if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		emit deployFinished(false, QStringLiteral("Cannot write pointnet_config.json"));
		return;
	}
	const std::string dumped = cfg.dump(2);
	out.write(dumped.data(), static_cast<int>(dumped.size()));
	out.close();
	emit deployFinished(true, QStringLiteral("Deployed to %1").arg(pointNetConfigPath));
}

void PointNetTrainingRunner::onProcessReadyRead()
{
	const QString out = decodeProcessText(m_process->readAllStandardOutput());
	const QString err = decodeProcessText(m_process->readAllStandardError());
	for (const QString& line : out.split(QLatin1Char('\n'), Qt::SkipEmptyParts))
	{
		emit logLine(line);
		parseStdoutLine(line);
	}
	for (const QString& line : err.split(QLatin1Char('\n'), Qt::SkipEmptyParts))
	{
		emit logLine(line);
	}
}

void PointNetTrainingRunner::parseStdoutLine(const QString& line)
{
	static const QRegularExpression epochRe(
		QStringLiteral(R"(Epoch\s+(\d+)/(\d+).+train_loss=([\d.]+).+acc=([\d.]+).+val_loss=([\d.]+).+acc=([\d.]+))"));
	const QRegularExpressionMatch m = epochRe.match(line);
	if (!m.hasMatch())
	{
		return;
	}
	TrainingEpochMetrics metrics;
	metrics.epoch = m.captured(1).toInt();
	metrics.totalEpochs = m.captured(2).toInt();
	metrics.trainLoss = m.captured(3).toDouble();
	metrics.trainAcc = m.captured(4).toDouble();
	metrics.valLoss = m.captured(5).toDouble();
	metrics.valAcc = m.captured(6).toDouble();
	emit metricsReceived(metrics);
}

void PointNetTrainingRunner::pollMetricsFile()
{
	readNewMetricsLines();
}

void PointNetTrainingRunner::readNewMetricsLines()
{
	if (m_metricsFilePath.isEmpty() || !QFile::exists(m_metricsFilePath))
	{
		return;
	}
	QFile f(m_metricsFilePath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		return;
	}
	if (!f.seek(m_metricsFilePos))
	{
		return;
	}
	while (!f.atEnd())
	{
		const QByteArray line = f.readLine().trimmed();
		if (line.isEmpty())
		{
			continue;
		}
		const QJsonDocument doc = QJsonDocument::fromJson(line);
		if (!doc.isObject())
		{
			continue;
		}
		const QJsonObject obj = doc.object();
		TrainingEpochMetrics metrics;
		metrics.epoch = obj.value(QStringLiteral("epoch")).toInt();
		metrics.totalEpochs = obj.value(QStringLiteral("total_epochs")).toInt();
		metrics.trainLoss = obj.value(QStringLiteral("train_loss")).toDouble();
		metrics.trainAcc = obj.value(QStringLiteral("train_acc")).toDouble();
		metrics.valLoss = obj.value(QStringLiteral("val_loss")).toDouble();
		metrics.valAcc = obj.value(QStringLiteral("val_acc")).toDouble();
		metrics.lr = obj.value(QStringLiteral("lr")).toDouble();
		metrics.elapsedS = obj.value(QStringLiteral("elapsed_s")).toDouble();
		metrics.isBest = obj.value(QStringLiteral("best")).toBool();
		emit metricsReceived(metrics);
	}
	m_metricsFilePos = f.pos();
}

void PointNetTrainingRunner::onProcessFinished(int exitCode, QProcess::ExitStatus status)
{
	m_metricsTimer->stop();
	readNewMetricsLines();
	emit runningChanged(false);

	const QString summaryPath = absoluteTrainingPath(m_outputDir + QStringLiteral("/training_summary.json"));
	if (QFile::exists(summaryPath))
	{
		QFile f(summaryPath);
		if (f.open(QIODevice::ReadOnly))
		{
			const QJsonObject obj = QJsonDocument::fromJson(f.readAll()).object();
			m_lastResult.status = obj.value(QStringLiteral("status")).toString();
			m_lastResult.bestValAcc = obj.value(QStringLiteral("best_val_acc")).toDouble();
			m_lastResult.bestCheckpoint = obj.value(QStringLiteral("best_checkpoint")).toString();
			m_lastResult.device = obj.value(QStringLiteral("device")).toString();
		}
	}
	else
	{
		m_lastResult.status = exitCode == 0 ? QStringLiteral("completed") : QStringLiteral("failed");
		m_lastResult.bestCheckpoint = absoluteTrainingPath(m_outputDir + QStringLiteral("/best.pth"));
	}

	const bool ok = status == QProcess::NormalExit && exitCode == 0;
	emit finished(ok, ok ? QStringLiteral("Training finished.") : QStringLiteral("Training failed."));
}
