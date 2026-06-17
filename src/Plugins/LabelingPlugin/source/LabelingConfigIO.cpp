#include "LabelingConfigIO.h"

#include <json.hpp>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>

namespace
{

QString resolvePathFromExe(const QString& relative)
{
	return QDir::cleanPath(QCoreApplication::applicationDirPath() + QStringLiteral("/") + relative);
}

QString resolveTrainingRoot(QString root)
{
	if (root.isEmpty())
	{
		root = QStringLiteral("../../CloudSim/tools/pointnet-training");
	}
	if (!QDir(root).isAbsolute())
	{
		root = resolvePathFromExe(root);
	}
	return QDir::cleanPath(root);
}

} // namespace

QString resolveLabelingConfigFilePath()
{
	const QString exeDir = QCoreApplication::applicationDirPath();
	const QString pluginCfg = QDir(exeDir).filePath(QStringLiteral("plugins/com.cloudsim.labeling/labeling_config.json"));
	if (QFile::exists(pluginCfg))
	{
		return pluginCfg;
	}
	const QString legacyCfg = QDir(exeDir).filePath(QStringLiteral("labeling_config.json"));
	if (QFile::exists(legacyCfg))
	{
		return legacyCfg;
	}
	return pluginCfg;
}

LabelingPluginConfig loadLabelingPluginConfig()
{
	LabelingPluginConfig out;
	out.configFilePath = resolveLabelingConfigFilePath();
	out.trainingRoot = resolveTrainingRoot({});
	out.defaultSegConfig = QStringLiteral("configs/seg_config.yaml");
	out.deployOnnxRel = QStringLiteral("models/pointnet_seg.onnx");
	out.deployConfigRel = QStringLiteral("plugins/com.cloudsim.pointnet/pointnet_config.json");

	if (!QFile::exists(out.configFilePath))
	{
		return out;
	}
	QFile f(out.configFilePath);
	if (!f.open(QIODevice::ReadOnly))
	{
		return out;
	}
	try
	{
		const nlohmann::json cfg = nlohmann::json::parse(f.readAll().constData(), nullptr, true);
		if (cfg.contains("python_executable"))
		{
			out.pythonExecutable = QString::fromStdString(cfg["python_executable"].get<std::string>());
		}
		if (cfg.contains("dataset_root"))
		{
			out.datasetRoot = QString::fromStdString(cfg["dataset_root"].get<std::string>());
		}
		if (cfg.contains("training_root"))
		{
			out.trainingRoot = resolveTrainingRoot(QString::fromStdString(cfg["training_root"].get<std::string>()));
		}
		if (cfg.contains("default_seg_config"))
		{
			out.defaultSegConfig = QString::fromStdString(cfg["default_seg_config"].get<std::string>());
		}
		if (cfg.contains("deploy"))
		{
			const auto& dep = cfg["deploy"];
			if (dep.contains("onnx_output"))
			{
				out.deployOnnxRel = QString::fromStdString(dep["onnx_output"].get<std::string>());
			}
			if (dep.contains("plugin_config"))
			{
				out.deployConfigRel = QString::fromStdString(dep["plugin_config"].get<std::string>());
			}
		}
	}
	catch (...)
	{
	}
	return out;
}

bool saveLabelingPluginConfig(const LabelingPluginConfig& cfg)
{
	nlohmann::json root = nlohmann::json::object();
	if (QFile::exists(cfg.configFilePath))
	{
		QFile in(cfg.configFilePath);
		if (in.open(QIODevice::ReadOnly))
		{
			try
			{
				root = nlohmann::json::parse(in.readAll().constData(), nullptr, true);
			}
			catch (...)
			{
				root = nlohmann::json::object();
			}
		}
	}

	root["python_executable"] = cfg.pythonExecutable.toStdString();
	root["dataset_root"] = cfg.datasetRoot.toStdString();
	if (!cfg.trainingRoot.isEmpty())
	{
		root["training_root"] = cfg.trainingRoot.toStdString();
	}
	if (!cfg.defaultSegConfig.isEmpty())
	{
		root["default_seg_config"] = cfg.defaultSegConfig.toStdString();
	}
	if (!root.contains("deploy"))
	{
		root["deploy"] = nlohmann::json::object();
	}
	if (!cfg.deployOnnxRel.isEmpty())
	{
		root["deploy"]["onnx_output"] = cfg.deployOnnxRel.toStdString();
	}
	if (!cfg.deployConfigRel.isEmpty())
	{
		root["deploy"]["plugin_config"] = cfg.deployConfigRel.toStdString();
	}

	QFileInfo info(cfg.configFilePath);
	QDir().mkpath(info.absolutePath());
	QFile out(cfg.configFilePath);
	if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		return false;
	}
	const std::string dumped = root.dump(2);
	out.write(dumped.data(), static_cast<int>(dumped.size()));
	return true;
}
