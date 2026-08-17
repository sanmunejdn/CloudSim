/// @file PointNetPlugin.cpp
/// @brief PointNet++ 插件入口与 AI 域注册

#include "PointNetPlugin.h"

#include "IAiAssistantHost.h"
#include "IPluginHostContext.h"
#include "PointNetDomainHandler.h"
#include "PointNetInference.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <json.hpp>

PointNetPlugin::PointNetPlugin()
{
	m_inference = std::make_unique<PointNetInference>();
}

PointNetPlugin::~PointNetPlugin() = default;

QString PointNetPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.pointnet");
}

QString PointNetPlugin::displayName() const
{
	return QStringLiteral("PointNet++ AI");
}

bool PointNetPlugin::initialize(IPluginHostContext* host)
{
	m_host = host;
	return true;
}

void PointNetPlugin::shutdown()
{
	shutdownAi();
	m_host = nullptr;
}

QString PointNetPlugin::aiPluginId() const
{
	return pluginId();
}

bool PointNetPlugin::initializeAi(IPluginHostContext* host, IAiAssistantHost* aiHost)
{
	m_host = host;
	m_aiHost = aiHost;

	QString configErr;
	if (!loadConfig(&configErr))
	{
		if (m_host)
			m_host->logWarn(QStringLiteral("[PointNet] 配置加载失败: %1").arg(configErr));
	}

	// 推理类域由 UI 直接 execute，不经 LLM 解析链
	m_clsHandler = std::make_unique<PointNetClassifyDomainHandler>(m_inference.get());
	AiDomainDescriptor clsDesc;
	clsDesc.domainId = QStringLiteral("pointnet.classify");
	clsDesc.displayName = QStringLiteral("PointNet++ 分类");
	clsDesc.outputKind = AiDomainOutputKind::StructuredJson;
	clsDesc.supportsMultimodal = false;
	m_aiHost->domainRegistry()->registerDomain(clsDesc, m_clsHandler.get());

	m_segHandler = std::make_unique<PointNetSegmentDomainHandler>(m_inference.get());
	AiDomainDescriptor segDesc;
	segDesc.domainId = QStringLiteral("pointnet.segment");
	segDesc.displayName = QStringLiteral("PointNet++ 分割");
	segDesc.outputKind = AiDomainOutputKind::StructuredJson;
	segDesc.supportsMultimodal = false;
	m_aiHost->domainRegistry()->registerDomain(segDesc, m_segHandler.get());

	if (m_host)
		m_host->logInfo(QStringLiteral("[PointNet] 插件初始化完成"));

	return true;
}

void PointNetPlugin::shutdownAi()
{
	m_clsHandler.reset();
	m_segHandler.reset();
	m_aiHost = nullptr;
}

bool PointNetPlugin::loadConfig(QString* err)
{
	// 依次搜：exe 目录、plugins/com.cloudsim.pointnet、上级 bin、上级 com.cloudsim.pointnet
	QString pluginDir;
	if (m_host)
		pluginDir = m_host->applicationDirPath();

	QStringList searchPaths;
	if (!pluginDir.isEmpty())
	{
		searchPaths.append(pluginDir);
		searchPaths.append(QDir::cleanPath(pluginDir + QStringLiteral("/plugins/com.cloudsim.pointnet")));
		QDir parentDir(pluginDir);
		if (parentDir.cdUp())
		{
			searchPaths.append(parentDir.absolutePath());
			searchPaths.append(QDir::cleanPath(parentDir.absolutePath() + QStringLiteral("/com.cloudsim.pointnet")));
		}
	}

	QString configPath;
	for (const QString& dir : searchPaths)
	{
		const QString candidate = QDir::cleanPath(dir + QStringLiteral("/pointnet_config.json"));
		if (QFile::exists(candidate))
		{
			configPath = candidate;
			break;
		}
	}

	if (configPath.isEmpty())
	{
		if (err)
			*err = QStringLiteral("未找到 pointnet_config.json");
		return false;
	}

	QFile f(configPath);
	if (!f.open(QIODevice::ReadOnly))
	{
		if (err)
			*err = QStringLiteral("无法打开配置文件: %1").arg(configPath);
		return false;
	}

	QByteArray data = f.readAll();
	f.close();

	nlohmann::json cfg;
	try
	{
		cfg = nlohmann::json::parse(data.constData(), nullptr, true);
	}
	catch (const std::exception& e)
	{
		if (err)
			*err = QStringLiteral("配置文件 JSON 解析失败: %1").arg(QString::fromStdString(e.what()));
		return false;
	}

	QString provider = QStringLiteral("cpu");
	if (cfg.contains("inference") && cfg["inference"].contains("provider"))
		provider = QString::fromStdString(cfg["inference"]["provider"].get<std::string>());

	if (cfg.contains("models") && cfg["models"].contains("classify"))
	{
		auto& cls = cfg["models"]["classify"];
		QString modelPath;
		int numPoints = 1024;
		QStringList classes;

		if (cls.contains("path"))
		{
			modelPath = QString::fromStdString(cls["path"].get<std::string>());
			// 模型 path 相对配置文件目录
			if (QFileInfo(modelPath).isRelative())
			{
				QString baseDir = QFileInfo(configPath).absolutePath();
				modelPath = QDir::cleanPath(baseDir + QStringLiteral("/") + modelPath);
			}
		}
		if (cls.contains("num_points"))
			numPoints = cls["num_points"].get<int>();
		if (cls.contains("classes"))
		{
			for (auto& c : cls["classes"])
				classes.append(QString::fromStdString(c.get<std::string>()));
		}

		if (!modelPath.isEmpty())
		{
			QString loadErr;
			if (!m_inference->loadClassifyModel(modelPath, numPoints, classes, &loadErr))
			{
				if (m_host)
					m_host->logWarn(QStringLiteral("[PointNet] 分类模型加载失败: %1").arg(loadErr));
			}
		}
	}

	if (cfg.contains("models") && cfg["models"].contains("segment"))
	{
		auto& seg = cfg["models"]["segment"];
		QString modelPath;
		int numPoints = 2048;
		int numClasses = 6;

		if (seg.contains("path"))
		{
			modelPath = QString::fromStdString(seg["path"].get<std::string>());
			if (QFileInfo(modelPath).isRelative())
			{
				QString baseDir = QFileInfo(configPath).absolutePath();
				modelPath = QDir::cleanPath(baseDir + QStringLiteral("/") + modelPath);
			}
		}
		if (seg.contains("num_points"))
			numPoints = seg["num_points"].get<int>();
		if (seg.contains("num_classes"))
			numClasses = seg["num_classes"].get<int>();

		if (!modelPath.isEmpty())
		{
			QString loadErr;
			if (!m_inference->loadSegmentModel(modelPath, numPoints, numClasses, &loadErr))
			{
				if (m_host)
					m_host->logWarn(QStringLiteral("[PointNet] 分割模型加载失败: %1").arg(loadErr));
			}
		}
	}

	return true;
}
