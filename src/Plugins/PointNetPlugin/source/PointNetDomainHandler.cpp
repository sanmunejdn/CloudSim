/// @file PointNetDomainHandler.cpp
/// @brief 从场景对象导出点云到临时 PLY 并读取 xyz 坐标

#include "PointNetDomainHandler.h"

#include "IPluginDocument.h"
#include "IPluginHostContext.h"
#include "PointNetInference.h"

#include <QDir>
#include <QFile>
#include <cmath>
#include <fstream>
#include <sstream>

#include <json.hpp>

// ============================================================
// 辅助函数
// ============================================================

namespace
{
/// 从场景对象导出点云到临时 PLY 并读取 xyz 坐标
/// 注意：当前插件 SDK 不直接暴露原始顶点数据，
/// 这里通过 exportMeshToPly + 解析 PLY 获取坐标
bool extractPointsFromSceneObject(IPluginHostContext* host, const std::string& backendId, std::vector<float>& outPoints,
								  int& outCount)
{
	IPluginDocument* doc = host->activeDocument();
	if (!doc)
		return false;

	// 获取对象类名以判断类型
	std::string className = doc->backendClassName(backendId);
	if (className.empty())
		return false;

	// 使用临时文件中转
	QString tmpPath = QDir::tempPath() + QStringLiteral("/pointnet_tmp_export.ply");
	std::string tmpPathUtf8 = tmpPath.toUtf8().constData();

	bool exported = false;
	if (className == "MeshBackendData" || className == "BrepBackendData")
	{
		exported = doc->exportMeshToPly(backendId, tmpPathUtf8, nullptr);
	}
	else if (className == "PointCloudBackendData")
	{
		// 点云也可尝试导出为 PLY
		exported = doc->exportMeshToPly(backendId, tmpPathUtf8, nullptr);
	}

	if (!exported)
		return false;

	// 解析 PLY 文件提取 xyz
	// 简易 PLY 解析：读取 ASCII 或二进制格式的顶点坐标
	std::ifstream file(tmpPath.toStdString(), std::ios::binary);
	if (!file.is_open())
		return false;

	std::string line;
	int vertexCount = 0;
	bool inHeader = true;
	bool isBinary = false;
	bool inVertexElement = false;
	int propertiesRead = 0;

	while (std::getline(file, line))
	{
		if (line.substr(0, 7) == "format ")
		{
			isBinary = line.find("binary") != std::string::npos;
		}
		else if (line.substr(0, 15) == "element vertex ")
		{
			vertexCount = std::stoi(line.substr(15));
			inVertexElement = true;
		}
		else if (line.substr(0, 8) == "element " && line.substr(0, 15) != "element vertex ")
		{
			inVertexElement = false;
		}
		else if (line == "end_header")
		{
			inHeader = false;
			break;
		}
		else if (inVertexElement && line.substr(0, 9) == "property ")
		{
			++propertiesRead;
		}
	}

	if (vertexCount <= 0 || inHeader)
	{
		file.close();
		QFile::remove(tmpPath);
		return false;
	}

	outPoints.resize(static_cast<size_t>(vertexCount) * 3);
	outCount = vertexCount;

	if (!isBinary)
	{
		// ASCII PLY
		for (int i = 0; i < vertexCount; ++i)
		{
			if (!std::getline(file, line))
				break;
			std::istringstream iss(line);
			iss >> outPoints[i * 3 + 0] >> outPoints[i * 3 + 1] >> outPoints[i * 3 + 2];
			// 跳过其他属性（颜色、法线等）
		}
	}
	else
	{
		// Binary PLY: 每个顶点有 propertiesRead 个 float 属性
		// 假设前 3 个是 x, y, z
		const int bytesPerVertex = propertiesRead * sizeof(float);
		std::vector<char> buf(bytesPerVertex);
		for (int i = 0; i < vertexCount; ++i)
		{
			if (!file.read(buf.data(), bytesPerVertex))
				break;
			const float* fdata = reinterpret_cast<const float*>(buf.data());
			outPoints[i * 3 + 0] = fdata[0];
			outPoints[i * 3 + 1] = fdata[1];
			outPoints[i * 3 + 2] = fdata[2];
		}
	}

	file.close();
	QFile::remove(tmpPath);
	return outCount > 0;
}

/// 从 JSON 中读取 backendId
std::string readBackendId(const nlohmann::json& j)
{
	if (j.contains("backend_id") && j["backend_id"].is_string())
		return j["backend_id"].get<std::string>();
	return {};
}

} // namespace

// ============================================================
// PointNetClassifyDomainHandler
// ============================================================

PointNetClassifyDomainHandler::PointNetClassifyDomainHandler(PointNetInference* inference) : m_inference(inference) {}

QString PointNetClassifyDomainHandler::domainId() const
{
	return QStringLiteral("pointnet.classify");
}

bool PointNetClassifyDomainHandler::validateOutput(const QByteArray& jsonUtf8, QString* err) const
{
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("JSON 解析失败");
		return false;
	}

	if (!j.is_object())
	{
		if (err)
			*err = QStringLiteral("输出必须为 JSON 对象");
		return false;
	}

	// 分类结果只需 class_id 或 class_name
	if (!j.contains("class_id") && !j.contains("class_name"))
	{
		if (err)
			*err = QStringLiteral("缺少 class_id 或 class_name");
		return false;
	}

	return true;
}

bool PointNetClassifyDomainHandler::execute(const QByteArray& jsonUtf8, IPluginHostContext* host,
											IAiAssistantHost* aiHost, QString* summary, QString* err)
{
	(void)aiHost;

	if (!m_inference || !m_inference->isClassifyModelLoaded())
	{
		if (err)
			*err = QStringLiteral("分类模型未加载");
		return false;
	}

	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("JSON 解析失败");
		return false;
	}

	std::string backendId = readBackendId(j);
	if (backendId.empty())
	{
		// 尝试从活动文档获取选中对象
		IPluginDocument* doc = host ? host->activeDocument() : nullptr;
		if (!doc)
		{
			if (err)
				*err = QStringLiteral("未指定 backend_id 且无活动文档");
			return false;
		}
		auto ids = doc->backendIds();
		if (ids.empty())
		{
			if (err)
				*err = QStringLiteral("文档中无对象");
			return false;
		}
		backendId = ids.front();
	}

	// 提取点云数据
	std::vector<float> points;
	int numPoints = 0;
	if (!extractPointsFromSceneObject(host, backendId, points, numPoints))
	{
		if (err)
			*err = QStringLiteral("无法提取点云数据");
		return false;
	}

	// 执行推理
	PointNetClassifyResult result = m_inference->classify(points, numPoints);
	if (result.classId < 0)
	{
		if (err)
			*err = QStringLiteral("分类推理失败");
		return false;
	}

	// 构造结果 JSON
	nlohmann::json out;
	out["version"] = 1;
	out["domain"] = "pointnet.classify";
	out["backend_id"] = backendId;
	out["class_id"] = result.classId;
	out["class_name"] = result.className.toStdString();
	out["confidence"] = result.confidence;

	if (summary)
	{
		*summary = QStringLiteral("分类结果: %1 (置信度: %2%)")
					   .arg(result.className)
					   .arg(QString::number(result.confidence * 100.0f, 'f', 1));
	}

	return true;
}

// ============================================================
// PointNetSegmentDomainHandler
// ============================================================

PointNetSegmentDomainHandler::PointNetSegmentDomainHandler(PointNetInference* inference) : m_inference(inference) {}

QString PointNetSegmentDomainHandler::domainId() const
{
	return QStringLiteral("pointnet.segment");
}

bool PointNetSegmentDomainHandler::validateOutput(const QByteArray& jsonUtf8, QString* err) const
{
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("JSON 解析失败");
		return false;
	}

	if (!j.is_object())
	{
		if (err)
			*err = QStringLiteral("输出必须为 JSON 对象");
		return false;
	}

	// 分割结果需要 backend_id
	if (!j.contains("backend_id") || !j["backend_id"].is_string())
	{
		if (err)
			*err = QStringLiteral("缺少 backend_id");
		return false;
	}

	return true;
}

bool PointNetSegmentDomainHandler::execute(const QByteArray& jsonUtf8, IPluginHostContext* host,
										   IAiAssistantHost* aiHost, QString* summary, QString* err)
{
	(void)aiHost;

	if (!m_inference || !m_inference->isSegmentModelLoaded())
	{
		if (err)
			*err = QStringLiteral("分割模型未加载");
		return false;
	}

	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(jsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("JSON 解析失败");
		return false;
	}

	std::string backendId = readBackendId(j);
	if (backendId.empty())
	{
		IPluginDocument* doc = host ? host->activeDocument() : nullptr;
		if (!doc)
		{
			if (err)
				*err = QStringLiteral("未指定 backend_id 且无活动文档");
			return false;
		}
		auto ids = doc->backendIds();
		if (ids.empty())
		{
			if (err)
				*err = QStringLiteral("文档中无对象");
			return false;
		}
		backendId = ids.front();
	}

	// 提取点云数据
	std::vector<float> points;
	int numPoints = 0;
	if (!extractPointsFromSceneObject(host, backendId, points, numPoints))
	{
		if (err)
			*err = QStringLiteral("无法提取点云数据");
		return false;
	}

	// 执行推理
	PointNetSegmentResult result = m_inference->segment(points, numPoints);
	if (result.labels.empty())
	{
		if (err)
			*err = QStringLiteral("分割推理失败");
		return false;
	}

	// 统计各类别点数
	std::vector<int> classCounts(result.numClasses, 0);
	for (int label : result.labels)
	{
		if (label >= 0 && label < result.numClasses)
			++classCounts[label];
	}

	// 构造结果 JSON
	nlohmann::json out;
	out["version"] = 1;
	out["domain"] = "pointnet.segment";
	out["backend_id"] = backendId;
	out["num_points"] = result.labels.size();
	out["num_classes"] = result.numClasses;
	out["labels"] = result.labels;

	// 输出各类别统计
	nlohmann::json stats = nlohmann::json::array();
	for (int c = 0; c < result.numClasses; ++c)
	{
		stats.push_back({{"class_id", c}, {"point_count", classCounts[c]}});
	}
	out["class_statistics"] = stats;

	if (summary)
	{
		*summary = QStringLiteral("分割完成: %1 个点, %2 个类别").arg(result.labels.size()).arg(result.numClasses);
	}

	return true;
}
