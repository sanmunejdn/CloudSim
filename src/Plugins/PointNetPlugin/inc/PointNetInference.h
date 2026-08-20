#ifndef POINTNETPLUGIN_POINTNETINFERENCE_H
#define POINTNETPLUGIN_POINTNETINFERENCE_H

/// @file PointNetInference.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief ONNX Runtime 推理封装，支持 PointNet++ 分类与分割

#include "PointNetTypes.h"

#include <QString>
#include <memory>
#include <string>
#include <vector>

// ONNX Runtime 前向声明（头文件不暴露 onnxruntime C++ API）
namespace Ort
{
class Env;
class Session;
class SessionOptions;
struct Value;
} // namespace Ort

/// ONNX Runtime 推理封装，支持 PointNet++ 分类与分割
class PointNetInference
{
public:
	PointNetInference();
	~PointNetInference();

	// 禁止拷贝
	PointNetInference(const PointNetInference&) = delete;
	PointNetInference& operator=(const PointNetInference&) = delete;

	/// 加载分类模型（ONNX 格式）
	bool loadClassifyModel(const QString& onnxPath, int numPoints, const QStringList& classes, QString* err = nullptr);

	/// 加载分割模型（ONNX 格式）
	bool loadSegmentModel(const QString& onnxPath, int numPoints, int numClasses, QString* err = nullptr);

	/// 分类推理：points 为 N×3 float 数组（xyz 展平）
	PointNetClassifyResult classify(const std::vector<float>& points, int numPoints) const;

	/// 分割推理：points 为 N×3 float 数组（xyz 展平）
	PointNetSegmentResult segment(const std::vector<float>& points, int numPoints) const;

	bool isClassifyModelLoaded() const { return m_clsLoaded; }
	bool isSegmentModelLoaded() const { return m_segLoaded; }

private:
	std::unique_ptr<Ort::Env> m_env;
	std::unique_ptr<Ort::Session> m_clsSession;
	std::unique_ptr<Ort::Session> m_segSession;

	bool m_clsLoaded = false;
	bool m_segLoaded = false;
	int m_clsNumPoints = 0;
	int m_segNumPoints = 0;
	QStringList m_clsClasses;
	int m_segNumClasses = 0;

	/// 将原始 xyz 点云预处理为模型输入格式（归一化 + 采样/填充）
	std::vector<float> preprocessPoints(const std::vector<float>& rawPoints, int srcCount, int targetCount) const;
};

#endif // POINTNETPLUGIN_POINTNETINFERENCE_H
