#include "PointNetInference.h"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

PointNetInference::PointNetInference()
{
	m_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "PointNetPlugin");
}

PointNetInference::~PointNetInference() = default;

bool PointNetInference::loadClassifyModel(const QString& onnxPath, int numPoints,
	const QStringList& classes, QString* err)
{
	try
	{
		auto opts = std::make_unique<Ort::SessionOptions>();
		opts->SetIntraOpNumThreads(1);
		opts->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

		std::wstring wPath = onnxPath.toStdWString();
		m_clsSession = std::make_unique<Ort::Session>(*m_env, wPath.c_str(), *opts);
		m_clsNumPoints = numPoints;
		m_clsClasses = classes;
		m_clsLoaded = true;
		return true;
	}
	catch (const Ort::Exception& e)
	{
		if (err)
			*err = QStringLiteral("加载分类模型失败: %1").arg(QString::fromStdString(e.what()));
		m_clsLoaded = false;
		return false;
	}
}

bool PointNetInference::loadSegmentModel(const QString& onnxPath, int numPoints,
	int numClasses, QString* err)
{
	try
	{
		auto opts = std::make_unique<Ort::SessionOptions>();
		opts->SetIntraOpNumThreads(1);
		opts->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

		std::wstring wPath = onnxPath.toStdWString();
		m_segSession = std::make_unique<Ort::Session>(*m_env, wPath.c_str(), *opts);
		m_segNumPoints = numPoints;
		m_segNumClasses = numClasses;
		m_segLoaded = true;
		return true;
	}
	catch (const Ort::Exception& e)
	{
		if (err)
			*err = QStringLiteral("加载分割模型失败: %1").arg(QString::fromStdString(e.what()));
		m_segLoaded = false;
		return false;
	}
}

std::vector<float> PointNetInference::preprocessPoints(const std::vector<float>& rawPoints,
	int srcCount, int targetCount) const
{
	// rawPoints: [x0,y0,z0, x1,y1,z1, ...] 展平
	// 输出: 目标点数的归一化 xyz

	std::vector<float> result;
	result.resize(static_cast<size_t>(targetCount) * 3);

	// 步骤1：选取有效点（采样或填充）
	if (srcCount >= targetCount)
	{
		// 均匀采样
		const float step = static_cast<float>(srcCount) / static_cast<float>(targetCount);
		for (int i = 0; i < targetCount; ++i)
		{
			const int srcIdx = static_cast<int>(i * step);
			const int srcOff = srcIdx * 3;
			const int dstOff = i * 3;
			result[dstOff + 0] = rawPoints[srcOff + 0];
			result[dstOff + 1] = rawPoints[srcOff + 1];
			result[dstOff + 2] = rawPoints[srcOff + 2];
		}
	}
	else
	{
		// 复制已有 + 随机重复填充
		for (int i = 0; i < srcCount; ++i)
		{
			const int srcOff = i * 3;
			const int dstOff = i * 3;
			result[dstOff + 0] = rawPoints[srcOff + 0];
			result[dstOff + 1] = rawPoints[srcOff + 1];
			result[dstOff + 2] = rawPoints[srcOff + 2];
		}
		for (int i = srcCount; i < targetCount; ++i)
		{
			const int srcIdx = i % srcCount;
			const int srcOff = srcIdx * 3;
			const int dstOff = i * 3;
			result[dstOff + 0] = rawPoints[srcOff + 0];
			result[dstOff + 1] = rawPoints[srcOff + 1];
			result[dstOff + 2] = rawPoints[srcOff + 2];
		}
	}

	// 步骤2：中心化（减质心）
	float cx = 0.0f, cy = 0.0f, cz = 0.0f;
	for (int i = 0; i < targetCount; ++i)
	{
		cx += result[i * 3 + 0];
		cy += result[i * 3 + 1];
		cz += result[i * 3 + 2];
	}
	cx /= targetCount;
	cy /= targetCount;
	cz /= targetCount;

	float maxDist = 0.0f;
	for (int i = 0; i < targetCount; ++i)
	{
		result[i * 3 + 0] -= cx;
		result[i * 3 + 1] -= cy;
		result[i * 3 + 2] -= cz;
		const float d = std::sqrt(result[i * 3 + 0] * result[i * 3 + 0] +
			result[i * 3 + 1] * result[i * 3 + 1] +
			result[i * 3 + 2] * result[i * 3 + 2]);
		maxDist = std::max(maxDist, d);
	}

	// 步骤3：归一化到单位球
	if (maxDist > 1e-8f)
	{
		for (auto& v : result)
			v /= maxDist;
	}

	return result;
}

PointNetClassifyResult PointNetInference::classify(const std::vector<float>& points, int numPoints) const
{
	PointNetClassifyResult result;
	if (!m_clsLoaded || !m_clsSession)
		return result;

	try
	{
		// 预处理
		std::vector<float> input = preprocessPoints(points, numPoints, m_clsNumPoints);

		// 构造 ONNX 输入张量: [1, N, 3]
		std::vector<int64_t> inputShape = { 1, m_clsNumPoints, 3 };
		auto memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
		auto inputTensor = Ort::Value::CreateTensor<float>(
			memInfo, input.data(), input.size(), inputShape.data(), inputShape.size());

		// 获取输入输出名称
		Ort::AllocatorWithDefaultOptions allocator;
		auto inputName = m_clsSession->GetInputNameAllocated(0, allocator);
		auto outputName = m_clsSession->GetOutputNameAllocated(0, allocator);

		const char* inputNames[] = { inputName.get() };
		const char* outputNames[] = { outputName.get() };

		auto outputs = m_clsSession->Run(Ort::RunOptions{ nullptr },
			inputNames, &inputTensor, 1, outputNames, 1);

		// 解析输出: [1, numClasses] softmax 概率
		const float* logits = outputs[0].GetTensorData<float>();
		auto outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
		int numClasses = static_cast<int>(outShape.back());

		// softmax
		std::vector<float> probs(numClasses);
		float maxLogit = *std::max_element(logits, logits + numClasses);
		float sumExp = 0.0f;
		for (int i = 0; i < numClasses; ++i)
		{
			probs[i] = std::exp(logits[i] - maxLogit);
			sumExp += probs[i];
		}
		for (auto& p : probs)
			p /= sumExp;

		// 取最大概率
		auto maxIt = std::max_element(probs.begin(), probs.end());
		result.classId = static_cast<int>(std::distance(probs.begin(), maxIt));
		result.confidence = *maxIt;
		result.probabilities = probs;

		if (result.classId >= 0 && result.classId < m_clsClasses.size())
			result.className = m_clsClasses[result.classId];
	}
	catch (const Ort::Exception&)
	{
		// 推理失败，返回空结果
	}

	return result;
}

PointNetSegmentResult PointNetInference::segment(const std::vector<float>& points, int numPoints) const
{
	PointNetSegmentResult result;
	if (!m_segLoaded || !m_segSession)
		return result;

	try
	{
		// 预处理
		std::vector<float> input = preprocessPoints(points, numPoints, m_segNumPoints);

		// 构造 ONNX 输入张量: [1, N, 3]
		std::vector<int64_t> inputShape = { 1, m_segNumPoints, 3 };
		auto memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
		auto inputTensor = Ort::Value::CreateTensor<float>(
			memInfo, input.data(), input.size(), inputShape.data(), inputShape.size());

		Ort::AllocatorWithDefaultOptions allocator;
		auto inputName = m_segSession->GetInputNameAllocated(0, allocator);
		auto outputName = m_segSession->GetOutputNameAllocated(0, allocator);

		const char* inputNames[] = { inputName.get() };
		const char* outputNames[] = { outputName.get() };

		auto outputs = m_segSession->Run(Ort::RunOptions{ nullptr },
			inputNames, &inputTensor, 1, outputNames, 1);

		// 解析输出: [1, N, numClasses]
		const float* logits = outputs[0].GetTensorData<float>();
		auto outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
		int numClasses = static_cast<int>(outShape.back());

		result.numClasses = numClasses;
		result.labels.resize(m_segNumPoints);
		result.scores.resize(m_segNumPoints);

		for (int i = 0; i < m_segNumPoints; ++i)
		{
			const float* clsLogits = logits + i * numClasses;

			// argmax + softmax 置信度
			int bestCls = 0;
			float bestVal = clsLogits[0];
			for (int c = 1; c < numClasses; ++c)
			{
				if (clsLogits[c] > bestVal)
				{
					bestVal = clsLogits[c];
					bestCls = c;
				}
			}

			// softmax for confidence
			float maxL = *std::max_element(clsLogits, clsLogits + numClasses);
			float sumExp = 0.0f;
			for (int c = 0; c < numClasses; ++c)
				sumExp += std::exp(clsLogits[c] - maxL);
			float confidence = std::exp(bestVal - maxL) / sumExp;

			result.labels[i] = bestCls;
			result.scores[i] = confidence;
		}
	}
	catch (const Ort::Exception&)
	{
		// 推理失败，返回空结果
	}

	return result;
}
