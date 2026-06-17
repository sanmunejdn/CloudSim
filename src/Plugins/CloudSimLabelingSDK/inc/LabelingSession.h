#pragma once

#include "labeling_sdk_global.h"

#include "LabelingTypes.h"

#include <cstddef>
#include <map>
#include <vector>

/// 纯逻辑标注会话（无 Qt/OSG）
class LABELING_SDK_EXPORT LabelingSession
{
public:
	LabelingSession() = default;

	bool beginPointCloud(const std::vector<float>& xyz, const LabelingSessionConfig& config);
	bool beginTriangleMesh(const std::vector<float>& triangleSoup, const LabelingSessionConfig& config);

	LabelingGeometryKind geometryKind() const { return m_kind; }
	const LabelingSessionConfig& config() const { return m_config; }
	int activeClassId() const { return m_activeClassId; }
	void setActiveClassId(int classId) { m_activeClassId = classId; }

	/// 更新类别定义与颜色，保留已有标签
	void updateSessionConfig(const LabelingSessionConfig& config);

	const std::vector<int>& pointLabels() const { return m_pointLabels; }
	const std::vector<int>& triangleLabels() const { return m_triangleLabels; }
	const std::vector<float>& pointXyz() const { return m_xyz; }
	const std::vector<float>& triangleSoup() const { return m_triangleSoup; }

	std::size_t totalElements() const;
	std::size_t labeledCount() const;
	std::map<int, std::size_t> classHistogram() const;

	bool applyPointLabels(const std::vector<std::size_t>& indices, int classId, bool erase);
	bool applyTriangleLabels(const std::vector<int>& triIndices, int classId, bool erase);
	bool importPointLabels(const std::vector<int>& labels, int numClasses);

	bool undo();
	bool redo();
	bool canUndo() const { return !m_undoStack.empty(); }
	bool canRedo() const { return !m_redoStack.empty(); }

	/// 按类别着色 RGB（0..1），与几何顶点一一对应
	void buildPointCloudRgba(std::vector<float>& outRgba) const;
	void buildMeshVertexRgb(std::vector<float>& outRgb) const;

	bool exportPointNetDataset(
		const std::string& outputDirUtf8,
		const LabelingDatasetExportOptions& options,
		LabelingDatasetExportResult& outResult,
		std::string* errMsg = nullptr) const;

	static bool sampleMeshLabelsToPointCloud(
		const std::vector<float>& soup,
		const std::vector<int>& triLabels,
		int sampleCount,
		std::vector<float>& outXyz,
		std::vector<int>& outLabels);

private:
	void pushUndo(const LabelingUndoPatch& patch);
	int resolveClassId(int classId, bool erase) const;
	const LabelingClassDef* findClass(int classId) const;

	LabelingGeometryKind m_kind = LabelingGeometryKind::PointCloud;
	LabelingSessionConfig m_config;
	int m_activeClassId = 1;

	std::vector<float> m_xyz;
	std::vector<float> m_triangleSoup;
	std::vector<int> m_pointLabels;
	std::vector<int> m_triangleLabels;

	std::vector<LabelingUndoPatch> m_undoStack;
	std::vector<LabelingUndoPatch> m_redoStack;
};
