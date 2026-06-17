#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include "PluginLabelingTypes.h"

class IPluginDocument;

/// 分割标注宿主 API（1.16.0+）；插件经 IPluginHostContext::labelingHost() 获取
class IPluginLabelingHost
{
public:
	virtual ~IPluginLabelingHost() = default;

	virtual PluginLabelingSessionId beginLabelingSession(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginLabelingSessionConfig& config,
		QString* outError = nullptr) = 0;

	virtual void clearLabelingSession(PluginLabelingSessionId sessionId) = 0;

	virtual bool getSessionSummary(
		PluginLabelingSessionId sessionId,
		PluginLabelingSessionSummary& outSummary,
		QString* outError = nullptr) const = 0;

	virtual bool setActiveClass(PluginLabelingSessionId sessionId, int classId, QString* outError = nullptr) = 0;

	/// 同步类别表（名称/颜色），不重置标签
	virtual bool syncSessionConfig(
		PluginLabelingSessionId sessionId,
		const PluginLabelingSessionConfig& config,
		QString* outError = nullptr) = 0;

	virtual bool applyLabels(
		PluginLabelingSessionId sessionId,
		const PluginLabelingSelectionResult& selection,
		int classId,
		bool erase,
		QString* outError = nullptr) = 0;

	virtual bool undo(PluginLabelingSessionId sessionId, QString* outError = nullptr) = 0;
	virtual bool redo(PluginLabelingSessionId sessionId, QString* outError = nullptr) = 0;

	virtual bool syncLabelVisualization(PluginLabelingSessionId sessionId, QString* outError = nullptr) = 0;

	virtual bool importPerPointLabels(
		PluginLabelingSessionId sessionId,
		const std::vector<int>& labels,
		int numClasses,
		QString* outError = nullptr) = 0;

	virtual bool exportPointNetDataset(
		PluginLabelingSessionId sessionId,
		const std::string& outputDirUtf8,
		const PluginLabelingDatasetExportOptions& options,
		PluginLabelingDatasetExportResult& outResult,
		QString* outError = nullptr) = 0;

	virtual void pickPointsOnce(
		PluginLabelingSessionId sessionId,
		PluginLabelingPickFinishedFn onFinished) = 0;

	virtual void brushStroke(
		PluginLabelingSessionId sessionId,
		float radiusPx,
		PluginLabelingBrushStrokeFn onStroke,
		PluginLabelingBrushFinishedFn onFinished) = 0;

	virtual void pickPolylineRegion(
		PluginLabelingSessionId sessionId,
		PluginLabelingPickFinishedFn onFinished) = 0;

	virtual void pickMeshFaceOnce(
		PluginLabelingSessionId sessionId,
		PluginLabelingPickFinishedFn onFinished) = 0;

	virtual void brushMeshFaces(
		PluginLabelingSessionId sessionId,
		float radiusPx,
		PluginLabelingBrushStrokeFn onStroke,
		PluginLabelingBrushFinishedFn onFinished) = 0;
};
