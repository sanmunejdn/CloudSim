#ifndef CLOUDSIMPLUGINHOST_PLUGINLABELINGHOSTIMPL_H
#define CLOUDSIMPLUGINHOST_PLUGINLABELINGHOSTIMPL_H

/// @file PluginLabelingHostImpl.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PluginLabelingHostImpl 接口

#include "IPluginLabelingHost.h"
#include "PluginLabelingTypes.h"

#include <QMetaObject>
#include <atomic>
#include <memory>
#include <unordered_map>

class PluginHostContext;
class LabelingSession;
class OsgWidget;

class PluginLabelingHostImpl : public IPluginLabelingHost
{
public:
	explicit PluginLabelingHostImpl(PluginHostContext* hostContext);

	PluginLabelingSessionId beginLabelingSession(IPluginDocument* doc, const std::string& backendIdUtf8,
												 const PluginLabelingSessionConfig& config,
												 QString* outError = nullptr) override;

	void clearLabelingSession(PluginLabelingSessionId sessionId) override;

	bool getSessionSummary(PluginLabelingSessionId sessionId, PluginLabelingSessionSummary& outSummary,
						   QString* outError = nullptr) const override;

	bool setActiveClass(PluginLabelingSessionId sessionId, int classId, QString* outError = nullptr) override;

	bool syncSessionConfig(PluginLabelingSessionId sessionId, const PluginLabelingSessionConfig& config,
						   QString* outError = nullptr) override;

	bool applyLabels(PluginLabelingSessionId sessionId, const PluginLabelingSelectionResult& selection, int classId,
					 bool erase, QString* outError = nullptr) override;

	bool undo(PluginLabelingSessionId sessionId, QString* outError = nullptr) override;
	bool redo(PluginLabelingSessionId sessionId, QString* outError = nullptr) override;

	bool syncLabelVisualization(PluginLabelingSessionId sessionId, QString* outError = nullptr) override;

	bool importPerPointLabels(PluginLabelingSessionId sessionId, const std::vector<int>& labels, int numClasses,
							  QString* outError = nullptr) override;

	bool exportPointNetDataset(PluginLabelingSessionId sessionId, const std::string& outputDirUtf8,
							   const PluginLabelingDatasetExportOptions& options,
							   PluginLabelingDatasetExportResult& outResult, QString* outError = nullptr) override;

	void pickPointsOnce(PluginLabelingSessionId sessionId, PluginLabelingPickFinishedFn onFinished) override;

	void brushStroke(PluginLabelingSessionId sessionId, float radiusPx, PluginLabelingBrushStrokeFn onStroke,
					 PluginLabelingBrushFinishedFn onFinished) override;

	void pickPolylineRegion(PluginLabelingSessionId sessionId, PluginLabelingPickFinishedFn onFinished) override;

	void pickMeshFaceOnce(PluginLabelingSessionId sessionId, PluginLabelingPickFinishedFn onFinished) override;

	void brushMeshFaces(PluginLabelingSessionId sessionId, float radiusPx, PluginLabelingBrushStrokeFn onStroke,
						PluginLabelingBrushFinishedFn onFinished) override;

	void cancelActiveLabelingPick() override;
	void abandonActiveLabelingPick() override;
	void setPickCancelledNotifier(PluginLabelingPickCancelledFn notifier) override;

private:
	struct ActivePickState
	{
		OsgWidget* viewportWidget = nullptr;
		bool meshFace = false;
		float brushRadius = 16.f;
		PluginLabelingSessionId sessionId = 0U;
		QMetaObject::Connection clickConn;
		QMetaObject::Connection brushStrokeConn;
		QMetaObject::Connection brushFinishConn;
		QMetaObject::Connection cancelConn;
		PluginLabelingBrushFinishedFn brushFinished;
	};

	void clearActivePickState(bool notify);
	struct SessionEntry
	{
		PluginLabelingSessionId id = 0U;
		std::string backendId;
		IPluginDocument* doc = nullptr;
		std::unique_ptr<LabelingSession> session;
		PluginLabelingGeometryKind kind = PluginLabelingGeometryKind::PointCloud;
	};

	SessionEntry* findSession(PluginLabelingSessionId sessionId);
	const SessionEntry* findSession(PluginLabelingSessionId sessionId) const;
	bool refreshBackendColors(SessionEntry& entry, QString* outError);

	PluginHostContext* m_host = nullptr;
	std::atomic<PluginLabelingSessionId> m_nextSessionId{1U};
	std::unordered_map<PluginLabelingSessionId, SessionEntry> m_sessions;
	PluginLabelingSessionId m_activePickSessionId = 0U;
	std::unique_ptr<ActivePickState> m_pickState;
	PluginLabelingPickCancelledFn m_pickCancelledNotifier;
};

#endif // CLOUDSIMPLUGINHOST_PLUGINLABELINGHOSTIMPL_H
