#ifndef CLOUDSIMHOST_HEADLESSGEOMODELBRIDGE_H
#define CLOUDSIMHOST_HEADLESSGEOMODELBRIDGE_H

/// @file HeadlessGeomodelBridge.h
/// @brief Web/Headless：Parametric Body 特征史（与桌面 GeometryHost 同源 rebuild）

#include "cloudsim_host_global.h"

#include <QByteArray>
#include <QJsonObject>
#include <QString>

#include <memory>
#include <vector>

class ParametricBrepBackendData;

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessGeomodelBridge
{
public:
	explicit HeadlessGeomodelBridge(DocumentHost& host);

	HeadlessGeomodelBridge(const HeadlessGeomodelBridge&) = delete;
	HeadlessGeomodelBridge& operator=(const HeadlessGeomodelBridge&) = delete;

	QJsonObject summaryJson() const;
	QJsonObject historyJson(const QString& backendId) const;
	QJsonObject setHistory(const QJsonObject& body);
	/// op: primitive|extrude|append|patch|delete|rebuild|setHistory|undo|redo|importHistory|exportHistory
	QJsonObject applyOp(const QJsonObject& body);
	/// 新建/打开工程后 data 已清空，快照里的 backendId 全部失效
	void resetHistoryStack();

private:
	struct HistoryUndoSnap
	{
		QString backendId;
		QByteArray historyJson;
		bool created = false;
	};

	QJsonObject createPrimitive(const QJsonObject& body);
	QJsonObject extrude(const QJsonObject& body);
	QJsonObject appendFeature(const QJsonObject& body);
	QJsonObject patchFeature(const QJsonObject& body);
	QJsonObject deleteFeature(const QJsonObject& body);
	QJsonObject rebuildBody(const QJsonObject& body);
	QJsonObject undoOp();
	QJsonObject redoOp();
	QJsonObject importHistoryFile(const QJsonObject& body);
	QJsonObject exportHistoryFile(const QJsonObject& body);

	QByteArray captureHistory(const ParametricBrepBackendData& body) const;
	bool restoreHistory(ParametricBrepBackendData& body, const QByteArray& bytes, QString* err) const;
	void pushUndo(HistoryUndoSnap snap);
	void dropSnapsWithId(const QString& backendId);
	QJsonObject okBody(const ParametricBrepBackendData& body) const;
	QJsonObject commitExisting(const std::shared_ptr<ParametricBrepBackendData>& param, const QByteArray& before);
	QJsonObject finishNewBody(const std::shared_ptr<ParametricBrepBackendData>& param);

	DocumentHost& m_host;
	std::vector<HistoryUndoSnap> m_undo;
	std::vector<HistoryUndoSnap> m_redo;
};

} // namespace cloudsim::host

#endif
