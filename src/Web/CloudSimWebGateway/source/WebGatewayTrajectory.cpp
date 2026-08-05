/// @file WebGatewayTrajectory.cpp
/// @brief 轨迹 / 特征拾取 GUI 线程 API

#include "WebGateway.h"

#include "CloudSimHost.h"
#include "DocumentHost.h"
#include "HeadlessTrajectorySession.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace cloudsim::web
{
namespace
{
cloudsim::host::HeadlessTrajectorySession* traj(cloudsim::core::IDocumentScope* doc, QString* err)
{
	auto* host = cloudsim::host::documentHostFromScope(doc);
	if (!host || !host->headlessTrajectorySession())
	{
		if (err)
			*err = QStringLiteral("no headless trajectory session");
		return nullptr;
	}
	return host->headlessTrajectorySession();
}
} // namespace

QByteArray WebGateway::trajectorySessionJsonOnGuiThread()
{
	QString err;
	auto* s = traj(m_document.get(), &err);
	if (!s)
		return QJsonDocument(QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}})
			.toJson(QJsonDocument::Compact);
	return QJsonDocument(s->sessionSummaryJson()).toJson(QJsonDocument::Compact);
}

QByteArray WebGateway::trajectoryPathPlansJsonOnGuiThread(const QString& sceneRootBackendId)
{
	QString err;
	auto* s = traj(m_document.get(), &err);
	QJsonObject root;
	if (!s)
	{
		root.insert(QStringLiteral("ok"), false);
		root.insert(QStringLiteral("error"), err);
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("pathPlans"), s->listPathPlansJson(sceneRootBackendId));
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::createPathPlanOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out)
{
	auto* s = traj(m_document.get(), err);
	if (!s)
		return false;
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	QString id;
	if (!s->createPathPlan(o.value(QStringLiteral("sceneRootBackendId")).toString(), &id, err))
		return false;
	if (out)
	{
		(*out)[QStringLiteral("pathPlanId")] = id;
		(*out)[QStringLiteral("session")] = s->sessionSummaryJson();
	}
	return true;
}

bool WebGateway::bindPathPlanOnGuiThread(const QByteArray& body, QString* err)
{
	auto* s = traj(m_document.get(), err);
	if (!s)
		return false;
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	return s->bindPathPlan(o.value(QStringLiteral("pathPlanId")).toString(), err,
						   o.value(QStringLiteral("sceneRootBackendId")).toString());
}

bool WebGateway::beginTrajectoryEditOnGuiThread(QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->beginEdit(err);
}

bool WebGateway::cancelTrajectoryEditOnGuiThread()
{
	auto* s = traj(m_document.get(), nullptr);
	if (!s)
		return false;
	s->cancelEdit();
	return true;
}

bool WebGateway::pickMeshElementOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out)
{
	auto* s = traj(m_document.get(), err);
	return s && s->pickMeshElement(body, out, err);
}

bool WebGateway::pickHoverOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out)
{
	auto* s = traj(m_document.get(), err);
	return s && s->pickHover(body, out, err);
}

QByteArray WebGateway::featureCatalogOnGuiThread(const QString& workpieceBackendId, QString* err)
{
	auto* s = traj(m_document.get(), err);
	if (!s)
		return {};
	QByteArray out;
	if (!s->featureCatalogJson(workpieceBackendId, &out, err))
		return {};
	return out;
}

bool WebGateway::featureSchemaOnGuiThread(const QString& strategyId, QString* err, QJsonObject* out)
{
	auto* s = traj(m_document.get(), err);
	return s && s->featureSchemaJson(strategyId, out, err);
}

bool WebGateway::discretizeFeaturesOnGuiThread(const QByteArray& body, QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->setFeaturesAndDiscretize(body, err);
}

bool WebGateway::trajectoryOpSchemaOnGuiThread(const QString& kind, int opIndex, QString* err, QJsonObject* out)
{
	auto* s = traj(m_document.get(), err);
	return s && s->opSchemaJson(kind, opIndex, out, err);
}

bool WebGateway::discretizeMeshSpecOnGuiThread(const QByteArray& body, QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->discretizeMeshSpec(body, err);
}

bool WebGateway::setTrajectoryPipelineOnGuiThread(const QByteArray& body, QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->setPipelineJson(body, err);
}

QByteArray WebGateway::trajectoryPipelineJsonOnGuiThread()
{
	auto* s = traj(m_document.get(), nullptr);
	if (!s)
		return QByteArrayLiteral("[]");
	return s->pipelineJson();
}

bool WebGateway::fillTrajectoryRecipeOnGuiThread(const QByteArray& body, QString* err)
{
	auto* s = traj(m_document.get(), err);
	if (!s)
		return false;
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	return s->fillRecipe(o.value(QStringLiteral("recipe")).toString(QStringLiteral("weld")), err);
}

bool WebGateway::previewTrajectoryOnGuiThread(QString* err, QJsonObject* out)
{
	auto* s = traj(m_document.get(), err);
	return s && s->preview(out, err);
}

bool WebGateway::applyTrajectoryOnGuiThread(QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->apply(err);
}

bool WebGateway::emitTrajectoryRawOnGuiThread(QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->emitRawProgram(err);
}

bool WebGateway::trajectoryOpPaletteOnGuiThread(QString* err, QJsonObject* out)
{
	auto* s = traj(m_document.get(), err);
	return s && s->opPaletteJson(out, err);
}

bool WebGateway::resetTrajectoryPipelineOnGuiThread(QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->resetPipeline(err);
}

bool WebGateway::undoTrajectoryDraftOnGuiThread(QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->undoDraft(err);
}

bool WebGateway::redoTrajectoryDraftOnGuiThread(QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->redoDraft(err);
}

QByteArray WebGateway::listTrajectoryTemplatesOnGuiThread(const QString& kind)
{
	auto* s = traj(m_document.get(), nullptr);
	QJsonObject root;
	root.insert(QStringLiteral("ok"), !!s);
	if (s)
		root.insert(QStringLiteral("templates"), s->listTemplatesJson(kind));
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::saveTrajectoryTemplateOnGuiThread(const QString& kind, const QByteArray& body, QString* err)
{
	auto* s = traj(m_document.get(), err);
	if (!s)
		return false;
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	const QString name = o.value(QStringLiteral("name")).toString();
	// payload 可能是管线数组或特征对象
	QByteArray payload;
	const QJsonValue pv = o.value(QStringLiteral("payload"));
	if (pv.isArray())
		payload = QJsonDocument(pv.toArray()).toJson(QJsonDocument::Compact);
	else if (pv.isObject())
		payload = QJsonDocument(pv.toObject()).toJson(QJsonDocument::Compact);
	else
		payload = body;
	return s->saveTemplate(kind, name, payload, err);
}

bool WebGateway::loadTrajectoryTemplateOnGuiThread(const QString& kind, const QString& name, QByteArray* out,
												   QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->loadTemplate(kind, name, out, err);
}

bool WebGateway::deleteTrajectoryTemplateOnGuiThread(const QString& kind, const QString& name, QString* err)
{
	auto* s = traj(m_document.get(), err);
	return s && s->deleteTemplate(kind, name, err);
}

} // namespace cloudsim::web
