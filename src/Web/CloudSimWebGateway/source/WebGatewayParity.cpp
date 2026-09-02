/// @file WebGatewayParity.cpp
/// @brief 网页全量对等扩展路由（机器人回放/导出/几何/AI/工作区等）

#include "AssemblyMateApply.h"
#include "CloudSimHost.h"
#include "DocumentHost.h"
#include "WebGateway.h"
#include "headless/HeadlessAiBridge.h"
#include "headless/HeadlessDrawingBridge.h"
#include "headless/HeadlessGeometryBridge.h"
#include "headless/HeadlessGeomodelBridge.h"
#include "headless/HeadlessLabelingBridge.h"
#include "headless/HeadlessProcessFlowBridge.h"
#include "headless/HeadlessProgramEditBridge.h"
#include "headless/HeadlessRobotCollisionBridge.h"
#include "headless/HeadlessRobotExportBridge.h"
#include "headless/HeadlessRobotPlaybackBridge.h"

#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMetaObject>
#include <QString>

#define CPPHTTPLIB_NO_EXCEPTIONS
#include "../third_party/httplib.h"

namespace cloudsim::web
{
namespace
{
cloudsim::host::DocumentHost* docHost(cloudsim::core::IDocumentScope* scope)
{
	return cloudsim::host::documentHostFromScope(scope);
}

void writeJson(httplib::Response& res, const QJsonObject& o, int errStatus = 400)
{
	if (!o.value(QStringLiteral("ok")).toBool())
		res.status = errStatus;
	const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
	res.set_content(out.constData(), static_cast<size_t>(out.size()), "application/json; charset=utf-8");
}
} // namespace

void WebGateway::registerParityRoutes(cloudsim::host::DocumentHost* host)
{
	// 回放：服务端 Executor + SSE PlaybackFrame
	if (host && host->headlessRobotPlaybackBridge())
	{
		host->headlessRobotPlaybackBridge()->setEventPushFn(
			[this](const QJsonObject& ev)
			{ pushEvent(QString::fromUtf8(QJsonDocument(ev).toJson(QJsonDocument::Compact))); });
	}

	httpServer().Post("/api/robot/run",
					  [this, host](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, host, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  if (!h || !h->headlessRobotPlaybackBridge())
								  {
									  out =
										  QJsonObject{{QStringLiteral("ok"), false},
													  {QStringLiteral("error"), QStringLiteral("No playback bridge.")}};
									  return;
								  }
								  out = h->headlessRobotPlaybackBridge()->start(QJsonDocument::fromJson(body).object());
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/robot/stop",
					  [this, host](const httplib::Request&, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, host, &out]()
							  {
								  auto* h = docHost(m_document.get());
								  if (!h || !h->headlessRobotPlaybackBridge())
								  {
									  out = QJsonObject{{QStringLiteral("ok"), false}};
									  return;
								  }
								  out = h->headlessRobotPlaybackBridge()->stop();
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Get("/api/robot/playback/status",
					 [this](const httplib::Request&, httplib::Response& res)
					 {
						 QJsonObject out;
						 QMetaObject::invokeMethod(
							 this,
							 [this, &out]()
							 {
								 auto* h = docHost(m_document.get());
								 if (!h || !h->headlessRobotPlaybackBridge())
									 out = QJsonObject{{QStringLiteral("ok"), false}};
								 else
									 out = h->headlessRobotPlaybackBridge()->statusJson();
							 },
							 Qt::BlockingQueuedConnection);
						 writeJson(res, out);
					 });

	httpServer().Post("/api/robot/export",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  if (!h || !h->headlessRobotExportBridge())
								  {
									  out = QJsonObject{{QStringLiteral("ok"), false},
														{QStringLiteral("error"), QStringLiteral("No export bridge.")}};
									  return;
								  }
								  out = h->headlessRobotExportBridge()->exportProgram(
									  QJsonDocument::fromJson(body).object());
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	auto postBridge = [this](const char* path, auto method)
	{
		httpServer().Post(path,
						  [this, method](const httplib::Request& req, httplib::Response& res)
						  {
							  QJsonObject out;
							  QMetaObject::invokeMethod(
								  this,
								  [this, body = QByteArray::fromStdString(req.body), method, &out]()
								  {
									  auto* h = docHost(m_document.get());
									  if (!h)
									  {
										  out = QJsonObject{{QStringLiteral("ok"), false},
															{QStringLiteral("error"), QStringLiteral("No host.")}};
										  return;
									  }
									  out = (h->*method)(QJsonDocument::fromJson(body).object());
								  },
								  Qt::BlockingQueuedConnection);
							  writeJson(res, out);
						  });
	};

	httpServer().Post("/api/geometry/op",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  if (!h || !h->headlessGeometryBridge())
								  {
									  out = QJsonObject{{QStringLiteral("ok"), false}};
									  return;
								  }
								  out = h->headlessGeometryBridge()->op(QJsonDocument::fromJson(body).object());
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/geometry/discretize",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out = h && h->headlessGeometryBridge() ? h->headlessGeometryBridge()->discretize(
																			   QJsonDocument::fromJson(body).object())
																		 : QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/ai/chat",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out = h && h->headlessAiBridge()
											? h->headlessAiBridge()->chat(QJsonDocument::fromJson(body).object())
											: QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Get("/api/robot/collision-settings",
					 [this](const httplib::Request&, httplib::Response& res)
					 {
						 QJsonObject out;
						 QMetaObject::invokeMethod(
							 this,
							 [this, &out]()
							 {
								 auto* h = docHost(m_document.get());
								 out = h && h->headlessRobotCollisionBridge()
										   ? h->headlessRobotCollisionBridge()->getSettings({})
										   : QJsonObject{{QStringLiteral("ok"), false}};
							 },
							 Qt::BlockingQueuedConnection);
						 writeJson(res, out);
					 });

	httpServer().Put("/api/robot/collision-settings",
					 [this](const httplib::Request& req, httplib::Response& res)
					 {
						 QJsonObject out;
						 QMetaObject::invokeMethod(
							 this,
							 [this, body = QByteArray::fromStdString(req.body), &out]()
							 {
								 auto* h = docHost(m_document.get());
								 out = h && h->headlessRobotCollisionBridge()
										   ? h->headlessRobotCollisionBridge()->putSettings(
												 QJsonDocument::fromJson(body).object())
										   : QJsonObject{{QStringLiteral("ok"), false}};
							 },
							 Qt::BlockingQueuedConnection);
						 writeJson(res, out);
					 });

	httpServer().Post("/api/robot/collision/plan",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out = h && h->headlessRobotCollisionBridge()
											? h->headlessRobotCollisionBridge()->plan(
												  QJsonDocument::fromJson(body).object())
											: QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/robot/collision/confirm",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out = h && h->headlessRobotCollisionBridge()
											? h->headlessRobotCollisionBridge()->confirm(
												  QJsonDocument::fromJson(body).object())
											: QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/robot/programs/switch",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out = h && h->headlessProgramEditBridge()
											? h->headlessProgramEditBridge()->switchProgram(
												  QJsonDocument::fromJson(body).object())
											: QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/robot/program-edit/undo",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out =
									  h && h->headlessProgramEditBridge()
										  ? h->headlessProgramEditBridge()->undo(QJsonDocument::fromJson(body).object())
										  : QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/robot/program-edit/redo",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out =
									  h && h->headlessProgramEditBridge()
										  ? h->headlessProgramEditBridge()->redo(QJsonDocument::fromJson(body).object())
										  : QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/robot/program-edit/groups",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out = h && h->headlessProgramEditBridge()
											? h->headlessProgramEditBridge()->groupCrud(
												  QJsonDocument::fromJson(body).object())
											: QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/robot/programs/crud",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out = h && h->headlessProgramEditBridge()
											? h->headlessProgramEditBridge()->programCrud(
												  QJsonDocument::fromJson(body).object())
											: QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Get("/api/processflow/graph",
					 [this](const httplib::Request&, httplib::Response& res)
					 {
						 QJsonObject out;
						 QMetaObject::invokeMethod(
							 this,
							 [this, &out]()
							 {
								 auto* h = docHost(m_document.get());
								 out = h && h->headlessProcessFlowBridge() ? h->headlessProcessFlowBridge()->getGraph()
																		   : QJsonObject{{QStringLiteral("ok"), false}};
							 },
							 Qt::BlockingQueuedConnection);
						 writeJson(res, out);
					 });

	httpServer().Put("/api/processflow/graph",
					 [this](const httplib::Request& req, httplib::Response& res)
					 {
						 QJsonObject out;
						 QMetaObject::invokeMethod(
							 this,
							 [this, body = QByteArray::fromStdString(req.body), &out]()
							 {
								 auto* h = docHost(m_document.get());
								 out = h && h->headlessProcessFlowBridge() ? h->headlessProcessFlowBridge()->putGraph(
																				 QJsonDocument::fromJson(body).object())
																		   : QJsonObject{{QStringLiteral("ok"), false}};
							 },
							 Qt::BlockingQueuedConnection);
						 writeJson(res, out);
					 });

	httpServer().Post("/api/processflow/sim/run",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out = h && h->headlessProcessFlowBridge()
											? h->headlessProcessFlowBridge()->runSim(
												  QJsonDocument::fromJson(body).object())
											: QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Post("/api/drawing/export",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject out;
						  QMetaObject::invokeMethod(
							  this,
							  [this, body = QByteArray::fromStdString(req.body), &out]()
							  {
								  auto* h = docHost(m_document.get());
								  out = h && h->headlessDrawingBridge() ? h->headlessDrawingBridge()->exportDrawing(
																			  QJsonDocument::fromJson(body).object())
																		: QJsonObject{{QStringLiteral("ok"), false}};
							  },
							  Qt::BlockingQueuedConnection);
						  writeJson(res, out);
					  });

	httpServer().Get("/api/geomodeling/summary",
					 [this](const httplib::Request&, httplib::Response& res)
					 {
						 QJsonObject out;
						 QMetaObject::invokeMethod(
							 this,
							 [this, &out]()
							 {
								 auto* h = docHost(m_document.get());
								 out = h && h->headlessGeomodelBridge() ? h->headlessGeomodelBridge()->summaryJson()
																		: QJsonObject{{QStringLiteral("ok"), false}};
							 },
							 Qt::BlockingQueuedConnection);
						 writeJson(res, out);
					 });

	httpServer().Get("/api/geomodeling/history",
					 [this](const httplib::Request& req, httplib::Response& res)
					 {
						 const QString backendId = QString::fromStdString(req.get_param_value("backendId"));
						 QJsonObject out;
						 QMetaObject::invokeMethod(
							 this,
							 [this, backendId, &out]()
							 {
								 auto* h = docHost(m_document.get());
								 out = h && h->headlessGeomodelBridge()
										   ? h->headlessGeomodelBridge()->historyJson(backendId)
										   : QJsonObject{
												 {QStringLiteral("ok"), false},
												 {QStringLiteral("error"), QStringLiteral("No geomodel bridge.")}};
							 },
							 Qt::BlockingQueuedConnection);
						 writeJson(res, out);
					 });

	httpServer().Put(
		"/api/geomodeling/history",
		[this](const httplib::Request& req, httplib::Response& res)
		{
			QJsonObject out;
			QMetaObject::invokeMethod(
				this,
				[this, body = QByteArray::fromStdString(req.body), &out]()
				{
					auto* h = docHost(m_document.get());
					out = h && h->headlessGeomodelBridge()
							  ? h->headlessGeomodelBridge()->setHistory(QJsonDocument::fromJson(body).object())
							  : QJsonObject{{QStringLiteral("ok"), false},
											{QStringLiteral("error"), QStringLiteral("No geomodel bridge.")}};
				},
				Qt::BlockingQueuedConnection);
			writeJson(res, out);
		});

	httpServer().Post(
		"/api/geomodeling/op",
		[this](const httplib::Request& req, httplib::Response& res)
		{
			QJsonObject out;
			QMetaObject::invokeMethod(
				this,
				[this, body = QByteArray::fromStdString(req.body), &out]()
				{
					auto* h = docHost(m_document.get());
					out = h && h->headlessGeomodelBridge()
							  ? h->headlessGeomodelBridge()->applyOp(QJsonDocument::fromJson(body).object())
							  : QJsonObject{{QStringLiteral("ok"), false},
											{QStringLiteral("error"), QStringLiteral("No geomodel bridge.")}};
				},
				Qt::BlockingQueuedConnection);
			writeJson(res, out);
		});

	httpServer().Get("/api/labeling/tasks",
					 [this](const httplib::Request&, httplib::Response& res)
					 {
						 QJsonObject out;
						 QMetaObject::invokeMethod(
							 this,
							 [this, &out]()
							 {
								 auto* h = docHost(m_document.get());
								 out = h && h->headlessLabelingBridge() ? h->headlessLabelingBridge()->tasksJson()
																		: QJsonObject{{QStringLiteral("ok"), false}};
							 },
							 Qt::BlockingQueuedConnection);
						 writeJson(res, out);
					 });

	httpServer().Post(
		"/api/assembly/mate",
		[this](const httplib::Request& req, httplib::Response& res)
		{
			QJsonObject out;
			QMetaObject::invokeMethod(
				this,
				[this, bodyBytes = QByteArray::fromStdString(req.body), &out]()
				{
					auto* h = docHost(m_document.get());
					if (!h)
					{
						out = QJsonObject{{QStringLiteral("ok"), false},
										  {QStringLiteral("error"), QStringLiteral("无文档")}};
						return;
					}
					const QJsonObject body = QJsonDocument::fromJson(bodyBytes).object();
					const QString action = body.value(QStringLiteral("action")).toString(QStringLiteral("apply"));

					auto parseMat16 = [](const QJsonArray& arr, BackendMat4& outMat) -> bool
					{
						if (arr.size() != 16)
							return false;
						for (int i = 0; i < 16; ++i)
							outMat.v[i] = arr.at(i).toDouble();
						return true;
					};
					auto matToJson = [](const BackendMat4& m) -> QJsonArray
					{
						QJsonArray a;
						for (int i = 0; i < 16; ++i)
							a.append(m.v[i]);
						return a;
					};
					auto parseFace = [](const QJsonObject& o, cloudsim::host::AssemblyMateFaceRef& face,
										QString* err) -> bool
					{
						face = {};
						face.backendId = o.value(QStringLiteral("backendId")).toString().toStdString();
						face.faceIndex = o.value(QStringLiteral("faceIndex")).toInt(-1);
						const QJsonArray pw = o.value(QStringLiteral("pickWorldMm")).toArray();
						if (face.backendId.empty() || face.faceIndex < 0 || pw.size() < 3)
						{
							if (err)
								*err = QStringLiteral("面引用缺 backendId/faceIndex/pickWorldMm");
							return false;
						}
						face.pickWorldMm[0] = pw.at(0).toDouble();
						face.pickWorldMm[1] = pw.at(1).toDouble();
						face.pickWorldMm[2] = pw.at(2).toDouble();
						return true;
					};

					QString err;
					if (action == QStringLiteral("restore"))
					{
						const QString bid = body.value(QStringLiteral("backendId")).toString();
						BackendMat4 snap{};
						if (bid.isEmpty() ||
							!parseMat16(body.value(QStringLiteral("movingWorldSnapshot")).toArray(), snap))
						{
							out = QJsonObject{{QStringLiteral("ok"), false},
											  {QStringLiteral("error"),
											   QStringLiteral("restore 需要 backendId 与 movingWorldSnapshot[16]")}};
							return;
						}
						if (!cloudsim::host::restoreBackendWorldMatrix(*h, bid.toStdString(), snap, &err))
						{
							out = QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
							return;
						}
						out = QJsonObject{{QStringLiteral("ok"), true}};
						return;
					}

					cloudsim::host::AssemblyMateFaceRef grounded;
					cloudsim::host::AssemblyMateFaceRef moving;
					if (!parseFace(body.value(QStringLiteral("grounded")).toObject(), grounded, &err) ||
						!parseFace(body.value(QStringLiteral("moving")).toObject(), moving, &err))
					{
						out = QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
						return;
					}

					static const QHash<QString, geoalgo::AssemblyMateKind> kKinds{
						{QStringLiteral("coincident"), geoalgo::AssemblyMateKind::Coincident},
						{QStringLiteral("parallel"), geoalgo::AssemblyMateKind::Parallel},
						{QStringLiteral("perpendicular"), geoalgo::AssemblyMateKind::Perpendicular},
						{QStringLiteral("tangent"), geoalgo::AssemblyMateKind::Tangent},
						{QStringLiteral("concentric"), geoalgo::AssemblyMateKind::Concentric},
						{QStringLiteral("lock"), geoalgo::AssemblyMateKind::Lock},
						{QStringLiteral("distance"), geoalgo::AssemblyMateKind::Distance},
						{QStringLiteral("angle"), geoalgo::AssemblyMateKind::Angle},
					};
					geoalgo::AssemblyMateParams params;
					const QString kindKey =
						body.value(QStringLiteral("kind")).toString(QStringLiteral("coincident")).toLower();
					params.kind = kKinds.value(kindKey, geoalgo::AssemblyMateKind::Coincident);
					const QString alignKey =
						body.value(QStringLiteral("alignment")).toString(QStringLiteral("antiAligned")).toLower();
					params.alignment = (alignKey == QStringLiteral("aligned"))
										   ? geoalgo::AssemblyMateAlignment::Aligned
										   : geoalgo::AssemblyMateAlignment::AntiAligned;
					params.distanceMm = body.value(QStringLiteral("distanceMm")).toDouble(0.0);
					params.angleDeg = body.value(QStringLiteral("angleDeg")).toDouble(90.0);
					const bool commit = body.value(QStringLiteral("commit")).toBool(true);

					BackendMat4 snap{};
					bool haveSnap = parseMat16(body.value(QStringLiteral("movingWorldSnapshot")).toArray(), snap);
					if (!haveSnap)
					{
						if (!cloudsim::host::snapshotBackendWorldMatrix(*h, moving.backendId, snap))
						{
							out = QJsonObject{{QStringLiteral("ok"), false},
											  {QStringLiteral("error"), QStringLiteral("无法快照动件 worldMatrix")}};
							return;
						}
						haveSnap = true;
					}

					if (!cloudsim::host::resolveAssemblyMatePick(*h, grounded.backendId, grounded.faceIndex,
																 grounded.pickWorldMm[0], grounded.pickWorldMm[1],
																 grounded.pickWorldMm[2], grounded, &err) ||
						!cloudsim::host::resolveAssemblyMatePick(*h, moving.backendId, moving.faceIndex,
																 moving.pickWorldMm[0], moving.pickWorldMm[1],
																 moving.pickWorldMm[2], moving, &err))
					{
						out = QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
						return;
					}

					if (!cloudsim::host::applyAssemblyMate(*h, grounded, moving, params, haveSnap ? &snap : nullptr,
														   commit, &err))
					{
						out = QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
						return;
					}

					out = QJsonObject{{QStringLiteral("ok"), true},
									  {QStringLiteral("movingWorldSnapshot"), matToJson(snap)},
									  {QStringLiteral("movingBackendId"), QString::fromStdString(moving.backendId)},
									  {QStringLiteral("commit"), commit}};
					if (commit)
					{
						pushEvent(QStringLiteral("{\"type\":\"PoseCommitted\",\"backendId\":\"%1\"}")
									  .arg(QString::fromStdString(moving.backendId)));
						pushEvent(QStringLiteral("{\"type\":\"SceneChanged\"}"));
					}
				},
				Qt::BlockingQueuedConnection);
			writeJson(res, out);
		});

	Q_UNUSED(host);
}

} // namespace cloudsim::web
