/// @file WebGatewayParity.cpp
/// @brief 网页全量对等扩展路由（机器人回放/导出/几何/AI/工作区等）

#include "WebGateway.h"

#include "CloudSimHost.h"
#include "DocumentHost.h"
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

#include <QJsonDocument>
#include <QMetaObject>

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
			{
				pushEvent(QString::fromUtf8(QJsonDocument(ev).toJson(QJsonDocument::Compact)));
			});
	}

	httpServer().Post("/api/robot/run", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, host, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				if (!h || !h->headlessRobotPlaybackBridge())
				{
					out = QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("No playback bridge.")}};
					return;
				}
				out = h->headlessRobotPlaybackBridge()->start(QJsonDocument::fromJson(body).object());
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Post("/api/robot/stop", [this, host](const httplib::Request&, httplib::Response& res)
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

	httpServer().Get("/api/robot/playback/status", [this](const httplib::Request&, httplib::Response& res)
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

	httpServer().Post("/api/robot/export", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				if (!h || !h->headlessRobotExportBridge())
				{
					out = QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("No export bridge.")}};
					return;
				}
				out = h->headlessRobotExportBridge()->exportProgram(QJsonDocument::fromJson(body).object());
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	auto postBridge = [this](const char* path, auto method)
	{
		httpServer().Post(path, [this, method](const httplib::Request& req, httplib::Response& res)
		{
			QJsonObject out;
			QMetaObject::invokeMethod(
				this,
				[this, body = QByteArray::fromStdString(req.body), method, &out]()
				{
					auto* h = docHost(m_document.get());
					if (!h)
					{
						out = QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("No host.")}};
						return;
					}
					out = (h->*method)(QJsonDocument::fromJson(body).object());
				},
				Qt::BlockingQueuedConnection);
			writeJson(res, out);
		});
	};

	httpServer().Post("/api/geometry/op", [this](const httplib::Request& req, httplib::Response& res)
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

	httpServer().Post("/api/geometry/discretize", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessGeometryBridge()
						  ? h->headlessGeometryBridge()->discretize(QJsonDocument::fromJson(body).object())
						  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Post("/api/ai/chat", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessAiBridge() ? h->headlessAiBridge()->chat(QJsonDocument::fromJson(body).object())
												 : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Get("/api/robot/collision-settings", [this](const httplib::Request&, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessRobotCollisionBridge() ? h->headlessRobotCollisionBridge()->getSettings({})
															 : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Put("/api/robot/collision-settings", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessRobotCollisionBridge()
						  ? h->headlessRobotCollisionBridge()->putSettings(QJsonDocument::fromJson(body).object())
						  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Post("/api/robot/collision/plan", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessRobotCollisionBridge()
						  ? h->headlessRobotCollisionBridge()->plan(QJsonDocument::fromJson(body).object())
						  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Post("/api/robot/collision/confirm", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessRobotCollisionBridge()
						  ? h->headlessRobotCollisionBridge()->confirm(QJsonDocument::fromJson(body).object())
						  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Post("/api/robot/programs/switch", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessProgramEditBridge()
						  ? h->headlessProgramEditBridge()->switchProgram(QJsonDocument::fromJson(body).object())
						  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Post("/api/robot/program-edit/undo", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessProgramEditBridge()
						  ? h->headlessProgramEditBridge()->undo(QJsonDocument::fromJson(body).object())
														  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Post("/api/robot/program-edit/redo", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessProgramEditBridge()
						  ? h->headlessProgramEditBridge()->redo(QJsonDocument::fromJson(body).object())
														  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Get("/api/processflow/graph", [this](const httplib::Request&, httplib::Response& res)
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

	httpServer().Put("/api/processflow/graph", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessProcessFlowBridge()
						  ? h->headlessProcessFlowBridge()->putGraph(QJsonDocument::fromJson(body).object())
						  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Post("/api/processflow/sim/run", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessProcessFlowBridge()
						  ? h->headlessProcessFlowBridge()->runSim(QJsonDocument::fromJson(body).object())
						  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Post("/api/drawing/export", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject out;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &out]()
			{
				auto* h = docHost(m_document.get());
				out = h && h->headlessDrawingBridge()
						  ? h->headlessDrawingBridge()->exportDrawing(QJsonDocument::fromJson(body).object())
						  : QJsonObject{{QStringLiteral("ok"), false}};
			},
			Qt::BlockingQueuedConnection);
		writeJson(res, out);
	});

	httpServer().Get("/api/geomodeling/summary", [this](const httplib::Request&, httplib::Response& res)
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

	httpServer().Get("/api/labeling/tasks", [this](const httplib::Request&, httplib::Response& res)
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

	httpServer().Post("/api/assembly/mate", [this](const httplib::Request& req, httplib::Response& res)
	{
		QJsonObject body = QJsonDocument::fromJson(QByteArray::fromStdString(req.body)).object();
		body.insert(QStringLiteral("ok"), true);
		body.insert(QStringLiteral("note"), QStringLiteral("Assembly mate API — wire Host AssemblyMateApply"));
		writeJson(res, body);
	});

	Q_UNUSED(host);
}

} // namespace cloudsim::web
