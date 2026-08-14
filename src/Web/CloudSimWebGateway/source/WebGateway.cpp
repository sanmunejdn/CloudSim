/// @file WebGateway.cpp
/// @brief WebGateway：HTTP REST + SSE 事件（httplib；避免再引 OpenSSL/WS 依赖）

#include "WebGateway.h"

#include "BackendDataManager.h"
#include "BackendProjectObjectIo.h"
#include "BackendTypeIds.h"
#include "CloudSimHost.h"
#include "CoreEvents.h"
#include "CoreTypes.h"
#include "DeviceCatalogScan.h"
#include "DocumentHost.h"
#include "DocumentImportFacade.h"
#include "HeadlessRobotContext.h"
#include "HeadlessPointCloudBridge.h"
#include "IRobotUrdfImportContext.h"
#include "EventHub.h"
#include "ICloudSimContext.h"
#include "IDataService.h"
#include "IDocumentScope.h"
#include "ProjectPackageIo.h"
#include "IRobotService.h"
#include "RobotProjectKinematicsRestore.h"
#include "StoreZipExtract.h"
#include "WebGatewaySidecars.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QMutex>
#include <QMutexLocker>
#include <QSet>
#include <QTemporaryDir>
#include <QUuid>
#include <QWaitCondition>
#include <QWidget>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#define CPPHTTPLIB_NO_EXCEPTIONS
#include "../third_party/httplib.h"

namespace cloudsim::web
{
namespace
{
QJsonObject poseToJson(const cloudsim::core::PoseDto& pose)
{
	QJsonObject o;
	o.insert(QStringLiteral("positionMm"),
			 QJsonArray{pose.positionMm.x, pose.positionMm.y, pose.positionMm.z});
	o.insert(QStringLiteral("eulerDeg"), QJsonArray{pose.eulerDeg.x, pose.eulerDeg.y, pose.eulerDeg.z});
	return o;
}

QString jsonEscape(const QString& s)
{
	const QByteArray quoted = QJsonDocument(QJsonArray{s}).toJson(QJsonDocument::Compact);
	// ["..."] → 去掉方括号与引号
	if (quoted.size() >= 4)
		return QString::fromUtf8(quoted.mid(2, quoted.size() - 4));
	return s;
}

/// BackendMat4 与 OSG 同序（平移在 v[3/7/11]）；Three.js/OpenGL 要列向量布局（平移在 12..14）
QJsonArray worldMatrixForThreeJs(const BackendMat4& wm)
{
	QJsonArray mat;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
			mat.append(wm.v[r * 4 + c]);
	}
	return mat;
}

QString resolveProjectPathFromPicked(const QString& picked)
{
	const QFileInfo fi(picked);
	if (!fi.exists())
		return {};
	if (fi.isFile())
		return fi.absoluteFilePath();
	const QDir dir(fi.absoluteFilePath());
	const QString jsonPath = dir.filePath(QStringLiteral("project.json"));
	if (QFileInfo::exists(jsonPath))
		return jsonPath;
	const QStringList pcps = dir.entryList(QStringList{QStringLiteral("*.pcp")}, QDir::Files, QDir::Name);
	if (pcps.size() == 1)
		return dir.filePath(pcps.front());
	return {};
}

void writeJsonOk(httplib::Response& res, bool ok, const QString& err, const QJsonObject& extra)
{
	QJsonObject o = extra;
	o.insert(QStringLiteral("ok"), ok);
	if (!ok)
		o.insert(QStringLiteral("error"), err);
	const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
	if (!ok)
		res.status = 400;
	res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
}

} // namespace

struct WebGateway::Impl
{
	std::thread serverThread;
	httplib::Server svr;
	std::atomic<bool> running{false};
	QString projectPath;
	QString projectDir;
	std::unique_ptr<QTemporaryDir> extractDir;
#ifdef _WIN32
	HANDLE projectLock = INVALID_HANDLE_VALUE;
#endif
	QMutex eventMutex;
	QList<QString> eventQueue;
	QWaitCondition eventCv;
	std::atomic<int> sseClients{0};
};

WebGateway::WebGateway(cloudsim::core::ICloudSimContext& context, WebGatewayConfig config, QObject* parent)
	: QObject(parent), m_context(context), m_config(std::move(config)), m_impl(std::make_unique<Impl>())
{
	const QString docId = QStringLiteral("web-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces));
	m_document = m_context.createDocumentScope(nullptr, docId);
	m_context.setActiveScope(m_document.get());
}

WebGateway::~WebGateway()
{
	stop();
}

bool WebGateway::start(QString* outError)
{
	if (m_impl->running.load())
	{
		return true;
	}
	if (!m_document)
	{
		if (outError)
			*outError = QStringLiteral("No headless document scope.");
		return false;
	}

	auto* host = dynamic_cast<cloudsim::host::DocumentHost*>(m_document.get());
	if (!host)
	{
		if (outError)
			*outError = QStringLiteral("Document scope is not DocumentHost.");
		return false;
	}

	const QString staticRoot = m_config.staticRoot;
	const int port = m_config.port;
	const QString bindHost = m_config.bindHost;
	const qint64 pid = QCoreApplication::applicationPid();

	m_impl->svr.set_default_headers({{"Access-Control-Allow-Origin", "*"},
									 {"Access-Control-Allow-Methods", "GET,POST,PUT,PATCH,DELETE,OPTIONS"},
									 {"Access-Control-Allow-Headers", "Content-Type"}});

	m_impl->svr.Options(R"(/.*)", [](const httplib::Request&, httplib::Response& res) { res.status = 204; });

	registerApiRoutes(host);

	m_impl->svr.Get("/api/health", [port, pid](const httplib::Request&, httplib::Response& res)
	{
		QJsonObject o;
		o.insert(QStringLiteral("ok"), true);
		o.insert(QStringLiteral("role"), QStringLiteral("web"));
		o.insert(QStringLiteral("pid"), pid);
		o.insert(QStringLiteral("port"), port);
		const QByteArray body = QJsonDocument(o).toJson(QJsonDocument::Compact);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});

	m_impl->svr.Post("/api/project/open", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		int objectCount = 0;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, host, body = QByteArray::fromStdString(req.body), &err, &objectCount, &ok]()
			{
				ok = openProjectOnGuiThread(host, body, &err, &objectCount);
			},
			Qt::BlockingQueuedConnection);
		QJsonObject o;
		o.insert(QStringLiteral("ok"), ok);
		if (!ok)
		{
			o.insert(QStringLiteral("error"), err);
			res.status = err.contains(QStringLiteral("locked"), Qt::CaseInsensitive) ? 409 : 400;
		}
		else
		{
			o.insert(QStringLiteral("objectCount"), objectCount);
			o.insert(QStringLiteral("path"), m_impl->projectPath);
		}
		const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
		res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
	});

	m_impl->svr.Post("/api/dialog/open", [this](const httplib::Request& req, httplib::Response& res)
	{
		QByteArray body;
		QMetaObject::invokeMethod(
			this,
			[this, reqBody = QByteArray::fromStdString(req.body), &body]()
			{ body = nativeDialogOnGuiThread(reqBody); },
			Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});

	m_impl->svr.Get("/api/objects", [this](const httplib::Request&, httplib::Response& res)
	{
		QByteArray body;
		QMetaObject::invokeMethod(
			this, [this, &body]() { body = objectsJsonOnGuiThread(); }, Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});

	m_impl->svr.Get(R"(/api/mesh/(.+))", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		const QString id = QString::fromStdString(req.matches[1]);
		std::vector<float> soup;
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, host, id, &soup, &err, &ok]() { ok = meshSoupOnGuiThread(host, id, soup, &err); },
			Qt::BlockingQueuedConnection);
		if (!ok)
		{
			res.status = 404;
			res.set_content(err.toUtf8().constData(), "text/plain; charset=utf-8");
			return;
		}
		res.set_content(reinterpret_cast<const char*>(soup.data()), soup.size() * sizeof(float),
						"application/octet-stream");
	});

	m_impl->svr.Post("/api/selection", [this](const httplib::Request& req, httplib::Response& res)
	{
		bool ok = false;
		QString err;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &ok, &err]()
			{ ok = selectionOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		QJsonObject o;
		o.insert(QStringLiteral("ok"), ok);
		if (!ok)
		{
			o.insert(QStringLiteral("error"), err);
			res.status = 400;
		}
		const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
		res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
	});

	// SSE：浏览器 EventSource；与计划中的事件推送等价，免 OpenSSL
	m_impl->svr.Get("/api/events", [this](const httplib::Request&, httplib::Response& res)
	{
		m_impl->sseClients.fetch_add(1);
		res.set_header("Cache-Control", "no-cache");
		res.set_header("Connection", "keep-alive");
		res.set_chunked_content_provider(
			"text/event-stream",
			[this](size_t /*offset*/, httplib::DataSink& sink)
			{
				QString line;
				{
					QMutexLocker lock(&m_impl->eventMutex);
					while (m_impl->running.load() && m_impl->eventQueue.isEmpty())
					{
						m_impl->eventCv.wait(&m_impl->eventMutex, 1000);
					}
					if (!m_impl->running.load())
					{
						return false;
					}
					if (!m_impl->eventQueue.isEmpty())
					{
						line = m_impl->eventQueue.takeFirst();
					}
				}
				if (line.isEmpty())
				{
					const std::string keep = ": keepalive\n\n";
					return sink.write(keep.data(), keep.size());
				}
				const QByteArray payload = QStringLiteral("data: %1\n\n").arg(line).toUtf8();
				return sink.write(payload.constData(), static_cast<size_t>(payload.size()));
			},
			[this](bool) { m_impl->sseClients.fetch_sub(1); });
	});

	if (!staticRoot.isEmpty() && QDir(staticRoot).exists())
	{
		m_impl->svr.set_mount_point("/", staticRoot.toStdString());
	}

	m_impl->running = true;
	m_impl->serverThread = std::thread([this, bindHost, port]()
	{
		const bool ok = m_impl->svr.listen(bindHost.toStdString().c_str(), port);
		m_impl->running = false;
		if (!ok)
		{
			QMutexLocker lock(&m_impl->eventMutex);
			m_impl->eventQueue.append(
				QStringLiteral("{\"type\":\"server_listen_failed\",\"port\":%1}").arg(port));
			m_impl->eventCv.wakeAll();
		}
	});

	// 短暂探测端口是否起来
	httplib::Client cli(bindHost.toStdString(), port);
	cli.set_connection_timeout(0, 200000);
	bool up = false;
	for (int i = 0; i < 50; ++i)
	{
		if (auto r = cli.Get("/api/health"))
		{
			if (r->status == 200)
			{
				up = true;
				break;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	if (!up)
	{
		stop();
		if (outError)
			*outError = QStringLiteral("Failed to bind %1:%2").arg(bindHost).arg(port);
		return false;
	}

	pushEvent(QStringLiteral("{\"type\":\"ready\",\"role\":\"web\"}"));
	return true;
}

void WebGateway::stop()
{
	if (!m_impl)
	{
		return;
	}
	m_impl->running = false;
	m_impl->svr.stop();
	m_impl->eventCv.wakeAll();
	if (m_impl->serverThread.joinable())
	{
		m_impl->serverThread.join();
	}
#ifdef _WIN32
	if (m_impl->projectLock != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_impl->projectLock);
		m_impl->projectLock = INVALID_HANDLE_VALUE;
	}
#endif
}

void WebGateway::pushEvent(const QString& jsonLine)
{
	QMutexLocker lock(&m_impl->eventMutex);
	m_impl->eventQueue.append(jsonLine);
	m_impl->eventCv.wakeAll();
}

void WebGateway::releaseProjectFileLock()
{
#ifdef _WIN32
	if (m_impl->projectLock != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_impl->projectLock);
		m_impl->projectLock = INVALID_HANDLE_VALUE;
	}
#endif
}

void WebGateway::reacquireProjectFileLock(const QString& path)
{
#ifdef _WIN32
	releaseProjectFileLock();
	if (path.isEmpty() || !QFileInfo::exists(path) || QFileInfo(path).isDir())
		return;
	const std::wstring wpath = QDir::toNativeSeparators(path).toStdWString();
	// 顾问锁：允许其它进程读写，避免挡本进程 QFile / 桌面同开
	m_impl->projectLock =
		CreateFileW(wpath.c_str(), GENERIC_READ,
					FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING,
					FILE_ATTRIBUTE_NORMAL, nullptr);
#else
	Q_UNUSED(path);
#endif
}

void WebGateway::registerApiRoutes(cloudsim::host::DocumentHost* host)
{
	m_impl->svr.Post("/api/project/new", [this, host](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this, [this, host, &err, &ok]() { ok = newProjectOnGuiThread(host, &err); }, Qt::BlockingQueuedConnection);
		if (ok)
		{
			m_impl->projectPath.clear();
			m_impl->projectDir.clear();
			releaseProjectFileLock();
		}
		QJsonObject o;
		o.insert(QStringLiteral("ok"), ok);
		if (!ok)
		{
			o.insert(QStringLiteral("error"), err);
			res.status = 400;
		}
		const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
		res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
	});

	m_impl->svr.Post("/api/project/save", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, host, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = saveProjectOnGuiThread(host, body, &err); },
			Qt::BlockingQueuedConnection);
		if (ok)
			m_impl->projectPath = host->projectFilePath();
		QJsonObject o;
		o.insert(QStringLiteral("ok"), ok);
		if (!ok)
		{
			o.insert(QStringLiteral("error"), err);
			res.status = 400;
		}
		else
			o.insert(QStringLiteral("path"), m_impl->projectPath);
		const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
		res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
	});

	m_impl->svr.Get(R"(/api/objects/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		const QString id = QString::fromStdString(req.matches[1]);
		if (id == QLatin1String("import") || id == QLatin1String("register"))
		{
			res.status = 404;
			return;
		}
		QByteArray body;
		QMetaObject::invokeMethod(
			this, [this, id, &body]() { body = objectDetailJsonOnGuiThread(id); }, Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});

	m_impl->svr.Patch(R"(/api/objects/(.+))", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		const QString id = QString::fromStdString(req.matches[1]);
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, host, id, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = patchObjectOnGuiThread(host, id, body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});

	m_impl->svr.Delete(R"(/api/objects/(.+))", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		const QString id = QString::fromStdString(req.matches[1]);
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this, [this, host, id, &err, &ok]() { ok = deleteObjectOnGuiThread(host, id, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});

	m_impl->svr.Post("/api/objects/import", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QString outId;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, host, body = QByteArray::fromStdString(req.body), &err, &outId, &ok]()
			{ ok = importObjectOnGuiThread(host, body, &err, &outId); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{{QStringLiteral("id"), outId}});
	});

	m_impl->svr.Post("/api/objects/attach", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, host, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = attachChildOnGuiThread(host, body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});

	m_impl->svr.Post("/api/objects/register", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QString id;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, host, body = QByteArray::fromStdString(req.body), &err, &id, &ok]()
			{
				const QJsonDocument doc = QJsonDocument::fromJson(body);
				if (!doc.isObject() || !host)
				{
					err = QStringLiteral("Invalid body.");
					ok = false;
					return;
				}
				cloudsim::core::RegisterObjectDto meta;
				meta.className = doc.object().value(QStringLiteral("className")).toString(QStringLiteral("Frame"));
				meta.name = doc.object().value(QStringLiteral("name")).toString(QStringLiteral("Frame"));
				meta.parentId = doc.object().value(QStringLiteral("parentId")).toString();
				id = host->data().registerObject(meta, &err);
				ok = !id.isEmpty();
				if (ok)
					pushEvent(QStringLiteral("{\"type\":\"BackendObjectRegistered\",\"backendId\":\"%1\"}").arg(id));
			},
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{{QStringLiteral("id"), id}});
	});

	m_impl->svr.Post("/api/objects/coordinate-frame", [this, host](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QString id;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, host, body = QByteArray::fromStdString(req.body), &err, &id, &ok]()
			{ ok = createCoordinateFrameOnGuiThread(host, body, &err, &id); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err,
					QJsonObject{{QStringLiteral("backendId"), id}, {QStringLiteral("id"), id}});
	});

	m_impl->svr.Get("/api/objects/coordinate-frames", [this](const httplib::Request&, httplib::Response& res)
	{
		QByteArray body;
		QMetaObject::invokeMethod(
			this, [this, &body]() { body = coordinateFramesJsonOnGuiThread(); }, Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});

	// P2 robot
	m_impl->svr.Get("/api/robot/programs", [this](const httplib::Request&, httplib::Response& res)
	{
		QByteArray body;
		QMetaObject::invokeMethod(
			this, [this, &body]() { body = robotProgramsJsonOnGuiThread(); }, Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Put("/api/robot/programs", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = setRobotProgramsOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/robot/joints", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = applyJointsOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Get("/api/robot/instances", [this](const httplib::Request&, httplib::Response& res)
	{
		QByteArray body;
		QMetaObject::invokeMethod(
			this, [this, &body]() { body = robotInstancesJsonOnGuiThread(); }, Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Get("/api/robot/joints", [this](const httplib::Request& req, httplib::Response& res)
	{
		QByteArray body;
		const QString rootId = QString::fromStdString(req.get_param_value("sceneRootBackendId"));
		QMetaObject::invokeMethod(
			this, [this, rootId, &body]() { body = robotJointsMetaJsonOnGuiThread(rootId); },
			Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Get("/api/robot/resolve", [this](const httplib::Request& req, httplib::Response& res)
	{
		QByteArray body;
		const QString backendId = QString::fromStdString(req.get_param_value("backendId"));
		QMetaObject::invokeMethod(
			this, [this, backendId, &body]() { body = robotResolveJsonOnGuiThread(backendId); },
			Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Post("/api/robot/place", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = placeRobotOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/robot/tcp-ik", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &extra, &ok]()
			{ ok = tcpIkRobotOnGuiThread(body, &err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Get("/api/robot/tcp-pose", [this](const httplib::Request& req, httplib::Response& res)
	{
		QByteArray body;
		const QString rootId = QString::fromStdString(req.get_param_value("sceneRootBackendId"));
		QMetaObject::invokeMethod(
			this, [this, rootId, &body]() { body = robotTcpPoseJsonOnGuiThread(rootId); },
			Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Get("/api/robot/frames", [this](const httplib::Request& req, httplib::Response& res)
	{
		QByteArray body;
		const QString rootId = QString::fromStdString(req.get_param_value("sceneRootBackendId"));
		QMetaObject::invokeMethod(
			this, [this, rootId, &body]() { body = robotFramesJsonOnGuiThread(rootId); },
			Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Put("/api/robot/frames", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = putRobotFramesOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/robot/frames/mutate", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &extra, &ok]()
			{ ok = mutateRobotFramesOnGuiThread(body, &err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Post("/api/robot/frames/capture-tool", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = captureRobotToolFrameOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/robot/frames/capture-user", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = captureRobotUserFrameOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/robot/frames/reset-tool", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = resetRobotToolFrameOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Get("/api/robot/frames/overlays", [this](const httplib::Request& req, httplib::Response& res)
	{
		QByteArray body;
		const QString rootId = QString::fromStdString(req.get_param_value("sceneRootBackendId"));
		QMetaObject::invokeMethod(
			this, [this, rootId, &body]() { body = robotFrameOverlaysJsonOnGuiThread(rootId); },
			Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Get(R"(/api/robot/instructions/(.+)/properties)",
					[this](const httplib::Request& req, httplib::Response& res)
					{
						QByteArray body;
						const QString id = QString::fromStdString(req.matches[1]);
						QMetaObject::invokeMethod(
							this, [this, id, &body]() { body = instructionPropertiesJsonOnGuiThread(id); },
							Qt::BlockingQueuedConnection);
						res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
					});
	m_impl->svr.Patch(R"(/api/robot/instructions/(.+))",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QString err;
						  bool ok = false;
						  const QString id = QString::fromStdString(req.matches[1]);
						  QMetaObject::invokeMethod(
							  this,
							  [this, id, body = QByteArray::fromStdString(req.body), &err, &ok]()
							  { ok = patchInstructionPropertyOnGuiThread(id, body, &err); },
							  Qt::BlockingQueuedConnection);
						  writeJsonOk(res, ok, err, QJsonObject{});
					  });
	m_impl->svr.Post("/api/robot/urdf/import", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &extra, &ok]()
			{ ok = registerUrdfOnGuiThread(body, &err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Post("/api/robot/plan", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &extra, &ok]()
			{ ok = planInstructionOnGuiThread(body, &err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});

	m_impl->svr.Get("/api/trajectory/session", [this](const httplib::Request&, httplib::Response& res)
	{
		QByteArray body;
		QMetaObject::invokeMethod(this, [this, &body]() { body = trajectorySessionJsonOnGuiThread(); },
								  Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Get("/api/trajectory/path-plans", [this](const httplib::Request& req, httplib::Response& res)
	{
		QByteArray body;
		const QString rootId = QString::fromStdString(req.get_param_value("sceneRootBackendId"));
		QMetaObject::invokeMethod(this, [this, rootId, &body]() { body = trajectoryPathPlansJsonOnGuiThread(rootId); },
								  Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Post("/api/robot/path-plan", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &extra, &ok]()
			{ ok = createPathPlanOnGuiThread(body, &err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Post("/api/trajectory/bind", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = bindPathPlanOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/trajectory/begin-edit", [this](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(this, [this, &err, &ok]() { ok = beginTrajectoryEditOnGuiThread(&err); },
								  Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/trajectory/cancel-edit", [this](const httplib::Request&, httplib::Response& res)
	{
		bool ok = false;
		QMetaObject::invokeMethod(this, [this, &ok]() { ok = cancelTrajectoryEditOnGuiThread(); },
								  Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, QString{}, QJsonObject{});
	});
	m_impl->svr.Post("/api/pick/mesh-element", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &extra, &ok]()
			{ ok = pickMeshElementOnGuiThread(body, &err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Post("/api/pick/hover", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &extra, &ok]()
			{ ok = pickHoverOnGuiThread(body, &err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Get("/api/trajectory/op-schema", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		const QString kind = QString::fromStdString(req.get_param_value("kind"));
		const int opIndex = req.has_param("opIndex") ? QString::fromStdString(req.get_param_value("opIndex")).toInt() : -1;
		QMetaObject::invokeMethod(
			this, [this, kind, opIndex, &err, &extra, &ok]()
			{ ok = trajectoryOpSchemaOnGuiThread(kind, opIndex, &err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Get("/api/trajectory/feature-catalog", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QByteArray body;
		const QString wp = QString::fromStdString(req.get_param_value("workpiece"));
		QMetaObject::invokeMethod(
			this, [this, wp, &body, &err]() { body = featureCatalogOnGuiThread(wp, &err); },
			Qt::BlockingQueuedConnection);
		if (body.isEmpty())
		{
			writeJsonOk(res, false, err.isEmpty() ? QStringLiteral("catalog failed") : err, QJsonObject{});
			return;
		}
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Get("/api/trajectory/feature-schema", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		const QString strategyId = QString::fromStdString(req.get_param_value("strategyId"));
		QMetaObject::invokeMethod(
			this, [this, strategyId, &err, &extra, &ok]()
			{ ok = featureSchemaOnGuiThread(strategyId, &err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Post("/api/trajectory/discretize", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = discretizeFeaturesOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/trajectory/mesh-spec", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = discretizeMeshSpecOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Get("/api/trajectory/pipeline", [this](const httplib::Request&, httplib::Response& res)
	{
		QByteArray body;
		QMetaObject::invokeMethod(this, [this, &body]() { body = trajectoryPipelineJsonOnGuiThread(); },
								  Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Put("/api/trajectory/pipeline", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = setTrajectoryPipelineOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/trajectory/recipe", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = fillTrajectoryRecipeOnGuiThread(body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/trajectory/preview", [this](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(this, [this, &err, &extra, &ok]() { ok = previewTrajectoryOnGuiThread(&err, &extra); },
								  Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	// 特征离散后预览：不对齐管线，只看 Raw→世界（桌面特征页）
	m_impl->svr.Post("/api/trajectory/preview-raw", [this](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(
			this, [this, &err, &extra, &ok]() { ok = previewTrajectoryRawOnGuiThread(&err, &extra); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Post("/api/trajectory/apply", [this](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(this, [this, &err, &ok]() { ok = applyTrajectoryOnGuiThread(&err); },
								  Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/trajectory/emit", [this](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(this, [this, &err, &ok]() { ok = emitTrajectoryRawOnGuiThread(&err); },
								  Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Get("/api/trajectory/op-palette", [this](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		QJsonObject extra;
		bool ok = false;
		QMetaObject::invokeMethod(this, [this, &err, &extra, &ok]() { ok = trajectoryOpPaletteOnGuiThread(&err, &extra); },
								  Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, extra);
	});
	m_impl->svr.Post("/api/trajectory/reset", [this](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(this, [this, &err, &ok]() { ok = resetTrajectoryPipelineOnGuiThread(&err); },
								  Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/trajectory/undo", [this](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(this, [this, &err, &ok]() { ok = undoTrajectoryDraftOnGuiThread(&err); },
								  Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	m_impl->svr.Post("/api/trajectory/redo", [this](const httplib::Request&, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(this, [this, &err, &ok]() { ok = redoTrajectoryDraftOnGuiThread(&err); },
								  Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});
	// 先注册 kind/name，避免被 templates/(.+) 吞掉
	m_impl->svr.Get(R"(/api/trajectory/templates/(.+)/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		QByteArray body;
		const QString kind = QString::fromStdString(req.matches[1]);
		const QString name = QString::fromStdString(req.matches[2]);
		bool ok = false;
		QMetaObject::invokeMethod(
			this, [this, kind, name, &body, &err, &ok]()
			{ ok = loadTrajectoryTemplateOnGuiThread(kind, name, &body, &err); },
			Qt::BlockingQueuedConnection);
		if (!ok)
		{
			writeJsonOk(res, false, err, QJsonObject{});
			return;
		}
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Delete(R"(/api/trajectory/templates/(.+)/(.+))",
					   [this](const httplib::Request& req, httplib::Response& res)
					   {
						   QString err;
						   bool ok = false;
						   const QString kind = QString::fromStdString(req.matches[1]);
						   const QString name = QString::fromStdString(req.matches[2]);
						   QMetaObject::invokeMethod(
							   this, [this, kind, name, &err, &ok]()
							   { ok = deleteTrajectoryTemplateOnGuiThread(kind, name, &err); },
							   Qt::BlockingQueuedConnection);
						   writeJsonOk(res, ok, err, QJsonObject{});
					   });
	m_impl->svr.Get(R"(/api/trajectory/templates/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		QByteArray body;
		const QString kind = QString::fromStdString(req.matches[1]);
		QMetaObject::invokeMethod(this, [this, kind, &body]() { body = listTrajectoryTemplatesOnGuiThread(kind); },
								  Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Post(R"(/api/trajectory/templates/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		QString err;
		bool ok = false;
		const QString kind = QString::fromStdString(req.matches[1]);
		QMetaObject::invokeMethod(
			this,
			[this, kind, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = saveTrajectoryTemplateOnGuiThread(kind, body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});

	m_impl->svr.Post("/api/robot/run",
					  [](const httplib::Request&, httplib::Response& res)
					  {
						  res.set_content(R"({"ok":true,"status":"accepted","note":"playback orchestrated by client SSE pose stream"})",
										  "application/json; charset=utf-8");
					  });
	m_impl->svr.Post("/api/robot/stop",
					  [](const httplib::Request&, httplib::Response& res)
					  {
						  res.set_content(R"({"ok":true,"status":"stopped"})", "application/json; charset=utf-8");
					  });
	m_impl->svr.Post("/api/robot/export",
					  [](const httplib::Request& req, httplib::Response& res)
					  {
						  QJsonObject o;
						  o.insert(QStringLiteral("ok"), true);
						  o.insert(QStringLiteral("format"), QStringLiteral("canonical-v1"));
						  o.insert(QStringLiteral("note"),
								   QStringLiteral("Brand scripts invoked out-of-process; body echoed for client pipeline"));
						  o.insert(QStringLiteral("requestBytes"), static_cast<int>(req.body.size()));
						  const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
						  res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
					  });

	// P3 geometry / point cloud (capability surface; heavy work via Host algorithms when linked)
	m_impl->svr.Post("/api/geometry/op",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(req.body));
						  const QString op =
							  doc.isObject() ? doc.object().value(QStringLiteral("op")).toString() : QString();
						  QJsonObject o;
						  o.insert(QStringLiteral("ok"), true);
						  o.insert(QStringLiteral("queued"), true);
						  o.insert(QStringLiteral("op"), op);
						  o.insert(QStringLiteral("note"),
								   QStringLiteral("GeometryAlgorithm jobs accepted; progress via SSE GeometryJobProgress"));
						  pushEvent(QStringLiteral("{\"type\":\"GeometryJobProgress\",\"op\":\"%1\",\"progress\":1.0}")
										.arg(op));
						  const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
						  res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
					  });
	m_impl->svr.Post("/api/pointcloud/op",
					  [this](const httplib::Request& req, httplib::Response& res)
					  {
						  QByteArray body;
						  QMetaObject::invokeMethod(
							  this,
							  [this, reqBody = QByteArray::fromStdString(req.body), &body]()
							  {
								  body = pointCloudPostJsonOnGuiThread(
									  reqBody, &cloudsim::host::HeadlessPointCloudBridge::deprecatedOp);
							  },
							  Qt::BlockingQueuedConnection);
						  res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
					  });
	registerPointCloudRoutes(host);

	// P4 modes / sidecars
	m_impl->svr.Get("/api/modes", [this](const httplib::Request&, httplib::Response& res)
	{
		const QByteArray body = modesCatalogJson();
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Get(R"(/api/sidecar/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		const QString key = QString::fromStdString(req.matches[1]);
		QByteArray body;
		QMetaObject::invokeMethod(
			this, [this, key, &body]() { body = sidecarGetOnGuiThread(key); }, Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Put(R"(/api/sidecar/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		const QString key = QString::fromStdString(req.matches[1]);
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, key, body = QByteArray::fromStdString(req.body), &err, &ok]()
			{ ok = sidecarPutOnGuiThread(key, body, &err); },
			Qt::BlockingQueuedConnection);
		writeJsonOk(res, ok, err, QJsonObject{});
	});

	// P5 AI / help / devices
	m_impl->svr.Get("/api/ai/status", [this](const httplib::Request&, httplib::Response& res)
	{
		const QByteArray body = aiStatusJson();
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Post("/api/ai/chat",
					 [](const httplib::Request& req, httplib::Response& res)
					 {
						 QJsonObject o;
						 o.insert(QStringLiteral("ok"), true);
						 o.insert(QStringLiteral("role"), QStringLiteral("assistant"));
						 o.insert(QStringLiteral("content"),
								  QStringLiteral("AI bridge stub — configure ai_config.json; request bytes=%1")
									  .arg(static_cast<int>(req.body.size())));
						 const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
						 res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
					 });
	m_impl->svr.Get("/api/help", [this](const httplib::Request&, httplib::Response& res)
	{
		const QByteArray body = helpIndexJson();
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});
	m_impl->svr.Get(R"(/api/i18n/(.+))",
					[](const httplib::Request& req, httplib::Response& res)
					{
						const QString lang = QString::fromStdString(req.matches[1]);
						QJsonObject o;
						o.insert(QStringLiteral("lang"), lang);
						o.insert(QStringLiteral("strings"),
								 QJsonObject{{QStringLiteral("app.title"), QStringLiteral("CloudSim Web")},
											 {QStringLiteral("menu.file"), QStringLiteral("文件")}});
						const QByteArray out = QJsonDocument(o).toJson(QJsonDocument::Compact);
						res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
					});
	m_impl->svr.Get("/api/devices/catalog",
					[](const httplib::Request&, httplib::Response& res)
					{
						const QString modelsRoot =
							QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("resource/models"));
						const QJsonObject catalog = scanDeviceCatalog(modelsRoot);
						const QByteArray out = QJsonDocument(catalog).toJson(QJsonDocument::Compact);
						res.set_content(out.constData(), out.size(), "application/json; charset=utf-8");
					});
	m_impl->svr.Get("/api/devices/thumbnail",
					[](const httplib::Request& req, httplib::Response& res)
					{
						const QString path = QString::fromStdString(req.get_param_value("path"));
						const QString modelsRoot = QDir::cleanPath(
							QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("resource/models")));
						const QString canon = QFileInfo(path).canonicalFilePath();
						if (canon.isEmpty() || !canon.startsWith(modelsRoot, Qt::CaseInsensitive) ||
							!QFileInfo::exists(canon))
						{
							res.status = 404;
							res.set_content(R"({"ok":false,"error":"thumbnail not found"})",
											"application/json; charset=utf-8");
							return;
						}
						QFile f(canon);
						if (!f.open(QIODevice::ReadOnly))
						{
							res.status = 404;
							res.set_content(R"({"ok":false,"error":"cannot read thumbnail"})",
											"application/json; charset=utf-8");
							return;
						}
						const QByteArray bytes = f.readAll();
						const QString suf = QFileInfo(canon).suffix().toLower();
						const char* mime = "image/png";
						if (suf == QLatin1String("jpg") || suf == QLatin1String("jpeg"))
							mime = "image/jpeg";
						else if (suf == QLatin1String("webp"))
							mime = "image/webp";
						else if (suf == QLatin1String("bmp"))
							mime = "image/bmp";
						res.set_content(bytes.constData(), bytes.size(), mime);
					});
	m_impl->svr.Get("/api/devices/plc",
					[](const httplib::Request&, httplib::Response& res)
					{
						res.set_content(R"({"ok":true,"devices":[],"note":"PLC panel deferred"})",
										"application/json; charset=utf-8");
					});
	m_impl->svr.Get("/api/devices/camera",
					[](const httplib::Request&, httplib::Response& res)
					{
						res.set_content(R"({"ok":true,"devices":[],"note":"Industrial camera panel deferred"})",
										"application/json; charset=utf-8");
					});

	Q_UNUSED(host);
}

bool WebGateway::openProjectOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err,
										int* objectCount)
{
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject())
	{
		if (err)
			*err = QStringLiteral("Invalid JSON body.");
		return false;
	}
	QString openPath = doc.object().value(QStringLiteral("path")).toString().trimmed();
	if (!openPath.isEmpty() && QFileInfo(openPath).isDir())
		openPath = resolveProjectPathFromPicked(openPath);
	if (openPath.isEmpty() || !QFileInfo::exists(openPath))
	{
		if (err)
			*err = QStringLiteral("Path missing or not found (file .pcp/.json, or folder with project.json).");
		return false;
	}

	// 先读盘再顾问锁：若先 CreateFile 且只 SHARE_READ，Qt QFile 会因要 SHARE_WRITE 而打不开
	releaseProjectFileLock();

	QString projectJsonPath = openPath;
	QString projectDir = QFileInfo(openPath).absolutePath();
	m_impl->extractDir.reset();
	if (isStoreZipArchive(openPath))
	{
		m_impl->extractDir = std::make_unique<QTemporaryDir>();
		if (!m_impl->extractDir->isValid())
		{
			if (err)
				*err = QStringLiteral("Cannot create temp dir.");
			return false;
		}
		QString zipErr;
		if (!extractStoreZipArchive(openPath, m_impl->extractDir->path(), &zipErr))
		{
			if (err)
				*err = zipErr.isEmpty() ? QStringLiteral("Failed to unpack .pcp") : zipErr;
			return false;
		}
		projectJsonPath = QDir(m_impl->extractDir->path()).filePath(QStringLiteral("project.json"));
		projectDir = m_impl->extractDir->path();
		if (!QFileInfo::exists(projectJsonPath))
		{
			if (err)
				*err = QStringLiteral("Archive has no project.json.");
			return false;
		}
	}

	QFile file(projectJsonPath);
	if (!file.open(QIODevice::ReadOnly))
	{
		if (err)
			*err = QStringLiteral("Failed to open project.json (%1).").arg(projectJsonPath);
		return false;
	}
	const QJsonDocument jsonDoc = QJsonDocument::fromJson(file.readAll());
	file.close();
	if (!jsonDoc.isObject())
	{
		if (err)
			*err = QStringLiteral("Invalid project.json.");
		return false;
	}
	const QJsonObject root = jsonDoc.object();
	if (root.value(QStringLiteral("version")).toInt(0) != 4)
	{
		if (err)
			*err = QStringLiteral("Unsupported project version (expect v4).");
		return false;
	}

	host->data().clear();
	host->backendSourcePath().clear();
	host->backendSourceType().clear();
	host->backendParentId().clear();
	if (cloudsim::host::HeadlessRobotContext* hrc = host->headlessRobotContext())
		hrc->clearRobotSimulationContext();

	const QVector<cloudsim::host::ProjectHierarchyEdge> pendingEdges =
		cloudsim::host::parseProjectEdgesJson(root.value(QStringLiteral("edges")).toArray());
	const bool useEdgesRelation = !pendingEdges.isEmpty();
	const QSet<QString> robotLinkMeshBackendIds = cloudsim::host::collectRobotLinkMeshBackendIds(root);

	cloudsim::host::ProjectObjectLoadOptions loadOpts;
	loadOpts.projectDir = projectDir;
	loadOpts.useEdgesRelation = useEdgesRelation;
	loadOpts.robotLinkMeshBackendIds = robotLinkMeshBackendIds;

	cloudsim::host::ProjectObjectLoadCallbacks loadCbs;
	loadCbs.legacyParentFollow = [](const std::string&, const std::string&) {};
	loadCbs.pointCloudWidgetImport = [](cloudsim::host::DocumentHost& h, const QString& loadPath,
										const QString& persistedId, QString& outImportedId, QString* outError) -> bool
	{
		cloudsim::core::ImportOptionsDto opt;
		opt.quietUi = true;
		opt.resetViewToHome = false;
		opt.persistedId = persistedId;
		opt.catalogTypeName = QLatin1String(backend_type::kCatalogPointCloud);
		const cloudsim::host::ImportFileResult imported = cloudsim::host::importFileIntoDocument(
			h, loadPath, cloudsim::host::ImportFileKind::PointCloud, opt, outError);
		outImportedId = imported.rootBackendId;
		return imported.ok;
	};

	QStringList warnings;
	cloudsim::host::loadProjectObjectsFromJson(*host, root.value(QStringLiteral("objects")).toArray(), loadOpts,
											   loadCbs, &warnings);
	cloudsim::host::finalizeProjectHierarchyAfterObjects(*host, useEdgesRelation, pendingEdges, &warnings);
	(void)cloudsim::host::loadRobotProgramsFromProjectJson(*host, root, nullptr);
	cloudsim::host::finalizeProjectLoadFollowAndViewport(*host, root, useEdgesRelation, pendingEdges, nullptr);

	// Headless 无 OSG finalize 早退；在此恢复 FK 绑定并写回连杆矩阵
	if (cloudsim::host::IRobotUrdfImportContext* ctx = host->robotUrdfImportContext())
	{
		const cloudsim::host::RobotKinematicsRestoreResult kin =
			cloudsim::host::restoreRobotKinematicsFromProjectJson(*ctx, root);
		if (kin.restoredInstanceCount > 0 && !kin.aggregatedJointAnglesRad.isEmpty())
		{
			QString kinErr;
			(void)cloudsim::host::applyRestoredJointAnglesToScene(*ctx, kin.aggregatedJointAnglesRad, &kinErr);
			if (cloudsim::host::HeadlessRobotContext* hrc = host->headlessRobotContext())
			{
				int offset = 0;
				for (const auto& info : hrc->listInstances())
				{
					QVector<double> local;
					local.reserve(info.jointCount);
					for (int i = 0; i < info.jointCount && offset + i < kin.aggregatedJointAnglesRad.size(); ++i)
						local.append(kin.aggregatedJointAnglesRad[offset + i]);
					while (local.size() < info.jointCount)
						local.append(0.0);
					hrc->recordJointAnglesForSceneRoot(info.sceneRootBackendId, local);
					offset += info.jointCount;
				}
			}
		}
	}
	webGatewayLoadSidecarsFromProject(root);

	host->setProjectFilePath(openPath);
	m_impl->projectPath = openPath;
	m_impl->projectDir = projectDir;
	reacquireProjectFileLock(openPath);
	if (objectCount)
		*objectCount = host->data().listAll().size();

	pushEvent(QStringLiteral("{\"type\":\"ProjectLoaded\",\"path\":\"%1\",\"objectCount\":%2}")
				  .arg(jsonEscape(openPath))
				  .arg(objectCount ? *objectCount : 0));
	return true;
}

QByteArray WebGateway::objectsJsonOnGuiThread()
{
	if (!m_document)
	{
		return QByteArrayLiteral("{\"objects\":[]}");
	}
	auto* host = dynamic_cast<cloudsim::host::DocumentHost*>(m_document.get());
	auto& data = m_document->data();
	QJsonArray arr;
	for (const auto& snap : data.listObjectSnapshots())
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), snap.id);
		o.insert(QStringLiteral("name"), snap.name);
		o.insert(QStringLiteral("className"), snap.className);
		o.insert(QStringLiteral("visible"), snap.visible);
		o.insert(QStringLiteral("hasGeometry"), snap.hasGeometry);
		o.insert(QStringLiteral("geometryKind"), static_cast<int>(snap.geometryKind));
		QJsonArray parents;
		for (const auto& p : snap.parentIds)
			parents.append(p);
		QJsonArray children;
		for (const auto& c : snap.childIds)
			children.append(c);
			o.insert(QStringLiteral("parentIds"), parents);
		o.insert(QStringLiteral("childIds"), children);
		o.insert(QStringLiteral("pose"), poseToJson(data.worldPoseMm(snap.id)));
		// 导出 Three.js/OpenGL 列主序，避免把 BackendMat4/OSG 布局直接塞给 fromArray
		if (host)
		{
			if (const auto obj = host->findObject(snap.id.toStdString()))
			{
				o.insert(QStringLiteral("worldMatrix"), worldMatrixForThreeJs(obj->worldMatrix()));
				if (obj->hasColorProperty())
				{
					const BackendColor c = obj->color();
					o.insert(QStringLiteral("color"),
							 QJsonObject{{QStringLiteral("r"), c.r},
										 {QStringLiteral("g"), c.g},
										 {QStringLiteral("b"), c.b},
										 {QStringLiteral("a"), c.a}});
				}
			}
		}
		arr.append(o);
	}
	QJsonObject root;
	root.insert(QStringLiteral("objects"), arr);
	root.insert(QStringLiteral("projectPath"), m_impl->projectPath);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::meshSoupOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, std::vector<float>& out,
									 QString* err)
{
	return cloudsim::host::exportBackendTriangleSoupMm(*host, id, out, err);
}

void WebGateway::registerPointCloudRoutes(cloudsim::host::DocumentHost*)
{
	auto postPc = [this](const char* path,
						 QJsonObject (cloudsim::host::HeadlessPointCloudBridge::*method)(const QJsonObject&))
	{
		m_impl->svr.Post(
			path,
			[this, method](const httplib::Request& req, httplib::Response& res)
			{
				QByteArray body;
				QMetaObject::invokeMethod(
					this,
					[this, reqBody = QByteArray::fromStdString(req.body), method, &body]()
					{ body = pointCloudPostJsonOnGuiThread(reqBody, method); },
					Qt::BlockingQueuedConnection);
				res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
			});
	};

	m_impl->svr.Get(R"(/api/pointcloud/info/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		const QString id = QString::fromStdString(req.matches[1]);
		QByteArray body;
		QMetaObject::invokeMethod(
			this, [this, id, &body]() { body = pointCloudInfoJsonOnGuiThread(id); }, Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});

	m_impl->svr.Get(R"(/api/pointcloud/measure/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		const QString id = QString::fromStdString(req.matches[1]);
		QByteArray body;
		QMetaObject::invokeMethod(
			this, [this, id, &body]() { body = pointCloudMeasureJsonOnGuiThread(id); }, Qt::BlockingQueuedConnection);
		res.set_content(body.constData(), body.size(), "application/json; charset=utf-8");
	});

	m_impl->svr.Get(R"(/api/pointcloud/preview/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		const QString id = QString::fromStdString(req.matches[1]);
		std::size_t maxPts = cloudsim::host::HeadlessPointCloudBridge::kDefaultPreviewMaxPoints;
		if (const auto v = req.get_param_value("maxPoints"); !v.empty())
		{
			const int parsed = std::atoi(v.c_str());
			maxPts = static_cast<std::size_t>(parsed > 0 ? parsed : 0);
		}
		std::vector<float> soup;
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, id, maxPts, &soup, &err, &ok]() { ok = pointCloudPreviewSoupOnGuiThread(id, maxPts, soup, &err); },
			Qt::BlockingQueuedConnection);
		if (!ok)
		{
			res.status = 404;
			res.set_content(err.toUtf8().constData(), "text/plain; charset=utf-8");
			return;
		}
		res.set_content(reinterpret_cast<const char*>(soup.data()), soup.size() * sizeof(float),
						"application/octet-stream");
	});

	m_impl->svr.Get(R"(/api/pointcloud/chunk/(.+))", [this](const httplib::Request& req, httplib::Response& res)
	{
		const QString id = QString::fromStdString(req.matches[1]);
		const int lod = req.has_param("lod") ? std::atoi(req.get_param_value("lod").c_str()) : 0;
		const int index = req.has_param("index") ? std::atoi(req.get_param_value("index").c_str()) : 0;
		std::size_t maxPts = cloudsim::host::HeadlessPointCloudBridge::kChunkPointCount;
		if (const auto v = req.get_param_value("maxPoints"); !v.empty())
		{
			const int parsed = std::atoi(v.c_str());
			maxPts = static_cast<std::size_t>(parsed > 0 ? parsed : 0);
		}
		std::vector<float> soup;
		QJsonObject meta;
		QString err;
		bool ok = false;
		QMetaObject::invokeMethod(
			this,
			[this, id, lod, index, maxPts, &soup, &meta, &err, &ok]()
			{ ok = pointCloudChunkSoupOnGuiThread(id, lod, index, maxPts, soup, &meta, &err); },
			Qt::BlockingQueuedConnection);
		if (!ok)
		{
			res.status = 404;
			res.set_content(err.toUtf8().constData(), "text/plain; charset=utf-8");
			return;
		}
		res.set_header("X-Chunk-Meta", QJsonDocument(meta).toJson(QJsonDocument::Compact).constData());
		res.set_content(reinterpret_cast<const char*>(soup.data()), soup.size() * sizeof(float),
						"application/octet-stream");
	});

	postPc("/api/pointcloud/downsample", &cloudsim::host::HeadlessPointCloudBridge::downsample);
	postPc("/api/pointcloud/crop", &cloudsim::host::HeadlessPointCloudBridge::crop);
	postPc("/api/pointcloud/preprocess", &cloudsim::host::HeadlessPointCloudBridge::preprocess);
	postPc("/api/pointcloud/register", &cloudsim::host::HeadlessPointCloudBridge::registerCloud);
	postPc("/api/pointcloud/reconstruct", &cloudsim::host::HeadlessPointCloudBridge::reconstruct);
	postPc("/api/pointcloud/mesh/post", &cloudsim::host::HeadlessPointCloudBridge::meshPost);
	postPc("/api/pointcloud/mesh/export-ply", &cloudsim::host::HeadlessPointCloudBridge::meshExportPly);
	postPc("/api/pointcloud/surface/run", &cloudsim::host::HeadlessPointCloudBridge::surfaceRun);
	postPc("/api/pointcloud/surface/reset", &cloudsim::host::HeadlessPointCloudBridge::surfaceReset);
}

bool WebGateway::selectionOnGuiThread(const QByteArray& body, QString* err)
{
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject() || !m_document)
	{
		if (err)
			*err = QStringLiteral("Invalid body.");
		return false;
	}
	const QString backendId = doc.object().value(QStringLiteral("backendId")).toString();
	cloudsim::core::SelectionChangedEvent ev;
	ev.documentId = m_document->documentId();
	ev.primaryId = backendId;
	ev.source = cloudsim::core::SelectionSource::OsgPick;
	m_context.events().publish(ev);
	pushEvent(QStringLiteral("{\"type\":\"SelectionChanged\",\"backendId\":\"%1\"}").arg(jsonEscape(backendId)));
	return true;
}

QByteArray WebGateway::nativeDialogOnGuiThread(const QByteArray& body)
{
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	const QJsonObject o = doc.isObject() ? doc.object() : QJsonObject{};
	const QString purpose = o.value(QStringLiteral("purpose")).toString(QStringLiteral("project"));
	QString startDir = o.value(QStringLiteral("startDir")).toString();
	if (startDir.isEmpty() && !m_impl->projectPath.isEmpty())
		startDir = QFileInfo(m_impl->projectPath).absolutePath();

	// 无主窗时对话框易被浏览器盖住；用置顶临时父窗保证可见
	QWidget dialogParent;
	dialogParent.setWindowFlags(Qt::Tool | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
	dialogParent.resize(1, 1);
	dialogParent.show();
	dialogParent.raise();
	dialogParent.activateWindow();
	QApplication::alert(&dialogParent);

	QJsonObject out;
	QString path;
	if (purpose == QStringLiteral("directory") || purpose == QStringLiteral("folder"))
	{
		path = QFileDialog::getExistingDirectory(&dialogParent, QStringLiteral("选择工程文件夹"), startDir,
												 QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
		if (path.isEmpty())
		{
			out.insert(QStringLiteral("ok"), false);
			out.insert(QStringLiteral("cancelled"), true);
			return QJsonDocument(out).toJson(QJsonDocument::Compact);
		}
		const QString resolved = resolveProjectPathFromPicked(path);
		if (resolved.isEmpty())
		{
			out.insert(QStringLiteral("ok"), false);
			out.insert(QStringLiteral("error"),
					   QStringLiteral("文件夹内未找到 project.json 或唯一的 .pcp"));
			out.insert(QStringLiteral("folder"), path);
			return QJsonDocument(out).toJson(QJsonDocument::Compact);
		}
		path = resolved;
	}
	else if (purpose == QStringLiteral("saveProject"))
	{
		path = QFileDialog::getSaveFileName(&dialogParent, QStringLiteral("保存工程"), startDir,
											QStringLiteral("Point Cloud Package (*.pcp);;JSON Files (*.json)"));
		if (path.isEmpty())
		{
			out.insert(QStringLiteral("ok"), false);
			out.insert(QStringLiteral("cancelled"), true);
			return QJsonDocument(out).toJson(QJsonDocument::Compact);
		}
	}
	else if (purpose == QStringLiteral("pointcloud"))
	{
		// 点云页导入：勿走默认 project（.pcp），与桌面「导入 PLY/XYZ」对齐
		const QString caption = o.value(QStringLiteral("title")).toString(QStringLiteral("导入点云"));
		const QString filter =
			o.value(QStringLiteral("filter"))
				.toString(QStringLiteral("点云 (*.ply *.xyz *.pcd *.las *.laz);;所有文件 (*.*)"));
		path = QFileDialog::getOpenFileName(&dialogParent, caption, startDir, filter);
		if (path.isEmpty())
		{
			out.insert(QStringLiteral("ok"), false);
			out.insert(QStringLiteral("cancelled"), true);
			return QJsonDocument(out).toJson(QJsonDocument::Compact);
		}
	}
	else if (purpose == QStringLiteral("import") || purpose == QStringLiteral("file"))
	{
		const QString caption = o.value(QStringLiteral("title")).toString(QStringLiteral("选择文件"));
		const QString filter =
			o.value(QStringLiteral("filter"))
				.toString(QStringLiteral(
					"Models (*.stl *.obj *.ply *.step *.stp *.iges *.igs);;Point Clouds (*.pcd *.ply *.las "
					"*.laz);;All Files (*.*)"));
		path = QFileDialog::getOpenFileName(&dialogParent, caption, startDir, filter);
		if (path.isEmpty())
		{
			out.insert(QStringLiteral("ok"), false);
			out.insert(QStringLiteral("cancelled"), true);
			return QJsonDocument(out).toJson(QJsonDocument::Compact);
		}
	}
	else if (purpose == QStringLiteral("urdf"))
	{
		const QString caption = o.value(QStringLiteral("title")).toString(QStringLiteral("选择 URDF"));
		path = QFileDialog::getOpenFileName(&dialogParent, caption, startDir,
											QStringLiteral("URDF (*.urdf);;Xacro (*.xacro);;All Files (*.*)"));
		if (path.isEmpty())
		{
			out.insert(QStringLiteral("ok"), false);
			out.insert(QStringLiteral("cancelled"), true);
			return QJsonDocument(out).toJson(QJsonDocument::Compact);
		}
	}
	else if (purpose == QStringLiteral("saveFile") || purpose == QStringLiteral("save"))
	{
		const QString caption = o.value(QStringLiteral("title")).toString(QStringLiteral("保存文件"));
		const QString filter = o.value(QStringLiteral("filter")).toString(QStringLiteral("All Files (*.*)"));
		path = QFileDialog::getSaveFileName(&dialogParent, caption, startDir, filter);
		if (path.isEmpty())
		{
			out.insert(QStringLiteral("ok"), false);
			out.insert(QStringLiteral("cancelled"), true);
			return QJsonDocument(out).toJson(QJsonDocument::Compact);
		}
	}
	else
	{
		const QString caption = o.value(QStringLiteral("title")).toString(QStringLiteral("打开工程"));
		path = QFileDialog::getOpenFileName(
			&dialogParent, caption, startDir,
			QStringLiteral(
				"Point Cloud Package (*.pcp);;PointCloud Project (*.pcproj.json);;JSON Files (*.json);;All Files (*.*)"));
		if (path.isEmpty())
		{
			out.insert(QStringLiteral("ok"), false);
			out.insert(QStringLiteral("cancelled"), true);
			return QJsonDocument(out).toJson(QJsonDocument::Compact);
		}
	}

	out.insert(QStringLiteral("ok"), true);
	out.insert(QStringLiteral("path"), QDir::toNativeSeparators(path));
	return QJsonDocument(out).toJson(QJsonDocument::Compact);
}

} // namespace cloudsim::web
