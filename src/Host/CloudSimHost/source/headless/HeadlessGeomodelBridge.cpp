/// @file HeadlessGeomodelBridge.cpp
/// @brief 网页几何建模：直接写 ParametricBrep + rebuild，不经 Qt 插件 UI

#include "headless/HeadlessGeomodelBridge.h"

#include "BackendDataBase.h"
#include "BackendFileImport.h"
#include "BackendTypeIds.h"
#include "DocumentHost.h"
#include "ParametricBrepBackendData.h"
#include "ParametricBrepFeature.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cloudsim::host
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxUndo = 48;

QJsonObject fail(const QString& err)
{
	return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
}

QJsonObject nlohmannToQJson(const nlohmann::json& j)
{
	const std::string dumped = j.dump();
	const QJsonDocument doc = QJsonDocument::fromJson(QByteArray(dumped.data(), static_cast<int>(dumped.size())));
	return doc.object();
}

nlohmann::json qJsonToNlohmann(const QJsonObject& o)
{
	const QByteArray bytes = QJsonDocument(o).toJson(QJsonDocument::Compact);
	try
	{
		return nlohmann::json::parse(bytes.constData(), bytes.constData() + bytes.size());
	}
	catch (const std::exception&)
	{
		return nlohmann::json::object();
	}
}

ParametricSketchPlane planeFromId(const QString& id)
{
	ParametricSketchPlane p;
	p.isPlanar = true;
	const QString key = id.trimmed().toUpper();
	if (key == QStringLiteral("XZ"))
	{
		p.axisXX = 1;
		p.axisXY = 0;
		p.axisXZ = 0;
		p.axisYX = 0;
		p.axisYY = 0;
		p.axisYZ = 1;
		p.normalX = 0;
		p.normalY = -1;
		p.normalZ = 0;
	}
	else if (key == QStringLiteral("YZ"))
	{
		p.axisXX = 0;
		p.axisXY = 1;
		p.axisXZ = 0;
		p.axisYX = 0;
		p.axisYY = 0;
		p.axisYZ = 1;
		p.normalX = 1;
		p.normalY = 0;
		p.normalZ = 0;
	}
	return p;
}

ParametricSketchPlane offsetPlane(ParametricSketchPlane p, double distMm)
{
	p.originX += p.normalX * distMm;
	p.originY += p.normalY * distMm;
	p.originZ += p.normalZ * distMm;
	return p;
}

std::vector<float> embedOnPlane(const std::vector<float>& uv, const ParametricSketchPlane& pl)
{
	std::vector<float> w;
	w.reserve(uv.size());
	for (size_t i = 0; i + 2 < uv.size(); i += 3)
	{
		const double u = static_cast<double>(uv[i]);
		const double v = static_cast<double>(uv[i + 1]);
		w.push_back(static_cast<float>(pl.originX + u * pl.axisXX + v * pl.axisYX));
		w.push_back(static_cast<float>(pl.originY + u * pl.axisXY + v * pl.axisYY));
		w.push_back(static_cast<float>(pl.originZ + u * pl.axisXZ + v * pl.axisYZ));
	}
	return w;
}

std::vector<float> closedRectXy(double lengthMm, double widthMm)
{
	const float L = static_cast<float>(lengthMm);
	const float W = static_cast<float>(widthMm);
	return {0.f, 0.f, 0.f, L, 0.f, 0.f, L, W, 0.f, 0.f, W, 0.f, 0.f, 0.f, 0.f};
}

std::vector<float> closedCircleXy(double cx, double cy, double radiusMm, int segments)
{
	const int n = segments < 8 ? 8 : segments;
	std::vector<float> poly;
	poly.reserve(static_cast<size_t>(n + 1) * 3);
	for (int i = 0; i <= n; ++i)
	{
		const double a = (2.0 * kPi * (i % n)) / n;
		poly.push_back(static_cast<float>(cx + radiusMm * std::cos(a)));
		poly.push_back(static_cast<float>(cy + radiusMm * std::sin(a)));
		poly.push_back(0.f);
	}
	return poly;
}

std::vector<float> closedPolygonXy(int sides, double radiusMm)
{
	const int n = sides < 3 ? 3 : (sides > 24 ? 24 : sides);
	std::vector<float> poly;
	poly.reserve(static_cast<size_t>(n + 1) * 3);
	for (int i = 0; i <= n; ++i)
	{
		const double a = (2.0 * kPi * (i % n)) / n;
		poly.push_back(static_cast<float>(radiusMm * std::cos(a)));
		poly.push_back(static_cast<float>(radiusMm * std::sin(a)));
		poly.push_back(0.f);
	}
	return poly;
}

std::vector<float> closedEllipseXy(double rx, double ry, int segments)
{
	const int n = segments < 8 ? 8 : segments;
	std::vector<float> poly;
	poly.reserve(static_cast<size_t>(n + 1) * 3);
	for (int i = 0; i <= n; ++i)
	{
		const double a = (2.0 * kPi * (i % n)) / n;
		poly.push_back(static_cast<float>(rx * std::cos(a)));
		poly.push_back(static_cast<float>(ry * std::sin(a)));
		poly.push_back(0.f);
	}
	return poly;
}

std::vector<float> closedSlotXy(double lengthMm, double widthMm, int segments)
{
	const double w = widthMm <= 0.1 ? 0.1 : widthMm;
	const double r = w * 0.5;
	double L = lengthMm < w ? w : lengthMm;
	const double halfStraight = (L * 0.5) - r;
	const int n = segments < 8 ? 8 : segments;
	std::vector<float> poly;
	poly.reserve(static_cast<size_t>(n * 2 + 2) * 3);
	for (int i = 0; i <= n; ++i)
	{
		const double a = -kPi * 0.5 + (kPi * i) / n;
		poly.push_back(static_cast<float>(halfStraight + r * std::cos(a)));
		poly.push_back(static_cast<float>(r * std::sin(a)));
		poly.push_back(0.f);
	}
	for (int i = 0; i <= n; ++i)
	{
		const double a = kPi * 0.5 + (kPi * i) / n;
		poly.push_back(static_cast<float>(-halfStraight + r * std::cos(a)));
		poly.push_back(static_cast<float>(r * std::sin(a)));
		poly.push_back(0.f);
	}
	if (!poly.empty())
	{
		poly.push_back(poly[0]);
		poly.push_back(poly[1]);
		poly.push_back(poly[2]);
	}
	return poly;
}

std::vector<float> polylineFromJson(const QJsonValue& v)
{
	std::vector<float> out;
	if (!v.isArray())
		return out;
	const QJsonArray a = v.toArray();
	out.reserve(static_cast<size_t>(a.size()));
	for (const QJsonValue& x : a)
		out.push_back(static_cast<float>(x.toDouble()));
	return out;
}

std::vector<int> intListFromJson(const QJsonValue& v)
{
	std::vector<int> out;
	if (!v.isArray())
		return out;
	const QJsonArray a = v.toArray();
	out.reserve(static_cast<size_t>(a.size()));
	for (const QJsonValue& x : a)
		out.push_back(x.toInt());
	return out;
}

std::vector<float> profileFromBody(const QJsonObject& body)
{
	const QString profile = body.value(QStringLiteral("profile")).toString(QStringLiteral("rectangle"));
	if (profile == QStringLiteral("polyline"))
		return polylineFromJson(body.value(QStringLiteral("polyline")));
	if (profile == QStringLiteral("circle"))
	{
		const double r = body.value(QStringLiteral("radiusMm")).toDouble(50.0);
		return closedCircleXy(0.0, 0.0, r, body.value(QStringLiteral("segments")).toInt(32));
	}
	if (profile == QStringLiteral("polygon"))
	{
		return closedPolygonXy(body.value(QStringLiteral("sides")).toInt(6),
							   body.value(QStringLiteral("radiusMm")).toDouble(50.0));
	}
	if (profile == QStringLiteral("ellipse"))
	{
		return closedEllipseXy(body.value(QStringLiteral("radiusMm")).toDouble(
								   body.value(QStringLiteral("lengthMm")).toDouble(60.0)),
							   body.value(QStringLiteral("radiusBMm")).toDouble(
								   body.value(QStringLiteral("widthMm")).toDouble(30.0)),
							   body.value(QStringLiteral("segments")).toInt(32));
	}
	if (profile == QStringLiteral("slot"))
	{
		return closedSlotXy(body.value(QStringLiteral("lengthMm")).toDouble(80.0),
							body.value(QStringLiteral("widthMm")).toDouble(20.0),
							body.value(QStringLiteral("segments")).toInt(12));
	}
	return closedRectXy(body.value(QStringLiteral("lengthMm")).toDouble(100.0),
						body.value(QStringLiteral("widthMm")).toDouble(100.0));
}

std::vector<float> defaultSweepPath(const ParametricSketchPlane& pl, double lenMm,
								   const std::vector<float>& profileWorld)
{
	double ox = pl.originX;
	double oy = pl.originY;
	double oz = pl.originZ;
	if (profileWorld.size() >= 12)
	{
		size_t n = profileWorld.size() / 3;
		if (n > 1)
			--n;
		ox = oy = oz = 0.0;
		for (size_t i = 0; i < n; ++i)
		{
			ox += static_cast<double>(profileWorld[i * 3]);
			oy += static_cast<double>(profileWorld[i * 3 + 1]);
			oz += static_cast<double>(profileWorld[i * 3 + 2]);
		}
		ox /= static_cast<double>(n);
		oy /= static_cast<double>(n);
		oz /= static_cast<double>(n);
	}
	return {static_cast<float>(ox),
			static_cast<float>(oy),
			static_cast<float>(oz),
			static_cast<float>(ox + pl.normalX * lenMm),
			static_cast<float>(oy + pl.normalY * lenMm),
			static_cast<float>(oz + pl.normalZ * lenMm)};
}

std::string lastSketchIdOf(const ParametricBrepBackendData& body)
{
	std::string id;
	for (const ParametricFeature& f : body.features())
	{
		if (f.kind == ParametricFeatureKind::Sketch && f.profileXyzMm.size() >= 12)
			id = f.id;
	}
	return id;
}

bool kindAllowsNewBody(ParametricFeatureKind k)
{
	switch (k)
	{
	case ParametricFeatureKind::Sketch:
	case ParametricFeatureKind::Pad:
	case ParametricFeatureKind::Sweep:
	case ParametricFeatureKind::Revolve:
	case ParametricFeatureKind::Loft:
		return true;
	default:
		return false;
	}
}

QJsonObject featureSummary(const ParametricFeature& f)
{
	QJsonObject o;
	o.insert(QStringLiteral("id"), QString::fromStdString(f.id));
	o.insert(QStringLiteral("name"), QString::fromStdString(f.name));
	o.insert(QStringLiteral("kind"), QString::fromUtf8(parametricFeatureKindToString(f.kind)));
	o.insert(QStringLiteral("suppressed"), f.suppressed);
	o.insert(QStringLiteral("visible"), f.visible);
	if (f.kind == ParametricFeatureKind::Pad || f.kind == ParametricFeatureKind::Pocket)
	{
		o.insert(QStringLiteral("lengthMm"), f.lengthMm);
		o.insert(QStringLiteral("draftAngleDeg"), f.draftAngleDeg);
		o.insert(QStringLiteral("endCondition"), QString::fromUtf8(parametricExtrudeEndToString(f.endCondition)));
	}
	if (f.kind == ParametricFeatureKind::Fillet)
		o.insert(QStringLiteral("radiusMm"), f.radiusMm);
	if (f.kind == ParametricFeatureKind::Chamfer)
		o.insert(QStringLiteral("chamferDistMm"), f.chamferDistMm);
	if (f.kind == ParametricFeatureKind::Revolve || f.kind == ParametricFeatureKind::RevolveCut)
		o.insert(QStringLiteral("revolveAngleDeg"), f.revolveAngleDeg);
	if (f.kind == ParametricFeatureKind::Shell)
		o.insert(QStringLiteral("shellThicknessMm"), f.shellThicknessMm);
	if (f.kind == ParametricFeatureKind::Sweep || f.kind == ParametricFeatureKind::SweepCut)
		o.insert(QStringLiteral("twistDeg"), f.twistDeg);
	if (f.kind == ParametricFeatureKind::LinearPattern || f.kind == ParametricFeatureKind::CircularPattern)
		o.insert(QStringLiteral("patternCount"), f.patternCount);
	if (!f.sketchRefId.empty())
		o.insert(QStringLiteral("sketchRefId"), QString::fromStdString(f.sketchRefId));
	return o;
}

std::shared_ptr<ParametricBrepBackendData> findParametric(DocumentHost& host, const QString& backendId, QString* err)
{
	if (backendId.isEmpty())
	{
		if (err)
			*err = QStringLiteral("backendId required.");
		return nullptr;
	}
	auto body = std::dynamic_pointer_cast<ParametricBrepBackendData>(host.findObject(backendId.toStdString()));
	if (!body && err)
		*err = QStringLiteral("Parametric Body not found.");
	return body;
}

nlohmann::json historyRootFromBytes(const QByteArray& bytes, QString* err)
{
	try
	{
		nlohmann::json root = nlohmann::json::parse(bytes.constData(), bytes.constData() + bytes.size());
		if (root.contains("parametricHistory") && root["parametricHistory"].is_object())
			root = root["parametricHistory"];
		if (root.contains("history") && root["history"].is_object())
			root = root["history"];
		return root;
	}
	catch (const std::exception& ex)
	{
		if (err)
			*err = QString::fromUtf8(ex.what());
		return nlohmann::json();
	}
}

} // namespace

HeadlessGeomodelBridge::HeadlessGeomodelBridge(DocumentHost& host) : m_host(host) {}

QByteArray HeadlessGeomodelBridge::captureHistory(const ParametricBrepBackendData& body) const
{
	const std::string dumped = body.historyToJson().dump();
	return QByteArray(dumped.data(), static_cast<int>(dumped.size()));
}

bool HeadlessGeomodelBridge::restoreHistory(ParametricBrepBackendData& body, const QByteArray& bytes, QString* err) const
{
	if (bytes.isEmpty())
	{
		if (err)
			*err = QStringLiteral("empty history snapshot");
		return false;
	}
	QString parseErr;
	nlohmann::json root = historyRootFromBytes(bytes, &parseErr);
	if (root.is_null() || !root.is_object())
	{
		if (err)
			*err = parseErr.isEmpty() ? QStringLiteral("invalid history snapshot") : parseErr;
		return false;
	}
	std::string pe;
	if (!body.historyFromJson(root, &pe))
	{
		if (err)
			*err = QString::fromStdString(pe.empty() ? "historyFromJson failed" : pe);
		return false;
	}
	return true;
}

void HeadlessGeomodelBridge::pushUndo(HistoryUndoSnap snap)
{
	m_undo.push_back(std::move(snap));
	while (static_cast<int>(m_undo.size()) > kMaxUndo)
		m_undo.erase(m_undo.begin());
	m_redo.clear();
}

void HeadlessGeomodelBridge::dropSnapsWithId(const QString& backendId)
{
	const auto pred = [&backendId](const HistoryUndoSnap& s) { return s.backendId == backendId; };
	m_undo.erase(std::remove_if(m_undo.begin(), m_undo.end(), pred), m_undo.end());
	m_redo.erase(std::remove_if(m_redo.begin(), m_redo.end(), pred), m_redo.end());
}

void HeadlessGeomodelBridge::resetHistoryStack()
{
	m_undo.clear();
	m_redo.clear();
}

QJsonObject HeadlessGeomodelBridge::okBody(const ParametricBrepBackendData& body) const
{
	emit m_host.visualSceneDirty();
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("backendId"), QString::fromStdString(body.id()));
	o.insert(QStringLiteral("name"), QString::fromStdString(body.name()));
	o.insert(QStringLiteral("featureCount"), static_cast<int>(body.features().size()));
	o.insert(QStringLiteral("hasGeometry"), body.hasGeometry());
	o.insert(QStringLiteral("undoCount"), static_cast<int>(m_undo.size()));
	o.insert(QStringLiteral("redoCount"), static_cast<int>(m_redo.size()));
	return o;
}

QJsonObject HeadlessGeomodelBridge::commitExisting(const std::shared_ptr<ParametricBrepBackendData>& param,
												  const QByteArray& before)
{
	if (!param)
		return fail(QStringLiteral("Parametric Body missing."));
	std::string rebuildErr;
	if (!param->rebuild(&rebuildErr))
	{
		restoreHistory(*param, before, nullptr);
		param->rebuild(nullptr);
		return fail(QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr));
	}
	pushUndo({QString::fromStdString(param->id()), before, false});
	return okBody(*param);
}

QJsonObject HeadlessGeomodelBridge::finishNewBody(const std::shared_ptr<ParametricBrepBackendData>& param)
{
	if (!param)
		return fail(QStringLiteral("Parametric Body missing."));
	std::string rebuildErr;
	if (!param->rebuild(&rebuildErr))
		return fail(QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr));
	QString regErr;
	if (!registerAdoptedBrepAndLoadScene(m_host, param, QStringLiteral("geomodel://parametric"),
										 QLatin1String(backend_type::kCatalogParametricBrep), QString(), true, &regErr))
		return fail(regErr.isEmpty() ? QStringLiteral("register Parametric Body failed") : regErr);
	pushUndo({QString::fromStdString(param->id()), captureHistory(*param), true});
	return okBody(*param);
}

QJsonObject HeadlessGeomodelBridge::summaryJson() const
{
	QJsonArray bodies;
	for (const std::shared_ptr<BackendDataBase>& obj : m_host.listObjects())
	{
		if (!obj || obj->className() != backend_type::kClassParametricBrep)
			continue;
		const auto* param = dynamic_cast<const ParametricBrepBackendData*>(obj.get());
		QJsonObject entry;
		entry.insert(QStringLiteral("backendId"), QString::fromStdString(obj->id()));
		entry.insert(QStringLiteral("name"), QString::fromStdString(obj->name()));
		entry.insert(QStringLiteral("className"), QString::fromStdString(obj->className()));
		entry.insert(QStringLiteral("hasGeometry"), obj->hasGeometry());
		if (param)
		{
			entry.insert(QStringLiteral("featureCount"), static_cast<int>(param->features().size()));
			QJsonArray feats;
			for (const ParametricFeature& f : param->features())
				feats.append(featureSummary(f));
			entry.insert(QStringLiteral("features"), feats);
		}
		bodies.append(entry);
	}

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("bodies"), bodies);
	o.insert(QStringLiteral("count"), bodies.size());
	o.insert(QStringLiteral("undoCount"), static_cast<int>(m_undo.size()));
	o.insert(QStringLiteral("redoCount"), static_cast<int>(m_redo.size()));
	return o;
}

QJsonObject HeadlessGeomodelBridge::historyJson(const QString& backendId) const
{
	QString err;
	auto body = findParametric(m_host, backendId, &err);
	if (!body)
		return fail(err);
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("backendId"), backendId);
	o.insert(QStringLiteral("history"), nlohmannToQJson(body->historyToJson()));
	o.insert(QStringLiteral("undoCount"), static_cast<int>(m_undo.size()));
	o.insert(QStringLiteral("redoCount"), static_cast<int>(m_redo.size()));
	return o;
}

QJsonObject HeadlessGeomodelBridge::setHistory(const QJsonObject& body)
{
	QString err;
	auto param = findParametric(m_host, body.value(QStringLiteral("backendId")).toString(), &err);
	if (!param)
		return fail(err);
	QJsonObject history = body.value(QStringLiteral("history")).toObject();
	if (history.isEmpty() && body.contains(QStringLiteral("features")))
		history = body;
	if (history.contains(QStringLiteral("parametricHistory")))
		history = history.value(QStringLiteral("parametricHistory")).toObject();
	const QByteArray before = captureHistory(*param);
	nlohmann::json root = qJsonToNlohmann(history);
	std::string parseErr;
	if (!param->historyFromJson(root, &parseErr))
		return fail(QString::fromStdString(parseErr.empty() ? "historyFromJson failed" : parseErr));
	return commitExisting(param, before);
}

QJsonObject HeadlessGeomodelBridge::applyOp(const QJsonObject& body)
{
	const QString op = body.value(QStringLiteral("op")).toString();
	if (op == QStringLiteral("primitive"))
		return createPrimitive(body);
	if (op == QStringLiteral("extrude"))
		return extrude(body);
	if (op == QStringLiteral("append"))
		return appendFeature(body);
	if (op == QStringLiteral("patch"))
		return patchFeature(body);
	if (op == QStringLiteral("delete"))
		return deleteFeature(body);
	if (op == QStringLiteral("rebuild"))
		return rebuildBody(body);
	if (op == QStringLiteral("setHistory"))
		return setHistory(body);
	if (op == QStringLiteral("undo"))
		return undoOp();
	if (op == QStringLiteral("redo"))
		return redoOp();
	if (op == QStringLiteral("importHistory"))
		return importHistoryFile(body);
	if (op == QStringLiteral("exportHistory"))
		return exportHistoryFile(body);
	return fail(QStringLiteral(
		"Unknown op. Use primitive|extrude|append|patch|delete|rebuild|setHistory|undo|redo|importHistory|exportHistory."));
}

QJsonObject HeadlessGeomodelBridge::createPrimitive(const QJsonObject& body)
{
	const QString kind = body.value(QStringLiteral("kind")).toString(QStringLiteral("box"));
	QJsonObject extrudeBody = body;
	extrudeBody.insert(QStringLiteral("mode"), QStringLiteral("pad"));
	if (kind == QStringLiteral("cylinder"))
	{
		extrudeBody.insert(QStringLiteral("profile"), QStringLiteral("circle"));
		if (!extrudeBody.contains(QStringLiteral("radiusMm")))
			extrudeBody.insert(QStringLiteral("radiusMm"), 50.0);
		if (!extrudeBody.contains(QStringLiteral("heightMm")))
			extrudeBody.insert(QStringLiteral("heightMm"), 100.0);
	}
	else if (kind == QStringLiteral("polygon"))
	{
		extrudeBody.insert(QStringLiteral("profile"), QStringLiteral("polygon"));
	}
	else if (kind == QStringLiteral("ellipse"))
	{
		extrudeBody.insert(QStringLiteral("profile"), QStringLiteral("ellipse"));
	}
	else if (kind == QStringLiteral("slot"))
	{
		extrudeBody.insert(QStringLiteral("profile"), QStringLiteral("slot"));
	}
	else
	{
		extrudeBody.insert(QStringLiteral("profile"), QStringLiteral("rectangle"));
		if (!extrudeBody.contains(QStringLiteral("lengthMm")))
			extrudeBody.insert(QStringLiteral("lengthMm"), 100.0);
		if (!extrudeBody.contains(QStringLiteral("widthMm")))
			extrudeBody.insert(QStringLiteral("widthMm"), 100.0);
	}
	if (!extrudeBody.contains(QStringLiteral("heightMm")) && !extrudeBody.contains(QStringLiteral("depthMm")))
		extrudeBody.insert(QStringLiteral("heightMm"), 100.0);
	return extrude(extrudeBody);
}

QJsonObject HeadlessGeomodelBridge::extrude(const QJsonObject& body)
{
	const QString mode = body.value(QStringLiteral("mode")).toString(QStringLiteral("pad")).toLower();
	const bool pocket = (mode == QStringLiteral("pocket"));
	const bool useLast = body.value(QStringLiteral("useLastSketch")).toBool(false);
	const ParametricSketchPlane plane = planeFromId(body.value(QStringLiteral("plane")).toString());
	std::vector<float> profile = embedOnPlane(profileFromBody(body), plane);
	if (!useLast && profile.size() < 12)
		return fail(QStringLiteral("Profile too short."));

	const double depth =
		body.contains(QStringLiteral("heightMm"))
			? body.value(QStringLiteral("heightMm")).toDouble(100.0)
			: body.value(QStringLiteral("depthMm")).toDouble(body.value(QStringLiteral("lengthMm")).toDouble(100.0));
	const bool reversed = body.value(QStringLiteral("reversed")).toBool(false);
	const QString targetId = body.value(QStringLiteral("backendId")).toString();
	const bool createNew = targetId.isEmpty();
	if (pocket && createNew)
		return fail(QStringLiteral("Pocket requires existing Parametric Body."));

	std::shared_ptr<ParametricBrepBackendData> param;
	if (createNew)
	{
		param = std::make_shared<ParametricBrepBackendData>();
		const QString name = body.value(QStringLiteral("name")).toString(QStringLiteral("ParametricBody"));
		param->setName(name.toStdString());
	}
	else
	{
		QString err;
		param = findParametric(m_host, targetId, &err);
		if (!param)
			return fail(err);
	}

	const QByteArray before = createNew ? QByteArray() : captureHistory(*param);
	if (useLast)
	{
		const std::string lastSk = lastSketchIdOf(*param);
		if (lastSk.empty())
			return fail(QStringLiteral("No sketch on body; draw a template first."));
		if (pocket)
			param->addPocket(lastSk, depth, reversed);
		else
			param->addPad(lastSk, depth, reversed);
	}
	else
	{
		const std::string sketchId = param->addSketch(plane);
		if (!param->setProfile(sketchId, profile))
		{
			if (!createNew)
				restoreHistory(*param, before, nullptr);
			return fail(QStringLiteral("Failed to set sketch profile."));
		}
		if (pocket)
			param->addPocket(sketchId, depth, reversed);
		else
			param->addPad(sketchId, depth, reversed);
	}
	if (ParametricFeature* extrudeFeat = param->findFeature(param->features().back().id))
	{
		extrudeFeat->draftAngleDeg = body.value(QStringLiteral("draftAngleDeg")).toDouble(0.0);
		extrudeFeat->startOffsetMm = body.value(QStringLiteral("startOffsetMm")).toDouble(0.0);
		extrudeFeat->length2Mm = body.value(QStringLiteral("length2Mm")).toDouble(0.0);
		const auto endParse =
			parametricExtrudeEndTryParse(body.value(QStringLiteral("endCondition")).toString().toStdString());
		if (endParse.ok)
			extrudeFeat->endCondition = endParse.value;
	}

	if (createNew)
		return finishNewBody(param);
	return commitExisting(param, before);
}

QJsonObject HeadlessGeomodelBridge::appendFeature(const QJsonObject& body)
{
	const QString kind = body.value(QStringLiteral("kind")).toString();
	const auto parsed = parametricFeatureKindTryParse(kind.toStdString());
	if (!parsed.ok)
		return fail(QStringLiteral("Unknown feature kind."));

	if (parsed.value == ParametricFeatureKind::Pad || parsed.value == ParametricFeatureKind::Pocket)
		return extrude(body);

	const QString targetId = body.value(QStringLiteral("backendId")).toString();
	const bool createNew = targetId.isEmpty();
	if (createNew && !kindAllowsNewBody(parsed.value))
		return fail(QStringLiteral("This kind requires an existing Parametric Body."));

	std::shared_ptr<ParametricBrepBackendData> param;
	if (createNew)
	{
		param = std::make_shared<ParametricBrepBackendData>();
		const QString name = body.value(QStringLiteral("name")).toString(QStringLiteral("ParametricBody"));
		param->setName(name.toStdString());
	}
	else
	{
		QString err;
		param = findParametric(m_host, targetId, &err);
		if (!param)
			return fail(err);
	}

	const ParametricSketchPlane plane = planeFromId(body.value(QStringLiteral("plane")).toString());
	const QByteArray before = createNew ? QByteArray() : captureHistory(*param);
	auto rollback = [&]() {
		if (!createNew)
			restoreHistory(*param, before, nullptr);
	};

	switch (parsed.value)
	{
	case ParametricFeatureKind::Sketch:
	{
		const std::vector<float> profile = embedOnPlane(profileFromBody(body), plane);
		if (profile.size() < 12)
			return fail(QStringLiteral("Sketch profile too short."));
		const std::string sketchId = param->addSketch(plane);
		if (!param->setProfile(sketchId, profile))
		{
			rollback();
			return fail(QStringLiteral("Failed to set sketch profile."));
		}
		break;
	}
	case ParametricFeatureKind::Fillet:
		param->addFillet(intListFromJson(body.value(QStringLiteral("edgeIndices"))),
						 body.value(QStringLiteral("radiusMm")).toDouble(1.0));
		break;
	case ParametricFeatureKind::Chamfer:
		param->addChamfer(intListFromJson(body.value(QStringLiteral("edgeIndices"))),
						  body.value(QStringLiteral("chamferDistMm")).toDouble(1.0));
		break;
	case ParametricFeatureKind::Revolve:
	case ParametricFeatureKind::RevolveCut:
	{
		const std::vector<float> profile = embedOnPlane(profileFromBody(body), plane);
		if (profile.size() < 12)
			return fail(QStringLiteral("Revolve profile too short."));
		const std::string sketchId = param->addSketch(plane);
		if (!param->setProfile(sketchId, profile))
		{
			rollback();
			return fail(QStringLiteral("Failed to set revolve profile."));
		}
		const QJsonArray axisO = body.value(QStringLiteral("axisO")).toArray();
		const QJsonArray axisD = body.value(QStringLiteral("axisD")).toArray();
		param->addRevolve(
			sketchId, body.value(QStringLiteral("revolveAngleDeg")).toDouble(360.0),
			axisO.size() > 0 ? axisO.at(0).toDouble() : plane.originX,
			axisO.size() > 1 ? axisO.at(1).toDouble() : plane.originY,
			axisO.size() > 2 ? axisO.at(2).toDouble() : plane.originZ,
			axisD.size() > 0 ? axisD.at(0).toDouble() : plane.axisYX,
			axisD.size() > 1 ? axisD.at(1).toDouble() : plane.axisYY,
			axisD.size() > 2 ? axisD.at(2).toDouble() : plane.axisYZ,
			parsed.value == ParametricFeatureKind::RevolveCut);
		break;
	}
	case ParametricFeatureKind::LinearPattern:
	{
		const QJsonArray d = body.value(QStringLiteral("patternD")).toArray();
		param->addLinearPattern(body.value(QStringLiteral("patternCount")).toInt(2),
								d.size() > 0 ? d.at(0).toDouble() : 10.0, d.size() > 1 ? d.at(1).toDouble() : 0.0,
								d.size() > 2 ? d.at(2).toDouble() : 0.0,
								body.value(QStringLiteral("sourceFeatureId")).toString().toStdString());
		break;
	}
	case ParametricFeatureKind::CircularPattern:
	{
		const QJsonArray axisO = body.value(QStringLiteral("axisO")).toArray();
		const QJsonArray axisD = body.value(QStringLiteral("axisD")).toArray();
		param->addCircularPattern(
			body.value(QStringLiteral("patternCount")).toInt(4),
			body.value(QStringLiteral("patternAngleDeg")).toDouble(360.0),
			axisO.size() > 0 ? axisO.at(0).toDouble() : plane.originX,
			axisO.size() > 1 ? axisO.at(1).toDouble() : plane.originY,
			axisO.size() > 2 ? axisO.at(2).toDouble() : plane.originZ,
			axisD.size() > 0 ? axisD.at(0).toDouble() : plane.normalX,
			axisD.size() > 1 ? axisD.at(1).toDouble() : plane.normalY,
			axisD.size() > 2 ? axisD.at(2).toDouble() : plane.normalZ,
			body.value(QStringLiteral("sourceFeatureId")).toString().toStdString());
		break;
	}
	case ParametricFeatureKind::Mirror3D:
		param->addMirror3D(plane, body.value(QStringLiteral("keepOriginal")).toBool(true));
		break;
	case ParametricFeatureKind::Shell:
		param->addShell(intListFromJson(body.value(QStringLiteral("faceIndices"))),
						body.value(QStringLiteral("shellThicknessMm")).toDouble(1.0));
		break;
	case ParametricFeatureKind::Draft:
		param->addDraft(intListFromJson(body.value(QStringLiteral("faceIndices"))),
						body.value(QStringLiteral("draftAngleDeg")).toDouble(5.0), plane);
		break;
	case ParametricFeatureKind::Sweep:
	case ParametricFeatureKind::SweepCut:
	{
		const std::vector<float> profile = embedOnPlane(profileFromBody(body), plane);
		if (profile.size() < 12)
			return fail(QStringLiteral("Sweep profile too short."));
		std::vector<float> path = polylineFromJson(body.value(QStringLiteral("path")));
		if (path.size() < 6)
			path = defaultSweepPath(plane, body.value(QStringLiteral("sweepLengthMm")).toDouble(80.0), profile);
		const std::string profileId = param->addSketch(plane);
		if (!param->setProfile(profileId, profile))
		{
			rollback();
			return fail(QStringLiteral("Failed to set sweep profile."));
		}
		const std::string pathId = param->addSketch(plane);
		if (!param->setProfile(pathId, path))
		{
			rollback();
			return fail(QStringLiteral("Failed to set sweep path."));
		}
		const std::string sweepId =
			param->addSweep(profileId, pathId, parsed.value == ParametricFeatureKind::SweepCut);
		if (ParametricFeature* sw = param->findFeature(sweepId))
			sw->twistDeg = body.value(QStringLiteral("twistDeg")).toDouble(0.0);
		break;
	}
	case ParametricFeatureKind::Loft:
	case ParametricFeatureKind::LoftCut:
	{
		const std::vector<float> profileA = embedOnPlane(profileFromBody(body), plane);
		if (profileA.size() < 12)
			return fail(QStringLiteral("Loft profile too short."));
		const double gap = body.value(QStringLiteral("loftGapMm")).toDouble(80.0);
		const ParametricSketchPlane planeB = offsetPlane(plane, gap);
		std::vector<float> profileB = polylineFromJson(body.value(QStringLiteral("polylineB")));
		if (profileB.size() < 12)
			profileB = embedOnPlane(profileFromBody(body), planeB);
		const std::string skA = param->addSketch(plane);
		const std::string skB = param->addSketch(planeB);
		if (!param->setProfile(skA, profileA) || !param->setProfile(skB, profileB))
		{
			rollback();
			return fail(QStringLiteral("Failed to set loft profiles."));
		}
		param->addLoft(skA, skB, parsed.value == ParametricFeatureKind::LoftCut);
		break;
	}
	default:
		return fail(QStringLiteral("Kind not supported via append; use extrude or setHistory."));
	}

	if (createNew)
		return finishNewBody(param);
	return commitExisting(param, before);
}

QJsonObject HeadlessGeomodelBridge::patchFeature(const QJsonObject& body)
{
	QString err;
	auto param = findParametric(m_host, body.value(QStringLiteral("backendId")).toString(), &err);
	if (!param)
		return fail(err);
	const std::string featureId = body.value(QStringLiteral("featureId")).toString().toStdString();
	ParametricFeature* feat = param->findFeature(featureId);
	if (!feat)
		return fail(QStringLiteral("Feature not found."));

	const QByteArray before = captureHistory(*param);
	if (body.contains(QStringLiteral("name")))
		feat->name = body.value(QStringLiteral("name")).toString().toStdString();
	if (body.contains(QStringLiteral("suppressed")))
		feat->suppressed = body.value(QStringLiteral("suppressed")).toBool();
	if (body.contains(QStringLiteral("visible")))
		feat->visible = body.value(QStringLiteral("visible")).toBool();
	if (body.contains(QStringLiteral("lengthMm")))
		feat->lengthMm = body.value(QStringLiteral("lengthMm")).toDouble();
	if (body.contains(QStringLiteral("length2Mm")))
		feat->length2Mm = body.value(QStringLiteral("length2Mm")).toDouble();
	if (body.contains(QStringLiteral("startOffsetMm")))
		feat->startOffsetMm = body.value(QStringLiteral("startOffsetMm")).toDouble();
	if (body.contains(QStringLiteral("draftAngleDeg")))
		feat->draftAngleDeg = body.value(QStringLiteral("draftAngleDeg")).toDouble();
	if (body.contains(QStringLiteral("radiusMm")))
		feat->radiusMm = body.value(QStringLiteral("radiusMm")).toDouble();
	if (body.contains(QStringLiteral("chamferDistMm")))
		feat->chamferDistMm = body.value(QStringLiteral("chamferDistMm")).toDouble();
	if (body.contains(QStringLiteral("shellThicknessMm")))
		feat->shellThicknessMm = body.value(QStringLiteral("shellThicknessMm")).toDouble();
	if (body.contains(QStringLiteral("revolveAngleDeg")))
		feat->revolveAngleDeg = body.value(QStringLiteral("revolveAngleDeg")).toDouble();
	if (body.contains(QStringLiteral("twistDeg")))
		feat->twistDeg = body.value(QStringLiteral("twistDeg")).toDouble();
	if (body.contains(QStringLiteral("reversed")))
		feat->reversed = body.value(QStringLiteral("reversed")).toBool();
	if (body.contains(QStringLiteral("endCondition")))
	{
		const auto endParse =
			parametricExtrudeEndTryParse(body.value(QStringLiteral("endCondition")).toString().toStdString());
		if (endParse.ok)
			feat->endCondition = endParse.value;
	}
	if (body.contains(QStringLiteral("polyline")) && feat->kind == ParametricFeatureKind::Sketch)
	{
		const std::vector<float> poly = polylineFromJson(body.value(QStringLiteral("polyline")));
		if (poly.size() >= 12)
			feat->profileXyzMm = poly;
	}

	return commitExisting(param, before);
}

QJsonObject HeadlessGeomodelBridge::deleteFeature(const QJsonObject& body)
{
	QString err;
	auto param = findParametric(m_host, body.value(QStringLiteral("backendId")).toString(), &err);
	if (!param)
		return fail(err);
	const std::string featureId = body.value(QStringLiteral("featureId")).toString().toStdString();
	const QByteArray before = captureHistory(*param);
	std::vector<ParametricFeature> kept;
	kept.reserve(param->features().size());
	bool found = false;
	for (const ParametricFeature& f : param->features())
	{
		if (f.id == featureId)
		{
			found = true;
			continue;
		}
		kept.push_back(f);
	}
	if (!found)
		return fail(QStringLiteral("Feature not found."));
	param->setFeatures(std::move(kept));
	return commitExisting(param, before);
}

QJsonObject HeadlessGeomodelBridge::rebuildBody(const QJsonObject& body)
{
	QString err;
	auto param = findParametric(m_host, body.value(QStringLiteral("backendId")).toString(), &err);
	if (!param)
		return fail(err);
	std::string rebuildErr;
	if (!param->rebuild(&rebuildErr))
		return fail(QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr));
	return okBody(*param);
}

QJsonObject HeadlessGeomodelBridge::undoOp()
{
	if (m_undo.empty())
		return fail(QStringLiteral("Nothing to undo."));
	HistoryUndoSnap snap = m_undo.back();
	m_undo.pop_back();
	if (snap.created)
	{
		auto param = findParametric(m_host, snap.backendId, nullptr);
		if (param)
			snap.historyJson = captureHistory(*param);
		m_host.removeBackendSubtree(snap.backendId);
		dropSnapsWithId(snap.backendId);
		m_redo.push_back(std::move(snap));
		emit m_host.visualSceneDirty();
		QJsonObject o;
		o.insert(QStringLiteral("ok"), true);
		o.insert(QStringLiteral("removed"), true);
		o.insert(QStringLiteral("undoCount"), static_cast<int>(m_undo.size()));
		o.insert(QStringLiteral("redoCount"), static_cast<int>(m_redo.size()));
		return o;
	}
	QString err;
	auto param = findParametric(m_host, snap.backendId, &err);
	if (!param)
		return fail(err);
	const QByteArray now = captureHistory(*param);
	if (!restoreHistory(*param, snap.historyJson, &err))
	{
		m_undo.push_back(std::move(snap));
		return fail(err);
	}
	std::string rebuildErr;
	if (!param->rebuild(&rebuildErr))
	{
		restoreHistory(*param, now, nullptr);
		param->rebuild(nullptr);
		m_undo.push_back(std::move(snap));
		return fail(QString::fromStdString(rebuildErr.empty() ? "undo rebuild failed" : rebuildErr));
	}
	m_redo.push_back({snap.backendId, now, false});
	return okBody(*param);
}

QJsonObject HeadlessGeomodelBridge::redoOp()
{
	if (m_redo.empty())
		return fail(QStringLiteral("Nothing to redo."));
	HistoryUndoSnap snap = m_redo.back();
	m_redo.pop_back();
	if (snap.created)
	{
		auto param = std::make_shared<ParametricBrepBackendData>();
		QString err;
		if (!restoreHistory(*param, snap.historyJson, &err))
		{
			m_redo.push_back(std::move(snap));
			return fail(err);
		}
		std::string rebuildErr;
		if (!param->rebuild(&rebuildErr))
		{
			m_redo.push_back(std::move(snap));
			return fail(QString::fromStdString(rebuildErr.empty() ? "redo rebuild failed" : rebuildErr));
		}
		QString regErr;
		if (!registerAdoptedBrepAndLoadScene(m_host, param, QStringLiteral("geomodel://parametric"),
											 QLatin1String(backend_type::kCatalogParametricBrep), QString(), true,
											 &regErr))
		{
			m_redo.push_back(std::move(snap));
			return fail(regErr.isEmpty() ? QStringLiteral("register Parametric Body failed") : regErr);
		}
		m_undo.push_back({QString::fromStdString(param->id()), captureHistory(*param), true});
		dropSnapsWithId(snap.backendId);
		return okBody(*param);
	}
	QString err;
	auto param = findParametric(m_host, snap.backendId, &err);
	if (!param)
		return fail(err);
	const QByteArray now = captureHistory(*param);
	if (!restoreHistory(*param, snap.historyJson, &err))
	{
		m_redo.push_back(std::move(snap));
		return fail(err);
	}
	std::string rebuildErr;
	if (!param->rebuild(&rebuildErr))
	{
		restoreHistory(*param, now, nullptr);
		param->rebuild(nullptr);
		m_redo.push_back(std::move(snap));
		return fail(QString::fromStdString(rebuildErr.empty() ? "redo rebuild failed" : rebuildErr));
	}
	m_undo.push_back({snap.backendId, now, false});
	return okBody(*param);
}

QJsonObject HeadlessGeomodelBridge::importHistoryFile(const QJsonObject& body)
{
	const QString path = body.value(QStringLiteral("path")).toString();
	if (path.isEmpty())
		return fail(QStringLiteral("path required."));
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
		return fail(QStringLiteral("Cannot read history file."));
	const QByteArray bytes = f.readAll();
	QString parseErr;
	nlohmann::json root = historyRootFromBytes(bytes, &parseErr);
	if (root.is_null() || !root.is_object())
		return fail(parseErr.isEmpty() ? QStringLiteral("Invalid history JSON") : parseErr);

	const bool createNew = body.value(QStringLiteral("createNew")).toBool(true);
	if (createNew)
	{
		auto param = std::make_shared<ParametricBrepBackendData>();
		const QString name = body.value(QStringLiteral("name")).toString(QFileInfo(path).completeBaseName());
		if (!name.isEmpty())
			param->setName(name.toStdString());
		std::string pe;
		if (!param->historyFromJson(root, &pe))
			return fail(QString::fromStdString(pe.empty() ? "historyFromJson failed" : pe));
		return finishNewBody(param);
	}

	QString err;
	auto param = findParametric(m_host, body.value(QStringLiteral("backendId")).toString(), &err);
	if (!param)
		return fail(err);
	const QByteArray before = captureHistory(*param);
	std::string pe;
	if (!param->historyFromJson(root, &pe))
		return fail(QString::fromStdString(pe.empty() ? "historyFromJson failed" : pe));
	return commitExisting(param, before);
}

QJsonObject HeadlessGeomodelBridge::exportHistoryFile(const QJsonObject& body)
{
	QString err;
	auto param = findParametric(m_host, body.value(QStringLiteral("backendId")).toString(), &err);
	if (!param)
		return fail(err);
	const QString path = body.value(QStringLiteral("path")).toString();
	if (path.isEmpty())
		return fail(QStringLiteral("path required."));
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
		return fail(QStringLiteral("Cannot write history file."));
	const QJsonDocument doc(nlohmannToQJson(param->historyToJson()));
	if (f.write(doc.toJson(QJsonDocument::Indented)) < 0)
		return fail(QStringLiteral("Write history file failed."));
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("backendId"), QString::fromStdString(param->id()));
	o.insert(QStringLiteral("path"), path);
	o.insert(QStringLiteral("undoCount"), static_cast<int>(m_undo.size()));
	o.insert(QStringLiteral("redoCount"), static_cast<int>(m_redo.size()));
	return o;
}

} // namespace cloudsim::host
