/// @file CloudSimGeomPython.cpp

#include <Windows.h>

#include <filesystem>
#include <stdexcept>
#include <string>

// 先于任何 Qt 头，避免 slots 宏打坏 Python object.h
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "CloudSimGeomPython.h"
#include "IAiAssistantHost.h"
#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "IPluginHostContext.h"
#include "PluginGeometryTypes.h"
#include "ScriptModelIo.h"

#include <QByteArray>
#include <QDialog>
#include <QDialogButtonBox>
#include <QEventLoop>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

namespace fs = std::filesystem;
namespace py = pybind11;

namespace CloudSimGeomPython
{
namespace
{
IPluginHostContext* g_host = nullptr;
bool g_moduleRegistered = false;

fs::path resolvePythonHome()
{
	wchar_t exePath[MAX_PATH] = {};
	if (GetModuleFileNameW(nullptr, exePath, MAX_PATH) == 0)
		return {};
	const fs::path exeDir = fs::path(exePath).parent_path();
	const fs::path candidates[] = {
		exeDir / L".." / L"SDK" / L"python311",
		exeDir / L"SDK" / L"python311",
		exeDir / L"python311",
	};
	for (const fs::path& c : candidates)
	{
		std::error_code ec;
		const fs::path canon = fs::weakly_canonical(c, ec);
		if (!ec && fs::exists(canon / L"python311.dll"))
			return canon;
	}
	return {};
}

IPluginDocument* activeDoc()
{
	return g_host ? g_host->activeDocument() : nullptr;
}

IPluginGeometryHost* geoHost()
{
	return g_host ? g_host->geometryHost() : nullptr;
}

IAiAssistantHost* aiHost()
{
	return g_host ? g_host->aiAssistantHost() : nullptr;
}

std::string firstBodyId(std::string* outError)
{
	IPluginDocument* doc = activeDoc();
	IPluginGeometryHost* geo = geoHost();
	if (!doc || !geo)
	{
		if (outError)
			*outError = "no document/geometry host";
		return {};
	}
	std::vector<std::string> ids;
	QString qerr;
	if (!geo->listParametricBodyIds(doc, ids, &qerr) || ids.empty())
	{
		if (outError)
			*outError = qerr.isEmpty() ? "no parametric body" : qerr.toStdString();
		return {};
	}
	return ids.front();
}

bool createBodySync(std::string& outBodyId, std::string* outError)
{
	IPluginDocument* doc = activeDoc();
	IPluginGeometryHost* geo = geoHost();
	if (!doc || !geo)
	{
		if (outError)
			*outError = "no document/geometry host";
		return false;
	}
	PluginSketchPlane plane;
	plane.origin = {0, 0, 0};
	plane.axisX = {1, 0, 0};
	plane.axisY = {0, 1, 0};
	plane.normal = {0, 0, 1};
	plane.isPlanar = true;
	const std::vector<float> profile = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0};
	PluginSketchExtrudeParams params;
	params.mode = PluginSketchExtrudeMode::Pad;
	params.lengthMm = 1.0;
	params.endCondition = PluginSketchExtrudeEnd::Blind;
	params.targetParametricBackendIdUtf8.clear();
	params.resultNameUtf8 = "ImportedBody";

	bool ok = false;
	QString err;
	QEventLoop loop;
	geo->extrudeSketchProfileToBrep(doc, profile, plane, params,
									[&](bool success, const QString& e, const PluginGeometryJobResult& result)
									{
										ok = success;
										err = e;
										if (success)
											outBodyId = result.newBackendId;
										loop.quit();
									});
	loop.exec();
	if (!ok)
	{
		if (outError)
			*outError = err.isEmpty() ? "create body failed" : err.toStdString();
		return false;
	}
	return true;
}
} // namespace

void setHost(IPluginHostContext* host)
{
	g_host = host;
}

IPluginHostContext* host()
{
	return g_host;
}

bool ensureReady(std::string* outError)
{
	try
	{
		if (Py_IsInitialized())
			return true;
		const fs::path pythonHome = resolvePythonHome();
		if (pythonHome.empty())
		{
			if (outError)
				*outError = "SDK python311 not found";
			return false;
		}
		static std::wstring pythonHomeStr = pythonHome.wstring();
		PyConfig config;
		PyConfig_InitPythonConfig(&config);
		PyStatus status = PyConfig_SetString(&config, &config.home, pythonHomeStr.c_str());
		if (PyStatus_Exception(status))
		{
			PyConfig_Clear(&config);
			if (outError)
				*outError = "PyConfig_SetString failed";
			return false;
		}
		status = Py_InitializeFromConfig(&config);
		PyConfig_Clear(&config);
		if (PyStatus_Exception(status))
		{
			if (outError)
				*outError = "Py_InitializeFromConfig failed";
			return false;
		}
		const fs::path dllBin = pythonHome / L"Library" / L"bin";
		if (fs::exists(dllBin))
			AddDllDirectory(dllBin.wstring().c_str());
		AddDllDirectory(pythonHome.wstring().c_str());
		return true;
	}
	catch (const std::exception& ex)
	{
		if (outError)
			*outError = ex.what();
		return false;
	}
}

std::string exportHistory(const std::string& bodyId, std::string* outError)
{
	IPluginDocument* doc = activeDoc();
	IPluginGeometryHost* geo = geoHost();
	if (!doc || !geo)
	{
		if (outError)
			*outError = "no document/geometry host";
		return {};
	}
	std::string id = bodyId;
	if (id.empty())
		id = firstBodyId(outError);
	if (id.empty())
		return {};
	QByteArray hist;
	QString qerr;
	if (!geo->queryParametricBodyHistoryJson(doc, id, hist, &qerr))
	{
		if (outError)
			*outError = qerr.toStdString();
		return {};
	}
	return std::string(hist.constData(), static_cast<size_t>(hist.size()));
}

void importHistory(const std::string& jsonUtf8, const std::string& bodyId, std::string* outError)
{
	IPluginDocument* doc = activeDoc();
	IPluginGeometryHost* geo = geoHost();
	if (!doc || !geo)
	{
		if (outError)
			*outError = "no document/geometry host";
		return;
	}
	const ScriptModelParseResult parsed =
		parseScriptModelJson(QByteArray(jsonUtf8.data(), static_cast<int>(jsonUtf8.size())));
	if (parsed.kind != ScriptModelJsonKind::History)
	{
		if (outError)
			*outError = parsed.error.isEmpty() ? "not history JSON" : parsed.error.toStdString();
		return;
	}
	std::string id = bodyId;
	if (id.empty())
	{
		std::vector<std::string> ids;
		QString qerr;
		(void)geo->listParametricBodyIds(doc, ids, &qerr);
		if (ids.empty())
		{
			if (!createBodySync(id, outError))
				return;
		}
		else
		{
			id = ids.front();
		}
	}
	bool ok = false;
	QString err;
	QEventLoop loop;
	geo->setParametricBodyHistoryJson(doc, id, parsed.payloadUtf8,
									  [&](bool success, const QString& e, const PluginGeometryJobResult&)
									  {
										  ok = success;
										  err = e;
										  loop.quit();
									  });
	loop.exec();
	if (!ok && outError)
		*outError = err.isEmpty() ? "import failed" : err.toStdString();
}

std::string runCompose(const std::string& jsonUtf8, std::string* outError)
{
	IAiAssistantHost* ai = aiHost();
	if (!ai)
	{
		if (outError)
			*outError = "AI host unavailable";
		return {};
	}
	const ScriptModelParseResult parsed =
		parseScriptModelJson(QByteArray(jsonUtf8.data(), static_cast<int>(jsonUtf8.size())));
	if (parsed.kind != ScriptModelJsonKind::Compose)
	{
		if (outError)
			*outError = parsed.error.isEmpty() ? "not compose JSON" : parsed.error.toStdString();
		return {};
	}
	QString summary;
	QString err;
	if (!ai->executeActionPlan(parsed.payloadUtf8, &summary, &err))
	{
		if (outError)
			*outError = err.isEmpty() ? "executeActionPlan failed" : err.toStdString();
		return {};
	}
	return summary.toStdString();
}

std::vector<std::string> listBodies(std::string* outError)
{
	IPluginDocument* doc = activeDoc();
	IPluginGeometryHost* geo = geoHost();
	std::vector<std::string> ids;
	if (!doc || !geo)
	{
		if (outError)
			*outError = "no document/geometry host";
		return ids;
	}
	QString qerr;
	if (!geo->listParametricBodyIds(doc, ids, &qerr) && outError)
		*outError = qerr.toStdString();
	return ids;
}

bool registerModule(std::string* outError)
{
	if (g_moduleRegistered)
		return true;
	if (!ensureReady(outError))
		return false;
	try
	{
		py::gil_scoped_acquire gil;
		py::module_ types = py::module_::import("types");
		py::module_ sys = py::module_::import("sys");
		py::object mod = types.attr("ModuleType")("cloudsim_geom");
		mod.attr("__doc__") = "CloudSim geometry scripting (history / feature.compose)";
		mod.attr("export_history") = py::cpp_function(
			[](py::object bodyId)
			{
				std::string id;
				if (!bodyId.is_none())
					id = py::str(bodyId);
				std::string err;
				const std::string out = exportHistory(id, &err);
				if (!err.empty())
					throw std::runtime_error(err);
				return out;
			},
			py::arg("body_id") = py::none());
		mod.attr("import_history") = py::cpp_function(
			[](const std::string& jsonStr, py::object bodyId)
			{
				std::string id;
				if (!bodyId.is_none())
					id = py::str(bodyId);
				std::string err;
				importHistory(jsonStr, id, &err);
				if (!err.empty())
					throw std::runtime_error(err);
			},
			py::arg("json_str"), py::arg("body_id") = py::none());
		mod.attr("run_compose") = py::cpp_function(
			[](const std::string& jsonStr)
			{
				std::string err;
				const std::string out = runCompose(jsonStr, &err);
				if (!err.empty())
					throw std::runtime_error(err);
				return out;
			},
			py::arg("json_str"));
		mod.attr("list_bodies") = py::cpp_function(
			[]()
			{
				std::string err;
				auto ids = listBodies(&err);
				if (!err.empty())
					throw std::runtime_error(err);
				return ids;
			});
		sys.attr("modules")["cloudsim_geom"] = mod;
		g_moduleRegistered = true;
		return true;
	}
	catch (const std::exception& ex)
	{
		if (outError)
			*outError = ex.what();
		return false;
	}
}

void openConsole(QWidget* parent)
{
	std::string err;
	if (!registerModule(&err))
	{
		auto* dlg = new QDialog(parent);
		dlg->setWindowTitle(QStringLiteral("Python"));
		auto* lay = new QVBoxLayout(dlg);
		lay->addWidget(new QLabel(err.empty() ? QStringLiteral("Python not ready") : QString::fromStdString(err), dlg));
		auto* box = new QDialogButtonBox(QDialogButtonBox::Close, dlg);
		QObject::connect(box, &QDialogButtonBox::rejected, dlg, &QDialog::reject);
		QObject::connect(box, &QDialogButtonBox::accepted, dlg, &QDialog::accept);
		lay->addWidget(box);
		dlg->exec();
		delete dlg;
		return;
	}

	auto* dlg = new QDialog(parent);
	dlg->setWindowTitle(QStringLiteral("cloudsim_geom"));
	dlg->resize(640, 420);
	auto* lay = new QVBoxLayout(dlg);
	lay->addWidget(new QLabel(
		QStringLiteral("import cloudsim_geom as g\ng.list_bodies()\ng.export_history()\ng.import_history(json)\n"
					   "g.run_compose(json)"),
		dlg));
	auto* edit = new QPlainTextEdit(dlg);
	edit->setPlainText(QStringLiteral("import cloudsim_geom as g\nprint(g.list_bodies())\n"));
	lay->addWidget(edit, 1);
	auto* out = new QPlainTextEdit(dlg);
	out->setReadOnly(true);
	lay->addWidget(out, 1);
	auto* row = new QHBoxLayout();
	auto* run = new QPushButton(QStringLiteral("Run"), dlg);
	auto* close = new QPushButton(QStringLiteral("Close"), dlg);
	row->addWidget(run);
	row->addStretch(1);
	row->addWidget(close);
	lay->addLayout(row);
	QObject::connect(close, &QPushButton::clicked, dlg, &QDialog::accept);
	QObject::connect(run, &QPushButton::clicked, dlg,
					 [edit, out]()
					 {
						 std::string err;
						 if (!registerModule(&err))
						 {
							 out->appendPlainText(QString::fromStdString(err));
							 return;
						 }
						 try
						 {
							 py::gil_scoped_acquire gil;
							 py::object scope = py::module_::import("__main__").attr("__dict__");
							 py::exec("import io,sys\n_buf=io.StringIO()\n_old=sys.stdout\nsys.stdout=_buf\n", scope);
							 py::exec(edit->toPlainText().toUtf8().constData(), scope);
							 py::exec("sys.stdout=_old\n_out=_buf.getvalue()\n", scope);
							 const std::string captured = scope["_out"].cast<std::string>();
							 out->appendPlainText(QString::fromStdString(captured));
							 out->appendPlainText(QStringLiteral("OK"));
						 }
						 catch (const py::error_already_set& e)
						 {
							 out->appendPlainText(QString::fromUtf8(e.what()));
							 if (PyErr_Occurred())
								 PyErr_Clear();
						 }
						 catch (const std::exception& e)
						 {
							 out->appendPlainText(QString::fromUtf8(e.what()));
						 }
					 });
	dlg->exec();
	delete dlg;
}

} // namespace CloudSimGeomPython
