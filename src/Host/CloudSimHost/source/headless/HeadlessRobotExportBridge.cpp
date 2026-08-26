/// @file HeadlessRobotExportBridge.cpp

#include "headless/HeadlessRobotExportBridge.h"

#include "DocumentHost.h"
#include "HeadlessRobotContext.h"
#include "RobotCanonicalProgramExport.h"
#include "RobotProgramStore.h"

#include <QDir>
#include <QFile>
#include <QTemporaryFile>

namespace cloudsim::host
{
namespace
{
QJsonObject fail(const QString& err)
{
	return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
}
} // namespace

HeadlessRobotExportBridge::HeadlessRobotExportBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessRobotExportBridge::exportProgram(const QJsonObject& body)
{
	HeadlessRobotContext* hrc = m_host.headlessRobotContext();
	if (!hrc)
		return fail(QStringLiteral("No headless robot context."));

	QString sceneRoot = body.value(QStringLiteral("sceneRootBackendId")).toString();
	if (sceneRoot.isEmpty())
	{
		const auto inst = hrc->listInstances();
		if (!inst.isEmpty())
			sceneRoot = inst.front().sceneRootBackendId;
	}
	const int instIdx = hrc->robotInstanceIndexForBackendId(sceneRoot);
	if (instIdx < 0)
		return fail(QStringLiteral("Robot instance not found."));

	const QString programId = body.value(QStringLiteral("programId")).toString();
	RobotInstruction::RobotProgramCatalog& catalog = m_host.robotProgramStore().catalogFor(sceneRoot);
	RobotInstruction::RobotProgram* exportProg = nullptr;
	if (!programId.isEmpty())
		exportProg = catalog.findProgram(programId.toStdString());
	if (!exportProg)
		exportProg = catalog.mainProgram();
	if (!exportProg)
		return fail(QStringLiteral("Program not found."));

	RobotCanonicalExport::InstructionRuntimeResolveContext ctx;
	ctx.robotInstanceIndex = instIdx;
	ctx.robotSceneBackendId = sceneRoot.toStdString();
	ctx.urdfPath = hrc->robotUrdfAbsolutePathForInstance(instIdx).toStdString();
	ctx.coordinateFrames = &hrc->robotCoordinateFramesForInstance(instIdx);

	RobotCanonicalExport::CanonicalProgramExportV1 exportDoc;
	std::string buildErr;
	if (!RobotCanonicalExport::buildCanonicalExportV1(*exportProg, ctx,
													  RobotCanonicalExport::CanonicalExportLayout::NestedTree, false,
													  nullptr, exportDoc, &buildErr))
	{
		return fail(QString::fromStdString(buildErr));
	}

	std::string fileBody;
	std::string writeErr;
	if (!RobotCanonicalExport::writeCanonicalExportV1ToJson(exportDoc, fileBody, &writeErr, false))
		return fail(QString::fromStdString(writeErr));

	QTemporaryFile canonicalTemp(QDir::temp().filePath(QStringLiteral("cloudsim_canonical_XXXXXX.json")));
	canonicalTemp.setAutoRemove(false);
	if (!canonicalTemp.open())
		return fail(QStringLiteral("Cannot create temporary canonical file."));
	canonicalTemp.write(fileBody.c_str(), static_cast<qint64>(fileBody.size()));
	canonicalTemp.flush();
	const QString canonicalPath = canonicalTemp.fileName();

	QString outPath = body.value(QStringLiteral("outputPath")).toString();
	if (outPath.isEmpty())
	{
		outPath = canonicalPath;
	}
	else
	{
		QFile outFile(outPath);
		if (!outFile.open(QIODevice::WriteOnly))
			return fail(QStringLiteral("Cannot write outputPath."));
		outFile.write(fileBody.c_str(), static_cast<qint64>(fileBody.size()));
	}

	(void)body.value(QStringLiteral("brand"));
	(void)body.value(QStringLiteral("scriptStem"));

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("path"), outPath);
	o.insert(QStringLiteral("canonicalPath"), canonicalPath);
	return o;
}

} // namespace cloudsim::host
