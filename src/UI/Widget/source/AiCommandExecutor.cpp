#include "AiCommandExecutor.h"

#include "AiCommandSchema.h"
#include "BackendDataBase.h"
#include "BackendPrimitiveGeometry.h"
#include "DocumentPage.h"
#include "MainWindow.h"
#include "MeshBackendData.h"

#include <memory>

namespace AiCreateMeshRunner
{
namespace
{
void applyPoseFromJson(MeshBackendData& mesh, const nlohmann::json& cmd)
{
	if (!cmd.contains("pose_mm") || !cmd["pose_mm"].is_object())
		return;
	const auto& p = cmd["pose_mm"];
	BackendVec3 pos;
	pos.x = p.value("x", 0.0);
	pos.y = p.value("y", 0.0);
	pos.z = p.value("z", 0.0);
	mesh.setPose(pos);
	if (cmd.contains("rotation_deg") && cmd["rotation_deg"].is_object())
	{
		const auto& r = cmd["rotation_deg"];
		BackendVec3 euler;
		euler.x = r.value("x", 0.0);
		euler.y = r.value("y", 0.0);
		euler.z = r.value("z", 0.0);
		mesh.setRotation(euler);
	}
}
}

bool executeFromJson(MainWindow& mw, const nlohmann::json& cmd, QString& outAssistantReply, QString& outError)
{
	outAssistantReply.clear();
	outError.clear();

	DocumentPage* doc = mw.currentPage();
	if (!doc)
	{
		outError = QStringLiteral("No active document. Create or open a document tab first.");
		return false;
	}

	BackendPrimitiveGeometry::PrimitiveMeshParams params;
	BackendPrimitiveGeometry::PrimitiveMeshQuality quality;
	std::string displayName;
	std::string sourcePath;
	std::string errStd;
	if (!AiCommandSchema::parseCreateMeshCommand(cmd, params, quality, displayName, sourcePath, errStd))
	{
		outError = QString::fromStdString(errStd);
		return false;
	}

	auto soup = BackendPrimitiveGeometry::makePrimitiveTriangleSoup(params, quality);
	if (soup.empty() || (soup.size() % 9U) != 0U)
	{
		outError = QStringLiteral("Failed to build primitive mesh.");
		return false;
	}

	auto mesh = std::make_shared<MeshBackendData>();
	mesh->setName(displayName);
	mesh->setTriangleSoup(std::move(soup));
	applyPoseFromJson(*mesh, cmd);

	if (!mw.registerExistingBackendObject(
			mesh, QString::fromStdString(sourcePath), QStringLiteral("Model"), QString(), true, QString()))
	{
		outError = QStringLiteral("Failed to register mesh in backend.");
		return false;
	}

	QString sceneErr;
	// Match general mesh import: lit plastic + wire; requires outward CCW winding in soup.
	if (!doc->loadMeshFromBackendIntoScene(*mesh, &sceneErr, true, true, true))
	{
		outError = sceneErr;
		return false;
	}

	const QString prim = QString::fromStdString(AiCommandSchema::primitiveKindToString(params.kind));
	outAssistantReply = QStringLiteral("Created %1: %2").arg(prim, QString::fromStdString(displayName));
	return true;
}

}