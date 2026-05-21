#include "RobotProgramExport.h"

#include "RobotCoordinateFrames.h"
#include "RobotInstructionProgram.h"

#include <json.hpp>

#include <sstream>

namespace RobotProgramExport
{

bool buildExportResult(
	const std::vector<const RobotInstruction::Base*>& motions,
	const std::vector<RobotInstruction::PlanResult>& plans,
	const std::string& robotSceneBackendId,
	const std::string& urdfPath,
	RobotProgramExportResult& out,
	std::string* errMsg)
{
	out = RobotProgramExportResult{};
	out.robotSceneBackendId = robotSceneBackendId;
	out.urdfPath = urdfPath;
	if (motions.size() != plans.size())
	{
		if (errMsg)
		{
			*errMsg = "Motion instruction count does not match plan result count";
		}
		return false;
	}
	out.points.reserve(motions.size());
	for (size_t i = 0; i < motions.size(); ++i)
	{
		const RobotInstruction::Base* ins = motions[i];
		const RobotInstruction::PlanResult& plan = plans[i];
		if (!ins)
		{
			continue;
		}
		MotionPointExport pt{};
		pt.pointIndex = RobotInstruction::motionPointIndex(*ins);
		pt.type = RobotInstruction::typeToString(ins->type());
		const RobotInstruction::Vec3 p = ins->pose();
		const RobotInstruction::Vec3 e = ins->eulerDeg();
		pt.posBaseMm[0] = p.x;
		pt.posBaseMm[1] = p.y;
		pt.posBaseMm[2] = p.z;
		pt.eulerBaseDeg[0] = e.x;
		pt.eulerBaseDeg[1] = e.y;
		pt.eulerBaseDeg[2] = e.z;
		const auto& ext = ins->extensionProperties();
		const auto itTool = ext.find(RobotCoordinate::kExtMotionToolFrameId);
		if (itTool != ext.end())
		{
			pt.toolFrameId = itTool->second;
		}
		const auto itUser = ext.find(RobotCoordinate::kExtMotionUserFrameId);
		if (itUser != ext.end())
		{
			pt.userFrameId = itUser->second;
		}
		pt.ikOk = plan.ok;
		if (plan.ok)
		{
			pt.jointRad = plan.jointTargetsRad;
		}
		else if (!plan.summary.empty())
		{
			pt.ikError = plan.summary;
		}
		out.points.push_back(std::move(pt));
	}
	return true;
}

bool writeExportResultToJson(const RobotProgramExportResult& result, std::string& outJson, std::string* errMsg)
{
	(void)errMsg;
	nlohmann::json root = nlohmann::json::object();
	root["robotSceneBackendId"] = result.robotSceneBackendId;
	root["urdfPath"] = result.urdfPath;
	root["coordinateFrame"] = "base_tool_origin_mm_deg";
	nlohmann::json points = nlohmann::json::array();
	for (const MotionPointExport& pt : result.points)
	{
		nlohmann::json item = nlohmann::json::object();
		item["pointIndex"] = pt.pointIndex;
		item["type"] = pt.type;
		item["positionBaseMm"] = { pt.posBaseMm[0], pt.posBaseMm[1], pt.posBaseMm[2] };
		item["eulerBaseDeg"] = { pt.eulerBaseDeg[0], pt.eulerBaseDeg[1], pt.eulerBaseDeg[2] };
		if (!pt.toolFrameId.empty())
		{
			item["toolFrameId"] = pt.toolFrameId;
		}
		if (!pt.userFrameId.empty())
		{
			item["userFrameId"] = pt.userFrameId;
		}
		item["ikOk"] = pt.ikOk;
		if (!pt.jointRad.empty())
		{
			item["jointRad"] = pt.jointRad;
		}
		if (!pt.ikError.empty())
		{
			item["ikError"] = pt.ikError;
		}
		points.push_back(std::move(item));
	}
	root["motionPoints"] = std::move(points);
	outJson = root.dump(2);
	return true;
}

bool writeExportResultToCsv(const RobotProgramExportResult& result, std::string& outCsv, std::string* errMsg)
{
	(void)errMsg;
	std::ostringstream oss;
	oss.imbue(std::locale::classic());
	oss << "pointIndex,type,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg,toolFrameId,userFrameId,ikOk,jointRad_csv\n";
	for (const MotionPointExport& pt : result.points)
	{
		oss << pt.pointIndex << ',' << pt.type << ',' << pt.posBaseMm[0] << ',' << pt.posBaseMm[1] << ','
			<< pt.posBaseMm[2] << ',' << pt.eulerBaseDeg[0] << ',' << pt.eulerBaseDeg[1] << ','
			<< pt.eulerBaseDeg[2] << ',' << pt.toolFrameId << ',' << pt.userFrameId << ','
			<< (pt.ikOk ? 1 : 0) << ',';
		for (size_t j = 0; j < pt.jointRad.size(); ++j)
		{
			if (j > 0)
			{
				oss << ' ';
			}
			oss << pt.jointRad[j];
		}
		oss << '\n';
	}
	outCsv = oss.str();
	return true;
}

} // namespace RobotProgramExport
