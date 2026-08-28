/// @file RobotInstructionIkContext.cpp
/// @brief 规划前工具上下文

#include "RobotInstructionIkContext.h"

#include <iomanip>
#include <sstream>

namespace RobotInstruction
{
namespace
{
std::string encodeJointRadCsv(const std::vector<double>& jointAnglesRad)
{
	std::ostringstream oss;
	oss.imbue(std::locale::classic());
	oss << std::setprecision(12);
	for (size_t i = 0; i < jointAnglesRad.size(); ++i)
	{
		if (i > 0)
		{
			oss << ',';
		}
		oss << jointAnglesRad[i];
	}
	return oss.str();
}

void syncInstructionToolContextFromToolFrame(Base& ins, const RobotCoordinate::RobotCoordinateFrameSet& frames,
											 const RobotCoordinate::RobotToolFrame& tool)
{
	const BackendMat4 toolMat = RobotCoordinate::frameToMat4(tool.T_flange_tool);
	ins.setExtensionProperty(RobotCoordinate::kExtContextToolFrameMat4, RobotCoordinate::encodeMat4Csv(toolMat));
	ins.setExtensionProperty("context.activeToolFrameId", tool.id);
	const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(frames, tool);
	if (!flangeLink.empty())
	{
		ins.setExtensionProperty("context.flangeLinkName", flangeLink);
	}
}
} // namespace

bool motionUsesActiveToolFrame(const Base& ins)
{
	const auto& ext = ins.extensionProperties();
	const auto itMotion = ext.find(RobotCoordinate::kExtMotionToolFrameId);
	return itMotion == ext.end() || itMotion->second.empty() || itMotion->second == "active";
}

void syncToolContextFromFrames(Base& ins, const RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	const RobotCoordinate::RobotToolFrame* tool = nullptr;
	if (motionUsesActiveToolFrame(ins))
	{
		// 跟随 active 必须用当前激活工具，不能走 resolve 里 stale 的 frozen id
		tool = RobotCoordinate::activeToolFrame(frames);
	}
	else if (const RobotCoordinate::RobotToolFrame* resolved =
				 RobotCoordinate::resolveToolFrameForExtension(frames, ins.extensionProperties()))
	{
		tool = resolved;
	}
	if (tool)
	{
		syncInstructionToolContextFromToolFrame(ins, frames, *tool);
	}
	else if (motionUsesActiveToolFrame(ins))
	{
		// 跟随 active 且当前无工具：清陈旧 toolMat，避免假 hasToolMat 抢法兰链
		ins.eraseExtensionProperty(RobotCoordinate::kExtContextToolFrameMat4);
		ins.eraseExtensionProperty("context.activeToolFrameId");
	}
}

void prepareInstructionIkContext(Base& ins, const std::vector<double>& rollingQ, const std::string& urdfPath,
								 const std::string& defaultTcpLinkName,
								 const RobotCoordinate::RobotCoordinateFrameSet* frames)
{
	// 仅本次 plan 临时注入种子；调用方须 backup/restore，禁止当作指令持久化字段
	ins.setExtensionProperty("context.currentJointRadCsv", encodeJointRadCsv(rollingQ));
	ins.setExtensionProperty("context.urdfPath", urdfPath);
	ins.setExtensionProperty("context.tcpLinkName", defaultTcpLinkName);
	if (!frames)
	{
		return;
	}
	// 先 sync：跟随 active 时刷新为当前工具矩阵
	syncToolContextFromFrames(ins, *frames);
	// 无工具时不要写单位 toolFrameMat4：否则下游 hasToolMat 恒真，强制法兰 IK，
	// 与示教时 tcpLink（可能≠法兰）目标脱节 → DLS 不收敛
	{
		const auto& ext = ins.extensionProperties();
		const auto itFlange = ext.find("context.flangeLinkName");
		if ((itFlange == ext.end() || itFlange->second.empty()) && !frames->flangeLinkName.empty())
		{
			ins.setExtensionProperty("context.flangeLinkName", frames->flangeLinkName);
		}
	}
}

} // namespace RobotInstruction
