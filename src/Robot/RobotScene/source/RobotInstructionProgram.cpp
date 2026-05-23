#include "RobotInstructionProgram.h"

#include <cstdio>
#include <string>

namespace RobotInstruction
{
namespace
{
void renumberMotionPointIndicesRecursive(std::vector<std::shared_ptr<Base>>& steps, int& nextIndex)
{
	for (const auto& ins : steps)
	{
		if (!ins)
		{
			continue;
		}
		if (isMotionWaypointType(ins->type()))
		{
			setMotionPointIndex(*ins, nextIndex++);
		}
		if (ins->type() == Type::IF)
		{
			renumberMotionPointIndicesRecursive(
				const_cast<std::vector<std::shared_ptr<Base>>&>(ins->nestedSteps()), nextIndex);
			renumberMotionPointIndicesRecursive(
				const_cast<std::vector<std::shared_ptr<Base>>&>(ins->elseSteps()), nextIndex);
		}
		else if (ins->type() == Type::WHILE)
		{
			renumberMotionPointIndicesRecursive(
				const_cast<std::vector<std::shared_ptr<Base>>&>(ins->nestedSteps()), nextIndex);
		}
	}
}
} // namespace

bool isMotionWaypointType(const Type t)
{
	return t == Type::PTP || t == Type::LINE;
}

int motionPointIndex(const Base& ins)
{
	if (!isMotionWaypointType(ins.type()))
	{
		return 0;
	}
	const auto& ext = ins.extensionProperties();
	const auto it = ext.find(kMotionPointIndexKey);
	if (it == ext.end() || it->second.empty())
	{
		return 0;
	}
	try
	{
		const int v = std::stoi(it->second);
		return v > 0 ? v : 0;
	}
	catch (...)
	{
		return 0;
	}
}

void setMotionPointIndex(Base& ins, const int oneBasedIndex)
{
	if (!isMotionWaypointType(ins.type()) || oneBasedIndex <= 0)
	{
		return;
	}
	ins.setExtensionProperty(kMotionPointIndexKey, std::to_string(oneBasedIndex));
}

std::string formatMotionPointName(const int oneBasedIndex)
{
	if (oneBasedIndex <= 0)
	{
		return {};
	}
	return "P" + std::to_string(oneBasedIndex);
}

void renumberMotionPointIndices(std::vector<std::shared_ptr<Base>>& program)
{
	int nextIndex = 1;
	renumberMotionPointIndicesRecursive(program, nextIndex);
}

std::string formatMotionWaypointSummary(const Base& ins, const bool chinese)
{
	const Vec3 p = ins.pose();
	char xyzBuf[128];
	std::snprintf(
		xyzBuf,
		sizeof(xyzBuf),
		"%.1f, %.1f, %.1f",
		p.x,
		p.y,
		p.z);

	const int pointIndex = motionPointIndex(ins);
	if (pointIndex > 0)
	{
		const std::string pointName = formatMotionPointName(pointIndex);
		if (chinese)
		{
			return pointName + " · 第" + std::to_string(pointIndex) + "点 · XYZ " + xyzBuf;
		}
		return pointName + " · Point " + std::to_string(pointIndex) + " · XYZ " + xyzBuf;
	}
	if (chinese)
	{
		return std::string("XYZ ") + xyzBuf;
	}
	return std::string("XYZ ") + xyzBuf;
}

void collectMotionInstructionsRecursive(
	const std::vector<std::shared_ptr<Base>>& steps,
	std::vector<const Base*>& out)
{
	for (const auto& ins : steps)
	{
		if (!ins)
		{
			continue;
		}
		if (ins->category() == Category::Motion)
		{
			out.push_back(ins.get());
		}
		else if (ins->type() == Type::IF)
		{
			collectMotionInstructionsRecursive(ins->nestedSteps(), out);
			collectMotionInstructionsRecursive(ins->elseSteps(), out);
		}
		else if (ins->type() == Type::WHILE)
		{
			collectMotionInstructionsRecursive(ins->nestedSteps(), out);
		}
	}
}

std::vector<const Base*> collectMotionInstructions(const std::vector<std::shared_ptr<Base>>& program)
{
	std::vector<const Base*> out;
	collectMotionInstructionsRecursive(program, out);
	return out;
}

void flattenInstructionsRecursive(
	const std::vector<std::shared_ptr<Base>>& steps,
	std::vector<std::shared_ptr<Base>>& out)
{
	for (const auto& ins : steps)
	{
		if (!ins)
		{
			continue;
		}
		out.push_back(ins);
		if (ins->type() == Type::IF)
		{
			const auto* ifIns = dynamic_cast<const IfInstruction*>(ins.get());
			if (ifIns)
			{
				flattenInstructionsRecursive(ifIns->nestedSteps(), out);
				flattenInstructionsRecursive(ifIns->elseSteps(), out);
			}
		}
		else if (ins->type() == Type::WHILE)
		{
			flattenInstructionsRecursive(ins->nestedSteps(), out);
		}
	}
}

} // namespace RobotInstruction
