#pragma once

#include "RobotInstructionModel.h"
#include "robot_scene_global.h"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace RobotInstruction
{

enum class StepContainerKind
{
	Root = 0,
	IfThen,
	IfElse,
	WhileBody
};

struct ROBOT_SCENE_API ProgramPath
{
	StepContainerKind container = StepContainerKind::Root;
	std::vector<size_t> parentInstructionIndices;
	size_t indexInContainer = 0;
};

class ROBOT_SCENE_API InstructionProgramDocument
{
public:
	explicit InstructionProgramDocument(std::vector<std::shared_ptr<Base>>* rootSteps);

	std::vector<std::shared_ptr<Base>>* rootSteps() { return m_root; }
	const std::vector<std::shared_ptr<Base>>* rootSteps() const { return m_root; }

	Base* findById(const std::string& instructionId) const;
	bool removeById(const std::string& instructionId);
	bool insertAtRoot(size_t index, std::shared_ptr<Base> instruction);
	bool appendToRoot(std::shared_ptr<Base> instruction);

	void collectIdMap(std::unordered_map<std::string, std::shared_ptr<Base>>& out) const;
	void renumberAndNotify();

	static void collectIdMapRecursive(
		const std::vector<std::shared_ptr<Base>>& steps,
		std::unordered_map<std::string, std::shared_ptr<Base>>& out);

private:
	static bool removeRecursive(std::vector<std::shared_ptr<Base>>& steps, const std::string& id);
	static Base* findRecursive(const std::vector<std::shared_ptr<Base>>& steps, const std::string& id);

	std::vector<std::shared_ptr<Base>>* m_root = nullptr;
};

} // namespace RobotInstruction
