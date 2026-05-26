#pragma once

#include "RobotInstructionModel.h"
#include "RobotProgramCatalog.h"
#include "robotwidget_global.h"

#include <QTreeWidget>

#include <memory>
#include <unordered_map>
#include <vector>

/// 层级指令程序编辑器，支持拖放改父
class ROBOTWIDGET_EXPORT InstructionProgramTreeWidget : public QTreeWidget
{
	Q_OBJECT

public:
	enum class NodeKind
	{
		Instruction = 0,
		ThenBranch,
		ElseBranch
	};

	explicit InstructionProgramTreeWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setProgram(std::vector<std::shared_ptr<RobotInstruction::Base>>* program);
	void setGroupMembership(const std::vector<RobotInstruction::InstructionGroup>* groups);
	void rebuildFromProgram();
	void syncToProgram();

	std::shared_ptr<RobotInstruction::Base> selectedInstruction() const;
	std::vector<std::shared_ptr<RobotInstruction::Base>> selectedMotionInstructions() const;
	QTreeWidgetItem* selectedItem() const { return currentItem(); }

	/// 相对当前选中插入指令（或追加到根）
	/// emitSelection 为 false 时不发 instructionSelected（避免调用方设工具扩展前触发预览/IK）
	void insertInstruction(const std::shared_ptr<RobotInstruction::Base>& ins, bool emitSelection = true);
	void removeSelected();
	void clearProgram();

signals:
	void programStructureChanged();
	void instructionSelected(std::shared_ptr<RobotInstruction::Base> instruction);

protected:
	void startDrag(Qt::DropActions supportedActions) override;
	void dragEnterEvent(QDragEnterEvent* event) override;
	void dragMoveEvent(QDragMoveEvent* event) override;
	void dropEvent(QDropEvent* event) override;

private:
	static NodeKind nodeKind(const QTreeWidgetItem* item);
	static RobotInstruction::Base* instructionRaw(const QTreeWidgetItem* item);
	static void setInstructionPtr(QTreeWidgetItem* item, RobotInstruction::Base* raw);
	static QString formatInstructionLabel(const RobotInstruction::Base& ins, bool chinese);

	QTreeWidgetItem* createInstructionItem(const std::shared_ptr<RobotInstruction::Base>& ins);
	void populateInstructionItem(QTreeWidgetItem* item, const std::shared_ptr<RobotInstruction::Base>& ins);
	QTreeWidgetItem* appendBranchHeader(QTreeWidgetItem* parent, NodeKind branch, bool chinese);
	void readStepsFromChildren(
		QTreeWidgetItem* container,
		std::vector<std::shared_ptr<RobotInstruction::Base>>& out,
		const std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>>& ptrMap) const;
	void readProgramFromTree(
		std::vector<std::shared_ptr<RobotInstruction::Base>>& root,
		const std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>>& ptrMap) const;

	bool canAcceptDrop(QTreeWidgetItem* dragged, QTreeWidgetItem* target, DropIndicatorPosition pos) const;
	void applyDrop(QTreeWidgetItem* dragged, QTreeWidgetItem* target, DropIndicatorPosition pos);
	QTreeWidgetItem* takeTreeItem(QTreeWidgetItem* item);
	void selectInstructionByRaw(RobotInstruction::Base* raw);

	std::vector<std::shared_ptr<RobotInstruction::Base>>* m_program = nullptr;
	const std::vector<RobotInstruction::InstructionGroup>* m_groups = nullptr;
	bool m_useChinese = false;
	bool m_syncing = false;
	QTreeWidgetItem* m_dragItem = nullptr;
};
