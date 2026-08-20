#ifndef ROBOTWIDGET_INSTRUCTIONPROGRAMTREEWIDGET_H
#define ROBOTWIDGET_INSTRUCTIONPROGRAMTREEWIDGET_H

/// @file InstructionProgramTreeWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 层级指令程序编辑器，支持拖放改父与分组嵌套显示

#include "robotwidget_global.h"

#include "RobotInstructionModel.h"
#include "RobotProgramCatalog.h"

#include <QTimer>
#include <QTreeWidget>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/// 层级指令程序编辑器，支持拖放改父与分组嵌套显示

class ROBOTWIDGET_EXPORT InstructionProgramTreeWidget : public QTreeWidget

{
	Q_OBJECT

public:
	enum class NodeKind
	{
		Instruction = 0,
		ThenBranch,
		ElseBranch,
		Group,
		PlanningSection,
		PathPlanOutputRef,
		WaypointDetail
	};

	explicit InstructionProgramTreeWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);

	void setProgram(std::vector<std::shared_ptr<RobotInstruction::Base>>* program);

	void setGroupMembership(std::vector<RobotInstruction::InstructionGroup>* groups);

	void setGroupVisibilityQuery(std::function<bool(const std::string& groupId)> query);

	void rebuildFromProgram();

	void syncToProgram();

	std::shared_ptr<RobotInstruction::Base> selectedInstruction() const;

	std::vector<std::shared_ptr<RobotInstruction::Base>> selectedMotionInstructions() const;

	std::vector<std::shared_ptr<RobotInstruction::Base>> selectedRootInstructions() const;

	QTreeWidgetItem* selectedItem() const { return currentItem(); }

	/// 相对当前选中插入指令（或追加到根）

	/// emitSelection 为 false 时不发 instructionSelected（避免调用方设工具扩展前触发预览/IK）

	void insertInstruction(const std::shared_ptr<RobotInstruction::Base>& ins, bool emitSelection = true);

	void removeSelected();

	void clearProgram();

	/// Run 期间按指令 id 选中树节点；调用方应 QSignalBlocker 避免触发 instructionSelected
	void selectInstructionByRaw(RobotInstruction::Base* raw);

signals:

	void programStructureChanged();

	void groupMembershipChanged();

	void instructionSelected(std::shared_ptr<RobotInstruction::Base> instruction);

	void createGroupRequested(const QString& name, const std::vector<std::string>& memberIds);

	void dissolveGroupRequested(const std::string& groupId);

	void renameGroupRequested(const std::string& groupId, const QString& newName);

	void groupVisibilityChangeRequested(const std::string& groupId, bool visible);

protected:
	void startDrag(Qt::DropActions supportedActions) override;

	void dragEnterEvent(QDragEnterEvent* event) override;

	void dragMoveEvent(QDragMoveEvent* event) override;

	void dropEvent(QDropEvent* event) override;

	void contextMenuEvent(QContextMenuEvent* event) override;

private:
	static NodeKind nodeKind(const QTreeWidgetItem* item);

	static QString instructionIdOf(const QTreeWidgetItem* item);

	RobotInstruction::Base* instructionRaw(const QTreeWidgetItem* item) const;

	static std::string groupIdFromItem(const QTreeWidgetItem* item);

	static void setInstructionId(QTreeWidgetItem* item, const std::string& id);

	static void setGroupPtr(QTreeWidgetItem* item, const std::string& groupId);

	static QString formatInstructionLabel(const RobotInstruction::Base& ins, bool chinese);

	static bool isRootLevelInstructionItem(const QTreeWidgetItem* item);

	bool isPathPlanInstructionItem(const QTreeWidgetItem* item) const;

	QTreeWidgetItem* findPlanningSectionItem() const;

	size_t countRootPathPlansInProgram() const;

	QTreeWidgetItem* createPlanningSectionItem();

	QTreeWidgetItem* createPathPlanOutputRefItem(

		const std::string& pathPlanId,

		const RobotInstruction::InstructionGroup& outputGroup,

		bool chinese);

	QTreeWidgetItem* createInstructionItem(const std::shared_ptr<RobotInstruction::Base>& ins);

	QTreeWidgetItem* createGroupItem(const RobotInstruction::InstructionGroup& group);

	void populateInstructionItem(QTreeWidgetItem* item, const std::shared_ptr<RobotInstruction::Base>& ins);

	QTreeWidgetItem* appendBranchHeader(QTreeWidgetItem* parent, NodeKind branch, bool chinese);

	void readStepsFromChildren(

		QTreeWidgetItem* container,

		std::vector<std::shared_ptr<RobotInstruction::Base>>& out,

		const std::unordered_map<std::string, std::shared_ptr<RobotInstruction::Base>>& idMap) const;

	void syncLogicBranchesFromTreeItem(
		QTreeWidgetItem* item,
		const std::unordered_map<std::string, std::shared_ptr<RobotInstruction::Base>>& idMap) const;

	void readProgramFromTree(

		std::vector<std::shared_ptr<RobotInstruction::Base>>& root,

		const std::unordered_map<std::string, std::shared_ptr<RobotInstruction::Base>>& idMap) const;

	void syncGroupsFromTree();

	bool canAcceptDrop(QTreeWidgetItem* dragged, QTreeWidgetItem* target, DropIndicatorPosition pos) const;

	void applyDrop(QTreeWidgetItem* dragged, QTreeWidgetItem* target, DropIndicatorPosition pos);

	QTreeWidgetItem* takeTreeItem(QTreeWidgetItem* item);

	void showContextMenu(const QPoint& globalPos);

	std::string resolveGroupIdForContextItem(const QTreeWidgetItem* item) const;

	std::vector<std::shared_ptr<RobotInstruction::Base>>* m_program = nullptr;

	std::vector<RobotInstruction::InstructionGroup>* m_groups = nullptr;

	std::function<bool(const std::string& groupId)> m_groupVisibilityQuery;

	bool m_useChinese = false;

	bool m_syncing = false;

	QTimer m_selectionDebounce;

	QTreeWidgetItem* m_dragItem = nullptr;
};

#endif // ROBOTWIDGET_INSTRUCTIONPROGRAMTREEWIDGET_H
