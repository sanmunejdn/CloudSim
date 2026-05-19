#include "InstructionProgramTreeWidget.h"
#include "RobotInstructionProgram.h"

#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QMimeData>

#include <algorithm>
#include <functional>

namespace
{
constexpr int kKindRole = Qt::UserRole;
constexpr int kInstrPtrRole = Qt::UserRole + 1;

QString branchLabel(InstructionProgramTreeWidget::NodeKind kind, bool chinese)
{
	switch (kind)
	{
	case InstructionProgramTreeWidget::NodeKind::ThenBranch:
		return chinese ? QStringLiteral("Then（真）") : QStringLiteral("Then");
	case InstructionProgramTreeWidget::NodeKind::ElseBranch:
		return chinese ? QStringLiteral("Else（假）") : QStringLiteral("Else");
	default:
		return QString();
	}
}

void collectPtrMapRecursive(
	const std::vector<std::shared_ptr<RobotInstruction::Base>>& steps,
	std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>>& out)
{
	for (const auto& ins : steps)
	{
		if (!ins)
		{
			continue;
		}
		out[ins.get()] = ins;
		if (ins->type() == RobotInstruction::Type::IF)
		{
			const auto* ifIns = dynamic_cast<const RobotInstruction::IfInstruction*>(ins.get());
			if (ifIns)
			{
				collectPtrMapRecursive(ifIns->nestedSteps(), out);
				collectPtrMapRecursive(ifIns->elseSteps(), out);
			}
		}
		else if (ins->type() == RobotInstruction::Type::WHILE)
		{
			collectPtrMapRecursive(ins->nestedSteps(), out);
		}
	}
}
} // namespace

InstructionProgramTreeWidget::InstructionProgramTreeWidget(QWidget* parent)
	: QTreeWidget(parent)
{
	setHeaderHidden(true);
	setRootIsDecorated(true);
	setAlternatingRowColors(true);
	setMinimumHeight(120);
	setSelectionMode(QAbstractItemView::SingleSelection);
	setDragEnabled(true);
	setAcceptDrops(true);
	setDropIndicatorShown(true);
	setDragDropMode(QAbstractItemView::DragDrop);
	setDefaultDropAction(Qt::MoveAction);

	connect(this, &QTreeWidget::itemSelectionChanged, this, [this]() {
		if (m_syncing)
		{
			return;
		}
		emit instructionSelected(selectedInstruction());
	});
}

void InstructionProgramTreeWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	rebuildFromProgram();
}

void InstructionProgramTreeWidget::setProgram(std::vector<std::shared_ptr<RobotInstruction::Base>>* program)
{
	m_program = program;
	rebuildFromProgram();
}

InstructionProgramTreeWidget::NodeKind InstructionProgramTreeWidget::nodeKind(const QTreeWidgetItem* item)
{
	if (!item)
	{
		return NodeKind::Instruction;
	}
	return static_cast<NodeKind>(item->data(0, kKindRole).toInt());
}

RobotInstruction::Base* InstructionProgramTreeWidget::instructionRaw(const QTreeWidgetItem* item)
{
	if (!item || nodeKind(item) != NodeKind::Instruction)
	{
		return nullptr;
	}
	return reinterpret_cast<RobotInstruction::Base*>(item->data(0, kInstrPtrRole).value<quintptr>());
}

void InstructionProgramTreeWidget::setInstructionPtr(QTreeWidgetItem* item, RobotInstruction::Base* raw)
{
	item->setData(0, kKindRole, static_cast<int>(NodeKind::Instruction));
	item->setData(0, kInstrPtrRole, QVariant::fromValue<quintptr>(reinterpret_cast<quintptr>(raw)));
}

QString InstructionProgramTreeWidget::formatInstructionLabel(const RobotInstruction::Base& ins, const bool chinese)
{
	auto typeLabel = [&](RobotInstruction::Type t) {
		switch (t)
		{
		case RobotInstruction::Type::LINE:
			return chinese ? QStringLiteral("直线") : QStringLiteral("LINE");
		case RobotInstruction::Type::WAIT:
			return chinese ? QStringLiteral("等待") : QStringLiteral("WAIT");
		case RobotInstruction::Type::IF:
			return chinese ? QStringLiteral("条件") : QStringLiteral("IF");
		case RobotInstruction::Type::WHILE:
			return chinese ? QStringLiteral("循环") : QStringLiteral("WHILE");
		case RobotInstruction::Type::SET_DO:
			return chinese ? QStringLiteral("数字输出") : QStringLiteral("SET_DO");
		case RobotInstruction::Type::SET_AO:
			return chinese ? QStringLiteral("模拟输出") : QStringLiteral("SET_AO");
		case RobotInstruction::Type::PTP:
		default:
			return chinese ? QStringLiteral("点到点") : QStringLiteral("PTP");
		}
	};

	QString summary;
	switch (ins.type())
	{
	case RobotInstruction::Type::WAIT:
		summary = chinese ? QStringLiteral("时长 %1 s").arg(ins.durationSec(), 0, 'f', 2)
						  : QStringLiteral("duration %1 s").arg(ins.durationSec(), 0, 'f', 2);
		break;
	case RobotInstruction::Type::IF:
	case RobotInstruction::Type::WHILE:
	{
		const RobotInstruction::Condition& c = ins.condition();
		switch (c.kind)
		{
		case RobotInstruction::ConditionKind::Never:
			summary = chinese ? QStringLiteral("永不") : QStringLiteral("never");
			break;
		case RobotInstruction::ConditionKind::Io:
			summary = QStringLiteral("IO%1==%2").arg(c.ioPort).arg(c.ioEquals ? 1 : 0);
			break;
		case RobotInstruction::ConditionKind::Compare:
			summary = QString::fromStdString(c.compareLeft + " " + c.compareOp + " "
				+ std::to_string(c.compareRight));
			break;
		default:
			summary = chinese ? QStringLiteral("始终") : QStringLiteral("always");
			break;
		}
		break;
	}
	case RobotInstruction::Type::SET_DO:
		summary = QStringLiteral("port %1 = %2").arg(ins.ioPort()).arg(ins.ioBoolValue() ? 1 : 0);
		break;
	case RobotInstruction::Type::SET_AO:
		summary = QStringLiteral("port %1 = %2").arg(ins.ioPort()).arg(ins.ioAnalogValue(), 0, 'f', 2);
		break;
	case RobotInstruction::Type::PTP:
	case RobotInstruction::Type::LINE:
	default:
		summary = QString::fromStdString(RobotInstruction::formatMotionWaypointSummary(ins, chinese));
		if (ins.hasMotionAxisConfigurationProperty())
		{
			const std::string axisSummary =
				RobotInstruction::formatMotionAxisConfigurationSummary(ins.motionAxisConfiguration(), chinese);
			if (!axisSummary.empty())
			{
				summary += chinese ? QStringLiteral(" · ") : QStringLiteral(" · ");
				summary += QString::fromStdString(axisSummary);
			}
		}
		break;
	}
	const int pointIndex = RobotInstruction::motionPointIndex(ins);
	if (pointIndex > 0 && RobotInstruction::isMotionWaypointType(ins.type()))
	{
		const QString pointName = QString::fromStdString(RobotInstruction::formatMotionPointName(pointIndex));
		return QStringLiteral("%1 [%2] %3").arg(pointName, typeLabel(ins.type()), summary);
	}
	return QStringLiteral("[%1] %2").arg(typeLabel(ins.type()), summary);
}

QTreeWidgetItem* InstructionProgramTreeWidget::createInstructionItem(
	const std::shared_ptr<RobotInstruction::Base>& ins)
{
	auto* item = new QTreeWidgetItem();
	item->setFlags(item->flags() | Qt::ItemIsDragEnabled | Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	populateInstructionItem(item, ins);
	return item;
}

void InstructionProgramTreeWidget::populateInstructionItem(
	QTreeWidgetItem* item,
	const std::shared_ptr<RobotInstruction::Base>& ins)
{
	if (!item || !ins)
	{
		return;
	}
	setInstructionPtr(item, ins.get());
	item->setText(0, formatInstructionLabel(*ins, m_useChinese));

	if (ins->type() == RobotInstruction::Type::IF)
	{
		appendBranchHeader(item, NodeKind::ThenBranch, m_useChinese);
		appendBranchHeader(item, NodeKind::ElseBranch, m_useChinese);
		const auto* ifIns = dynamic_cast<const RobotInstruction::IfInstruction*>(ins.get());
		if (ifIns)
		{
			QTreeWidgetItem* thenHdr = item->child(0);
			QTreeWidgetItem* elseHdr = item->child(1);
			for (const auto& step : ifIns->nestedSteps())
			{
				if (step)
				{
					thenHdr->addChild(createInstructionItem(step));
				}
			}
			for (const auto& step : ifIns->elseSteps())
			{
				if (step)
				{
					elseHdr->addChild(createInstructionItem(step));
				}
			}
		}
		item->setExpanded(true);
	}
	else if (ins->type() == RobotInstruction::Type::WHILE)
	{
		for (const auto& step : ins->nestedSteps())
		{
			if (step)
			{
				item->addChild(createInstructionItem(step));
			}
		}
		item->setExpanded(true);
	}
}

QTreeWidgetItem* InstructionProgramTreeWidget::appendBranchHeader(
	QTreeWidgetItem* parent,
	const NodeKind branch,
	const bool chinese)
{
	auto* hdr = new QTreeWidgetItem(parent);
	hdr->setText(0, branchLabel(branch, chinese));
	hdr->setData(0, kKindRole, static_cast<int>(branch));
	hdr->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDropEnabled | Qt::ItemIsSelectable);
	hdr->setExpanded(true);
	return hdr;
}

void InstructionProgramTreeWidget::rebuildFromProgram()
{
	m_syncing = true;
	if (m_program)
	{
		RobotInstruction::renumberMotionPointIndices(*m_program);
	}
	clear();
	if (!m_program)
	{
		m_syncing = false;
		return;
	}
	for (const auto& ins : *m_program)
	{
		if (ins)
		{
			addTopLevelItem(createInstructionItem(ins));
		}
	}
	expandAll();
	m_syncing = false;
}

void InstructionProgramTreeWidget::readStepsFromChildren(
	QTreeWidgetItem* container,
	std::vector<std::shared_ptr<RobotInstruction::Base>>& out,
	const std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>>& ptrMap) const
{
	out.clear();
	if (!container)
	{
		return;
	}
	for (int i = 0; i < container->childCount(); ++i)
	{
		QTreeWidgetItem* ch = container->child(i);
		if (nodeKind(ch) != NodeKind::Instruction)
		{
			continue;
		}
		RobotInstruction::Base* raw = instructionRaw(ch);
		const auto it = ptrMap.find(raw);
		if (it != ptrMap.end())
		{
			out.push_back(it->second);
		}
	}
}

void InstructionProgramTreeWidget::readProgramFromTree(
	std::vector<std::shared_ptr<RobotInstruction::Base>>& root,
	const std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>>& ptrMap) const
{
	root.clear();
	for (int i = 0; i < topLevelItemCount(); ++i)
	{
		QTreeWidgetItem* item = topLevelItem(i);
		if (nodeKind(item) != NodeKind::Instruction)
		{
			continue;
		}
		RobotInstruction::Base* raw = instructionRaw(item);
		const auto it = ptrMap.find(raw);
		if (it == ptrMap.end())
		{
			continue;
		}
		std::shared_ptr<RobotInstruction::Base> ins = it->second;
		root.push_back(ins);

		if (ins->type() == RobotInstruction::Type::IF)
		{
			auto* ifIns = dynamic_cast<RobotInstruction::IfInstruction*>(ins.get());
			if (ifIns && item->childCount() >= 2)
			{
				readStepsFromChildren(item->child(0), ifIns->thenSteps(), ptrMap);
				readStepsFromChildren(item->child(1), ifIns->elseStepsMut(), ptrMap);
			}
		}
		else if (ins->type() == RobotInstruction::Type::WHILE)
		{
			auto* whileIns = dynamic_cast<RobotInstruction::WhileInstruction*>(ins.get());
			if (whileIns)
			{
				std::vector<std::shared_ptr<RobotInstruction::Base>> body;
				for (int c = 0; c < item->childCount(); ++c)
				{
					QTreeWidgetItem* ch = item->child(c);
					if (nodeKind(ch) != NodeKind::Instruction)
					{
						continue;
					}
					RobotInstruction::Base* rawChild = instructionRaw(ch);
					const auto itChild = ptrMap.find(rawChild);
					if (itChild != ptrMap.end())
					{
						body.push_back(itChild->second);
					}
				}
				whileIns->bodySteps() = std::move(body);
			}
		}
	}
}

void InstructionProgramTreeWidget::syncToProgram()
{
	if (!m_program)
	{
		return;
	}
	std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>> ptrMap;
	collectPtrMapRecursive(*m_program, ptrMap);

	std::vector<std::shared_ptr<RobotInstruction::Base>> newRoot;
	readProgramFromTree(newRoot, ptrMap);
	*m_program = std::move(newRoot);
	emit programStructureChanged();
}

std::shared_ptr<RobotInstruction::Base> InstructionProgramTreeWidget::selectedInstruction() const
{
	QTreeWidgetItem* item = currentItem();
	if (!item)
	{
		return nullptr;
	}
	RobotInstruction::Base* raw = instructionRaw(item);
	if (!raw || !m_program)
	{
		return nullptr;
	}
	std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>> ptrMap;
	collectPtrMapRecursive(*m_program, ptrMap);
	const auto it = ptrMap.find(raw);
	return it != ptrMap.end() ? it->second : nullptr;
}

QTreeWidgetItem* InstructionProgramTreeWidget::takeTreeItem(QTreeWidgetItem* item)
{
	if (!item)
	{
		return nullptr;
	}
	if (QTreeWidgetItem* parent = item->parent())
	{
		return parent->takeChild(parent->indexOfChild(item));
	}
	return takeTopLevelItem(indexOfTopLevelItem(item));
}

void InstructionProgramTreeWidget::selectInstructionByRaw(RobotInstruction::Base* raw)
{
	if (!raw)
	{
		return;
	}
	const auto walk = [&](auto&& self, QTreeWidgetItem* node) -> QTreeWidgetItem* {
		if (!node)
		{
			return nullptr;
		}
		if (instructionRaw(node) == raw)
		{
			return node;
		}
		for (int i = 0; i < node->childCount(); ++i)
		{
			if (QTreeWidgetItem* found = self(self, node->child(i)))
			{
				return found;
			}
		}
		return nullptr;
	};
	for (int i = 0; i < topLevelItemCount(); ++i)
	{
		if (QTreeWidgetItem* found = walk(walk, topLevelItem(i)))
		{
			setCurrentItem(found);
			return;
		}
	}
}

void InstructionProgramTreeWidget::insertInstruction(
	const std::shared_ptr<RobotInstruction::Base>& ins,
	const bool emitSelection)
{
	if (!ins || !m_program)
	{
		return;
	}
	syncToProgram();

	QTreeWidgetItem* sel = currentItem();
	if (!sel)
	{
		m_program->push_back(ins);
	}
	else
	{
		const NodeKind k = nodeKind(sel);
		if (k == NodeKind::ThenBranch || k == NodeKind::ElseBranch)
		{
			QTreeWidgetItem* ifItem = sel->parent();
			auto* ifIns = dynamic_cast<RobotInstruction::IfInstruction*>(instructionRaw(ifItem));
			if (ifIns)
			{
				if (k == NodeKind::ThenBranch)
				{
					ifIns->thenSteps().push_back(ins);
				}
				else
				{
					ifIns->elseStepsMut().push_back(ins);
				}
			}
		}
		else if (RobotInstruction::Base* raw = instructionRaw(sel))
		{
			if (raw->type() == RobotInstruction::Type::WHILE)
			{
				if (auto* whileIns = dynamic_cast<RobotInstruction::WhileInstruction*>(raw))
				{
					whileIns->bodySteps().push_back(ins);
				}
			}
			else if (raw->type() == RobotInstruction::Type::IF)
			{
				if (auto* ifIns = dynamic_cast<RobotInstruction::IfInstruction*>(raw))
				{
					ifIns->thenSteps().push_back(ins);
				}
			}
			else if (QTreeWidgetItem* branch = sel->parent())
			{
				const NodeKind pk = nodeKind(branch);
				if (pk == NodeKind::ThenBranch || pk == NodeKind::ElseBranch)
				{
					if (auto* ifIns = dynamic_cast<RobotInstruction::IfInstruction*>(instructionRaw(branch->parent())))
					{
						if (pk == NodeKind::ThenBranch)
						{
							ifIns->thenSteps().push_back(ins);
						}
						else
						{
							ifIns->elseStepsMut().push_back(ins);
						}
					}
				}
				else
				{
					m_program->push_back(ins);
				}
			}
			else
			{
				m_program->push_back(ins);
			}
		}
	}

	rebuildFromProgram();
	m_syncing = true;
	selectInstructionByRaw(ins.get());
	m_syncing = false;
	if (emitSelection)
	{
		emit instructionSelected(ins);
	}
}

void InstructionProgramTreeWidget::removeSelected()
{
	QTreeWidgetItem* sel = currentItem();
	if (!sel || nodeKind(sel) != NodeKind::Instruction)
	{
		return;
	}
	RobotInstruction::Base* raw = instructionRaw(sel);
	syncToProgram();

	std::function<bool(std::vector<std::shared_ptr<RobotInstruction::Base>>&)> removeRecursive;
	removeRecursive = [&](std::vector<std::shared_ptr<RobotInstruction::Base>>& steps) -> bool {
		for (auto it = steps.begin(); it != steps.end(); ++it)
		{
			if (!*it)
			{
				continue;
			}
			if (it->get() == raw)
			{
				steps.erase(it);
				return true;
			}
			if ((*it)->type() == RobotInstruction::Type::IF)
			{
				auto* ifIns = dynamic_cast<RobotInstruction::IfInstruction*>(it->get());
				if (ifIns)
				{
					if (removeRecursive(ifIns->thenSteps()) || removeRecursive(ifIns->elseStepsMut()))
					{
						return true;
					}
				}
			}
			else if ((*it)->type() == RobotInstruction::Type::WHILE)
			{
				if (auto* whileIns = dynamic_cast<RobotInstruction::WhileInstruction*>(it->get()))
				{
					if (removeRecursive(whileIns->bodySteps()))
					{
						return true;
					}
				}
			}
		}
		return false;
	};

	if (m_program)
	{
		(void)removeRecursive(*m_program);
	}
	rebuildFromProgram();
	emit instructionSelected(nullptr);
}

void InstructionProgramTreeWidget::clearProgram()
{
	clear();
	if (m_program)
	{
		m_program->clear();
	}
	emit programStructureChanged();
	emit instructionSelected(nullptr);
}

void InstructionProgramTreeWidget::startDrag(Qt::DropActions supportedActions)
{
	m_dragItem = currentItem();
	if (!m_dragItem || nodeKind(m_dragItem) != NodeKind::Instruction)
	{
		m_dragItem = nullptr;
		return;
	}
	QTreeWidget::startDrag(supportedActions);
}

void InstructionProgramTreeWidget::dragEnterEvent(QDragEnterEvent* event)
{
	if (event->source() == this)
	{
		event->acceptProposedAction();
	}
	else
	{
		event->ignore();
	}
}

bool InstructionProgramTreeWidget::canAcceptDrop(
	QTreeWidgetItem* dragged,
	QTreeWidgetItem* target,
	const DropIndicatorPosition pos) const
{
	if (!dragged || nodeKind(dragged) != NodeKind::Instruction)
	{
		return false;
	}
	if (dragged == target)
	{
		return false;
	}
	// Cannot drop into own descendants.
	for (QTreeWidgetItem* p = target; p; p = p->parent())
	{
		if (p == dragged)
		{
			return false;
		}
	}

	if (!target)
	{
		return pos == OnViewport || pos == BelowItem;
	}

	const NodeKind tk = nodeKind(target);
	if (tk == NodeKind::ThenBranch || tk == NodeKind::ElseBranch)
	{
		return pos == OnItem || pos == BelowItem || pos == AboveItem;
	}
	if (tk == NodeKind::Instruction)
	{
		RobotInstruction::Base* raw = instructionRaw(target);
		if (raw && (raw->type() == RobotInstruction::Type::IF || raw->type() == RobotInstruction::Type::WHILE))
		{
			if (pos == OnItem)
			{
				return true;
			}
		}
		return pos == BelowItem || pos == AboveItem;
	}
	return false;
}

void InstructionProgramTreeWidget::applyDrop(
	QTreeWidgetItem* dragged,
	QTreeWidgetItem* target,
	const DropIndicatorPosition pos)
{
	QTreeWidgetItem* item = takeTreeItem(dragged);
	if (!item)
	{
		return;
	}

	auto insertInto = [&](QTreeWidgetItem* parent, int index) {
		if (!parent)
		{
			insertTopLevelItem(index, item);
		}
		else
		{
			parent->insertChild(index, item);
			parent->setExpanded(true);
		}
	};

	if (!target || pos == OnViewport)
	{
		insertInto(nullptr, topLevelItemCount());
		setCurrentItem(item);
		return;
	}

	if (const NodeKind tk = nodeKind(target); tk == NodeKind::ThenBranch || tk == NodeKind::ElseBranch)
	{
		int insertAt = target->childCount();
		if (pos == AboveItem)
		{
			insertAt = 0;
		}
		target->insertChild(insertAt, item);
		target->setExpanded(true);
		setCurrentItem(item);
		return;
	}

	if (nodeKind(target) == NodeKind::Instruction)
	{
		if (RobotInstruction::Base* raw = instructionRaw(target))
		{
			if ((raw->type() == RobotInstruction::Type::IF || raw->type() == RobotInstruction::Type::WHILE)
				&& pos == OnItem)
			{
				QTreeWidgetItem* container = target;
				if (raw->type() == RobotInstruction::Type::IF && target->childCount() > 0)
				{
					container = target->child(0);
				}
				container->addChild(item);
				container->setExpanded(true);
				setCurrentItem(item);
				return;
			}
		}
		QTreeWidgetItem* parent = target->parent();
		int index = parent ? parent->indexOfChild(target) : indexOfTopLevelItem(target);
		if (pos == BelowItem)
		{
			++index;
		}
		insertInto(parent, index);
		setCurrentItem(item);
		return;
	}

	insertInto(nullptr, topLevelItemCount());
	setCurrentItem(item);
}

void InstructionProgramTreeWidget::dragMoveEvent(QDragMoveEvent* event)
{
	if (event->source() != this || !m_dragItem)
	{
		event->ignore();
		return;
	}
	const QPoint pos = event->pos();
	QTreeWidgetItem* target = itemAt(pos);
	const DropIndicatorPosition dropPos = dropIndicatorPosition();
	if (canAcceptDrop(m_dragItem, target, dropPos))
	{
		event->acceptProposedAction();
	}
	else
	{
		event->ignore();
	}
}

void InstructionProgramTreeWidget::dropEvent(QDropEvent* event)
{
	if (event->source() != this || !m_dragItem)
	{
		event->ignore();
		return;
	}
	const QPoint pos = event->pos();
	QTreeWidgetItem* target = itemAt(pos);
	const DropIndicatorPosition dropPos = dropIndicatorPosition();
	if (!canAcceptDrop(m_dragItem, target, dropPos))
	{
		event->ignore();
		return;
	}
	applyDrop(m_dragItem, target, dropPos);
	m_dragItem = nullptr;
	syncToProgram();
	rebuildFromProgram();
	event->acceptProposedAction();
}
