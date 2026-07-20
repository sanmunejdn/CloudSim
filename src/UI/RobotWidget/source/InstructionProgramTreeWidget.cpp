/// @file InstructionProgramTreeWidget.cpp
/// @brief InstructionProgramTreeWidget 实现

#include "InstructionProgramTreeWidget.h"

#include "RawTrajectory.h"
#include "RobotInstructionProgram.h"

#include <QContextMenuEvent>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QInputDialog>
#include <QLineEdit>
#include <QMenu>
#include <QMimeData>
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include <json.hpp>

namespace
{
constexpr int kKindRole = Qt::UserRole;
constexpr int kInstrPtrRole = Qt::UserRole + 1;
constexpr int kGroupIdRole = Qt::UserRole + 2;

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

void collectPtrMapRecursive(const std::vector<std::shared_ptr<RobotInstruction::Base>>& steps,
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

InstructionProgramTreeWidget::InstructionProgramTreeWidget(QWidget* parent) : QTreeWidget(parent)
{
	setHeaderHidden(true);
	setRootIsDecorated(true);
	setAlternatingRowColors(true);
	setMinimumHeight(120);
	setSelectionMode(QAbstractItemView::ExtendedSelection);
	setDragEnabled(true);
	setAcceptDrops(true);
	setDropIndicatorShown(true);
	setDragDropMode(QAbstractItemView::DragDrop);
	setDefaultDropAction(Qt::MoveAction);
	setContextMenuPolicy(Qt::DefaultContextMenu);

	m_selectionDebounce.setSingleShot(true);
	m_selectionDebounce.setInterval(50);
	connect(&m_selectionDebounce, &QTimer::timeout, this,
			[this]()
			{
				if (m_syncing)
				{
					return;
				}
				emit instructionSelected(selectedInstruction());
			});

	connect(this, &QTreeWidget::itemSelectionChanged, this,
			[this]()
			{
				if (m_syncing)
				{
					return;
				}
				m_selectionDebounce.start();
			});
}

void InstructionProgramTreeWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
}

void InstructionProgramTreeWidget::setProgram(std::vector<std::shared_ptr<RobotInstruction::Base>>* program)
{
	m_program = program;
}

void InstructionProgramTreeWidget::setGroupMembership(std::vector<RobotInstruction::InstructionGroup>* groups)
{
	m_groups = groups;
}

void InstructionProgramTreeWidget::setGroupVisibilityQuery(std::function<bool(const std::string& groupId)> query)
{
	m_groupVisibilityQuery = std::move(query);
}

std::string InstructionProgramTreeWidget::resolveGroupIdForContextItem(const QTreeWidgetItem* item) const
{
	if (!item)
	{
		return {};
	}
	const NodeKind kind = nodeKind(item);
	if (kind == NodeKind::Group)
	{
		return groupIdFromItem(item);
	}
	if (kind == NodeKind::PathPlanOutputRef && m_groups)
	{
		const std::string pathPlanId = item->data(0, kGroupIdRole).toString().toStdString();
		for (const RobotInstruction::InstructionGroup& group : *m_groups)
		{
			if (group.role == RobotInstruction::InstructionGroupRole::PathPlanOutput &&
				group.pathPlanInstructionId == pathPlanId)
			{
				return group.id;
			}
		}
	}
	return {};
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

std::string InstructionProgramTreeWidget::groupIdFromItem(const QTreeWidgetItem* item)
{
	if (!item || nodeKind(item) != NodeKind::Group)
	{
		return {};
	}
	return item->data(0, kGroupIdRole).toString().toStdString();
}

void InstructionProgramTreeWidget::setInstructionPtr(QTreeWidgetItem* item, RobotInstruction::Base* raw)
{
	item->setData(0, kKindRole, static_cast<int>(NodeKind::Instruction));
	item->setData(0, kInstrPtrRole, QVariant::fromValue<quintptr>(reinterpret_cast<quintptr>(raw)));
}

void InstructionProgramTreeWidget::setGroupPtr(QTreeWidgetItem* item, const std::string& groupId)
{
	item->setData(0, kKindRole, static_cast<int>(NodeKind::Group));
	item->setData(0, kGroupIdRole, QString::fromStdString(groupId));
}

bool InstructionProgramTreeWidget::isRootLevelInstructionItem(const QTreeWidgetItem* item)
{
	if (!item || nodeKind(item) != NodeKind::Instruction)
	{
		return false;
	}
	const QTreeWidgetItem* parent = item->parent();
	if (!parent)
	{
		return true;
	}
	const NodeKind pk = nodeKind(parent);
	return pk == NodeKind::Group || pk == NodeKind::PlanningSection;
}

bool InstructionProgramTreeWidget::isPathPlanInstructionItem(const QTreeWidgetItem* item)
{
	if (!item || nodeKind(item) != NodeKind::Instruction)
	{
		return false;
	}
	const RobotInstruction::Base* raw = instructionRaw(item);
	return raw && raw->type() == RobotInstruction::Type::PathPlan;
}

QTreeWidgetItem* InstructionProgramTreeWidget::findPlanningSectionItem() const
{
	for (int i = 0; i < topLevelItemCount(); ++i)
	{
		if (nodeKind(topLevelItem(i)) == NodeKind::PlanningSection)
		{
			return topLevelItem(i);
		}
	}
	return nullptr;
}

size_t InstructionProgramTreeWidget::countRootPathPlansInProgram() const
{
	if (!m_program)
	{
		return 0;
	}
	size_t n = 0;
	for (const std::shared_ptr<RobotInstruction::Base>& ins : *m_program)
	{
		if (ins && ins->type() == RobotInstruction::Type::PathPlan)
		{
			++n;
		}
	}
	return n;
}

QTreeWidgetItem* InstructionProgramTreeWidget::createPlanningSectionItem()
{
	auto* item = new QTreeWidgetItem();
	item->setData(0, kKindRole, static_cast<int>(NodeKind::PlanningSection));
	item->setText(0, m_useChinese ? QStringLiteral("路径规划") : QStringLiteral("Path planning"));
	item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDropEnabled | Qt::ItemIsSelectable);
	item->setExpanded(true);
	return item;
}

QTreeWidgetItem* InstructionProgramTreeWidget::createPathPlanOutputRefItem(
	const std::string& pathPlanId, const RobotInstruction::InstructionGroup& outputGroup, const bool chinese)
{
	auto* item = new QTreeWidgetItem();
	item->setData(0, kKindRole, static_cast<int>(NodeKind::PathPlanOutputRef));
	item->setData(0, kGroupIdRole, QString::fromStdString(pathPlanId));
	const int memberCount = static_cast<int>(outputGroup.memberInstructionIds.size());
	const QString groupName = QString::fromStdString(outputGroup.name);
	item->setText(0, chinese ? QStringLiteral("↳ 输出: %1（%2 点）").arg(groupName).arg(memberCount)
							 : QStringLiteral("↳ Output: %1 (%2 pts)").arg(groupName).arg(memberCount));
	item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	return item;
}

QString InstructionProgramTreeWidget::formatInstructionLabel(const RobotInstruction::Base& ins, const bool chinese)
{
	auto typeLabel = [&](RobotInstruction::Type t)
	{
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
		case RobotInstruction::Type::PathPlan:
			return chinese ? QStringLiteral("路径规划") : QStringLiteral("PATH_PLAN");
		case RobotInstruction::Type::PTP:
		default:
			return chinese ? QStringLiteral("点到点") : QStringLiteral("PTP");
		}
	};

	QString summary;
	if (ins.type() == RobotInstruction::Type::PathPlan)
	{
		const RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(ins);
		const QString phase = pp && pp->phase() == RobotInstruction::PathPlanPhase::Applied
								  ? (chinese ? QStringLiteral("已应用") : QStringLiteral("applied"))
								  : (pp && pp->phase() == RobotInstruction::PathPlanPhase::RawReady
										 ? (chinese ? QStringLiteral("已离散") : QStringLiteral("raw_ready"))
										 : (chinese ? QStringLiteral("草稿") : QStringLiteral("draft")));
		QString title = QString::fromStdString(ins.name());
		if (title.isEmpty())
		{
			title = chinese ? QStringLiteral("路径规划") : QStringLiteral("Path plan");
		}
		if (pp && !pp->sourceFeatureJson().empty())
		{
			RobotInstruction::RawTrajectory traj;
			traj.sourceFeatureJson = pp->sourceFeatureJson();
			const std::string featureId = RobotInstruction::rawTrajectoryFeatureId(traj);
			if (!featureId.empty())
			{
				title += QStringLiteral(" · ") + QString::fromStdString(featureId);
			}
		}
		return title + QStringLiteral(" · ") + phase;
	}

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
			summary = QString::fromStdString(c.compareLeft + " " + c.compareOp + " " + std::to_string(c.compareRight));
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

QTreeWidgetItem* InstructionProgramTreeWidget::createInstructionItem(const std::shared_ptr<RobotInstruction::Base>& ins)
{
	auto* item = new QTreeWidgetItem();
	item->setFlags(item->flags() | Qt::ItemIsDragEnabled | Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	populateInstructionItem(item, ins);
	return item;
}

QTreeWidgetItem* InstructionProgramTreeWidget::createGroupItem(const RobotInstruction::InstructionGroup& group)
{
	auto* item = new QTreeWidgetItem();
	setGroupPtr(item, group.id);
	QString label = m_useChinese ? QStringLiteral("分组: %1").arg(QString::fromStdString(group.name))
								 : QStringLiteral("Group: %1").arg(QString::fromStdString(group.name));
	if (m_groupVisibilityQuery && !m_groupVisibilityQuery(group.id))
	{
		label = (m_useChinese ? QStringLiteral("[隐藏] ") : QStringLiteral("[hidden] ")) + label;
	}
	item->setText(0, label);
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDropEnabled);
	item->setExpanded(true);
	return item;
}

void InstructionProgramTreeWidget::populateInstructionItem(QTreeWidgetItem* item,
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

QTreeWidgetItem* InstructionProgramTreeWidget::appendBranchHeader(QTreeWidgetItem* parent, const NodeKind branch,
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

	std::unordered_map<std::string, std::string> instrToGroupId;
	if (m_groups)
	{
		for (const RobotInstruction::InstructionGroup& group : *m_groups)
		{
			for (const std::string& memberId : group.memberInstructionIds)
			{
				instrToGroupId[memberId] = group.id;
			}
		}
	}

	std::vector<std::shared_ptr<RobotInstruction::Base>> rootPathPlans;
	std::vector<std::shared_ptr<RobotInstruction::Base>> motionRoots;
	rootPathPlans.reserve(m_program->size());
	motionRoots.reserve(m_program->size());
	for (const std::shared_ptr<RobotInstruction::Base>& ins : *m_program)
	{
		if (!ins)
		{
			continue;
		}
		if (ins->type() == RobotInstruction::Type::PathPlan)
		{
			rootPathPlans.push_back(ins);
		}
		else
		{
			motionRoots.push_back(ins);
		}
	}

	if (!rootPathPlans.empty())
	{
		QTreeWidgetItem* section = createPlanningSectionItem();
		for (const std::shared_ptr<RobotInstruction::Base>& ppIns : rootPathPlans)
		{
			QTreeWidgetItem* ppItem = createInstructionItem(ppIns);
			section->addChild(ppItem);
			if (m_groups)
			{
				for (const RobotInstruction::InstructionGroup& group : *m_groups)
				{
					if (group.role == RobotInstruction::InstructionGroupRole::PathPlanOutput &&
						group.pathPlanInstructionId == ppIns->id() && !group.memberInstructionIds.empty())
					{
						ppItem->addChild(createPathPlanOutputRefItem(ppIns->id(), group, m_useChinese));
						break;
					}
				}
			}
		}
		addTopLevelItem(section);
	}

	std::unordered_set<std::string> groupsRendered;
	for (const std::shared_ptr<RobotInstruction::Base>& ins : motionRoots)
	{
		if (!ins)
		{
			continue;
		}
		const auto groupIt = instrToGroupId.find(ins->id());
		if (groupIt != instrToGroupId.end())
		{
			const std::string& groupId = groupIt->second;
			if (groupsRendered.count(groupId) != 0)
			{
				continue;
			}
			const RobotInstruction::InstructionGroup* groupDef = nullptr;
			if (m_groups)
			{
				for (const RobotInstruction::InstructionGroup& group : *m_groups)
				{
					if (group.id == groupId)
					{
						groupDef = &group;
						break;
					}
				}
			}
			if (!groupDef)
			{
				addTopLevelItem(createInstructionItem(ins));
				continue;
			}
			QTreeWidgetItem* groupItem = createGroupItem(*groupDef);
			for (const std::shared_ptr<RobotInstruction::Base>& step : motionRoots)
			{
				if (!step)
				{
					continue;
				}
				const auto memberIt = instrToGroupId.find(step->id());
				if (memberIt != instrToGroupId.end() && memberIt->second == groupId)
				{
					groupItem->addChild(createInstructionItem(step));
				}
			}
			addTopLevelItem(groupItem);
			groupsRendered.insert(groupId);
		}
		else
		{
			addTopLevelItem(createInstructionItem(ins));
		}
	}
	if (!m_program || m_program->size() <= 100U)
	{
		expandAll();
	}
	m_syncing = false;
}

void InstructionProgramTreeWidget::readStepsFromChildren(
	QTreeWidgetItem* container, std::vector<std::shared_ptr<RobotInstruction::Base>>& out,
	const std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>>& ptrMap) const
{
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
		const NodeKind kind = nodeKind(item);
		if (kind == NodeKind::PlanningSection)
		{
			for (int c = 0; c < item->childCount(); ++c)
			{
				QTreeWidgetItem* child = item->child(c);
				if (nodeKind(child) != NodeKind::Instruction)
				{
					continue;
				}
				RobotInstruction::Base* raw = instructionRaw(child);
				const auto it = ptrMap.find(raw);
				if (it != ptrMap.end() && it->second)
				{
					root.push_back(it->second);
				}
			}
			continue;
		}
		if (kind == NodeKind::PathPlanOutputRef)
		{
			continue;
		}
		if (kind == NodeKind::Group)
		{
			std::vector<std::shared_ptr<RobotInstruction::Base>> groupSteps;
			readStepsFromChildren(item, groupSteps, ptrMap);
			for (const std::shared_ptr<RobotInstruction::Base>& ins : groupSteps)
			{
				if (ins)
				{
					root.push_back(ins);
				}
			}
			continue;
		}
		if (kind != NodeKind::Instruction)
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

void InstructionProgramTreeWidget::syncGroupsFromTree()
{
	if (!m_groups)
	{
		return;
	}
	std::unordered_map<std::string, std::vector<std::string>> membership;
	for (int i = 0; i < topLevelItemCount(); ++i)
	{
		QTreeWidgetItem* item = topLevelItem(i);
		if (nodeKind(item) != NodeKind::Group)
		{
			continue;
		}
		const std::string groupId = groupIdFromItem(item);
		if (groupId.empty())
		{
			continue;
		}
		std::vector<std::string>& members = membership[groupId];
		for (int c = 0; c < item->childCount(); ++c)
		{
			QTreeWidgetItem* ch = item->child(c);
			if (RobotInstruction::Base* raw = instructionRaw(ch))
			{
				members.push_back(raw->id());
			}
		}
	}
	for (RobotInstruction::InstructionGroup& group : *m_groups)
	{
		const auto it = membership.find(group.id);
		if (it != membership.end())
		{
			group.memberInstructionIds = it->second;
		}
		else
		{
			group.memberInstructionIds.clear();
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
	syncGroupsFromTree();
	emit programStructureChanged();
	emit groupMembershipChanged();
}

std::shared_ptr<RobotInstruction::Base> InstructionProgramTreeWidget::selectedInstruction() const
{
	QTreeWidgetItem* item = currentItem();
	if (!item)
	{
		return nullptr;
	}
	if (nodeKind(item) == NodeKind::PathPlanOutputRef)
	{
		if (QTreeWidgetItem* parent = item->parent())
		{
			item = parent;
		}
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

std::vector<std::shared_ptr<RobotInstruction::Base>> InstructionProgramTreeWidget::selectedMotionInstructions() const
{
	std::vector<std::shared_ptr<RobotInstruction::Base>> out;
	if (!m_program)
	{
		return out;
	}
	std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>> ptrMap;
	collectPtrMapRecursive(*m_program, ptrMap);
	const QList<QTreeWidgetItem*> items = selectedItems();
	out.reserve(static_cast<size_t>(items.size()));
	for (QTreeWidgetItem* item : items)
	{
		RobotInstruction::Base* raw = instructionRaw(item);
		if (!raw || !RobotInstruction::isMotionWaypointType(raw->type()))
		{
			continue;
		}
		const auto it = ptrMap.find(raw);
		if (it != ptrMap.end() && it->second)
		{
			out.push_back(it->second);
		}
	}
	return out;
}

std::vector<std::shared_ptr<RobotInstruction::Base>> InstructionProgramTreeWidget::selectedRootInstructions() const
{
	std::vector<std::shared_ptr<RobotInstruction::Base>> out;
	if (!m_program)
	{
		return out;
	}
	std::unordered_map<RobotInstruction::Base*, std::shared_ptr<RobotInstruction::Base>> ptrMap;
	collectPtrMapRecursive(*m_program, ptrMap);
	const QList<QTreeWidgetItem*> items = selectedItems();
	out.reserve(static_cast<size_t>(items.size()));
	for (QTreeWidgetItem* item : items)
	{
		if (!isRootLevelInstructionItem(item))
		{
			continue;
		}
		RobotInstruction::Base* raw = instructionRaw(item);
		if (!raw)
		{
			continue;
		}
		const auto it = ptrMap.find(raw);
		if (it != ptrMap.end() && it->second)
		{
			out.push_back(it->second);
		}
	}
	return out;
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
	const auto walk = [&](auto&& self, QTreeWidgetItem* node) -> QTreeWidgetItem*
	{
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

void InstructionProgramTreeWidget::insertInstruction(const std::shared_ptr<RobotInstruction::Base>& ins,
													 const bool emitSelection)
{
	if (!ins || !m_program)
	{
		return;
	}
	syncToProgram();

	if (ins->type() == RobotInstruction::Type::PathPlan)
	{
		const size_t idx = countRootPathPlansInProgram();
		m_program->insert(m_program->begin() + static_cast<std::ptrdiff_t>(idx), ins);
		rebuildFromProgram();
		m_syncing = true;
		selectInstructionByRaw(ins.get());
		m_syncing = false;
		if (emitSelection)
		{
			emit instructionSelected(ins);
		}
		return;
	}

	QTreeWidgetItem* sel = currentItem();
	if (!sel)
	{
		m_program->push_back(ins);
	}
	else
	{
		const NodeKind k = nodeKind(sel);
		if (k == NodeKind::Group)
		{
			m_program->push_back(ins);
			if (m_groups)
			{
				const std::string groupId = groupIdFromItem(sel);
				for (RobotInstruction::InstructionGroup& group : *m_groups)
				{
					if (group.id == groupId)
					{
						group.memberInstructionIds.push_back(ins->id());
						break;
					}
				}
			}
		}
		else if (k == NodeKind::ThenBranch || k == NodeKind::ElseBranch)
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
				if (pk == NodeKind::Group)
				{
					m_program->push_back(ins);
					const std::string groupId = groupIdFromItem(branch);
					if (m_groups)
					{
						for (RobotInstruction::InstructionGroup& group : *m_groups)
						{
							if (group.id == groupId)
							{
								group.memberInstructionIds.push_back(ins->id());
								break;
							}
						}
					}
				}
				else if (pk == NodeKind::ThenBranch || pk == NodeKind::ElseBranch)
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
	if (!sel)
	{
		return;
	}
	if (nodeKind(sel) == NodeKind::PathPlanOutputRef && sel->parent())
	{
		sel = sel->parent();
	}
	if (nodeKind(sel) != NodeKind::Instruction)
	{
		return;
	}
	RobotInstruction::Base* raw = instructionRaw(sel);
	syncToProgram();

	std::function<bool(std::vector<std::shared_ptr<RobotInstruction::Base>>&)> removeRecursive;
	removeRecursive = [&](std::vector<std::shared_ptr<RobotInstruction::Base>>& steps) -> bool
	{
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
	if (m_groups && raw)
	{
		for (RobotInstruction::InstructionGroup& group : *m_groups)
		{
			group.memberInstructionIds.erase(
				std::remove(group.memberInstructionIds.begin(), group.memberInstructionIds.end(), raw->id()),
				group.memberInstructionIds.end());
		}
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
	if (m_groups)
	{
		for (RobotInstruction::InstructionGroup& group : *m_groups)
		{
			group.memberInstructionIds.clear();
		}
	}
	emit programStructureChanged();
	emit groupMembershipChanged();
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
	const RobotInstruction::Base* raw = instructionRaw(m_dragItem);
	if (raw && raw->type() == RobotInstruction::Type::PathPlan)
	{
		const QTreeWidgetItem* parent = m_dragItem->parent();
		if (!parent || nodeKind(parent) != NodeKind::PlanningSection)
		{
			m_dragItem = nullptr;
			return;
		}
	}
	else if (!isRootLevelInstructionItem(m_dragItem))
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

bool InstructionProgramTreeWidget::canAcceptDrop(QTreeWidgetItem* dragged, QTreeWidgetItem* target,
												 const DropIndicatorPosition pos) const
{
	if (!dragged || nodeKind(dragged) != NodeKind::Instruction)
	{
		return false;
	}
	const RobotInstruction::Base* dragRaw = instructionRaw(dragged);
	const bool dragIsPathPlan = dragRaw && dragRaw->type() == RobotInstruction::Type::PathPlan;
	if (dragIsPathPlan)
	{
		const QTreeWidgetItem* dragParent = dragged->parent();
		if (!dragParent || nodeKind(dragParent) != NodeKind::PlanningSection)
		{
			return false;
		}
		if (!target)
		{
			return false;
		}
		if (nodeKind(target) == NodeKind::PlanningSection)
		{
			return pos == OnItem;
		}
		if (nodeKind(target) == NodeKind::PathPlanOutputRef)
		{
			target = target->parent();
		}
		if (isPathPlanInstructionItem(target))
		{
			const QTreeWidgetItem* targetParent = target->parent();
			return targetParent && nodeKind(targetParent) == NodeKind::PlanningSection &&
				   (pos == BelowItem || pos == AboveItem);
		}
		return false;
	}
	if (!isRootLevelInstructionItem(dragged))
	{
		return false;
	}
	if (dragged == target)
	{
		return false;
	}
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

	for (QTreeWidgetItem* p = target; p; p = p->parent())
	{
		if (nodeKind(p) == NodeKind::PlanningSection)
		{
			return false;
		}
	}

	const NodeKind tk = nodeKind(target);
	if (tk == NodeKind::PlanningSection || tk == NodeKind::PathPlanOutputRef)
	{
		return false;
	}
	if (tk == NodeKind::Group)
	{
		return pos == OnItem || pos == BelowItem || pos == AboveItem;
	}
	if (tk == NodeKind::ThenBranch || tk == NodeKind::ElseBranch)
	{
		return false;
	}
	if (tk == NodeKind::Instruction)
	{
		if (QTreeWidgetItem* parent = target->parent())
		{
			if (nodeKind(parent) == NodeKind::Group)
			{
				return pos == BelowItem || pos == AboveItem;
			}
		}
		RobotInstruction::Base* raw = instructionRaw(target);
		if (raw && (raw->type() == RobotInstruction::Type::IF || raw->type() == RobotInstruction::Type::WHILE))
		{
			if (pos == OnItem)
			{
				return false;
			}
		}
		return pos == BelowItem || pos == AboveItem;
	}
	return false;
}

void InstructionProgramTreeWidget::applyDrop(QTreeWidgetItem* dragged, QTreeWidgetItem* target,
											 const DropIndicatorPosition pos)
{
	QTreeWidgetItem* item = takeTreeItem(dragged);
	if (!item)
	{
		return;
	}

	auto insertInto = [&](QTreeWidgetItem* parent, int index)
	{
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

	const RobotInstruction::Base* dragRawApply = instructionRaw(item);
	const bool dragPathPlanApply = dragRawApply && dragRawApply->type() == RobotInstruction::Type::PathPlan;

	if (!target || pos == OnViewport)
	{
		if (dragPathPlanApply)
		{
			delete item;
			return;
		}
		insertInto(nullptr, topLevelItemCount());
		setCurrentItem(item);
		return;
	}

	if (dragPathPlanApply)
	{
		QTreeWidgetItem* section = findPlanningSectionItem();
		if (!section)
		{
			section = createPlanningSectionItem();
			insertTopLevelItem(0, section);
		}
		if (nodeKind(target) == NodeKind::PlanningSection && pos == OnItem)
		{
			section->addChild(item);
			section->setExpanded(true);
		}
		else
		{
			QTreeWidgetItem* refTarget = target;
			if (nodeKind(refTarget) == NodeKind::PathPlanOutputRef && refTarget->parent())
			{
				refTarget = refTarget->parent();
			}
			int index = section->indexOfChild(refTarget);
			if (index < 0)
			{
				index = section->childCount();
			}
			if (pos == BelowItem)
			{
				++index;
			}
			section->insertChild(index, item);
			section->setExpanded(true);
		}
		setCurrentItem(item);
		return;
	}

	if (const NodeKind tk = nodeKind(target); tk == NodeKind::Group)
	{
		if (pos == OnItem)
		{
			target->addChild(item);
			target->setExpanded(true);
		}
		else
		{
			QTreeWidgetItem* parent = target->parent();
			int index = parent ? parent->indexOfChild(target) : indexOfTopLevelItem(target);
			if (pos == BelowItem)
			{
				++index;
			}
			insertInto(parent, index);
		}
		setCurrentItem(item);
		return;
	}

	if (nodeKind(target) == NodeKind::Instruction)
	{
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

void InstructionProgramTreeWidget::contextMenuEvent(QContextMenuEvent* event)
{
	showContextMenu(event->globalPos());
	event->accept();
}

void InstructionProgramTreeWidget::showContextMenu(const QPoint& globalPos)
{
	QTreeWidgetItem* item = itemAt(viewport()->mapFromGlobal(globalPos));
	QMenu menu(this);

	const std::string groupId = resolveGroupIdForContextItem(item);
	if (!groupId.empty())
	{
		const bool visible = !m_groupVisibilityQuery || m_groupVisibilityQuery(groupId);
		const QString hideText = m_useChinese ? QStringLiteral("隐藏分组") : QStringLiteral("Hide group");
		const QString showText = m_useChinese ? QStringLiteral("显示分组") : QStringLiteral("Show group");
		if (visible)
		{
			menu.addAction(hideText, this, [this, groupId]() { emit groupVisibilityChangeRequested(groupId, false); });
		}
		else
		{
			menu.addAction(showText, this, [this, groupId]() { emit groupVisibilityChangeRequested(groupId, true); });
		}
	}

	if (item && nodeKind(item) == NodeKind::Group)
	{
		const QString renameText = m_useChinese ? QStringLiteral("重命名分组…") : QStringLiteral("Rename group…");
		const QString dissolveText = m_useChinese ? QStringLiteral("解散分组") : QStringLiteral("Dissolve group");
		menu.addAction(renameText, this,
					   [this, groupId]()
					   {
						   bool ok = false;
						   const QString title =
							   m_useChinese ? QStringLiteral("重命名分组") : QStringLiteral("Rename group");
						   const QString label = m_useChinese ? QStringLiteral("名称") : QStringLiteral("Name");
						   const QString newName =
							   QInputDialog::getText(this, title, label, QLineEdit::Normal, QString(), &ok);
						   if (ok && !newName.isEmpty())
						   {
							   emit renameGroupRequested(groupId, newName);
						   }
					   });
		menu.addAction(dissolveText, this, [this, groupId]() { emit dissolveGroupRequested(groupId); });
	}
	else
	{
		const std::vector<std::shared_ptr<RobotInstruction::Base>> selected = selectedRootInstructions();
		if (selected.size() >= 2)
		{
			const QString createText = m_useChinese ? QStringLiteral("创建分组…") : QStringLiteral("Create group…");
			menu.addAction(
				createText, this,
				[this, selected]()
				{
					bool ok = false;
					const QString title = m_useChinese ? QStringLiteral("创建分组") : QStringLiteral("Create group");
					const QString label = m_useChinese ? QStringLiteral("名称") : QStringLiteral("Name");
					const QString defaultName = m_useChinese ? QStringLiteral("分组") : QStringLiteral("Group");
					const QString name = QInputDialog::getText(this, title, label, QLineEdit::Normal, defaultName, &ok);
					if (!ok || name.isEmpty())
					{
						return;
					}
					std::vector<std::string> memberIds;
					memberIds.reserve(selected.size());
					for (const std::shared_ptr<RobotInstruction::Base>& ins : selected)
					{
						if (ins)
						{
							memberIds.push_back(ins->id());
						}
					}
					emit createGroupRequested(name, memberIds);
				});
		}
	}

	if (!menu.isEmpty())
	{
		menu.exec(globalPos);
	}
}
