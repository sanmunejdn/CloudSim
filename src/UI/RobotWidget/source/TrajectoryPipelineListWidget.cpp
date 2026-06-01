#include "TrajectoryPipelineListWidget.h"

#include <ITrajectoryOp.h>
#include "TrajectoryOpBridge.h"

#include <QDrag>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QSignalBlocker>

#include <cmath>
#include <cstring>

namespace
{
RobotInstruction::TrajectoryOpKind kindFromInt(const int v)
{
	switch (v)
	{
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::Rotate):
		return RobotInstruction::TrajectoryOpKind::Rotate;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::Mirror):
		return RobotInstruction::TrajectoryOpKind::Mirror;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::Delete):
		return RobotInstruction::TrajectoryOpKind::Delete;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::Duplicate):
		return RobotInstruction::TrajectoryOpKind::Duplicate;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::Reorder):
		return RobotInstruction::TrajectoryOpKind::Reorder;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::RecipeWeld):
		return RobotInstruction::TrajectoryOpKind::RecipeWeld;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::RecipeGlue):
		return RobotInstruction::TrajectoryOpKind::RecipeGlue;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::RecipeGrind):
		return RobotInstruction::TrajectoryOpKind::RecipeGrind;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::Approach):
		return RobotInstruction::TrajectoryOpKind::Approach;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::Retract):
		return RobotInstruction::TrajectoryOpKind::Retract;
	case static_cast<int>(RobotInstruction::TrajectoryOpKind::Translate):
	default:
		return RobotInstruction::TrajectoryOpKind::Translate;
	}
}
} // namespace

TrajectoryPipelineListWidget::TrajectoryPipelineListWidget(QWidget* parent)
	: QListWidget(parent)
{
	setSelectionMode(QAbstractItemView::SingleSelection);
	setAcceptDrops(true);
	setDropIndicatorShown(true);
	setDragDropMode(QAbstractItemView::DropOnly);

	connect(this, &QListWidget::itemSelectionChanged, this, [this]() {
		emit selectedOpChanged(selectedOpIndex());
	});
}

void TrajectoryPipelineListWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	rebuildItems();
}

void TrajectoryPipelineListWidget::setOps(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	m_ops = ops;
	rebuildItems();
}

std::vector<RobotInstruction::TrajectoryOpDescriptor> TrajectoryPipelineListWidget::ops() const
{
	return m_ops;
}

int TrajectoryPipelineListWidget::selectedOpIndex() const
{
	const QListWidgetItem* item = currentItem();
	return item ? row(item) : -1;
}

RobotInstruction::TrajectoryOpDescriptor TrajectoryPipelineListWidget::selectedOp() const
{
	const int idx = selectedOpIndex();
	if (idx < 0 || idx >= static_cast<int>(m_ops.size()))
	{
		return {};
	}
	return m_ops[static_cast<size_t>(idx)];
}

void TrajectoryPipelineListWidget::appendOp(RobotInstruction::TrajectoryOpDescriptor op)
{
	m_ops.push_back(std::move(op));
	rebuildItems();
	setCurrentRow(count() - 1);
	emit opsChanged();
}

void TrajectoryPipelineListWidget::removeSelectedOp()
{
	const int idx = selectedOpIndex();
	if (idx < 0 || idx >= static_cast<int>(m_ops.size()))
	{
		return;
	}
	m_ops.erase(m_ops.begin() + idx);
	rebuildItems();
	emit opsChanged();
}

void TrajectoryPipelineListWidget::moveSelectedOp(const int delta)
{
	const int idx = selectedOpIndex();
	if (idx < 0)
	{
		return;
	}
	const int target = idx + delta;
	if (target < 0 || target >= static_cast<int>(m_ops.size()))
	{
		return;
	}
	std::swap(m_ops[static_cast<size_t>(idx)], m_ops[static_cast<size_t>(target)]);
	rebuildItems();
	setCurrentRow(target);
	emit opsChanged();
}

void TrajectoryPipelineListWidget::updateSelectedOp(const RobotInstruction::TrajectoryOpDescriptor& op)
{
	const int idx = selectedOpIndex();
	if (idx < 0 || idx >= static_cast<int>(m_ops.size()))
	{
		return;
	}
	m_ops[static_cast<size_t>(idx)] = op;
	// 仅改参数时不重建列表，避免 itemSelectionChanged → loadSelectedOpToParams → clearRows 重入
	if (QListWidgetItem* listItem = QListWidget::item(idx))
	{
		listItem->setText(formatOpSummary(op));
	}
}

void TrajectoryPipelineListWidget::setDefaultOpFactory(DefaultOpFactory factory)
{
	m_defaultOpFactory = std::move(factory);
}

void TrajectoryPipelineListWidget::dragEnterEvent(QDragEnterEvent* event)
{
	if (event->mimeData() && event->mimeData()->hasFormat(kMimeType))
	{
		event->acceptProposedAction();
		return;
	}
	QListWidget::dragEnterEvent(event);
}

void TrajectoryPipelineListWidget::dragMoveEvent(QDragMoveEvent* event)
{
	if (event->mimeData() && event->mimeData()->hasFormat(kMimeType))
	{
		event->acceptProposedAction();
		return;
	}
	QListWidget::dragMoveEvent(event);
}

void TrajectoryPipelineListWidget::dropEvent(QDropEvent* event)
{
	if (event->mimeData() && event->mimeData()->hasFormat(kMimeType))
	{
		RobotInstruction::TrajectoryOpDescriptor op{};
		const QByteArray raw = event->mimeData()->data(kMimeType);
		if (raw.size() >= static_cast<int>(sizeof(int)))
		{
			int kindInt = 0;
			std::memcpy(&kindInt, raw.constData(), sizeof(int));
			const RobotInstruction::TrajectoryOpKind kind = kindFromInt(kindInt);
			if (m_defaultOpFactory)
			{
				op = m_defaultOpFactory(kind);
			}
			else
			{
				op = opFromMime(event->mimeData());
			}
		}
		int insertRow = count();
		if (QListWidgetItem* target = itemAt(event->pos()))
		{
			insertRow = row(target);
		}
		m_ops.insert(m_ops.begin() + insertRow, op);
		{
			QSignalBlocker blocker(this);
			rebuildItems();
			setCurrentRow(insertRow);
		}
		emit selectedOpChanged(selectedOpIndex());
		emit opsChanged();
		event->acceptProposedAction();
		return;
	}
	event->ignore();
}

void TrajectoryPipelineListWidget::startDrag(Qt::DropActions supportedActions)
{
	(void)supportedActions;
}

void TrajectoryPipelineListWidget::rebuildItems()
{
	const int prev = selectedOpIndex();
	blockSignals(true);
	clear();
	for (const RobotInstruction::TrajectoryOpDescriptor& op : m_ops)
	{
		auto* item = new QListWidgetItem(formatOpSummary(op), this);
		item->setData(Qt::UserRole, static_cast<int>(op.kind));
	}
	blockSignals(false);
	// 外层 QSignalBlocker（如 dropEvent）期间勿恢复选中，避免 itemSelectionChanged 与手动 emit 叠加
	if (!signalsBlocked() && prev >= 0 && prev < count())
	{
		setCurrentRow(prev);
	}
}

QString TrajectoryPipelineListWidget::formatOpSummary(const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	RobotInstruction::ensureTrajectoryOpBuiltinsRegistered();
	const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(op.kind);
	if (algo)
	{
		return QString::fromStdString(algo->formatSummary(op, m_useChinese));
	}
	return m_useChinese ? QStringLiteral("未知块") : QStringLiteral("Unknown");
}

RobotInstruction::TrajectoryOpDescriptor TrajectoryPipelineListWidget::opFromMime(const QMimeData* mime) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	if (!mime || !mime->hasFormat(kMimeType))
	{
		return op;
	}
	const QByteArray raw = mime->data(kMimeType);
	if (raw.size() < static_cast<int>(sizeof(int)))
	{
		return op;
	}
	int kindInt = 0;
	std::memcpy(&kindInt, raw.constData(), sizeof(int));
	op.kind = kindFromInt(kindInt);
	return op;
}

QByteArray TrajectoryPipelineListWidget::mimeFromOp(const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	QByteArray raw;
	const int kindInt = static_cast<int>(op.kind);
	raw.resize(static_cast<int>(sizeof(int)));
	std::memcpy(raw.data(), &kindInt, sizeof(int));
	return raw;
}
