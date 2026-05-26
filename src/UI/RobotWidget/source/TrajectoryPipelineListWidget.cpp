#include "TrajectoryPipelineListWidget.h"

#include <QDrag>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>

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
	rebuildItems();
	setCurrentRow(idx);
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
		rebuildItems();
		setCurrentRow(insertRow);
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
	if (prev >= 0 && prev < count())
	{
		setCurrentRow(prev);
	}
}

QString TrajectoryPipelineListWidget::formatOpSummary(const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	QString kindLabel;
	switch (op.kind)
	{
	case RobotInstruction::TrajectoryOpKind::Rotate:
		kindLabel = m_useChinese ? QStringLiteral("旋转") : QStringLiteral("Rotate");
		break;
	case RobotInstruction::TrajectoryOpKind::Mirror:
		kindLabel = m_useChinese ? QStringLiteral("镜像") : QStringLiteral("Mirror");
		break;
	case RobotInstruction::TrajectoryOpKind::Delete:
		kindLabel = m_useChinese ? QStringLiteral("删除") : QStringLiteral("Delete");
		break;
	case RobotInstruction::TrajectoryOpKind::Duplicate:
		kindLabel = m_useChinese ? QStringLiteral("复制") : QStringLiteral("Duplicate");
		break;
	case RobotInstruction::TrajectoryOpKind::Reorder:
		kindLabel = m_useChinese ? QStringLiteral("移动顺序") : QStringLiteral("Reorder");
		break;
	case RobotInstruction::TrajectoryOpKind::Translate:
	default:
		kindLabel = m_useChinese ? QStringLiteral("平移") : QStringLiteral("Translate");
		break;
	}
	QString scopeLabel;
	switch (op.scope.kind)
	{
	case RobotInstruction::OpScope::Kind::EntireProgram:
		scopeLabel = m_useChinese ? QStringLiteral("全程序") : QStringLiteral("Entire");
		break;
	case RobotInstruction::OpScope::Kind::PointIndexRange:
		scopeLabel = QStringLiteral("P%1-P%2").arg(op.scope.pointFrom).arg(op.scope.pointTo);
		break;
	case RobotInstruction::OpScope::Kind::InstructionIds:
		scopeLabel = m_useChinese ? QStringLiteral("指定路点") : QStringLiteral("Ids");
		break;
	case RobotInstruction::OpScope::Kind::Group:
	default:
		if (!op.scope.groupId.empty())
		{
			scopeLabel = QString::fromStdString(op.scope.groupId);
		}
		else
		{
			scopeLabel = m_useChinese ? QStringLiteral("分组") : QStringLiteral("Group");
		}
		break;
	}
	QString paramLabel;
	switch (op.kind)
	{
	case RobotInstruction::TrajectoryOpKind::Translate:
		paramLabel = QStringLiteral("Δ(%1,%2,%3)")
			.arg(op.translate.dxMm, 0, 'f', 1)
			.arg(op.translate.dyMm, 0, 'f', 1)
			.arg(op.translate.dzMm, 0, 'f', 1);
		break;
	case RobotInstruction::TrajectoryOpKind::Rotate:
	{
		const char axis = std::abs(op.rotate.axisZ) >= std::abs(op.rotate.axisX)
			&& std::abs(op.rotate.axisZ) >= std::abs(op.rotate.axisY)
			? 'Z'
			: (std::abs(op.rotate.axisY) >= std::abs(op.rotate.axisX) ? 'Y' : 'X');
		paramLabel = QStringLiteral("%1°@%2").arg(op.rotate.angleDeg, 0, 'f', 1).arg(axis);
		break;
	}
	default:
		break;
	}
	if (paramLabel.isEmpty())
	{
		return kindLabel + QStringLiteral(" | ") + scopeLabel;
	}
	return kindLabel + QStringLiteral(" | ") + scopeLabel + QStringLiteral(" | ") + paramLabel;
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
