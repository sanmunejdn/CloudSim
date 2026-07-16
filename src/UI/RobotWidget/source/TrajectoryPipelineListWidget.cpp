#include "TrajectoryPipelineListWidget.h"

#include <ITrajectoryOp.h>
#include "TrajectoryOpBridge.h"

#include <QCheckBox>
#include <QDrag>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QMimeData>
#include <QSignalBlocker>
#include <QSizePolicy>

#include <cmath>
#include <cstring>

namespace
{
RobotInstruction::TrajectoryOpKind kindFromInt(const int v)
{
	const int maxKind = static_cast<int>(RobotInstruction::TrajectoryOpKind::NonRigidRegistration);
	if (v < 0 || v > maxKind)
	{
		return RobotInstruction::TrajectoryOpKind::Translate;
	}
	return static_cast<RobotInstruction::TrajectoryOpKind>(v);
}
} // namespace

TrajectoryPipelineListWidget::TrajectoryPipelineListWidget(QWidget* parent)
	: QListWidget(parent)
{
	setSelectionMode(QAbstractItemView::SingleSelection);
	setAcceptDrops(true);
	setDropIndicatorShown(true);
	setDragDropMode(QAbstractItemView::DragDrop);
	setDefaultDropAction(Qt::MoveAction);

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
	return opAt(selectedOpIndex());
}

RobotInstruction::TrajectoryOpDescriptor TrajectoryPipelineListWidget::opAt(const int index) const
{
	if (index < 0 || index >= static_cast<int>(m_ops.size()))
	{
		return {};
	}
	return m_ops[static_cast<size_t>(index)];
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
	updateOpAt(selectedOpIndex(), op);
}

void TrajectoryPipelineListWidget::updateOpAt(
	const int index,
	const RobotInstruction::TrajectoryOpDescriptor& op)
{
	if (index < 0 || index >= static_cast<int>(m_ops.size()))
	{
		return;
	}
	m_ops[static_cast<size_t>(index)] = op;
	// 仅刷新行控件，避免 itemSelectionChanged → loadSelectedOpToParams 重入
	refreshRowWidget(index);
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
		const QByteArray raw = event->mimeData()->data(kMimeType);
		if (raw.size() >= static_cast<int>(sizeof(int) * 2))
		{
			int srcRow = -1;
			std::memcpy(&srcRow, raw.constData() + sizeof(int), sizeof(int));
			if (srcRow >= 0 && srcRow < static_cast<int>(m_ops.size()))
			{
				int insertRow = count();
				if (QListWidgetItem* target = itemAt(event->pos()))
				{
					insertRow = row(target);
				}
				if (insertRow > srcRow)
				{
					--insertRow;
				}
				if (insertRow != srcRow)
				{
					RobotInstruction::TrajectoryOpDescriptor moved = m_ops[static_cast<size_t>(srcRow)];
					m_ops.erase(m_ops.begin() + srcRow);
					const int clamped = std::max(0, std::min(insertRow, static_cast<int>(m_ops.size())));
					m_ops.insert(m_ops.begin() + clamped, moved);
					{
						QSignalBlocker blocker(this);
						rebuildItems();
						setCurrentRow(clamped);
					}
					emit selectedOpChanged(selectedOpIndex());
					emit opsChanged();
					event->acceptProposedAction();
					return;
				}
			}
		}
		RobotInstruction::TrajectoryOpDescriptor op{};
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
	const int idx = selectedOpIndex();
	if (idx < 0 || idx >= static_cast<int>(m_ops.size()))
	{
		return;
	}
	auto* mime = new QMimeData();
	QByteArray payload = mimeFromOp(m_ops[static_cast<size_t>(idx)]);
	const int srcRow = idx;
	payload.append(reinterpret_cast<const char*>(&srcRow), sizeof(int));
	mime->setData(kMimeType, payload);
	auto* drag = new QDrag(this);
	drag->setMimeData(mime);
	drag->exec(supportedActions, Qt::MoveAction);
}

void TrajectoryPipelineListWidget::rebuildItems()
{
	const int prev = selectedOpIndex();
	blockSignals(true);
	clear();
	for (int i = 0; i < static_cast<int>(m_ops.size()); ++i)
	{
		auto* item = new QListWidgetItem(this);
		item->setData(Qt::UserRole, static_cast<int>(m_ops[static_cast<size_t>(i)].kind));
		QWidget* rowWidget = createRowWidget(i, m_ops[static_cast<size_t>(i)]);
		item->setSizeHint(rowWidget->sizeHint());
		setItemWidget(item, rowWidget);
	}
	blockSignals(false);
	// 外层 QSignalBlocker（如 dropEvent）期间勿恢复选中，避免 itemSelectionChanged 与手动 emit 叠加
	if (!signalsBlocked() && prev >= 0 && prev < count())
	{
		setCurrentRow(prev);
	}
}

void TrajectoryPipelineListWidget::refreshRowWidget(const int index)
{
	if (index < 0 || index >= static_cast<int>(m_ops.size()) || index >= count())
	{
		return;
	}
	QListWidgetItem* listItem = QListWidget::item(index);
	if (!listItem)
	{
		return;
	}
	QWidget* rowWidget = createRowWidget(index, m_ops[static_cast<size_t>(index)]);
	listItem->setSizeHint(rowWidget->sizeHint());
	setItemWidget(listItem, rowWidget);
}

QWidget* TrajectoryPipelineListWidget::createRowWidget(
	const int index,
	const RobotInstruction::TrajectoryOpDescriptor& op)
{
	auto* row = new QWidget(this);
	auto* layout = new QHBoxLayout(row);
	layout->setContentsMargins(4, 1, 4, 1);
	layout->setSpacing(6);

	auto* summary = new QLabel(formatOpSummary(op), row);
	summary->setObjectName(QStringLiteral("opSummary"));
	summary->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
	if (!op.enabled)
	{
		summary->setStyleSheet(QStringLiteral("color: palette(mid);"));
	}
	layout->addWidget(summary, 1);

	auto* enable = new QCheckBox(
		m_useChinese ? QStringLiteral("启用") : QStringLiteral("On"),
		row);
	enable->setObjectName(QStringLiteral("opEnabled"));
	enable->setProperty("opIndex", index);
	{
		const QSignalBlocker blocker(enable);
		enable->setChecked(op.enabled);
	}
	connect(enable, &QCheckBox::toggled, this, &TrajectoryPipelineListWidget::onEnableCheckToggled);
	layout->addWidget(enable, 0);

	row->setMinimumHeight(26);
	return row;
}

void TrajectoryPipelineListWidget::onEnableCheckToggled(const bool checked)
{
	auto* box = qobject_cast<QCheckBox*>(sender());
	if (!box)
	{
		return;
	}
	const int index = box->property("opIndex").toInt();
	if (index < 0 || index >= static_cast<int>(m_ops.size()))
	{
		return;
	}
	if (m_ops[static_cast<size_t>(index)].enabled == checked)
	{
		return;
	}
	m_ops[static_cast<size_t>(index)].enabled = checked;
	refreshRowWidget(index);
	emit opsChanged();
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
