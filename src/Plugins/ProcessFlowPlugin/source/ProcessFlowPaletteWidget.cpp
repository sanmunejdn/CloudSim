/// @file ProcessFlowPaletteWidget.cpp
/// @brief 左侧节点库；拖出 MIME / 双击添加

#include "ProcessFlowPaletteWidget.h"

#include "ProcessFlowNodeProps.h"
#include "ProcessFlowPropertyPanel.h"
#include "ProcessFlowUiStyle.h"

#include <QAbstractItemView>
#include <QColor>
#include <QDrag>
#include <QFrame>
#include <QIcon>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMimeData>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QSize>
#include <QSplitter>
#include <QVBoxLayout>

namespace
{
struct NodeTypeDef
{
	const char* kind;
	const char* titleZh;
	const char* titleEn;
	const char* subtitleZh;
	const char* subtitleEn;
	const char* color;
};

const NodeTypeDef kTypes[] = {
	{"start", "开始", "Start", "流程入口", "Entry", "#2E7DD1"},
	{"station", "工位", "Station", "加工工位", "Work cell", "#1F9D63"},
	{"buffer", "缓冲", "Buffer", "物料缓冲", "Buffer", "#D87516"},
	{"warehouse", "仓库", "Warehouse", "大容量存贮", "Store", "#B45309"},
	{"conveyor", "输送", "Conveyor", "运输延时", "Transport", "#0EA5E9"},
	{"assembly", "装配", "Assembly", "汇合加工", "Join", "#DB2777"},
	{"inspect", "检测", "Inspect", "质量检测", "Inspection", "#7A5CFA"},
	{"end", "结束", "End", "流程出口", "Exit", "#E9573F"},
};

QIcon colorDotIcon(const QColor& color)
{
	QPixmap pm(18, 18);
	pm.fill(Qt::transparent);
	QPainter p(&pm);
	p.setRenderHint(QPainter::Antialiasing, true);
	p.setPen(QPen(color.darker(115), 1.0));
	p.setBrush(color);
	p.drawEllipse(QRectF(2.5, 2.5, 13.0, 13.0));
	return QIcon(pm);
}

class PaletteListWidget final : public QListWidget
{
public:
	explicit PaletteListWidget(QWidget* parent = nullptr) : QListWidget(parent)
	{
		setDragEnabled(true);
		setDefaultDropAction(Qt::CopyAction);
		setSelectionMode(QAbstractItemView::SingleSelection);
		setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
		setSpacing(0);
		setIconSize(QSize(18, 18));
	}

protected:
	void startDrag(Qt::DropActions) override
	{
		QListWidgetItem* item = currentItem();
		if (!item)
		{
			return;
		}
		QJsonObject o;
		o.insert(QStringLiteral("kind"), item->data(Qt::UserRole).toString());
		o.insert(QStringLiteral("title"), item->data(Qt::UserRole + 1).toString());
		o.insert(QStringLiteral("subtitle"), item->data(Qt::UserRole + 2).toString());
		o.insert(QStringLiteral("color"), item->data(Qt::UserRole + 3).toString());
		auto* mime = new QMimeData;
		mime->setData(QString::fromLatin1(processFlowNodeMimeType()),
					  QJsonDocument(o).toJson(QJsonDocument::Compact));
		auto* drag = new QDrag(this);
		drag->setMimeData(mime);
		drag->exec(Qt::CopyAction);
	}
};

QFrame* makeCard(QWidget* parent)
{
	auto* card = new QFrame(parent);
	card->setObjectName(QStringLiteral("ProcessFlowCard"));
	card->setFrameShape(QFrame::NoFrame);
	return card;
}
} // namespace

ProcessFlowPaletteWidget::ProcessFlowPaletteWidget(QWidget* parent) : QWidget(parent)
{
	setObjectName(QStringLiteral("ProcessFlowSideRoot"));
	setStyleSheet(processFlowSideChromeStyle());

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(10, 10, 10, 10);
	layout->setSpacing(10);

	m_title = new QLabel(QStringLiteral("节点库"), this);
	m_title->setObjectName(QStringLiteral("ProcessFlowSectionTitle"));

	m_hint = new QLabel(QStringLiteral("拖到画布或双击添加"), this);
	m_hint->setObjectName(QStringLiteral("ProcessFlowHint"));

	m_list = new PaletteListWidget(this);
	m_list->setObjectName(QStringLiteral("ProcessFlowNodeList"));
	for (const NodeTypeDef& type : kTypes)
	{
		const QString titleZh = QString::fromUtf8(type.titleZh);
		const QString subZh = QString::fromUtf8(type.subtitleZh);
		auto* item = new QListWidgetItem(m_list);
		item->setIcon(colorDotIcon(QColor(QString::fromUtf8(type.color))));
		item->setText(titleZh + QStringLiteral("  ·  ") + subZh);
		item->setToolTip(subZh);
		item->setSizeHint(QSize(0, 36));
		item->setData(Qt::UserRole, QString::fromUtf8(type.kind));
		item->setData(Qt::UserRole + 1, titleZh);
		item->setData(Qt::UserRole + 2, subZh);
		item->setData(Qt::UserRole + 3, QString::fromUtf8(type.color));
		item->setData(Qt::UserRole + 4, QString::fromUtf8(type.titleEn));
		item->setData(Qt::UserRole + 5, QString::fromUtf8(type.subtitleEn));
	}
	if (m_list->count() > 0)
	{
		m_list->setCurrentRow(0);
	}
	connect(m_list, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*) { emitSelectedType(); });

	m_propertyPanel = new ProcessFlowPropertyPanel(this);

	auto* libraryCard = makeCard(this);
	auto* libraryLayout = new QVBoxLayout(libraryCard);
	libraryLayout->setContentsMargins(12, 12, 12, 10);
	libraryLayout->setSpacing(6);
	libraryLayout->addWidget(m_title);
	libraryLayout->addWidget(m_hint);
	libraryLayout->addWidget(m_list, 1);

	auto* propsCard = makeCard(this);
	auto* propsLayout = new QVBoxLayout(propsCard);
	propsLayout->setContentsMargins(12, 10, 12, 12);
	propsLayout->setSpacing(0);
	propsLayout->addWidget(m_propertyPanel);

	auto* splitter = new QSplitter(Qt::Vertical, this);
	splitter->setChildrenCollapsible(false);
	splitter->addWidget(libraryCard);
	splitter->addWidget(propsCard);
	splitter->setStretchFactor(0, 2);
	splitter->setStretchFactor(1, 3);
	layout->addWidget(splitter);
}

void ProcessFlowPaletteWidget::applyLanguage(bool useChinese)
{
	if (m_title)
	{
		m_title->setText(useChinese ? QStringLiteral("节点库") : QStringLiteral("Node Library"));
	}
	if (m_hint)
	{
		m_hint->setText(useChinese ? QStringLiteral("拖到画布或双击添加")
								   : QStringLiteral("Drag to canvas or double-click"));
	}
	for (int i = 0; i < m_list->count() && i < static_cast<int>(sizeof(kTypes) / sizeof(kTypes[0])); ++i)
	{
		QListWidgetItem* item = m_list->item(i);
		const NodeTypeDef& type = kTypes[i];
		const QString title = useChinese ? QString::fromUtf8(type.titleZh) : QString::fromUtf8(type.titleEn);
		const QString sub = useChinese ? QString::fromUtf8(type.subtitleZh) : QString::fromUtf8(type.subtitleEn);
		item->setText(title + QStringLiteral("  ·  ") + sub);
		item->setToolTip(sub);
		item->setData(Qt::UserRole + 1, title);
		item->setData(Qt::UserRole + 2, sub);
	}
	if (m_propertyPanel)
	{
		m_propertyPanel->applyLanguage(useChinese);
	}
}

void ProcessFlowPaletteWidget::emitSelectedType()
{
	QListWidgetItem* item = m_list ? m_list->currentItem() : nullptr;
	if (!item)
	{
		return;
	}
	emit addNodeRequested(item->data(Qt::UserRole).toString(), item->data(Qt::UserRole + 1).toString(),
						  item->data(Qt::UserRole + 2).toString(), QColor(item->data(Qt::UserRole + 3).toString()));
}
