/// @file DrawingSidePanel.cpp
/// @brief 工程图左侧：模型 + 视角缩略图拖放

#include "DrawingSidePanel.h"

#include <QAbstractItemView>
#include <QDrag>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QListWidget>
#include <QMimeData>
#include <QPainter>
#include <QVBoxLayout>

namespace
{
class ViewPaletteList final : public QListWidget
{
public:
	explicit ViewPaletteList(QWidget* parent = nullptr) : QListWidget(parent)
	{
		setViewMode(QListView::IconMode);
		setIconSize(QSize(148, 108));
		setGridSize(QSize(158, 140));
		setResizeMode(QListView::Adjust);
		setMovement(QListView::Static);
		setSpacing(6);
		setDragEnabled(true);
		setDefaultDropAction(Qt::CopyAction);
		setSelectionMode(QAbstractItemView::SingleSelection);
		setWordWrap(true);
	}

protected:
	void startDrag(Qt::DropActions) override
	{
		QListWidgetItem* item = currentItem();
		if (!item || !item->data(Qt::UserRole + 1).toBool())
			return;
		QJsonObject o;
		o.insert(QStringLiteral("kind"), item->data(Qt::UserRole).toString());
		o.insert(QStringLiteral("title"), item->text());
		auto* mime = new QMimeData;
		mime->setData(QString::fromLatin1(drawingViewMimeType()),
					  QJsonDocument(o).toJson(QJsonDocument::Compact));
		auto* drag = new QDrag(this);
		drag->setMimeData(mime);
		const QPixmap pm = item->icon().pixmap(iconSize());
		if (!pm.isNull())
			drag->setPixmap(pm.scaled(96, 72, Qt::KeepAspectRatio, Qt::SmoothTransformation));
		drag->exec(Qt::CopyAction);
	}
};

QRectF polyBounds(const QVector<DrawingSheetCanvasWidget::Polyline2d>& visible,
				  const QVector<DrawingSheetCanvasWidget::Polyline2d>& hidden)
{
	bool any = false;
	double minX = 0, minY = 0, maxX = 0, maxY = 0;
	auto acc = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys) {
		for (const auto& poly : polys)
		{
			for (const QPointF& p : poly.points)
			{
				if (!any)
				{
					minX = maxX = p.x();
					minY = maxY = p.y();
					any = true;
				}
				else
				{
					minX = qMin(minX, p.x());
					minY = qMin(minY, p.y());
					maxX = qMax(maxX, p.x());
					maxY = qMax(maxY, p.y());
				}
			}
		}
	};
	acc(visible);
	acc(hidden);
	if (!any)
		return QRectF(0, 0, 10, 10);
	return QRectF(minX, minY, qMax(1.0, maxX - minX), qMax(1.0, maxY - minY)).adjusted(-2, -2, 2, 2);
}
} // namespace

QPixmap renderDrawingViewThumbnail(const QVector<DrawingSheetCanvasWidget::Polyline2d>& visible,
								   const QVector<DrawingSheetCanvasWidget::Polyline2d>& hidden, const QSize& size)
{
	QPixmap pm(size);
	pm.fill(QColor(0xF5, 0xF7, 0xFA));
	QPainter p(&pm);
	p.setRenderHint(QPainter::Antialiasing, true);
	const QRectF box = polyBounds(visible, hidden);
	const double sx = (size.width() - 12.0) / qMax(1.0, box.width());
	const double sy = (size.height() - 12.0) / qMax(1.0, box.height());
	const double scale = qMin(sx, sy);
	const QPointF origin(size.width() * 0.5, size.height() * 0.5);
	const QPointF center = box.center();
	auto mapPt = [&](const QPointF& pt) { return origin + (pt - center) * scale; };
	auto drawPolys = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys, const QPen& pen) {
		p.setPen(pen);
		for (const auto& poly : polys)
		{
			if (poly.points.size() < 2)
				continue;
			QPolygonF polyW;
			for (const QPointF& pt : poly.points)
				polyW << mapPt(pt);
			p.drawPolyline(polyW);
		}
	};
	drawPolys(hidden, QPen(QColor(160, 165, 175), 1.0, Qt::DashLine));
	drawPolys(visible, QPen(QColor(30, 34, 42), 1.4, Qt::SolidLine));
	p.setPen(QPen(QColor(180, 186, 196), 1));
	p.drawRect(QRectF(0.5, 0.5, size.width() - 1.0, size.height() - 1.0));
	return pm;
}

DrawingSidePanel::DrawingSidePanel(QWidget* parent) : QWidget(parent)
{
	setWindowTitle(QStringLiteral("工程图"));
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(8, 8, 8, 8);
	root->setSpacing(8);

	m_modelTitle = new QLabel(QStringLiteral("可出图模型"), this);
	m_modelList = new QListWidget(this);
	m_modelList->setMaximumHeight(120);
	m_modelList->setMinimumHeight(72);

	m_viewTitle = new QLabel(QStringLiteral("视角（拖到图幅添加）"), this);
	m_viewList = new ViewPaletteList(this);

	root->addWidget(m_modelTitle);
	root->addWidget(m_modelList);
	root->addWidget(m_viewTitle);
	root->addWidget(m_viewList, 1);

	connect(m_modelList, &QListWidget::currentRowChanged, this, [this](int row) {
		if (row < 0 || row >= m_backendIds.size())
		{
			emit selectionChanged(QString());
			return;
		}
		emit selectionChanged(m_backendIds.at(row));
	});
	connect(m_viewList, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
		if (!item || !item->data(Qt::UserRole + 1).toBool())
			return;
		emit viewTemplateActivated(item->data(Qt::UserRole).toString());
	});

	// 占位：尚未投影时仍显示四视角卡片
	QVector<DrawingViewTemplate> placeholders;
	for (const char* kind : {"front", "top", "right", "iso"})
	{
		DrawingViewTemplate t;
		t.kind = QString::fromLatin1(kind);
		placeholders.push_back(t);
	}
	setViewTemplates(placeholders);
}

void DrawingSidePanel::setBackends(const QStringList& displayNames, const QStringList& backendIds)
{
	m_backendIds = backendIds;
	m_modelList->clear();
	m_modelList->addItems(displayNames);
	if (!displayNames.isEmpty())
		m_modelList->setCurrentRow(0);
}

QString DrawingSidePanel::selectedBackendId() const
{
	const int row = m_modelList ? m_modelList->currentRow() : -1;
	if (row < 0 || row >= m_backendIds.size())
		return {};
	return m_backendIds.at(row);
}

void DrawingSidePanel::setViewTemplates(const QVector<DrawingViewTemplate>& templates)
{
	m_templates = templates;
	rebuildViewList();
}

void DrawingSidePanel::rebuildViewList()
{
	if (!m_viewList)
		return;
	m_viewList->clear();
	auto titleOf = [this](const QString& kind) -> QString {
		if (kind == QLatin1String("front"))
			return m_useChinese ? QStringLiteral("正视图") : QStringLiteral("Front");
		if (kind == QLatin1String("top"))
			return m_useChinese ? QStringLiteral("俯视图") : QStringLiteral("Top");
		if (kind == QLatin1String("right"))
			return m_useChinese ? QStringLiteral("右视图") : QStringLiteral("Right");
		if (kind == QLatin1String("iso"))
			return m_useChinese ? QStringLiteral("轴测图") : QStringLiteral("Iso");
		if (kind == QLatin1String("section"))
			return m_useChinese ? QStringLiteral("剖视图") : QStringLiteral("Section");
		return kind;
	};

	for (const DrawingViewTemplate& t : m_templates)
	{
		const bool ready = !t.visible.isEmpty() || !t.hidden.isEmpty();
		QPixmap thumb = t.thumbnail;
		if (thumb.isNull())
		{
			if (ready)
				thumb = renderDrawingViewThumbnail(t.visible, t.hidden, QSize(148, 108));
			else
			{
				thumb = QPixmap(148, 108);
				thumb.fill(QColor(0xEE, 0xF1, 0xF5));
				QPainter p(&thumb);
				p.setPen(QColor(140, 148, 160));
				p.drawText(thumb.rect(), Qt::AlignCenter,
						   m_useChinese ? QStringLiteral("待生成") : QStringLiteral("Pending"));
			}
		}
		auto* item = new QListWidgetItem(QIcon(thumb), titleOf(t.kind));
		item->setData(Qt::UserRole, t.kind);
		item->setData(Qt::UserRole + 1, ready);
		item->setFlags(ready ? (Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled)
							 : (Qt::ItemIsEnabled | Qt::ItemIsSelectable));
		item->setToolTip(ready ? (m_useChinese ? QStringLiteral("拖到图幅添加此视图")
											   : QStringLiteral("Drag onto sheet to add"))
							   : (m_useChinese ? QStringLiteral("先选择模型并生成图纸")
											   : QStringLiteral("Generate drawing first")));
		m_viewList->addItem(item);
	}
}

void DrawingSidePanel::applyLanguage(bool useChinese)
{
	m_useChinese = useChinese;
	setWindowTitle(useChinese ? QStringLiteral("工程图") : QStringLiteral("Drawing"));
	if (m_modelTitle)
		m_modelTitle->setText(useChinese ? QStringLiteral("可出图模型") : QStringLiteral("Drawable models"));
	if (m_viewTitle)
		m_viewTitle->setText(useChinese ? QStringLiteral("视角（拖到图幅添加）")
										: QStringLiteral("Views (drag to sheet)"));
	rebuildViewList();
}
