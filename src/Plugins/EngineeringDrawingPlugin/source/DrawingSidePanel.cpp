/// @file DrawingSidePanel.cpp
/// @brief 工程图左侧：模型 + 视角缩略图拖放

#include "DrawingSidePanel.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <QAbstractItemView>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDrag>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMimeData>
#include <QPainter>
#include <QPushButton>
#include <QSignalBlocker>
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
	auto fitCircle = [&](const QVector<QPointF>& pts, QPointF& c, double& r) -> bool {
		if (pts.size() < 5)
			return false;
		double sx = 0, sy = 0;
		for (const QPointF& p : pts)
		{
			sx += p.x();
			sy += p.y();
		}
		c = QPointF(sx / pts.size(), sy / pts.size());
		r = 0;
		double err = 0;
		for (const QPointF& p : pts)
			r += QLineF(c, p).length();
		r /= pts.size();
		if (!(r > 1e-6))
			return false;
		for (const QPointF& p : pts)
			err = qMax(err, std::abs(QLineF(c, p).length() - r));
		return err <= qMax(0.03 * r, 0.25);
	};
	auto drawPolys = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys, const QPen& pen) {
		p.setPen(pen);
		p.setBrush(Qt::NoBrush);
		for (const auto& poly : polys)
		{
			if (poly.points.size() < 2)
				continue;
			QPointF c;
			double r = 0;
			if (fitCircle(poly.points, c, r) &&
				QLineF(poly.points.first(), poly.points.last()).length() <= qMax(0.04 * r, 0.3))
			{
				const QPointF wc = mapPt(c);
				p.drawEllipse(wc, r * scale, r * scale);
				continue;
			}
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

	m_detailTitle = new QLabel(QStringLiteral("局部视图"), this);
	m_detailList = new QListWidget(this);
	m_detailList->setMaximumHeight(100);
	m_detailList->setMinimumHeight(56);
	auto* detailBtns = new QWidget(this);
	auto* detailBar = new QHBoxLayout(detailBtns);
	detailBar->setContentsMargins(0, 0, 0, 0);
	detailBar->setSpacing(4);
	m_detailRenameBtn = new QPushButton(QStringLiteral("改名"), detailBtns);
	m_detailScaleBtn = new QPushButton(QStringLiteral("倍率"), detailBtns);
	m_detailDeleteBtn = new QPushButton(QStringLiteral("删除"), detailBtns);
	detailBar->addWidget(m_detailRenameBtn);
	detailBar->addWidget(m_detailScaleBtn);
	detailBar->addWidget(m_detailDeleteBtn);
	detailBar->addStretch(1);

	m_layerTitle = new QLabel(QStringLiteral("图层"), this);
	m_layerList = new QListWidget(this);
	m_layerList->setMaximumHeight(140);
	m_layerList->setMinimumHeight(72);
	auto* layerBtns = new QWidget(this);
	auto* layerBar = new QHBoxLayout(layerBtns);
	layerBar->setContentsMargins(0, 0, 0, 0);
	layerBar->setSpacing(4);
	m_layerAddBtn = new QPushButton(QStringLiteral("新建"), layerBtns);
	m_layerRenameBtn = new QPushButton(QStringLiteral("重命名"), layerBtns);
	m_layerDeleteBtn = new QPushButton(QStringLiteral("删除"), layerBtns);
	m_layerMoveBtn = new QPushButton(QStringLiteral("移到当前层"), layerBtns);
	layerBar->addWidget(m_layerAddBtn);
	layerBar->addWidget(m_layerRenameBtn);
	layerBar->addWidget(m_layerDeleteBtn);
	layerBar->addWidget(m_layerMoveBtn);
	layerBar->addStretch(1);

	auto* layerStyle = new QWidget(this);
	auto* styleBar = new QHBoxLayout(layerStyle);
	styleBar->setContentsMargins(0, 0, 0, 0);
	styleBar->setSpacing(4);
	m_layerColorBtn = new QPushButton(QStringLiteral("颜色"), layerStyle);
	m_layerColorBtn->setFixedWidth(48);
	m_layerLineTypeLabel = new QLabel(QStringLiteral("线型"), layerStyle);
	m_layerLineTypeCombo = new QComboBox(layerStyle);
	m_layerLineTypeCombo->addItem(QStringLiteral("实线"), static_cast<int>(SheetLineType::Continuous));
	m_layerLineTypeCombo->addItem(QStringLiteral("虚线"), static_cast<int>(SheetLineType::Dashed));
	m_layerLineTypeCombo->addItem(QStringLiteral("中心线"), static_cast<int>(SheetLineType::Center));
	m_layerLineTypeCombo->addItem(QStringLiteral("点划线"), static_cast<int>(SheetLineType::DashDot));
	m_layerLineTypeCombo->setMinimumWidth(84);
	m_layerWidthLabel = new QLabel(QStringLiteral("线宽"), layerStyle);
	m_layerWidthSpin = new QDoubleSpinBox(layerStyle);
	m_layerWidthSpin->setRange(0.13, 2.0);
	m_layerWidthSpin->setDecimals(2);
	m_layerWidthSpin->setSingleStep(0.05);
	m_layerWidthSpin->setSuffix(QStringLiteral(" mm"));
	m_layerWidthSpin->setValue(0.35);
	m_layerWidthSpin->setMaximumWidth(88);
	styleBar->addWidget(m_layerColorBtn);
	styleBar->addWidget(m_layerLineTypeLabel);
	styleBar->addWidget(m_layerLineTypeCombo, 1);
	styleBar->addWidget(m_layerWidthLabel);
	styleBar->addWidget(m_layerWidthSpin);

	auto* layerFlags = new QWidget(this);
	auto* flagBar = new QHBoxLayout(layerFlags);
	flagBar->setContentsMargins(0, 0, 0, 0);
	flagBar->setSpacing(8);
	m_layerFrozenCheck = new QCheckBox(QStringLiteral("冻结"), layerFlags);
	m_layerPlotCheck = new QCheckBox(QStringLiteral("可打印"), layerFlags);
	m_layerPlotCheck->setChecked(true);
	flagBar->addWidget(m_layerFrozenCheck);
	flagBar->addWidget(m_layerPlotCheck);
	flagBar->addStretch(1);

	root->addWidget(m_modelTitle);
	root->addWidget(m_modelList);
	root->addWidget(m_viewTitle);
	root->addWidget(m_viewList, 1);
	root->addWidget(m_detailTitle);
	root->addWidget(m_detailList);
	root->addWidget(detailBtns);
	root->addWidget(m_layerTitle);
	root->addWidget(m_layerList);
	root->addWidget(layerBtns);
	root->addWidget(layerStyle);
	root->addWidget(layerFlags);

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
	connect(m_layerList, &QListWidget::itemClicked, this, [this](QListWidgetItem* item) {
		if (m_layerUiBusy || !m_canvas || !item)
			return;
		m_canvas->setCurrentLayer(item->data(Qt::UserRole).toString());
		rebuildLayerList();
	});
	connect(m_layerList, &QListWidget::currentItemChanged, this, [this](QListWidgetItem*, QListWidgetItem*) {
		if (!m_layerUiBusy)
			syncLayerStyleUi();
	});
	connect(m_layerList, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
		if (m_layerUiBusy || !m_canvas || !item)
			return;
		const QString id = item->data(Qt::UserRole).toString();
		const auto* L = m_canvas->layerById(id);
		if (!L)
			return;
		m_canvas->setLayerLocked(id, !L->locked);
		rebuildLayerList();
	});
	connect(m_layerList, &QListWidget::itemChanged, this, [this](QListWidgetItem* item) {
		if (m_layerUiBusy || !m_canvas || !item)
			return;
		const QString id = item->data(Qt::UserRole).toString();
		const bool vis = item->checkState() == Qt::Checked;
		m_canvas->setLayerVisible(id, vis);
	});
	connect(m_layerColorBtn, &QPushButton::clicked, this, [this]() {
		if (!m_canvas)
			return;
		const QString id = selectedLayerId();
		const auto* L = m_canvas->layerById(id);
		if (!L)
			return;
		const QColor c = QColorDialog::getColor(L->color, this, m_useChinese ? QStringLiteral("图层颜色")
																			 : QStringLiteral("Layer Color"));
		if (!c.isValid())
			return;
		m_canvas->setLayerColor(id, c);
		rebuildLayerList();
	});
	connect(m_layerLineTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
		if (m_layerUiBusy || !m_canvas || !m_layerLineTypeCombo)
			return;
		const QString id = selectedLayerId();
		const SheetLineType t = static_cast<SheetLineType>(m_layerLineTypeCombo->currentData().toInt());
		m_canvas->setLayerLineType(id, t);
	});
	connect(m_layerWidthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double w) {
		if (m_layerUiBusy || !m_canvas)
			return;
		m_canvas->setLayerLineWidth(selectedLayerId(), w);
	});
	connect(m_layerFrozenCheck, &QCheckBox::toggled, this, [this](bool on) {
		if (m_layerUiBusy || !m_canvas)
			return;
		m_canvas->setLayerFrozen(selectedLayerId(), on);
	});
	connect(m_layerPlotCheck, &QCheckBox::toggled, this, [this](bool on) {
		if (m_layerUiBusy || !m_canvas)
			return;
		m_canvas->setLayerPlottable(selectedLayerId(), on);
	});
	connect(m_layerAddBtn, &QPushButton::clicked, this, [this]() {
		if (!m_canvas)
			return;
		bool ok = false;
		const QString name = QInputDialog::getText(this, QStringLiteral("新建图层"), QStringLiteral("名称"),
												   QLineEdit::Normal, QStringLiteral("图层"), &ok);
		if (!ok)
			return;
		m_canvas->addLayer(name);
		rebuildLayerList();
	});
	connect(m_layerRenameBtn, &QPushButton::clicked, this, [this]() {
		if (!m_canvas || !m_layerList || !m_layerList->currentItem())
			return;
		const QString id = m_layerList->currentItem()->data(Qt::UserRole).toString();
		const auto* L = m_canvas->layerById(id);
		if (!L)
			return;
		bool ok = false;
		const QString name =
			QInputDialog::getText(this, QStringLiteral("重命名图层"), QStringLiteral("名称"), QLineEdit::Normal, L->name,
								  &ok);
		if (!ok || name.trimmed().isEmpty())
			return;
		if (!m_canvas->renameLayer(id, name))
			return;
		rebuildLayerList();
	});
	connect(m_layerDeleteBtn, &QPushButton::clicked, this, [this]() {
		if (!m_canvas || !m_layerList || !m_layerList->currentItem())
			return;
		const QString id = m_layerList->currentItem()->data(Qt::UserRole).toString();
		if (!m_canvas->removeLayer(id))
			return;
		rebuildLayerList();
	});
	connect(m_layerMoveBtn, &QPushButton::clicked, this, [this]() {
		if (m_canvas)
			m_canvas->reassignSelectionToCurrentLayer();
	});
	connect(m_detailRenameBtn, &QPushButton::clicked, this, [this]() {
		if (!m_canvas || !m_detailList || !m_detailList->currentItem())
			return;
		const QString id = m_detailList->currentItem()->data(Qt::UserRole).toString();
		bool ok = false;
		const QString name = QInputDialog::getText(this, QStringLiteral("局部视图"), QStringLiteral("标题"),
												   QLineEdit::Normal, m_detailList->currentItem()->text(), &ok);
		if (!ok || name.trimmed().isEmpty())
			return;
		m_canvas->renameView(id, name);
		rebuildDetailList();
	});
	connect(m_detailScaleBtn, &QPushButton::clicked, this, [this]() {
		if (!m_canvas || !m_detailList || !m_detailList->currentItem())
			return;
		const QString id = m_detailList->currentItem()->data(Qt::UserRole).toString();
		const double cur = m_detailList->currentItem()->data(Qt::UserRole + 1).toDouble();
		bool ok = false;
		const double scale =
			QInputDialog::getDouble(this, QStringLiteral("局部倍率"), QStringLiteral("倍率"), cur, 1.5, 10.0, 1, &ok);
		if (!ok)
			return;
		m_canvas->setDetailViewScale(id, scale);
		rebuildDetailList();
	});
	connect(m_detailDeleteBtn, &QPushButton::clicked, this, [this]() {
		if (!m_canvas || !m_detailList || !m_detailList->currentItem())
			return;
		m_canvas->removeView(m_detailList->currentItem()->data(Qt::UserRole).toString());
		rebuildDetailList();
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

void DrawingSidePanel::bindCanvas(DrawingSheetCanvasWidget* canvas)
{
	if (m_canvas == canvas)
	{
		rebuildLayerList();
		return;
	}
	if (m_canvas)
		disconnect(m_canvas, nullptr, this, nullptr);
	m_canvas = canvas;
	if (m_canvas)
	{
		connect(m_canvas, &DrawingSheetCanvasWidget::layersChanged, this, [this]() { rebuildLayerList(); });
		connect(m_canvas, &DrawingSheetCanvasWidget::sheetChanged, this, [this]() {
			rebuildLayerList();
			rebuildDetailList();
		});
	}
	rebuildLayerList();
	rebuildDetailList();
}

void DrawingSidePanel::rebuildDetailList()
{
	if (!m_detailList)
		return;
	m_detailList->clear();
	if (!m_canvas)
		return;
	for (const auto& v : m_canvas->detailViews())
	{
		auto* item = new QListWidgetItem(v.title.isEmpty() ? v.id : v.title);
		item->setData(Qt::UserRole, v.id);
		item->setData(Qt::UserRole + 1, v.contentScale);
		m_detailList->addItem(item);
	}
}

void DrawingSidePanel::rebuildLayerList()
{
	if (!m_layerList)
		return;
	m_layerUiBusy = true;
	m_layerList->clear();
	if (!m_canvas)
	{
		m_layerUiBusy = false;
		syncLayerStyleUi();
		return;
	}
	const QString current = m_canvas->currentLayerId();
	auto lineTypeName = [this](SheetLineType t) -> QString {
		switch (t)
		{
		case SheetLineType::Dashed:
			return m_useChinese ? QStringLiteral("虚线") : QStringLiteral("Dashed");
		case SheetLineType::Center:
			return m_useChinese ? QStringLiteral("中心线") : QStringLiteral("Center");
		case SheetLineType::DashDot:
			return m_useChinese ? QStringLiteral("点划线") : QStringLiteral("DashDot");
		case SheetLineType::Continuous:
		default:
			return m_useChinese ? QStringLiteral("实线") : QStringLiteral("Continuous");
		}
	};
	for (const auto& L : m_canvas->layers())
	{
		QString text = L.name;
		if (L.locked)
			text = QStringLiteral("[锁] %1").arg(text);
		if (L.frozen)
			text = QStringLiteral("[冻] %1").arg(text);
		if (!L.plottable)
			text = QStringLiteral("[不打印] %1").arg(text);
		if (L.id == current)
			text = QStringLiteral("▶ %1").arg(text);
		auto* item = new QListWidgetItem(text);
		item->setData(Qt::UserRole, L.id);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(L.visible ? Qt::Checked : Qt::Unchecked);
		item->setForeground(L.color);
		const QString tip =
			m_useChinese
				? QStringLiteral("%1 · %2 mm · 单击当前层；勾选显示；双击锁定；下方冻结/打印")
					  .arg(lineTypeName(L.lineType))
					  .arg(L.lineWidthMm, 0, 'f', 2)
				: QStringLiteral("%1 · %2 mm · click=current; check=visible; dbl=lock; freeze/plot below")
					  .arg(lineTypeName(L.lineType))
					  .arg(L.lineWidthMm, 0, 'f', 2);
		item->setToolTip(tip);
		m_layerList->addItem(item);
		if (L.id == current)
			m_layerList->setCurrentItem(item);
	}
	m_layerUiBusy = false;
	syncLayerStyleUi();
}

QString DrawingSidePanel::selectedLayerId() const
{
	if (m_layerList && m_layerList->currentItem())
		return m_layerList->currentItem()->data(Qt::UserRole).toString();
	return m_canvas ? m_canvas->currentLayerId() : QStringLiteral("L0");
}

void DrawingSidePanel::syncLayerStyleUi()
{
	if (!m_layerColorBtn || !m_layerLineTypeCombo || !m_layerWidthSpin)
		return;
	const bool has = m_canvas != nullptr;
	m_layerColorBtn->setEnabled(has);
	m_layerLineTypeCombo->setEnabled(has);
	m_layerWidthSpin->setEnabled(has);
	if (m_layerFrozenCheck)
		m_layerFrozenCheck->setEnabled(has);
	if (m_layerPlotCheck)
		m_layerPlotCheck->setEnabled(has);
	if (!has)
		return;
	const auto* L = m_canvas->layerById(selectedLayerId());
	if (!L)
		return;
	const QSignalBlocker b1(m_layerLineTypeCombo);
	const QSignalBlocker b2(m_layerWidthSpin);
	const int idx = m_layerLineTypeCombo->findData(static_cast<int>(L->lineType));
	if (idx >= 0)
		m_layerLineTypeCombo->setCurrentIndex(idx);
	m_layerWidthSpin->setValue(L->lineWidthMm);
	m_layerColorBtn->setStyleSheet(QStringLiteral("QPushButton { background-color: %1; color: %2; }")
									   .arg(L->color.name(QColor::HexRgb),
											L->color.lightness() > 140 ? QStringLiteral("#111")
																	   : QStringLiteral("#fff")));
	if (m_layerFrozenCheck)
	{
		const QSignalBlocker b(m_layerFrozenCheck);
		m_layerFrozenCheck->setChecked(L->frozen);
	}
	if (m_layerPlotCheck)
	{
		const QSignalBlocker b(m_layerPlotCheck);
		m_layerPlotCheck->setChecked(L->plottable);
	}
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
	if (m_detailTitle)
		m_detailTitle->setText(useChinese ? QStringLiteral("局部视图") : QStringLiteral("Details"));
	if (m_detailRenameBtn)
		m_detailRenameBtn->setText(useChinese ? QStringLiteral("改名") : QStringLiteral("Rename"));
	if (m_detailScaleBtn)
		m_detailScaleBtn->setText(useChinese ? QStringLiteral("倍率") : QStringLiteral("Scale"));
	if (m_detailDeleteBtn)
		m_detailDeleteBtn->setText(useChinese ? QStringLiteral("删除") : QStringLiteral("Delete"));
	if (m_layerTitle)
		m_layerTitle->setText(useChinese ? QStringLiteral("图层") : QStringLiteral("Layers"));
	if (m_layerAddBtn)
		m_layerAddBtn->setText(useChinese ? QStringLiteral("新建") : QStringLiteral("Add"));
	if (m_layerRenameBtn)
		m_layerRenameBtn->setText(useChinese ? QStringLiteral("重命名") : QStringLiteral("Rename"));
	if (m_layerDeleteBtn)
		m_layerDeleteBtn->setText(useChinese ? QStringLiteral("删除") : QStringLiteral("Delete"));
	if (m_layerMoveBtn)
		m_layerMoveBtn->setText(useChinese ? QStringLiteral("移到当前层") : QStringLiteral("Move to current"));
	if (m_layerColorBtn)
		m_layerColorBtn->setText(useChinese ? QStringLiteral("颜色") : QStringLiteral("Color"));
	if (m_layerLineTypeLabel)
		m_layerLineTypeLabel->setText(useChinese ? QStringLiteral("线型") : QStringLiteral("Type"));
	if (m_layerWidthLabel)
		m_layerWidthLabel->setText(useChinese ? QStringLiteral("线宽") : QStringLiteral("Width"));
	if (m_layerFrozenCheck)
		m_layerFrozenCheck->setText(useChinese ? QStringLiteral("冻结") : QStringLiteral("Frozen"));
	if (m_layerPlotCheck)
		m_layerPlotCheck->setText(useChinese ? QStringLiteral("可打印") : QStringLiteral("Plot"));
	if (m_layerLineTypeCombo)
	{
		const QSignalBlocker b(m_layerLineTypeCombo);
		m_layerLineTypeCombo->setItemText(0, useChinese ? QStringLiteral("实线") : QStringLiteral("Continuous"));
		m_layerLineTypeCombo->setItemText(1, useChinese ? QStringLiteral("虚线") : QStringLiteral("Dashed"));
		m_layerLineTypeCombo->setItemText(2, useChinese ? QStringLiteral("中心线") : QStringLiteral("Center"));
		m_layerLineTypeCombo->setItemText(3, useChinese ? QStringLiteral("点划线") : QStringLiteral("DashDot"));
	}
	rebuildViewList();
	rebuildLayerList();
	rebuildDetailList();
}
