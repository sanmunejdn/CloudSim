/// @file GeometricModelingPage.cpp

#include "GeometricModelingPage.h"
#include "GeomodelingI18n.h"

#include "SketchGeom.h"
#include "IPluginHostContext.h"

#include <QAction>
#include <QApplication>
#include <QEvent>
#include <QAbstractItemView>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFrame>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QPushButton>
#include <QShortcut>
#include <QKeySequence>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSplitter>
#include <QStackedWidget>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

namespace {
QIcon makeSketchEyeIcon(bool visible)
{
	// 路径内缩，缩放后描边不被按钮/列边界裁掉
	constexpr int s = 32;
	QPixmap pm(s, s);
	pm.fill(Qt::transparent);
	QPainter p(&pm);
	p.setRenderHint(QPainter::Antialiasing, true);

	QPainterPath eye;
	eye.moveTo(5.0, 16.0);
	eye.cubicTo(10.5, 7.5, 21.5, 7.5, 27.0, 16.0);
	eye.cubicTo(21.5, 24.5, 10.5, 24.5, 5.0, 16.0);

	if (visible)
	{
		p.setPen(QPen(QColor(0x0f, 0x17, 0x2a), 2.0));
		p.setBrush(QColor(0xff, 0xff, 0xff));
		p.drawPath(eye);
		p.setPen(Qt::NoPen);
		p.setBrush(QColor(0x1d, 0x4e, 0xd8));
		p.drawEllipse(QRectF(12.0, 11.0, 8.0, 8.0));
		p.setBrush(QColor(0x0f, 0x17, 0x2a));
		p.drawEllipse(QRectF(14.5, 13.5, 3.0, 3.0));
	}
	else
	{
		p.setPen(QPen(QColor(0x64, 0x74, 0x8b), 2.0));
		p.setBrush(QColor(0xf1, 0xf5, 0xf9));
		p.drawPath(eye);
		p.setPen(QPen(QColor(0xdc, 0x26, 0x26), 2.8, Qt::SolidLine, Qt::RoundCap));
		p.drawLine(QPointF(8.0, 8.0), QPointF(24.0, 24.0));
	}
	return QIcon(pm);
}

QString featureTreeTitle(const QString& idOrName, GeomodelingFeatureKind kind, bool zh)
{
	const QString num = idOrName.section(QLatin1Char('_'), -1);
	QString baseEn = QStringLiteral("Sketch");
	QString 		baseZh = QStringLiteral("\u8349\u56fe");
	if (kind == GeomodelingFeatureKind::Pad || idOrName.startsWith(QLatin1String("Pad_")))
	{
		baseEn = QStringLiteral("Pad");
		baseZh = QStringLiteral("\u62c9\u4f38");
	}
	else if (kind == GeomodelingFeatureKind::Pocket || idOrName.startsWith(QLatin1String("Pocket_")))
	{
		baseEn = QStringLiteral("Pocket");
		baseZh = QStringLiteral("\u5207\u9664");
	}
	else if (kind == GeomodelingFeatureKind::Sweep || idOrName.startsWith(QLatin1String("Sweep_")))
	{
		baseEn = QStringLiteral("Sweep");
		baseZh = QStringLiteral("\u626b\u63cf");
	}
	else if (kind == GeomodelingFeatureKind::SweepCut || idOrName.startsWith(QLatin1String("SweepCut_")))
	{
		baseEn = QStringLiteral("SweepCut");
		baseZh = QStringLiteral("\u626b\u63cf\u5207\u9664");
	}
	else if (kind == GeomodelingFeatureKind::Fillet || idOrName.startsWith(QLatin1String("Fillet_")))
	{
		baseEn = QStringLiteral("Fillet");
		baseZh = QStringLiteral("\u5706\u89d2");
	}
	else if (kind == GeomodelingFeatureKind::Chamfer || idOrName.startsWith(QLatin1String("Chamfer_")))
	{
		baseEn = QStringLiteral("Chamfer");
		baseZh = QStringLiteral("\u5012\u89d2");
	}
	else if (kind == GeomodelingFeatureKind::Revolve || idOrName.startsWith(QLatin1String("Revolve_")))
	{
		baseEn = QStringLiteral("Revolve");
		baseZh = QStringLiteral("\u65cb\u8f6c");
	}
	else if (kind == GeomodelingFeatureKind::RevolveCut || idOrName.startsWith(QLatin1String("RevolveCut_")))
	{
		baseEn = QStringLiteral("RevolveCut");
		baseZh = QStringLiteral("\u65cb\u8f6c\u5207\u9664");
	}
	else if (kind == GeomodelingFeatureKind::LinearPattern || idOrName.startsWith(QLatin1String("LinearPattern_")))
	{
		baseEn = QStringLiteral("LinearPattern");
		baseZh = QStringLiteral("\u7ebf\u6027\u9635\u5217");
	}
	else if (kind == GeomodelingFeatureKind::Mirror3D || idOrName.startsWith(QLatin1String("Mirror3D_")))
	{
		baseEn = QStringLiteral("Mirror3D");
		baseZh = QStringLiteral("\u955c\u50cf");
	}
	else if (kind == GeomodelingFeatureKind::Loft || idOrName.startsWith(QLatin1String("Loft_")))
	{
		baseEn = QStringLiteral("Loft");
		baseZh = QStringLiteral("\u653e\u6837");
	}
	else if (kind == GeomodelingFeatureKind::LoftCut || idOrName.startsWith(QLatin1String("LoftCut_")))
	{
		baseEn = QStringLiteral("LoftCut");
		baseZh = QStringLiteral("\u653e\u6837\u5207\u9664");
	}
	else if (kind == GeomodelingFeatureKind::Shell || idOrName.startsWith(QLatin1String("Shell_")))
	{
		baseEn = QStringLiteral("Shell");
		baseZh = QStringLiteral("\u62bd\u58f3");
	}
	else if (kind == GeomodelingFeatureKind::Draft || idOrName.startsWith(QLatin1String("Draft_")))
	{
		baseEn = QStringLiteral("Draft");
		baseZh = QStringLiteral("\u62d4\u6a21");
	}
	else if (kind == GeomodelingFeatureKind::DatumPlane || idOrName.startsWith(QLatin1String("DatumPlane_")))
	{
		baseEn = QStringLiteral("DatumPlane");
		baseZh = QStringLiteral("\u57fa\u51c6\u9762");
	}
	return zh ? QStringLiteral("%1_%2").arg(baseZh, num) : QStringLiteral("%1_%2").arg(baseEn, num);
}

constexpr char kOriginId[] = "__origin__";
constexpr char kOriginPointId[] = "__origin_point__";
constexpr char kOriginXyId[] = "__origin_xy__";
constexpr char kOriginXzId[] = "__origin_xz__";
constexpr char kOriginYzId[] = "__origin_yz__";

bool isVirtualOriginId(const QString& id)
{
	return id.startsWith(QStringLiteral("__origin"));
}

int originPlaneIndexFromId(const QString& id)
{
	if (id == QLatin1String(kOriginXyId))
		return 0;
	if (id == QLatin1String(kOriginXzId))
		return 1;
	if (id == QLatin1String(kOriginYzId))
		return 2;
	return -1;
}
} // namespace

GeometricModelingPage::GeometricModelingPage(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent), m_host(host)
{
	m_useChinese = !host || host->useChinese();
	m_commands = std::make_unique<CommandStack>(this);
	hide();

	m_sidePanel = new QWidget();
	m_sidePanel->setWindowTitle(QStringLiteral("特征树"));
	m_sidePanel->setMinimumWidth(220);
	auto* sideLay = new QVBoxLayout(m_sidePanel);
	sideLay->setContentsMargins(0, 0, 0, 0);
	sideLay->setSpacing(0);

	m_bodyCaption = new QLabel(QStringLiteral("活动实体"), m_sidePanel);
	m_bodyCombo = new QComboBox(m_sidePanel);
	m_bodyCombo->setMinimumHeight(24);
	connect(m_bodyCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			[this](int)
			{
				const QString id = m_bodyCombo->currentData().toString();
				if (id.isEmpty() || id == m_activeBodyId)
					return;
				m_activeBodyId = id;
				emit activeBodyChanged(id);
			});
	sideLay->addWidget(m_bodyCaption);
	sideLay->addWidget(m_bodyCombo);

	m_tree = new QTreeWidget();
	// 单列：眼睛跟名称同行，避免末列被滚动条/面板边裁切
	m_tree->setColumnCount(1);
	m_tree->setHeaderLabels({QStringLiteral("\u7279\u5f81\u6811")});
	m_tree->header()->setStretchLastSection(true);
	m_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
	m_tree->setRootIsDecorated(true);
	m_tree->setUniformRowHeights(true);
	m_tree->setIndentation(12);
	m_tree->setSelectionMode(QAbstractItemView::SingleSelection);
	m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(m_tree, &QTreeWidget::itemClicked, this,
			[this](QTreeWidgetItem* item, int)
			{
				if (!item)
					return;
				m_selectedFeatureId = item->data(0, Qt::UserRole).toString();
			});
	connect(m_tree, &QTreeWidget::itemDoubleClicked, this,
			[this](QTreeWidgetItem* item, int column)
			{
				if (!item || column != 0)
					return;
				const QString id = item->data(0, Qt::UserRole).toString();
				if (id.isEmpty())
					return;
				const int planeIdx = originPlaneIndexFromId(id);
				if (planeIdx >= 0)
				{
					emit originPlaneSketchRequested(planeIdx);
					return;
				}
				if (id == QLatin1String(kOriginPointId) || id == QLatin1String(kOriginId))
				{
					emit fixPointToOriginRequested();
					return;
				}
				if (!isVirtualOriginId(id))
					emit featureEditRequested(id);
			});
	auto* delShortcut = new QShortcut(QKeySequence::Delete, m_tree);
	delShortcut->setContext(Qt::WidgetWithChildrenShortcut);
	connect(delShortcut, &QShortcut::activated, this,
			[this]()
			{
				QTreeWidgetItem* item = m_tree->currentItem();
				if (!item)
					return;
				const QString id = item->data(0, Qt::UserRole).toString();
				if (!id.isEmpty() && !isVirtualOriginId(id))
					emit featureDeleteRequested(id);
			});
	connect(m_tree, &QTreeWidget::customContextMenuRequested, this,
			[this](const QPoint& pos)
			{
				QTreeWidgetItem* item = m_tree->itemAt(pos);
				QMenu menu(m_tree);
				QAction* editAct = nullptr;
				QAction* visAct = nullptr;
				QAction* delAct = nullptr;
				QAction* pickAct = nullptr;
				QAction* rollbackAct = nullptr;
				QAction* sketchOnPlaneAct = nullptr;
				QAction* fixOriginAct = nullptr;
				QString fid;
				bool sketchVisible = true;
				bool isSketch = false;
				const bool virtualOrigin = item && isVirtualOriginId(item->data(0, Qt::UserRole).toString());
				if (item)
				{
					fid = item->data(0, Qt::UserRole).toString();
					if (!fid.isEmpty() && !virtualOrigin)
					{
						editAct = menu.addAction(i18n(QStringLiteral("Edit"), QStringLiteral("\u7f16\u8f91")));
						if (const GeomodelingFeature* f = m_features.find(fid))
						{
							if (f->kind == GeomodelingFeatureKind::Sketch)
							{
								isSketch = true;
								sketchVisible = f->visible;
								visAct = menu.addAction(sketchVisible
															? i18n(QStringLiteral("Hide"), QStringLiteral("\u9690\u85cf"))
															: i18n(QStringLiteral("Show"), QStringLiteral("\u663e\u793a")));
							}
						}
						delAct = menu.addAction(i18n(QStringLiteral("Delete"), QStringLiteral("\u5220\u9664")));
						rollbackAct =
							menu.addAction(i18n(QStringLiteral("Rollback here"), QStringLiteral("\u56de\u9000\u81f3\u6b64")));
					}
					else if (virtualOrigin)
					{
						const int planeIdx = originPlaneIndexFromId(fid);
						if (planeIdx >= 0)
							sketchOnPlaneAct = menu.addAction(
								i18n(QStringLiteral("New sketch on plane"), QStringLiteral("\u5728\u6b64\u5e73\u9762\u65b0\u5efa\u8349\u56fe")));
						if (fid == QLatin1String(kOriginPointId) || fid == QLatin1String(kOriginId))
							fixOriginAct = menu.addAction(
								i18n(QStringLiteral("Fix point to origin"), QStringLiteral("\u56fa\u5b9a\u70b9\u5230\u539f\u70b9")));
						const bool ov = originNodeVisible(fid);
						visAct = menu.addAction(ov ? i18n(QStringLiteral("Hide"), QStringLiteral("\u9690\u85cf"))
												   : i18n(QStringLiteral("Show"), QStringLiteral("\u663e\u793a")));
					}
				}
				else
				{
					pickAct = menu.addAction(
						i18n(QStringLiteral("Pick feature in viewport"), QStringLiteral("\u89c6\u53e3\u70b9\u9009\u7279\u5f81")));
				}
				QAction* exitRb =
					menu.addAction(i18n(QStringLiteral("Exit rollback"), QStringLiteral("\u9000\u51fa\u56de\u9000")));
				exitRb->setEnabled(!m_features.rollbackAfterFeatureId().isEmpty());
				QAction* chosen = menu.exec(m_tree->viewport()->mapToGlobal(pos));
				if (!chosen)
					return;
				if (chosen == editAct && !fid.isEmpty())
					emit featureEditRequested(fid);
				else if (chosen == visAct && isSketch && !fid.isEmpty())
					emit sketchVisibilityToggleRequested(fid);
				else if (chosen == visAct && virtualOrigin && !fid.isEmpty())
					toggleOriginVisibility(fid);
				else if (chosen == delAct && !fid.isEmpty())
					emit featureDeleteRequested(fid);
				else if (chosen == rollbackAct && !fid.isEmpty())
					emit featureRollbackRequested(fid);
				else if (chosen == sketchOnPlaneAct)
					emit originPlaneSketchRequested(originPlaneIndexFromId(fid));
				else if (chosen == fixOriginAct)
					emit fixPointToOriginRequested();
				else if (chosen == pickAct)
					emit viewportFeaturePickRequested();
				else if (chosen == exitRb)
					emit exitRollbackRequested();
			});

	auto* props = new QWidget();
	auto* propsLay = new QVBoxLayout(props);
	propsLay->setContentsMargins(8, 8, 8, 8);
	propsLay->setSpacing(6);

	m_toolStack = new QStackedWidget(props);

	m_pageEmpty = new QWidget(m_toolStack);
	auto* emptyLay = new QVBoxLayout(m_pageEmpty);
	emptyLay->setContentsMargins(0, 0, 0, 0);
	m_emptyHint = new QLabel(QStringLiteral("选中工具后在此设置参数"), m_pageEmpty);
	emptyLay->addWidget(m_emptyHint);
	emptyLay->addStretch(1);
	m_toolStack->addWidget(m_pageEmpty);

	m_pageExtrude = new QWidget(m_toolStack);
	auto* exLay = new QVBoxLayout(m_pageExtrude);
	exLay->setContentsMargins(0, 0, 0, 0);
	m_extrudeTitle = new QLabel(QStringLiteral("拉伸参数"), m_pageExtrude);
	exLay->addWidget(m_extrudeTitle);
	m_length = new QDoubleSpinBox(m_pageExtrude);
	m_length->setRange(0.1, 1e6);
	m_length->setValue(10.0);
	m_length->setSuffix(QStringLiteral(" mm"));
	connect(m_length, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&GeometricModelingPage::lengthEdited);
	exLay->addWidget(m_length);
	m_draftAngle = new QDoubleSpinBox(m_pageExtrude);
	m_draftAngle->setRange(-45.0, 45.0);
	m_draftAngle->setValue(0.0);
	m_draftAngle->setDecimals(1);
	m_draftAngle->setSingleStep(1.0);
	m_draftAngle->setSuffix(QStringLiteral(" \u00b0"));
	m_draftAngle->setToolTip(QStringLiteral("\u62d4\u6a21\u659c\u5ea6"));
	connect(m_draftAngle, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[this](double) { emit extrudeOptionsChanged(); });
	exLay->addWidget(m_draftAngle);
	m_chkReversed = new QCheckBox(QStringLiteral("\u53cd\u5411"), m_pageExtrude);
	connect(m_chkReversed, &QCheckBox::toggled, this, [this](bool) { emit extrudeOptionsChanged(); });
	exLay->addWidget(m_chkReversed);
	m_endCondition = new QComboBox(m_pageExtrude);
	m_endCondition->addItem(QStringLiteral("定长"), static_cast<int>(GeomodelingExtrudeEnd::Blind));
	m_endCondition->addItem(QStringLiteral("到面"), static_cast<int>(GeomodelingExtrudeEnd::UpToFace));
	m_endCondition->addItem(QStringLiteral("对称"), static_cast<int>(GeomodelingExtrudeEnd::MidPlane));
	m_endCondition->addItem(QStringLiteral("贯通"), static_cast<int>(GeomodelingExtrudeEnd::ThroughAll));
	m_endCondition->addItem(QStringLiteral("到顶点"), static_cast<int>(GeomodelingExtrudeEnd::UpToVertex));
	m_endCondition->addItem(QStringLiteral("到面偏移"), static_cast<int>(GeomodelingExtrudeEnd::OffsetFromFace));
	connect(m_endCondition, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			[this](int)
			{
				syncExtrudeEndUi();
				emit extrudeOptionsChanged();
			});
	exLay->addWidget(m_endCondition);
	m_btnPickFace = new QPushButton(QStringLiteral("选择终止面"), m_pageExtrude);
	connect(m_btnPickFace, &QPushButton::clicked, this, &GeometricModelingPage::pickUpToFaceRequested);
	exLay->addWidget(m_btnPickFace);
	m_btnPickVertex = new QPushButton(QStringLiteral("选择顶点"), m_pageExtrude);
	connect(m_btnPickVertex, &QPushButton::clicked, this, &GeometricModelingPage::pickUpToVertexRequested);
	exLay->addWidget(m_btnPickVertex);
	m_offsetFromFace = new QDoubleSpinBox(m_pageExtrude);
	m_offsetFromFace->setRange(-1e6, 1e6);
	m_offsetFromFace->setDecimals(2);
	m_offsetFromFace->setSuffix(QStringLiteral(" mm"));
	m_offsetFromFace->setValue(0.0);
	connect(m_offsetFromFace, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[this](double) { emit extrudeOptionsChanged(); });
	exLay->addWidget(m_offsetFromFace);
	m_upToFaceStatus = new QLabel(QStringLiteral("未选择终止面"), m_pageExtrude);
	m_upToFaceStatus->setWordWrap(true);
	exLay->addWidget(m_upToFaceStatus);
	m_chkNewBody = new QCheckBox(QStringLiteral("新建实体"), m_pageExtrude);
	exLay->addWidget(m_chkNewBody);
	auto* exBtn = new QHBoxLayout();
	m_btnConfirm = new QPushButton(QStringLiteral("确认"), m_pageExtrude);
	m_btnCancel = new QPushButton(QStringLiteral("取消"), m_pageExtrude);
	exBtn->addWidget(m_btnConfirm);
	exBtn->addWidget(m_btnCancel);
	exLay->addLayout(exBtn);
	exLay->addStretch(1);
	connect(m_btnConfirm, &QPushButton::clicked, this, &GeometricModelingPage::confirmExtrudeRequested);
	connect(m_btnCancel, &QPushButton::clicked, this, &GeometricModelingPage::cancelExtrudeRequested);
	m_toolStack->addWidget(m_pageExtrude);
	syncExtrudeEndUi();

	m_pageMirror = new QWidget(m_toolStack);
	auto* miLay = new QVBoxLayout(m_pageMirror);
	miLay->setContentsMargins(0, 0, 0, 0);
	miLay->setSpacing(6);
	m_mirrorTitle = new QLabel(QStringLiteral("镜像参数"), m_pageMirror);
	auto* miTitle = m_mirrorTitle;
	QFont tf = miTitle->font();
	tf.setBold(true);
	miTitle->setFont(tf);
	miLay->addWidget(miTitle);

	m_mirrorAxisCaption = new QLabel(QStringLiteral("镜像轴"), m_pageMirror);
	miLay->addWidget(m_mirrorAxisCaption);
	m_mirrorAxis = new QLabel(QStringLiteral("未选择（点选直线）"), m_pageMirror);
	m_mirrorAxis->setWordWrap(true);
	miLay->addWidget(m_mirrorAxis);
	auto* axisRow = new QHBoxLayout();
	m_btnPickAxis = new QPushButton(QStringLiteral("选择轴"), m_pageMirror);
	m_btnPickAxis->setCheckable(true);
	m_btnPickAxis->setChecked(true);
	axisRow->addWidget(m_btnPickAxis);
	miLay->addLayout(axisRow);

	m_mirrorEntCaption = new QLabel(QStringLiteral("要镜像的图元"), m_pageMirror);
	miLay->addWidget(m_mirrorEntCaption);
	m_btnPickEnt = new QPushButton(QStringLiteral("选择图元"), m_pageMirror);
	m_btnPickEnt->setCheckable(true);
	miLay->addWidget(m_btnPickEnt);
	m_mirrorList = new QListWidget(m_pageMirror);
	m_mirrorList->setMinimumHeight(90);
	m_mirrorList->setSelectionMode(QAbstractItemView::SingleSelection);
	miLay->addWidget(m_mirrorList);
	m_btnClearEnt = new QPushButton(QStringLiteral("清空图元"), m_pageMirror);
	miLay->addWidget(m_btnClearEnt);

	m_keepOriginal = new QCheckBox(QStringLiteral("保留原图元"), m_pageMirror);
	m_keepOriginal->setChecked(true);
	m_keepOriginal->setEnabled(false);
	m_keepOriginal->setToolTip(QStringLiteral("当前版本始终保留原图元"));
	miLay->addWidget(m_keepOriginal);

	auto* miBtn = new QHBoxLayout();
	m_btnMirrorOk = new QPushButton(QStringLiteral("确认镜像"), m_pageMirror);
	m_btnMirrorCancel = new QPushButton(QStringLiteral("取消"), m_pageMirror);
	m_btnMirrorOk->setEnabled(false);
	miBtn->addWidget(m_btnMirrorOk);
	miBtn->addWidget(m_btnMirrorCancel);
	miLay->addLayout(miBtn);
	miLay->addStretch(1);

	connect(m_btnPickAxis, &QPushButton::clicked, this,
			[this]()
			{
				m_btnPickAxis->setChecked(true);
				m_btnPickEnt->setChecked(false);
				emit mirrorPickAxisRequested();
			});
	connect(m_btnPickEnt, &QPushButton::clicked, this,
			[this]()
			{
				m_btnPickEnt->setChecked(true);
				m_btnPickAxis->setChecked(false);
				emit mirrorPickEntitiesRequested();
			});
	connect(m_btnClearEnt, &QPushButton::clicked, this, &GeometricModelingPage::mirrorClearEntitiesRequested);
	connect(m_btnMirrorOk, &QPushButton::clicked, this, &GeometricModelingPage::mirrorConfirmRequested);
	connect(m_btnMirrorCancel, &QPushButton::clicked, this, &GeometricModelingPage::mirrorCancelRequested);
	connect(m_mirrorList, &QListWidget::itemDoubleClicked, this,
			[this](QListWidgetItem* item)
			{
				if (!item)
					return;
				emit mirrorRemoveEntityRequested(item->data(Qt::UserRole).toInt());
			});
	m_toolStack->addWidget(m_pageMirror);

	m_pageTrim = new QWidget(m_toolStack);
	auto* trLay = new QVBoxLayout(m_pageTrim);
	trLay->setContentsMargins(0, 0, 0, 0);
	m_trimTitle = new QLabel(QStringLiteral("修剪"), m_pageTrim);
	auto* trTitle = m_trimTitle;
	trTitle->setFont(tf);
	trLay->addWidget(trTitle);
	m_trimHint = new QLabel(QStringLiteral("点选与其它直线相交的线段段进行裁剪。"), m_pageTrim);
	m_trimHint->setWordWrap(true);
	trLay->addWidget(m_trimHint);
	trLay->addStretch(1);
	m_toolStack->addWidget(m_pageTrim);

	m_pageSweep = new QWidget(m_toolStack);
	auto* swLay = new QVBoxLayout(m_pageSweep);
	swLay->setContentsMargins(0, 0, 0, 0);
	m_sweepTitle = new QLabel(QStringLiteral("\u626b\u63cf\u53c2\u6570"), m_pageSweep);
	m_sweepTitle->setFont(tf);
	swLay->addWidget(m_sweepTitle);
	m_sweepProfileCaption = new QLabel(QStringLiteral("\u8f6e\u5ed3\u8349\u56fe"), m_pageSweep);
	swLay->addWidget(m_sweepProfileCaption);
	m_sweepProfileCombo = new QComboBox(m_pageSweep);
	swLay->addWidget(m_sweepProfileCombo);
	m_btnPickSweepProfile = new QPushButton(QStringLiteral("\u70b9\u9009\u8f6e\u5ed3"), m_pageSweep);
	swLay->addWidget(m_btnPickSweepProfile);
	m_sweepPathCaption = new QLabel(QStringLiteral("\u8def\u5f84\u8349\u56fe"), m_pageSweep);
	swLay->addWidget(m_sweepPathCaption);
	m_sweepPathCombo = new QComboBox(m_pageSweep);
	swLay->addWidget(m_sweepPathCombo);
	m_btnPickSweepPath = new QPushButton(QStringLiteral("\u70b9\u9009\u8def\u5f84"), m_pageSweep);
	swLay->addWidget(m_btnPickSweepPath);
	m_btnPickSweepEdgePath = new QPushButton(QStringLiteral("\u6a21\u578b\u8fb9\u8def\u5f84"), m_pageSweep);
	swLay->addWidget(m_btnPickSweepEdgePath);
	m_sweepTwist = new QDoubleSpinBox(m_pageSweep);
	m_sweepTwist->setRange(-360.0, 360.0);
	m_sweepTwist->setDecimals(1);
	m_sweepTwist->setSuffix(QStringLiteral(" \u00b0"));
	m_sweepTwist->setValue(0.0);
	connect(m_sweepTwist, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[this](double) { emit sweepSelectionChanged(); });
	swLay->addWidget(new QLabel(QStringLiteral("\u626d\u8f6c\u89d2\u5ea6"), m_pageSweep));
	swLay->addWidget(m_sweepTwist);
	auto* swBtns = new QHBoxLayout();
	m_btnSweepOk = new QPushButton(QStringLiteral("\u786e\u8ba4"), m_pageSweep);
	m_btnSweepCancel = new QPushButton(QStringLiteral("\u53d6\u6d88"), m_pageSweep);
	swBtns->addWidget(m_btnSweepOk);
	swBtns->addWidget(m_btnSweepCancel);
	swLay->addLayout(swBtns);
	m_sweepStatus = new QLabel(m_pageSweep);
	m_sweepStatus->setWordWrap(true);
	m_sweepStatus->setStyleSheet(QStringLiteral("color:#b45309;"));
	swLay->addWidget(m_sweepStatus);
	swLay->addStretch(1);
	m_toolStack->addWidget(m_pageSweep);

	const auto emitSweepSel = [this](int)
	{
		emit sweepSelectionChanged();
	};
	connect(m_sweepProfileCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, emitSweepSel);
	connect(m_sweepPathCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, emitSweepSel);
	connect(m_btnPickSweepProfile, &QPushButton::clicked, this, &GeometricModelingPage::pickSweepProfileRequested);
	connect(m_btnPickSweepPath, &QPushButton::clicked, this, &GeometricModelingPage::pickSweepPathRequested);
	connect(m_btnPickSweepEdgePath, &QPushButton::clicked, this, &GeometricModelingPage::pickSweepEdgePathRequested);
	connect(m_btnSweepOk, &QPushButton::clicked, this, &GeometricModelingPage::confirmSweepRequested);
	connect(m_btnSweepCancel, &QPushButton::clicked, this, &GeometricModelingPage::cancelSweepRequested);

	auto makeEdgePanel = [&](QWidget*& page, QLabel*& title, QDoubleSpinBox*& spin, QLabel*& countLbl,
							 QPushButton*& pickBtn, QPushButton*& okBtn, QPushButton*& cancelBtn,
							 const QString& titleText, const QString& spinSuffix, double spinVal,
							 const char* pickSignal, const char* okSignal, const char* cancelSignal)
	{
		page = new QWidget(m_toolStack);
		auto* lay = new QVBoxLayout(page);
		lay->setContentsMargins(0, 0, 0, 0);
		title = new QLabel(titleText, page);
		title->setFont(tf);
		lay->addWidget(title);
		spin = new QDoubleSpinBox(page);
		spin->setRange(0.01, 1e6);
		spin->setDecimals(2);
		spin->setSuffix(spinSuffix);
		spin->setValue(spinVal);
		lay->addWidget(spin);
		countLbl = new QLabel(QStringLiteral("\u5df2\u9009 0 \u6761\u8fb9"), page);
		lay->addWidget(countLbl);
		pickBtn = new QPushButton(QStringLiteral("\u70b9\u9009\u8fb9"), page);
		okBtn = new QPushButton(QStringLiteral("\u786e\u8ba4"), page);
		cancelBtn = new QPushButton(QStringLiteral("\u53d6\u6d88"), page);
		lay->addWidget(pickBtn);
		auto* btns = new QHBoxLayout();
		btns->addWidget(okBtn);
		btns->addWidget(cancelBtn);
		lay->addLayout(btns);
		lay->addStretch(1);
		connect(pickBtn, SIGNAL(clicked()), this, pickSignal);
		connect(okBtn, SIGNAL(clicked()), this, okSignal);
		connect(cancelBtn, SIGNAL(clicked()), this, cancelSignal);
		m_toolStack->addWidget(page);
	};

	makeEdgePanel(m_pageFillet, m_filletTitle, m_filletRadius, m_filletEdgeCount, m_btnPickFilletEdge, m_btnFilletOk,
				  m_btnFilletCancel, QStringLiteral("\u5706\u89d2"), QStringLiteral(" mm"), 1.0,
				  SIGNAL(pickFilletEdgeRequested()), SIGNAL(filletConfirmRequested()), SIGNAL(filletCancelRequested()));
	makeEdgePanel(m_pageChamfer, m_chamferTitle, m_chamferDist, m_chamferEdgeCount, m_btnPickChamferEdge,
				  m_btnChamferOk, m_btnChamferCancel, QStringLiteral("\u5012\u89d2"), QStringLiteral(" mm"), 1.0,
				  SIGNAL(pickChamferEdgeRequested()), SIGNAL(chamferConfirmRequested()), SIGNAL(chamferCancelRequested()));

	m_pageRevolve = new QWidget(m_toolStack);
	{
		auto* lay = new QVBoxLayout(m_pageRevolve);
		lay->setContentsMargins(0, 0, 0, 0);
		m_revolveTitle = new QLabel(QStringLiteral("\u65cb\u8f6c"), m_pageRevolve);
		m_revolveTitle->setFont(tf);
		lay->addWidget(m_revolveTitle);
		lay->addWidget(new QLabel(QStringLiteral("\u8f6e\u5ed3\u8349\u56fe"), m_pageRevolve));
		m_revolveSketchCombo = new QComboBox(m_pageRevolve);
		lay->addWidget(m_revolveSketchCombo);
		m_revolveAngle = new QDoubleSpinBox(m_pageRevolve);
		m_revolveAngle->setRange(0.01, 360.0);
		m_revolveAngle->setDecimals(1);
		m_revolveAngle->setSuffix(QStringLiteral("\u00b0"));
		m_revolveAngle->setValue(360.0);
		lay->addWidget(new QLabel(QStringLiteral("\u89d2\u5ea6"), m_pageRevolve));
		lay->addWidget(m_revolveAngle);
		m_revolveStatus = new QLabel(m_pageRevolve);
		m_revolveStatus->setWordWrap(true);
		lay->addWidget(m_revolveStatus);
		auto* btns = new QHBoxLayout();
		m_btnRevolveOk = new QPushButton(QStringLiteral("\u786e\u8ba4"), m_pageRevolve);
		m_btnRevolveCancel = new QPushButton(QStringLiteral("\u53d6\u6d88"), m_pageRevolve);
		btns->addWidget(m_btnRevolveOk);
		btns->addWidget(m_btnRevolveCancel);
		lay->addLayout(btns);
		lay->addStretch(1);
		connect(m_revolveSketchCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				&GeometricModelingPage::revolveSelectionChanged);
		connect(m_btnRevolveOk, &QPushButton::clicked, this, &GeometricModelingPage::revolveConfirmRequested);
		connect(m_btnRevolveCancel, &QPushButton::clicked, this, &GeometricModelingPage::revolveCancelRequested);
		m_toolStack->addWidget(m_pageRevolve);
	}

	m_pagePattern = new QWidget(m_toolStack);
	{
		auto* lay = new QVBoxLayout(m_pagePattern);
		lay->setContentsMargins(0, 0, 0, 0);
		m_patternTitle = new QLabel(QStringLiteral("\u7ebf\u6027\u9635\u5217"), m_pagePattern);
		m_patternTitle->setFont(tf);
		lay->addWidget(m_patternTitle);
		m_patternSource = new QComboBox(m_pagePattern);
		lay->addWidget(m_patternSource);
		m_patternCount = new QDoubleSpinBox(m_pagePattern);
		m_patternCount->setRange(2, 1000);
		m_patternCount->setDecimals(0);
		m_patternCount->setValue(2);
		lay->addWidget(new QLabel(QStringLiteral("\u6570\u91cf"), m_pagePattern));
		lay->addWidget(m_patternCount);
		m_patternDx = new QDoubleSpinBox(m_pagePattern);
		m_patternDy = new QDoubleSpinBox(m_pagePattern);
		m_patternDz = new QDoubleSpinBox(m_pagePattern);
		for (QDoubleSpinBox* s : {m_patternDx, m_patternDy, m_patternDz})
		{
			s->setRange(-1e6, 1e6);
			s->setDecimals(2);
			s->setSuffix(QStringLiteral(" mm"));
		}
		m_patternDx->setValue(10.0);
		lay->addWidget(new QLabel(QStringLiteral("dX"), m_pagePattern));
		lay->addWidget(m_patternDx);
		lay->addWidget(new QLabel(QStringLiteral("dY"), m_pagePattern));
		lay->addWidget(m_patternDy);
		lay->addWidget(new QLabel(QStringLiteral("dZ"), m_pagePattern));
		lay->addWidget(m_patternDz);
		auto* btns = new QHBoxLayout();
		m_btnPatternOk = new QPushButton(QStringLiteral("\u786e\u8ba4"), m_pagePattern);
		m_btnPatternCancel = new QPushButton(QStringLiteral("\u53d6\u6d88"), m_pagePattern);
		btns->addWidget(m_btnPatternOk);
		btns->addWidget(m_btnPatternCancel);
		lay->addLayout(btns);
		lay->addStretch(1);
		connect(m_btnPatternOk, &QPushButton::clicked, this, &GeometricModelingPage::patternConfirmRequested);
		connect(m_btnPatternCancel, &QPushButton::clicked, this, &GeometricModelingPage::patternCancelRequested);
		connect(m_patternCount, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&GeometricModelingPage::patternOptionsChanged);
		connect(m_patternDx, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&GeometricModelingPage::patternOptionsChanged);
		connect(m_patternDy, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&GeometricModelingPage::patternOptionsChanged);
		connect(m_patternDz, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&GeometricModelingPage::patternOptionsChanged);
		connect(m_patternSource, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				&GeometricModelingPage::patternOptionsChanged);
		m_toolStack->addWidget(m_pagePattern);
	}

	m_pageMirror3d = new QWidget(m_toolStack);
	{
		auto* lay = new QVBoxLayout(m_pageMirror3d);
		lay->setContentsMargins(0, 0, 0, 0);
		m_mirror3dTitle = new QLabel(QStringLiteral("3D \u955c\u50cf"), m_pageMirror3d);
		m_mirror3dTitle->setFont(tf);
		lay->addWidget(m_mirror3dTitle);
		m_mirror3dPlane = new QComboBox(m_pageMirror3d);
		m_mirror3dPlane->addItem(QStringLiteral("XY"), 0);
		m_mirror3dPlane->addItem(QStringLiteral("XZ"), 1);
		m_mirror3dPlane->addItem(QStringLiteral("YZ"), 2);
		lay->addWidget(new QLabel(QStringLiteral("\u955c\u50cf\u5e73\u9762"), m_pageMirror3d));
		lay->addWidget(m_mirror3dPlane);
		m_mirror3dKeep = new QCheckBox(QStringLiteral("\u4fdd\u7559\u539f\u5b9e\u4f53"), m_pageMirror3d);
		m_mirror3dKeep->setChecked(true);
		lay->addWidget(m_mirror3dKeep);
		auto* btns = new QHBoxLayout();
		m_btnMirror3dOk = new QPushButton(QStringLiteral("\u786e\u8ba4"), m_pageMirror3d);
		m_btnMirror3dCancel = new QPushButton(QStringLiteral("\u53d6\u6d88"), m_pageMirror3d);
		btns->addWidget(m_btnMirror3dOk);
		btns->addWidget(m_btnMirror3dCancel);
		lay->addLayout(btns);
		lay->addStretch(1);
		connect(m_mirror3dPlane, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				&GeometricModelingPage::mirror3dOptionsChanged);
		connect(m_mirror3dKeep, &QCheckBox::toggled, this, &GeometricModelingPage::mirror3dOptionsChanged);
		connect(m_btnMirror3dOk, &QPushButton::clicked, this, &GeometricModelingPage::mirror3dConfirmRequested);
		connect(m_btnMirror3dCancel, &QPushButton::clicked, this, &GeometricModelingPage::mirror3dCancelRequested);
		m_toolStack->addWidget(m_pageMirror3d);
	}

	m_pageLoft = new QWidget(m_toolStack);
	{
		auto* lay = new QVBoxLayout(m_pageLoft);
		lay->setContentsMargins(0, 0, 0, 0);
		m_loftTitle = new QLabel(QStringLiteral("\u653e\u6837"), m_pageLoft);
		m_loftTitle->setFont(tf);
		lay->addWidget(m_loftTitle);
		lay->addWidget(new QLabel(QStringLiteral("\u8349\u56fe A"), m_pageLoft));
		m_loftSketchA = new QComboBox(m_pageLoft);
		lay->addWidget(m_loftSketchA);
		lay->addWidget(new QLabel(QStringLiteral("\u8349\u56fe B"), m_pageLoft));
		m_loftSketchB = new QComboBox(m_pageLoft);
		lay->addWidget(m_loftSketchB);
		m_loftStatus = new QLabel(m_pageLoft);
		m_loftStatus->setWordWrap(true);
		lay->addWidget(m_loftStatus);
		auto* btns = new QHBoxLayout();
		m_btnLoftOk = new QPushButton(QStringLiteral("\u786e\u8ba4"), m_pageLoft);
		m_btnLoftCancel = new QPushButton(QStringLiteral("\u53d6\u6d88"), m_pageLoft);
		btns->addWidget(m_btnLoftOk);
		btns->addWidget(m_btnLoftCancel);
		lay->addLayout(btns);
		lay->addStretch(1);
		connect(m_loftSketchA, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				&GeometricModelingPage::loftSelectionChanged);
		connect(m_loftSketchB, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				&GeometricModelingPage::loftSelectionChanged);
		connect(m_btnLoftOk, &QPushButton::clicked, this, &GeometricModelingPage::loftConfirmRequested);
		connect(m_btnLoftCancel, &QPushButton::clicked, this, &GeometricModelingPage::loftCancelRequested);
		m_toolStack->addWidget(m_pageLoft);
	}

	m_pageShell = new QWidget(m_toolStack);
	{
		auto* lay = new QVBoxLayout(m_pageShell);
		lay->setContentsMargins(0, 0, 0, 0);
		m_shellTitle = new QLabel(QStringLiteral("\u62bd\u58f3"), m_pageShell);
		m_shellTitle->setFont(tf);
		lay->addWidget(m_shellTitle);
		m_shellThickness = new QDoubleSpinBox(m_pageShell);
		m_shellThickness->setRange(0.01, 1e6);
		m_shellThickness->setDecimals(2);
		m_shellThickness->setSuffix(QStringLiteral(" mm"));
		m_shellThickness->setValue(1.0);
		lay->addWidget(new QLabel(QStringLiteral("\u58c1\u539a"), m_pageShell));
		lay->addWidget(m_shellThickness);
		m_shellFaceCount = new QLabel(QStringLiteral("\u5df2\u9009 0 \u4e2a\u9762"), m_pageShell);
		lay->addWidget(m_shellFaceCount);
		m_btnPickShellFace = new QPushButton(QStringLiteral("\u70b9\u9009\u9762"), m_pageShell);
		lay->addWidget(m_btnPickShellFace);
		m_shellStatus = new QLabel(m_pageShell);
		m_shellStatus->setWordWrap(true);
		lay->addWidget(m_shellStatus);
		auto* btns = new QHBoxLayout();
		m_btnShellOk = new QPushButton(QStringLiteral("\u786e\u8ba4"), m_pageShell);
		m_btnShellCancel = new QPushButton(QStringLiteral("\u53d6\u6d88"), m_pageShell);
		btns->addWidget(m_btnShellOk);
		btns->addWidget(m_btnShellCancel);
		lay->addLayout(btns);
		lay->addStretch(1);
		connect(m_btnPickShellFace, &QPushButton::clicked, this, &GeometricModelingPage::shellPickFaceRequested);
		connect(m_btnShellOk, &QPushButton::clicked, this, &GeometricModelingPage::shellConfirmRequested);
		connect(m_btnShellCancel, &QPushButton::clicked, this, &GeometricModelingPage::shellCancelRequested);
		m_toolStack->addWidget(m_pageShell);
	}

	m_pageDraft = new QWidget(m_toolStack);
	{
		auto* lay = new QVBoxLayout(m_pageDraft);
		lay->setContentsMargins(0, 0, 0, 0);
		m_draftTitle = new QLabel(QStringLiteral("\u62d4\u6a21"), m_pageDraft);
		m_draftTitle->setFont(tf);
		lay->addWidget(m_draftTitle);
		m_draftAngle = new QDoubleSpinBox(m_pageDraft);
		m_draftAngle->setRange(-45.0, 45.0);
		m_draftAngle->setDecimals(1);
		m_draftAngle->setSuffix(QStringLiteral(" \u00b0"));
		m_draftAngle->setValue(1.0);
		lay->addWidget(m_draftAngle);
		m_draftFaceCount = new QLabel(QStringLiteral("\u5df2\u9009 0 \u4e2a\u9762"), m_pageDraft);
		lay->addWidget(m_draftFaceCount);
		m_btnPickDraftFace = new QPushButton(QStringLiteral("\u70b9\u9009\u62d4\u6a21\u9762"), m_pageDraft);
		lay->addWidget(m_btnPickDraftFace);
		m_draftNeutralLabel = new QLabel(QStringLiteral("\u4e2d\u6027\u9762\uff1a\u9ed8\u8ba4 XY"), m_pageDraft);
		lay->addWidget(m_draftNeutralLabel);
		m_btnPickDraftNeutral = new QPushButton(QStringLiteral("\u70b9\u9009\u4e2d\u6027\u9762"), m_pageDraft);
		lay->addWidget(m_btnPickDraftNeutral);
		m_draftStatus = new QLabel(m_pageDraft);
		m_draftStatus->setWordWrap(true);
		lay->addWidget(m_draftStatus);
		auto* btns = new QHBoxLayout();
		m_btnDraftOk = new QPushButton(QStringLiteral("\u786e\u8ba4"), m_pageDraft);
		m_btnDraftCancel = new QPushButton(QStringLiteral("\u53d6\u6d88"), m_pageDraft);
		btns->addWidget(m_btnDraftOk);
		btns->addWidget(m_btnDraftCancel);
		lay->addLayout(btns);
		lay->addStretch(1);
		connect(m_btnPickDraftFace, &QPushButton::clicked, this, &GeometricModelingPage::draftPickFaceRequested);
		connect(m_btnPickDraftNeutral, &QPushButton::clicked, this, &GeometricModelingPage::draftPickNeutralRequested);
		connect(m_btnDraftOk, &QPushButton::clicked, this, &GeometricModelingPage::draftConfirmRequested);
		connect(m_btnDraftCancel, &QPushButton::clicked, this, &GeometricModelingPage::draftCancelRequested);
		m_toolStack->addWidget(m_pageDraft);
	}

	propsLay->addWidget(m_toolStack);
	propsLay->addStretch(1);

	auto* sideSplit = new QSplitter(Qt::Vertical, m_sidePanel);
	sideSplit->addWidget(m_tree);
	sideSplit->addWidget(props);
	sideSplit->setStretchFactor(0, 3);
	sideSplit->setStretchFactor(1, 2);
	sideLay->addWidget(sideSplit);

	buildLegendPanel();
	setSideToolPanel(SideToolPanel::None);
}

QWidget* GeometricModelingPage::featureTreePanel() const
{
	return m_sidePanel;
}

double GeometricModelingPage::extrudeLengthMm() const
{
	return m_length ? m_length->value() : 10.0;
}

double GeometricModelingPage::extrudeDraftAngleDeg() const
{
	return m_draftAngle ? m_draftAngle->value() : 0.0;
}

bool GeometricModelingPage::extrudeReversed() const
{
	return m_chkReversed && m_chkReversed->isChecked();
}

bool GeometricModelingPage::extrudeCreateNewBody() const
{
	return m_chkNewBody && m_chkNewBody->isChecked();
}

GeomodelingExtrudeEnd GeometricModelingPage::extrudeEndCondition() const
{
	if (!m_endCondition)
		return GeomodelingExtrudeEnd::Blind;
	return static_cast<GeomodelingExtrudeEnd>(m_endCondition->currentData().toInt());
}

void GeometricModelingPage::setUpToFacePlane(const PluginSketchPlane& plane, const QString& backendId, int faceIndex)
{
	m_upToFacePlane = plane;
	m_hasUpToFacePlane = plane.isPlanar;
	m_upToFaceBackendId = backendId;
	m_upToFaceIndex = faceIndex;
	if (m_upToFaceStatus)
		m_upToFaceStatus->setText(m_hasUpToFacePlane
									  ? i18n(QStringLiteral("Up-to face selected"), QStringLiteral("已选择终止面"))
									  : i18n(QStringLiteral("No up-to face"), QStringLiteral("未选择终止面")));
	emit extrudeOptionsChanged();
}

void GeometricModelingPage::clearUpToFacePlane()
{
	m_hasUpToFacePlane = false;
	m_upToFacePlane = {};
	m_upToFaceBackendId.clear();
	m_upToFaceIndex = -1;
	if (m_upToFaceStatus)
		m_upToFaceStatus->setText(i18n(QStringLiteral("No up-to face"), QStringLiteral("未选择终止面")));
}

void GeometricModelingPage::setUpToVertex(const PluginPoint3d& v)
{
	m_upToVertex = v;
	m_hasUpToVertex = true;
	if (m_upToFaceStatus)
	{
		m_upToFaceStatus->setText(
			i18n(QStringLiteral("Vertex (%1,%2,%3)"), QStringLiteral("\u9876\u70b9 (%1,%2,%3)"))
				.arg(v.x, 0, 'f', 2)
				.arg(v.y, 0, 'f', 2)
				.arg(v.z, 0, 'f', 2));
	}
	emit extrudeOptionsChanged();
}

void GeometricModelingPage::clearUpToVertex()
{
	m_hasUpToVertex = false;
	if (m_upToFaceStatus && !m_hasUpToFacePlane)
		m_upToFaceStatus->setText(i18n(QStringLiteral("No up-to vertex"), QStringLiteral("\u672a\u9009\u62e9\u9876\u70b9")));
}

double GeometricModelingPage::offsetFromFaceMm() const
{
	return m_offsetFromFace ? m_offsetFromFace->value() : 0.0;
}

double GeometricModelingPage::sweepTwistDeg() const
{
	return m_sweepTwist ? m_sweepTwist->value() : 0.0;
}

void GeometricModelingPage::setExtrudeUi(double lengthMm, bool reversed, GeomodelingExtrudeEnd end, bool createNewBody,
										 double draftAngleDeg)
{
	if (m_length)
	{
		const QSignalBlocker b(m_length);
		m_length->setValue(lengthMm);
	}
	if (m_draftAngle)
	{
		const QSignalBlocker b(m_draftAngle);
		m_draftAngle->setValue(draftAngleDeg);
	}
	if (m_chkReversed)
	{
		const QSignalBlocker b(m_chkReversed);
		m_chkReversed->setChecked(reversed);
	}
	if (m_endCondition)
	{
		const QSignalBlocker b(m_endCondition);
		const int idx = m_endCondition->findData(static_cast<int>(end));
		if (idx >= 0)
			m_endCondition->setCurrentIndex(idx);
	}
	if (m_chkNewBody)
	{
		const QSignalBlocker b(m_chkNewBody);
		m_chkNewBody->setChecked(createNewBody && !m_pocketMode);
		m_chkNewBody->setEnabled(m_editingFeatureId.isEmpty() && !m_pocketMode);
	}
	syncExtrudeEndUi();
}

void GeometricModelingPage::setExtrudeOperationMode(bool pocket)
{
	m_pocketMode = pocket;
	if (m_chkNewBody)
	{
		const QSignalBlocker b(m_chkNewBody);
		if (pocket)
		{
			m_chkNewBody->setChecked(false);
			m_chkNewBody->setEnabled(false);
		}
		else if (m_editingFeatureId.isEmpty())
		{
			m_chkNewBody->setEnabled(true);
		}
	}
	if (m_bodyCaption)
	{
		m_bodyCaption->setText(pocket ? i18n(QStringLiteral("Pocket target body"), QStringLiteral("切除目标实体"))
									  : i18n(QStringLiteral("Active body"), QStringLiteral("活动实体")));
	}
}

void GeometricModelingPage::syncExtrudeEndUi()
{
	const GeomodelingExtrudeEnd end = extrudeEndCondition();
	const bool upTo = end == GeomodelingExtrudeEnd::UpToFace;
	const bool upVertex = end == GeomodelingExtrudeEnd::UpToVertex;
	const bool offsetFace = end == GeomodelingExtrudeEnd::OffsetFromFace;
	const bool through = end == GeomodelingExtrudeEnd::ThroughAll;
	if (m_length)
		m_length->setEnabled(!upTo && !through && !upVertex && !offsetFace);
	if (m_btnPickFace)
		m_btnPickFace->setEnabled(upTo || offsetFace);
	if (m_btnPickVertex)
		m_btnPickVertex->setEnabled(upVertex);
	if (m_offsetFromFace)
		m_offsetFromFace->setEnabled(offsetFace);
	if (m_upToFaceStatus)
		m_upToFaceStatus->setVisible(upTo || upVertex || offsetFace);
}

void GeometricModelingPage::setActiveBodyId(const QString& id)
{
	m_activeBodyId = id;
	if (!m_bodyCombo)
		return;
	const QSignalBlocker b(m_bodyCombo);
	const int idx = m_bodyCombo->findData(id);
	if (idx >= 0)
		m_bodyCombo->setCurrentIndex(idx);
}

void GeometricModelingPage::setBodyIdList(const QStringList& ids)
{
	m_bodyIds = ids;
	if (!m_bodyCombo)
		return;
	const QSignalBlocker b(m_bodyCombo);
	m_bodyCombo->clear();
	for (const QString& id : ids)
		m_bodyCombo->addItem(id, id);
	const int idx = m_bodyCombo->findData(m_activeBodyId);
	if (idx >= 0)
		m_bodyCombo->setCurrentIndex(idx);
	else if (!ids.isEmpty())
	{
		m_activeBodyId = ids.first();
		m_bodyCombo->setCurrentIndex(0);
	}
}

void GeometricModelingPage::setSideToolPanel(SideToolPanel panel)
{
	if (!m_toolStack)
		return;
	switch (panel)
	{
	case SideToolPanel::Extrude:
		m_toolStack->setCurrentWidget(m_pageExtrude);
		break;
	case SideToolPanel::Mirror:
		m_toolStack->setCurrentWidget(m_pageMirror);
		break;
	case SideToolPanel::Trim:
		m_toolStack->setCurrentWidget(m_pageTrim);
		break;
	case SideToolPanel::Sweep:
		m_toolStack->setCurrentWidget(m_pageSweep);
		break;
	case SideToolPanel::Fillet:
		m_toolStack->setCurrentWidget(m_pageFillet);
		break;
	case SideToolPanel::Chamfer:
		m_toolStack->setCurrentWidget(m_pageChamfer);
		break;
	case SideToolPanel::Revolve:
		m_toolStack->setCurrentWidget(m_pageRevolve);
		break;
	case SideToolPanel::Pattern:
		m_toolStack->setCurrentWidget(m_pagePattern);
		break;
	case SideToolPanel::Mirror3D:
		m_toolStack->setCurrentWidget(m_pageMirror3d);
		break;
	case SideToolPanel::Loft:
		m_toolStack->setCurrentWidget(m_pageLoft);
		break;
	case SideToolPanel::Shell:
		m_toolStack->setCurrentWidget(m_pageShell);
		break;
	case SideToolPanel::Draft:
		m_toolStack->setCurrentWidget(m_pageDraft);
		break;
	default:
		m_toolStack->setCurrentWidget(m_pageEmpty);
		break;
	}
}

void GeometricModelingPage::setExtrudePreviewUi(bool active)
{
	setSideToolPanel(active ? SideToolPanel::Extrude : SideToolPanel::None);
}

void GeometricModelingPage::setSweepPreviewUi(bool active, bool cutMode)
{
	m_sweepCutMode = cutMode;
	if (m_sweepTitle)
		m_sweepTitle->setText(cutMode ? i18n(QStringLiteral("Sweep Cut"), QStringLiteral("\u626b\u63cf\u5207\u9664"))
									  : i18n(QStringLiteral("Sweep"), QStringLiteral("\u626b\u63cf\u51f8\u53f0")));
	if (!active)
		setSweepStatus(QString());
	setSideToolPanel(active ? SideToolPanel::Sweep : SideToolPanel::None);
}

void GeometricModelingPage::setSweepStatus(const QString& text)
{
	if (!m_sweepStatus)
		return;
	m_sweepStatus->setText(text);
	m_sweepStatus->setVisible(!text.isEmpty());
}

void GeometricModelingPage::fillSweepSketchCombos(const QString& selectedProfileId, const QString& selectedPathId)
{
	if (!m_sweepProfileCombo || !m_sweepPathCombo)
		return;
	const QSignalBlocker b1(m_sweepProfileCombo);
	const QSignalBlocker b2(m_sweepPathCombo);
	m_sweepProfileCombo->clear();
	m_sweepPathCombo->clear();

	struct SkItem
	{
		QString id;
		QString title;
		bool closedOk = false;
		bool openOk = false;
	};
	std::vector<SkItem> items;
	for (const auto& f : m_features.features())
	{
		if (f.kind != GeomodelingFeatureKind::Sketch)
			continue;
		SkItem it;
		it.id = f.id;
		it.title = featureTreeTitle(f.name.isEmpty() ? f.id : f.name, f.kind, m_useChinese);
		SketchDocument2d doc;
		if (!f.sketchDocumentUtf8.isEmpty() && doc.fromJsonUtf8(f.sketchDocumentUtf8))
		{
			std::vector<float> closed;
			std::string e;
			it.closedOk = doc.exportClosedProfileXyz(f.plane, closed, &e) && closed.size() >= 12;
			e.clear();
			std::vector<PluginSketchSweepPathSegment> segs;
			it.openOk = doc.exportOpenPathSegments(f.plane, segs, &e) && !segs.empty();
		}
		else
		{
			it.closedOk = f.profileXyzMm.size() >= 12;
			it.openOk = f.profileXyzMm.size() >= 6 && !it.closedOk;
		}
		m_sweepProfileCombo->addItem(it.title, it.id);
		m_sweepPathCombo->addItem(it.title, it.id);
		items.push_back(std::move(it));
	}

	auto findId = [&](const QString& id) -> int
	{
		return m_sweepProfileCombo->findData(id);
	};

	int profileIdx = findId(selectedProfileId);
	int pathIdx = m_sweepPathCombo->findData(selectedPathId);
	if (profileIdx < 0 || pathIdx < 0)
	{
		int preferClosed = -1;
		int preferOpen = -1;
		for (int i = 0; i < static_cast<int>(items.size()); ++i)
		{
			if (preferClosed < 0 && items[static_cast<std::size_t>(i)].closedOk)
				preferClosed = i;
			if (preferOpen < 0 && items[static_cast<std::size_t>(i)].openOk && !items[static_cast<std::size_t>(i)].closedOk)
				preferOpen = i;
		}
		if (preferOpen < 0)
		{
			for (int i = 0; i < static_cast<int>(items.size()); ++i)
			{
				if (items[static_cast<std::size_t>(i)].openOk && i != preferClosed)
				{
					preferOpen = i;
					break;
				}
			}
		}
		if (profileIdx < 0)
			profileIdx = preferClosed >= 0 ? preferClosed : 0;
		if (pathIdx < 0)
		{
			pathIdx = preferOpen >= 0 ? preferOpen : (m_sweepPathCombo->count() > 1 ? (profileIdx == 0 ? 1 : 0) : 0);
			if (pathIdx == profileIdx && m_sweepPathCombo->count() > 1)
				pathIdx = (profileIdx + 1) % m_sweepPathCombo->count();
		}
	}
	if (profileIdx >= 0)
		m_sweepProfileCombo->setCurrentIndex(profileIdx);
	if (pathIdx >= 0)
		m_sweepPathCombo->setCurrentIndex(pathIdx);
}

QString GeometricModelingPage::sweepProfileSketchId() const
{
	return m_sweepProfileCombo ? m_sweepProfileCombo->currentData().toString() : QString();
}

QString GeometricModelingPage::sweepPathSketchId() const
{
	return m_sweepPathCombo ? m_sweepPathCombo->currentData().toString() : QString();
}

void GeometricModelingPage::selectSweepProfileSketch(const QString& id)
{
	if (!m_sweepProfileCombo || id.isEmpty())
		return;
	const int idx = m_sweepProfileCombo->findData(id);
	if (idx >= 0)
		m_sweepProfileCombo->setCurrentIndex(idx);
}

void GeometricModelingPage::selectSweepPathSketch(const QString& id)
{
	if (!m_sweepPathCombo || id.isEmpty())
		return;
	const int idx = m_sweepPathCombo->findData(id);
	if (idx >= 0)
		m_sweepPathCombo->setCurrentIndex(idx);
}

void GeometricModelingPage::updateMirrorPanel(int /*axisId*/, const QString& axisText,
											  const std::vector<std::pair<int, QString>>& entities, bool pickingAxis,
											  bool canConfirm)
{
	if (m_mirrorAxis)
		m_mirrorAxis->setText(axisText);
	if (m_btnPickAxis)
		m_btnPickAxis->setChecked(pickingAxis);
	if (m_btnPickEnt)
		m_btnPickEnt->setChecked(!pickingAxis);
	if (m_mirrorList)
	{
		m_mirrorList->clear();
		for (const auto& e : entities)
		{
			auto* item = new QListWidgetItem(e.second);
			item->setData(Qt::UserRole, e.first);
			item->setToolTip(i18n(QStringLiteral("Double-click to remove"), QStringLiteral("双击移除")));
			m_mirrorList->addItem(item);
		}
	}
	if (m_btnMirrorOk)
		m_btnMirrorOk->setEnabled(canConfirm);
}

void GeometricModelingPage::setTrimHint(const QString& text)
{
	if (m_trimHint)
		m_trimHint->setText(text);
}



QString GeometricModelingPage::i18n(const QString& en, const QString& zh) const
{
	return gmTr(m_useChinese, en, zh);
}

void GeometricModelingPage::applyLanguage(bool useChinese)
{
	m_useChinese = useChinese;
	if (m_sidePanel)
		m_sidePanel->setWindowTitle(i18n(QStringLiteral("Feature Tree"), QStringLiteral("特征树")));
	if (m_tree)
	{
		m_tree->setColumnCount(1);
		m_tree->setHeaderLabels({i18n(QStringLiteral("Feature Tree"), QStringLiteral("\u7279\u5f81\u6811"))});
	}
	if (m_emptyHint)
		m_emptyHint->setText(i18n(QStringLiteral("Select a tool to edit parameters here"), QStringLiteral("选中工具后在此设置参数")));
	if (m_extrudeTitle)
		m_extrudeTitle->setText(i18n(QStringLiteral("Extrude"), QStringLiteral("拉伸参数")));
	if (m_bodyCaption)
	{
		m_bodyCaption->setText(m_pocketMode ? i18n(QStringLiteral("Pocket target body"), QStringLiteral("切除目标实体"))
											: i18n(QStringLiteral("Active body"), QStringLiteral("活动实体")));
	}
	if (m_chkReversed)
		m_chkReversed->setText(i18n(QStringLiteral("Reverse"), QStringLiteral("反向")));
	if (m_draftAngle)
		m_draftAngle->setToolTip(i18n(QStringLiteral("Draft angle"), QStringLiteral("拔模斜度")));
	if (m_chkNewBody)
		m_chkNewBody->setText(i18n(QStringLiteral("New body"), QStringLiteral("新建实体")));
	if (m_endCondition && m_endCondition->count() >= 6)
	{
		m_endCondition->setItemText(0, i18n(QStringLiteral("Blind"), QStringLiteral("定长")));
		m_endCondition->setItemText(1, i18n(QStringLiteral("Up to face"), QStringLiteral("到面")));
		m_endCondition->setItemText(2, i18n(QStringLiteral("Mid plane"), QStringLiteral("对称")));
		m_endCondition->setItemText(3, i18n(QStringLiteral("Through all"), QStringLiteral("贯通")));
		m_endCondition->setItemText(4, i18n(QStringLiteral("Up to vertex"), QStringLiteral("到顶点")));
		m_endCondition->setItemText(5, i18n(QStringLiteral("Offset from face"), QStringLiteral("到面偏移")));
	}
	if (m_btnPickFace)
		m_btnPickFace->setText(i18n(QStringLiteral("Pick end face"), QStringLiteral("选择终止面")));
	if (m_upToFaceStatus && !m_hasUpToFacePlane)
		m_upToFaceStatus->setText(i18n(QStringLiteral("No up-to face"), QStringLiteral("未选择终止面")));
	if (m_btnConfirm)
		m_btnConfirm->setText(i18n(QStringLiteral("OK"), QStringLiteral("确认")));
	if (m_btnCancel)
		m_btnCancel->setText(i18n(QStringLiteral("Cancel"), QStringLiteral("取消")));
	if (m_mirrorTitle)
		m_mirrorTitle->setText(i18n(QStringLiteral("Mirror"), QStringLiteral("镜像参数")));
	if (m_mirrorAxisCaption)
		m_mirrorAxisCaption->setText(i18n(QStringLiteral("Mirror axis"), QStringLiteral("镜像轴")));
	if (m_mirrorEntCaption)
		m_mirrorEntCaption->setText(i18n(QStringLiteral("Entities to mirror"), QStringLiteral("要镜像的图元")));
	if (m_btnPickAxis)
		m_btnPickAxis->setText(i18n(QStringLiteral("Pick axis"), QStringLiteral("选择轴")));
	if (m_btnPickEnt)
		m_btnPickEnt->setText(i18n(QStringLiteral("Pick entities"), QStringLiteral("选择图元")));
	if (m_btnClearEnt)
		m_btnClearEnt->setText(i18n(QStringLiteral("Clear entities"), QStringLiteral("清空图元")));
	if (m_keepOriginal)
	{
		m_keepOriginal->setText(i18n(QStringLiteral("Keep original"), QStringLiteral("保留原图元")));
		m_keepOriginal->setToolTip(i18n(QStringLiteral("Current version always keeps originals"), QStringLiteral("当前版本始终保留原图元")));
	}
	if (m_btnMirrorOk)
		m_btnMirrorOk->setText(i18n(QStringLiteral("Confirm mirror"), QStringLiteral("确认镜像")));
	if (m_btnMirrorCancel)
		m_btnMirrorCancel->setText(i18n(QStringLiteral("Cancel"), QStringLiteral("取消")));
	if (m_trimTitle)
		m_trimTitle->setText(i18n(QStringLiteral("Trim"), QStringLiteral("修剪")));
	if (m_sweepTitle)
		m_sweepTitle->setText(m_sweepCutMode ? i18n(QStringLiteral("Sweep Cut"), QStringLiteral("\u626b\u63cf\u5207\u9664"))
											: i18n(QStringLiteral("Sweep"), QStringLiteral("\u626b\u63cf\u51f8\u53f0")));
	if (m_sweepProfileCaption)
		m_sweepProfileCaption->setText(i18n(QStringLiteral("Profile sketch"), QStringLiteral("\u8f6e\u5ed3\u8349\u56fe")));
	if (m_sweepPathCaption)
		m_sweepPathCaption->setText(i18n(QStringLiteral("Path sketch"), QStringLiteral("\u8def\u5f84\u8349\u56fe")));
	if (m_btnSweepOk)
		m_btnSweepOk->setText(i18n(QStringLiteral("OK"), QStringLiteral("\u786e\u8ba4")));
	if (m_btnSweepCancel)
		m_btnSweepCancel->setText(i18n(QStringLiteral("Cancel"), QStringLiteral("\u53d6\u6d88")));
	if (m_sweepStatus && m_sweepStatus->text().isEmpty())
		m_sweepStatus->setVisible(false);
	if (m_trimHint && m_toolStack && m_toolStack->currentWidget() == m_pageTrim)
	{
		// keep custom hint if set; default only when empty-ish
	}
	if (m_legendPanel)
		rebuildLegendContent();
	refreshFeatureTree();
}

void GeometricModelingPage::rebuildLegendContent()
{
	if (!m_legendPanel)
		return;
	if (!m_legendBodyLay)
	{
		m_legendBodyLay = qobject_cast<QVBoxLayout*>(m_legendPanel->layout());
		if (!m_legendBodyLay)
			return;
	}
	while (QLayoutItem* it = m_legendBodyLay->takeAt(0))
	{
		if (QWidget* w = it->widget())
			w->deleteLater();
		delete it;
	}

	auto* legTitle = new QLabel(i18n(QStringLiteral("Legend && Units"), QStringLiteral("图例与单位")), m_legendPanel);
	QFont legTf = legTitle->font();
	legTf.setBold(true);
	legTf.setPointSize(legTf.pointSize() + 1);
	legTitle->setFont(legTf);
	m_legendBodyLay->addWidget(legTitle);

	auto addColorRow = [&](const QColor& c, const QString& text, bool dashed = false) {
		auto* row = new QWidget(m_legendPanel);
		auto* hl = new QHBoxLayout(row);
		hl->setContentsMargins(0, 0, 0, 0);
		hl->setSpacing(8);
		auto* swatch = new QFrame(row);
		swatch->setFixedSize(28, 10);
		swatch->setFrameShape(QFrame::NoFrame);
		if (dashed)
			swatch->setStyleSheet(QStringLiteral("background: transparent; border-top: 2px dashed %1; margin-top: 4px;").arg(c.name()));
		else
			swatch->setStyleSheet(QStringLiteral("background-color: %1; border-radius: 2px;").arg(c.name()));
		auto* lab = new QLabel(text, row);
		lab->setWordWrap(true);
		hl->addWidget(swatch, 0, Qt::AlignVCenter);
		hl->addWidget(lab, 1);
		m_legendBodyLay->addWidget(row);
	};

	addColorRow(QColor::fromRgbF(0.20, 0.85, 1.00), i18n(QStringLiteral("Normal entity"), QStringLiteral("普通图元")));
	addColorRow(QColor::fromRgbF(0.70, 0.70, 0.72), i18n(QStringLiteral("Construction"), QStringLiteral("构造线")), true);
	addColorRow(QColor::fromRgbF(1.00, 0.92, 0.20), i18n(QStringLiteral("Hover / select / snap"), QStringLiteral("悬停 / 选中 / 捕捉")));
	addColorRow(QColor::fromRgbF(1.00, 0.72, 0.12), i18n(QStringLiteral("Dimension"), QStringLiteral("尺寸标注")));
	addColorRow(QColor::fromRgbF(1.00, 0.55, 0.15), i18n(QStringLiteral("Redundant constraint"), QStringLiteral("冗余约束")));
	addColorRow(QColor::fromRgbF(1.00, 0.22, 0.18), i18n(QStringLiteral("Constraint conflict"), QStringLiteral("约束冲突")));

	auto* sep = new QFrame(m_legendPanel);
	sep->setFrameShape(QFrame::HLine);
	sep->setStyleSheet(QStringLiteral("color: #64748b;"));
	m_legendBodyLay->addWidget(sep);

	auto* unitTitle = new QLabel(i18n(QStringLiteral("Units"), QStringLiteral("单位")), m_legendPanel);
	unitTitle->setFont(legTf);
	m_legendBodyLay->addWidget(unitTitle);

	auto addUnitRow = [&](const QString& name, const QString& unit) {
		auto* row = new QWidget(m_legendPanel);
		auto* hl = new QHBoxLayout(row);
		hl->setContentsMargins(0, 0, 0, 0);
		hl->setSpacing(8);
		hl->addWidget(new QLabel(name, row), 1);
		auto* u = new QLabel(unit, row);
		u->setStyleSheet(QStringLiteral("color: #5eead4; font-weight: 600;"));
		hl->addWidget(u, 0, Qt::AlignRight);
		m_legendBodyLay->addWidget(row);
	};
	addUnitRow(i18n(QStringLiteral("Length / distance / radius"), QStringLiteral("长度 / 距离 / 半径")), QStringLiteral("mm"));
	addUnitRow(i18n(QStringLiteral("Extrude depth"), QStringLiteral("拉伸深度")), QStringLiteral("mm"));
	addUnitRow(i18n(QStringLiteral("Angle"), QStringLiteral("角度")), i18n(QStringLiteral("deg"), QStringLiteral("°（度）")));
	addUnitRow(i18n(QStringLiteral("Model coordinates"), QStringLiteral("模型坐标")), QStringLiteral("mm"));
	m_legendBodyLay->addStretch(1);
	m_legendPanel->adjustSize();
}

void GeometricModelingPage::buildLegendPanel()
{
	m_legendPanel = new QWidget(this, Qt::Tool | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
	m_legendPanel->setObjectName(QStringLiteral("SketchLegendOverlay"));
	m_legendPanel->setAttribute(Qt::WA_ShowWithoutActivating);
	m_legendPanel->setFixedWidth(220);
	m_legendPanel->setStyleSheet(QStringLiteral(
		"QWidget#SketchLegendOverlay {"
		"  background-color: rgba(15, 23, 42, 220);"
		"  border: 1px solid #475569;"
		"  border-radius: 8px;"
		"  color: #e2e8f0;"
		"}"
		"QLabel { color: #e2e8f0; background: transparent; }"
		"QFrame { background: transparent; }"));
	m_legendBodyLay = new QVBoxLayout(m_legendPanel);
	m_legendBodyLay->setContentsMargins(12, 10, 12, 10);
	m_legendBodyLay->setSpacing(6);
	rebuildLegendContent();
	m_legendPanel->hide();
}

QWidget* GeometricModelingPage::findViewportHost() const
{
	QWidget* best = nullptr;
	int bestArea = 0;
	const auto widgets = QApplication::allWidgets();
	for (QWidget* w : widgets)
	{
		if (!w || !w->isVisible())
			continue;
		if (QString::fromLatin1(w->metaObject()->className()) != QLatin1String("OsgWidget"))
			continue;
		const int area = w->width() * w->height();
		if (area > bestArea)
		{
			bestArea = area;
			best = w;
		}
	}
	return best;
}

void GeometricModelingPage::repositionLegend()
{
	if (!m_legendPanel || !m_legendAnchor)
		return;
	const int margin = 12;
	m_legendPanel->adjustSize();
	const QSize sz = m_legendPanel->size().expandedTo(m_legendPanel->sizeHint());
	const QPoint topLeft = m_legendAnchor->mapToGlobal(
		QPoint(m_legendAnchor->width() - sz.width() - margin, m_legendAnchor->height() - sz.height() - margin));
	m_legendPanel->setFixedSize(sz);
	m_legendPanel->move(topLeft);
	m_legendPanel->raise();
}

void GeometricModelingPage::showLegendOverlay()
{
	if (!m_legendPanel)
		buildLegendPanel();
	QWidget* host = findViewportHost();
	if (!host)
	{
		m_legendPanel->hide();
		return;
	}
	if (m_legendAnchor && m_legendAnchor != host)
	{
		m_legendAnchor->removeEventFilter(this);
		m_legendAnchor = nullptr;
	}
	if (m_legendAnchor != host)
	{
		m_legendAnchor = host;
		m_legendAnchor->installEventFilter(this);
	}
	repositionLegend();
	m_legendPanel->show();
	m_legendPanel->raise();
}

void GeometricModelingPage::hideLegendOverlay()
{
	if (m_legendAnchor)
	{
		m_legendAnchor->removeEventFilter(this);
		m_legendAnchor = nullptr;
	}
	if (m_legendPanel)
		m_legendPanel->hide();
}

bool GeometricModelingPage::eventFilter(QObject* watched, QEvent* event)
{
	if (watched == m_legendAnchor && m_legendPanel && m_legendPanel->isVisible())
	{
		if (event->type() == QEvent::Resize || event->type() == QEvent::Move || event->type() == QEvent::Show)
			repositionLegend();
	}
	return QWidget::eventFilter(watched, event);
}

bool GeometricModelingPage::originNodeVisible(const QString& syntheticId) const
{
	if (syntheticId == QLatin1String(kOriginPointId))
		return m_originPointVisible;
	if (syntheticId == QLatin1String(kOriginXyId))
		return m_originXyVisible;
	if (syntheticId == QLatin1String(kOriginXzId))
		return m_originXzVisible;
	if (syntheticId == QLatin1String(kOriginYzId))
		return m_originYzVisible;
	if (syntheticId == QLatin1String(kOriginId))
		return m_originPointVisible || m_originXyVisible || m_originXzVisible || m_originYzVisible;
	return true;
}

void GeometricModelingPage::toggleOriginVisibility(const QString& syntheticId)
{
	if (syntheticId == QLatin1String(kOriginId))
	{
		const bool next = !originNodeVisible(kOriginId);
		m_originPointVisible = next;
		m_originXyVisible = next;
		m_originXzVisible = next;
		m_originYzVisible = next;
	}
	else if (syntheticId == QLatin1String(kOriginPointId))
		m_originPointVisible = !m_originPointVisible;
	else if (syntheticId == QLatin1String(kOriginXyId))
		m_originXyVisible = !m_originXyVisible;
	else if (syntheticId == QLatin1String(kOriginXzId))
		m_originXzVisible = !m_originXzVisible;
	else if (syntheticId == QLatin1String(kOriginYzId))
		m_originYzVisible = !m_originYzVisible;
	else
		return;
	refreshFeatureTree();
	emit originVisibilityChanged();
}

void GeometricModelingPage::restoreOriginVisibility(bool point, bool xy, bool xz, bool yz)
{
	m_originPointVisible = point;
	m_originXyVisible = xy;
	m_originXzVisible = xz;
	m_originYzVisible = yz;
}

void GeometricModelingPage::refreshFeatureTree()
{
	m_tree->clear();
	const QString rb = m_features.rollbackAfterFeatureId();
	if (!rb.isEmpty())
	{
		GeomodelingFeatureKind rbKind = GeomodelingFeatureKind::Sketch;
		for (const auto& f : m_features.features())
		{
			if (f.id == rb)
			{
				rbKind = f.kind;
				break;
			}
		}
		auto* banner = new QTreeWidgetItem(m_tree);
		banner->setText(0, i18n(QStringLiteral("Rollback to: %1"), QStringLiteral("回退至: %1"))
							   .arg(featureTreeTitle(rb, rbKind, m_useChinese)));
		banner->setFlags(Qt::ItemIsEnabled);
		QFont f = banner->font(0);
		f.setItalic(true);
		banner->setFont(0, f);
		banner->setForeground(0, QColor(0x0f, 0x76, 0x6e));
	}

	auto attachEyeRow = [this](QTreeWidgetItem* item, const QString& title, bool visible, const QString& toggleId)
	{
		item->setText(0, QString());
		auto* row = new QWidget(m_tree);
		auto* rowLay = new QHBoxLayout(row);
		rowLay->setContentsMargins(4, 0, 10, 0);
		rowLay->setSpacing(4);
		auto* name = new QLabel(title, row);
		name->setAttribute(Qt::WA_TransparentForMouseEvents);
		name->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
		if (!visible)
		{
			QFont font = name->font();
			font.setItalic(true);
			name->setFont(font);
			name->setStyleSheet(QStringLiteral("color: #a1a1aa;"));
		}
		auto* eye = new QToolButton(row);
		eye->setAutoRaise(true);
		eye->setFocusPolicy(Qt::NoFocus);
		eye->setCursor(Qt::PointingHandCursor);
		eye->setFixedSize(26, 26);
		eye->setIconSize(QSize(18, 18));
		eye->setIcon(makeSketchEyeIcon(visible));
		eye->setStyleSheet(QStringLiteral(
			"QToolButton { border: none; background: transparent; padding: 0; margin: 0; }"
			"QToolButton:hover { background: #e2e8f0; border-radius: 4px; }"));
		eye->setToolTip(visible ? i18n(QStringLiteral("Hide"), QStringLiteral("\u9690\u85cf"))
								: i18n(QStringLiteral("Show"), QStringLiteral("\u663e\u793a")));
		connect(eye, &QToolButton::clicked, this, [this, toggleId]() { toggleOriginVisibility(toggleId); });
		rowLay->addWidget(name, 1);
		rowLay->addWidget(eye, 0, Qt::AlignVCenter | Qt::AlignRight);
		m_tree->setItemWidget(item, 0, row);
	};

	// 虚拟原点：不进 Body 历史，仅方便开草图/约束到平面原点
	auto* originRoot = new QTreeWidgetItem(m_tree);
	originRoot->setData(0, Qt::UserRole, QLatin1String(kOriginId));
	originRoot->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	{
		const bool allVis = originNodeVisible(QLatin1String(kOriginId));
		QString title = i18n(QStringLiteral("Origin"), QStringLiteral("原点"));
		if (!allVis)
			title = QStringLiteral("%1 (%2)").arg(title, m_useChinese ? QStringLiteral("\u9690\u85cf")
																	 : QStringLiteral("Hidden"));
		attachEyeRow(originRoot, title, allVis, QLatin1String(kOriginId));
	}
	auto addOriginChild = [&](const QString& id, const QString& en, const QString& zh, bool visible)
	{
		auto* child = new QTreeWidgetItem(originRoot);
		child->setData(0, Qt::UserRole, id);
		child->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
		QString title = i18n(en, zh);
		if (!visible)
			title = QStringLiteral("%1 (%2)").arg(title, m_useChinese ? QStringLiteral("\u9690\u85cf")
																	 : QStringLiteral("Hidden"));
		attachEyeRow(child, title, visible, id);
	};
	addOriginChild(QLatin1String(kOriginPointId), QStringLiteral("Origin Point"), QStringLiteral("原点"),
				   m_originPointVisible);
	addOriginChild(QLatin1String(kOriginXyId), QStringLiteral("Front Plane (XY)"), QStringLiteral("前视平面 (XY)"),
				   m_originXyVisible);
	addOriginChild(QLatin1String(kOriginXzId), QStringLiteral("Top Plane (XZ)"), QStringLiteral("上视平面 (XZ)"),
				   m_originXzVisible);
	addOriginChild(QLatin1String(kOriginYzId), QStringLiteral("Right Plane (YZ)"), QStringLiteral("右视平面 (YZ)"),
				   m_originYzVisible);
	originRoot->setExpanded(true);

	for (const auto& f : m_features.features())
	{
		const QString src = f.name.isEmpty() ? f.id : f.name;
		QString title = featureTreeTitle(src, f.kind, m_useChinese);
		auto* item = new QTreeWidgetItem(m_tree);
		item->setData(0, Qt::UserRole, f.id);
		const bool dimmed = f.suppressed || (f.kind == GeomodelingFeatureKind::Sketch && !f.visible);
		if (f.kind == GeomodelingFeatureKind::Sketch && !f.visible)
			title = QStringLiteral("%1 (%2)").arg(title, m_useChinese ? QStringLiteral("\u9690\u85cf")
																	 : QStringLiteral("Hidden"));

		if (f.kind == GeomodelingFeatureKind::Sketch)
		{
			item->setText(0, QString());
			auto* row = new QWidget(m_tree);
			auto* rowLay = new QHBoxLayout(row);
			rowLay->setContentsMargins(4, 0, 10, 0);
			rowLay->setSpacing(4);
			auto* name = new QLabel(title, row);
			name->setAttribute(Qt::WA_TransparentForMouseEvents);
			name->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
			if (dimmed)
			{
				QFont font = name->font();
				font.setItalic(true);
				name->setFont(font);
				name->setStyleSheet(QStringLiteral("color: #a1a1aa;"));
			}
			auto* eye = new QToolButton(row);
			eye->setAutoRaise(true);
			eye->setFocusPolicy(Qt::NoFocus);
			eye->setCursor(Qt::PointingHandCursor);
			eye->setFixedSize(26, 26);
			eye->setIconSize(QSize(18, 18));
			eye->setIcon(makeSketchEyeIcon(f.visible));
			eye->setStyleSheet(QStringLiteral(
				"QToolButton { border: none; background: transparent; padding: 0; margin: 0; }"
				"QToolButton:hover { background: #e2e8f0; border-radius: 4px; }"));
			eye->setToolTip(f.visible ? i18n(QStringLiteral("Hide"), QStringLiteral("\u9690\u85cf"))
									  : i18n(QStringLiteral("Show"), QStringLiteral("\u663e\u793a")));
			const QString fid = f.id;
			connect(eye, &QToolButton::clicked, this, [this, fid]() { emit sketchVisibilityToggleRequested(fid); });
			rowLay->addWidget(name, 1);
			rowLay->addWidget(eye, 0, Qt::AlignVCenter | Qt::AlignRight);
			m_tree->setItemWidget(item, 0, row);
		}
		else
		{
			item->setText(0, title);
			if (dimmed)
			{
				QFont font = item->font(0);
				font.setItalic(true);
				item->setFont(0, font);
				item->setForeground(0, QColor(0xa1, 0xa1, 0xaa));
			}
		}
		item->setExpanded(true);
	}
}

void GeometricModelingPage::fillClosedSketchCombo(QComboBox* combo, const QString& selectedId)
{
	if (!combo)
		return;
	const QSignalBlocker b(combo);
	combo->clear();
	int pickIdx = -1;
	for (const auto& f : m_features.features())
	{
		if (f.kind != GeomodelingFeatureKind::Sketch)
			continue;
		const QString title = featureTreeTitle(f.name.isEmpty() ? f.id : f.name, f.kind, m_useChinese);
		combo->addItem(title, f.id);
		if (f.id == selectedId)
			pickIdx = combo->count() - 1;
	}
	if (pickIdx >= 0)
		combo->setCurrentIndex(pickIdx);
}

void GeometricModelingPage::setFilletUi(bool active)
{
	setSideToolPanel(active ? SideToolPanel::Fillet : SideToolPanel::None);
}

void GeometricModelingPage::setFilletEdgeCount(int n)
{
	if (m_filletEdgeCount)
		m_filletEdgeCount->setText(i18n(QStringLiteral("%1 edge(s) selected"), QStringLiteral("\u5df2\u9009 %1 \u6761\u8fb9")).arg(n));
}

double GeometricModelingPage::filletRadiusMm() const
{
	return m_filletRadius ? m_filletRadius->value() : 1.0;
}

void GeometricModelingPage::setChamferUi(bool active)
{
	setSideToolPanel(active ? SideToolPanel::Chamfer : SideToolPanel::None);
}

void GeometricModelingPage::setChamferEdgeCount(int n)
{
	if (m_chamferEdgeCount)
		m_chamferEdgeCount->setText(i18n(QStringLiteral("%1 edge(s) selected"), QStringLiteral("\u5df2\u9009 %1 \u6761\u8fb9")).arg(n));
}

double GeometricModelingPage::chamferDistanceMm() const
{
	return m_chamferDist ? m_chamferDist->value() : 1.0;
}

void GeometricModelingPage::setRevolveUi(bool active, bool cutMode)
{
	m_revolveCutMode = cutMode;
	if (m_revolveTitle)
		m_revolveTitle->setText(cutMode ? i18n(QStringLiteral("Revolve Cut"), QStringLiteral("\u65cb\u8f6c\u5207\u9664"))
										: i18n(QStringLiteral("Revolve"), QStringLiteral("\u65cb\u8f6c")));
	if (!active && m_revolveStatus)
		m_revolveStatus->clear();
	setSideToolPanel(active ? SideToolPanel::Revolve : SideToolPanel::None);
}

void GeometricModelingPage::fillRevolveSketchCombo(const QString& selectedId)
{
	fillClosedSketchCombo(m_revolveSketchCombo, selectedId);
}

QString GeometricModelingPage::revolveSketchId() const
{
	return m_revolveSketchCombo ? m_revolveSketchCombo->currentData().toString() : QString();
}

double GeometricModelingPage::revolveAngleDeg() const
{
	return m_revolveAngle ? m_revolveAngle->value() : 360.0;
}

void GeometricModelingPage::setRevolveStatus(const QString& text)
{
	if (!m_revolveStatus)
		return;
	m_revolveStatus->setText(text);
	m_revolveStatus->setVisible(!text.isEmpty());
}

void GeometricModelingPage::setPatternUi(bool active)
{
	if (active)
		fillPatternSourceCombo();
	setSideToolPanel(active ? SideToolPanel::Pattern : SideToolPanel::None);
}

void GeometricModelingPage::fillPatternSourceCombo()
{
	if (!m_patternSource)
		return;
	const QSignalBlocker b(m_patternSource);
	m_patternSource->clear();
	m_patternSource->addItem(i18n(QStringLiteral("Entire tip (current body)"), QStringLiteral("整个实体（当前 tip）")),
							 QString());
	for (const auto& f : m_features.features())
	{
		if (f.kind == GeomodelingFeatureKind::Sketch || f.kind == GeomodelingFeatureKind::DatumPlane || f.suppressed)
			continue;
		const QString title = featureTreeTitle(f.name.isEmpty() ? f.id : f.name, f.kind, m_useChinese);
		m_patternSource->addItem(title, f.id);
	}
}

QString GeometricModelingPage::patternSourceFeatureId() const
{
	if (!m_patternSource || m_patternSource->currentIndex() < 0)
		return {};
	return m_patternSource->currentData().toString();
}

int GeometricModelingPage::patternCount() const
{
	return m_patternCount ? static_cast<int>(m_patternCount->value()) : 2;
}

double GeometricModelingPage::patternDxMm() const
{
	return m_patternDx ? m_patternDx->value() : 10.0;
}

double GeometricModelingPage::patternDyMm() const
{
	return m_patternDy ? m_patternDy->value() : 0.0;
}

double GeometricModelingPage::patternDzMm() const
{
	return m_patternDz ? m_patternDz->value() : 0.0;
}

void GeometricModelingPage::setMirror3dUi(bool active)
{
	setSideToolPanel(active ? SideToolPanel::Mirror3D : SideToolPanel::None);
}

bool GeometricModelingPage::mirror3dKeepOriginal() const
{
	return !m_mirror3dKeep || m_mirror3dKeep->isChecked();
}

int GeometricModelingPage::mirror3dPlaneIndex() const
{
	return m_mirror3dPlane ? m_mirror3dPlane->currentData().toInt() : 0;
}

void GeometricModelingPage::setLoftUi(bool active, bool cutMode)
{
	m_loftCutMode = cutMode;
	if (m_loftTitle)
		m_loftTitle->setText(cutMode ? i18n(QStringLiteral("Loft Cut"), QStringLiteral("\u653e\u6837\u5207\u9664"))
									 : i18n(QStringLiteral("Loft"), QStringLiteral("\u653e\u6837")));
	if (!active && m_loftStatus)
		m_loftStatus->clear();
	setSideToolPanel(active ? SideToolPanel::Loft : SideToolPanel::None);
}

void GeometricModelingPage::fillLoftSketchCombos(const QString& selectedA, const QString& selectedB)
{
	fillClosedSketchCombo(m_loftSketchA, selectedA);
	fillClosedSketchCombo(m_loftSketchB, selectedB);
}

QString GeometricModelingPage::loftSketchAId() const
{
	return m_loftSketchA ? m_loftSketchA->currentData().toString() : QString();
}

QString GeometricModelingPage::loftSketchBId() const
{
	return m_loftSketchB ? m_loftSketchB->currentData().toString() : QString();
}

void GeometricModelingPage::setLoftStatus(const QString& text)
{
	if (!m_loftStatus)
		return;
	m_loftStatus->setText(text);
	m_loftStatus->setVisible(!text.isEmpty());
}

void GeometricModelingPage::setShellUi(bool active)
{
	setSideToolPanel(active ? SideToolPanel::Shell : SideToolPanel::None);
}

double GeometricModelingPage::shellThicknessMm() const
{
	return m_shellThickness ? m_shellThickness->value() : 1.0;
}

void GeometricModelingPage::setShellFaceCount(int n)
{
	if (m_shellFaceCount)
		m_shellFaceCount->setText(i18n(QStringLiteral("%1 face(s) selected"), QStringLiteral("\u5df2\u9009 %1 \u4e2a\u9762")).arg(n));
}

void GeometricModelingPage::setShellStatus(const QString& text)
{
	if (!m_shellStatus)
		return;
	m_shellStatus->setText(text);
	m_shellStatus->setVisible(!text.isEmpty());
}

void GeometricModelingPage::setDraftUi(bool active)
{
	setSideToolPanel(active ? SideToolPanel::Draft : SideToolPanel::None);
}

double GeometricModelingPage::draftAngleDeg() const
{
	return m_draftAngle ? m_draftAngle->value() : 1.0;
}

void GeometricModelingPage::setDraftFaceCount(int n)
{
	if (m_draftFaceCount)
		m_draftFaceCount->setText(i18n(QStringLiteral("%1 face(s) selected"), QStringLiteral("\u5df2\u9009 %1 \u4e2a\u9762")).arg(n));
}

void GeometricModelingPage::setDraftStatus(const QString& text)
{
	if (!m_draftStatus)
		return;
	m_draftStatus->setText(text);
	m_draftStatus->setVisible(!text.isEmpty());
}

void GeometricModelingPage::setDraftNeutralPlane(const PluginSketchPlane& plane)
{
	m_draftNeutralPlane = plane;
	m_hasDraftNeutral = plane.isPlanar;
	if (m_draftNeutralLabel)
	{
		m_draftNeutralLabel->setText(m_hasDraftNeutral
										 ? i18n(QStringLiteral("Neutral: planar face"), QStringLiteral("中性面：已选平面"))
										 : i18n(QStringLiteral("Neutral: default XY"), QStringLiteral("中性面：默认 XY")));
	}
}

void GeometricModelingPage::clearDraftNeutralPlane()
{
	m_draftNeutralPlane = {};
	m_hasDraftNeutral = false;
	if (m_draftNeutralLabel)
		m_draftNeutralLabel->setText(i18n(QStringLiteral("Neutral: default XY"), QStringLiteral("中性面：默认 XY")));
}
