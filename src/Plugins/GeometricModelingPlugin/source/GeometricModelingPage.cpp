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
	return zh ? QStringLiteral("%1_%2").arg(baseZh, num) : QStringLiteral("%1_%2").arg(baseEn, num);
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
	m_tree->setRootIsDecorated(false);
	m_tree->setUniformRowHeights(true);
	m_tree->setIndentation(8);
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
				if (!id.isEmpty())
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
				if (!id.isEmpty())
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
				QString fid;
				bool sketchVisible = true;
				bool isSketch = false;
				if (item)
				{
					fid = item->data(0, Qt::UserRole).toString();
					if (!fid.isEmpty())
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
				else if (chosen == delAct && !fid.isEmpty())
					emit featureDeleteRequested(fid);
				else if (chosen == rollbackAct && !fid.isEmpty())
					emit featureRollbackRequested(fid);
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
	m_sweepPathCaption = new QLabel(QStringLiteral("\u8def\u5f84\u8349\u56fe"), m_pageSweep);
	swLay->addWidget(m_sweepPathCaption);
	m_sweepPathCombo = new QComboBox(m_pageSweep);
	swLay->addWidget(m_sweepPathCombo);
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
	connect(m_btnSweepOk, &QPushButton::clicked, this, &GeometricModelingPage::confirmSweepRequested);
	connect(m_btnSweepCancel, &QPushButton::clicked, this, &GeometricModelingPage::cancelSweepRequested);

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
	const bool through = end == GeomodelingExtrudeEnd::ThroughAll;
	if (m_length)
		m_length->setEnabled(!upTo && !through);
	if (m_btnPickFace)
		m_btnPickFace->setEnabled(upTo);
	if (m_upToFaceStatus)
		m_upToFaceStatus->setVisible(upTo);
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
	if (m_endCondition && m_endCondition->count() >= 4)
	{
		m_endCondition->setItemText(0, i18n(QStringLiteral("Blind"), QStringLiteral("定长")));
		m_endCondition->setItemText(1, i18n(QStringLiteral("Up to face"), QStringLiteral("到面")));
		m_endCondition->setItemText(2, i18n(QStringLiteral("Mid plane"), QStringLiteral("对称")));
		m_endCondition->setItemText(3, i18n(QStringLiteral("Through all"), QStringLiteral("贯通")));
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
