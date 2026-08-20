/// @file AssemblyMatePanel.cpp
/// @brief Insert 装配一次定位面板

#include "AssemblyMatePanel.h"

#include "ApplicationStyle.h"
#include "BackendDataBase.h"
#include "DocumentPage.h"
#include "MainWindow.h"
#include "MainWindowRobotHost.h"
#include "OsgWidget.h"
#include "PickTypes.h"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>

namespace
{
const char* kKindEn[] = {"Coincident", "Parallel", "Perpendicular", "Tangent",
						 "Concentric", "Lock",     "Distance",      "Angle"};
const char* kKindZh[] = {"重合", "平行", "垂直", "相切", "同轴心", "锁定", "距离", "角度"};

geoalgo::AssemblyMateKind kindFromId(const int id)
{
	return static_cast<geoalgo::AssemblyMateKind>(id);
}

} // namespace

AssemblyMatePanel::AssemblyMatePanel(MainWindow* mw, QWidget* parent) : QWidget(parent), m_mw(mw)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(8, 8, 8, 8);
	root->setSpacing(6);

	m_kindTitle = new QLabel(this);
	root->addWidget(m_kindTitle);
	m_kindGroup = new QButtonGroup(this);
	m_kindGroup->setExclusive(true);
	auto* kindGrid = new QGridLayout();
	kindGrid->setSpacing(4);
	for (int i = 0; i < 8; ++i)
	{
		auto* btn = new QPushButton(this);
		btn->setCheckable(true);
		m_kindGroup->addButton(btn, i);
		kindGrid->addWidget(btn, i / 4, i % 4);
	}
	m_kindGroup->button(0)->setChecked(true);
	root->addLayout(kindGrid);

	auto* valueRow = new QHBoxLayout();
	m_distanceLabel = new QLabel(this);
	m_distanceSpin = new QDoubleSpinBox(this);
	m_distanceSpin->setRange(-1.0e6, 1.0e6);
	m_distanceSpin->setDecimals(3);
	m_distanceSpin->setSuffix(QStringLiteral(" mm"));
	m_angleLabel = new QLabel(this);
	m_angleSpin = new QDoubleSpinBox(this);
	m_angleSpin->setRange(0.0, 180.0);
	m_angleSpin->setDecimals(2);
	m_angleSpin->setValue(90.0);
	m_angleSpin->setSuffix(QStringLiteral(" °"));
	valueRow->addWidget(m_distanceLabel);
	valueRow->addWidget(m_distanceSpin);
	valueRow->addWidget(m_angleLabel);
	valueRow->addWidget(m_angleSpin);
	root->addLayout(valueRow);

	m_alignTitle = new QLabel(this);
	root->addWidget(m_alignTitle);
	auto* alignRow = new QHBoxLayout();
	m_antiAlignRadio = new QRadioButton(this);
	m_alignRadio = new QRadioButton(this);
	m_antiAlignRadio->setChecked(true);
	alignRow->addWidget(m_antiAlignRadio);
	alignRow->addWidget(m_alignRadio);
	alignRow->addStretch(1);
	root->addLayout(alignRow);

	auto* faceGrid = new QGridLayout();
	m_pickFace1Btn = new QPushButton(this);
	m_pickFace2Btn = new QPushButton(this);
	m_face1Label = new QLabel(this);
	m_face2Label = new QLabel(this);
	faceGrid->addWidget(m_pickFace1Btn, 0, 0);
	faceGrid->addWidget(m_face1Label, 0, 1);
	faceGrid->addWidget(m_pickFace2Btn, 1, 0);
	faceGrid->addWidget(m_face2Label, 1, 1);
	root->addLayout(faceGrid);

	m_statusLabel = new QLabel(this);
	m_statusLabel->setWordWrap(true);
	root->addWidget(m_statusLabel);

	auto* btnRow = new QHBoxLayout();
	btnRow->addStretch(1);
	m_okBtn = new QPushButton(this);
	m_cancelBtn = new QPushButton(this);
	m_okBtn->setEnabled(false);
	btnRow->addWidget(m_okBtn);
	btnRow->addWidget(m_cancelBtn);
	root->addLayout(btnRow);
	root->addStretch(1);

	connect(m_kindGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this,
			[this](int)
			{
				updateValueEditors();
				previewFromSnapshot();
			});
	connect(m_distanceSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[this](double) { previewFromSnapshot(); });
	connect(m_angleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[this](double) { previewFromSnapshot(); });
	connect(m_antiAlignRadio, &QRadioButton::toggled, this,
			[this](bool on)
			{
				if (on)
				{
					previewFromSnapshot();
				}
			});
	connect(m_alignRadio, &QRadioButton::toggled, this,
			[this](bool on)
			{
				if (on)
				{
					previewFromSnapshot();
				}
			});
	connect(m_pickFace1Btn, &QPushButton::clicked, this, [this]() { startFacePick(0); });
	connect(m_pickFace2Btn, &QPushButton::clicked, this, [this]() { startFacePick(1); });
	connect(m_okBtn, &QPushButton::clicked, this,
			[this]()
			{
				if (!m_mw)
				{
					return;
				}
				cloudsim::host::DocumentHost* host = m_mw->currentDocumentHost();
				if (!host)
				{
					return;
				}
				QString err;
				if (!cloudsim::host::applyAssemblyMate(*host, m_face1, m_face2, currentParams(),
													   m_haveSnapshot ? &m_movingSnapshot : nullptr, true, &err))
				{
					setStatus(err, true);
					return;
				}
				m_previewed = false;
				m_haveSnapshot = false;
				resetForNextMate();
			});
	connect(m_cancelBtn, &QPushButton::clicked, this,
			[this]()
			{
				endSession();
				if (m_mw && m_mw->m_assemblyMateDock)
				{
					m_mw->m_assemblyMateDock->hide();
				}
			});

	updateValueEditors();
	applyLanguage();
}

void AssemblyMatePanel::applyLanguage()
{
	if (!m_mw)
	{
		return;
	}
	const auto tr = [this](const QString& en, const QString& zh) { return m_mw->i18n(en, zh); };
	m_kindTitle->setText(tr(QStringLiteral("Mate type"), QStringLiteral("配合类型")));
	for (int i = 0; i < 8; ++i)
	{
		if (QAbstractButton* b = m_kindGroup->button(i))
		{
			b->setText(tr(QString::fromUtf8(kKindEn[i]), QString::fromUtf8(kKindZh[i])));
		}
	}
	m_distanceLabel->setText(tr(QStringLiteral("Distance"), QStringLiteral("距离")));
	m_angleLabel->setText(tr(QStringLiteral("Angle"), QStringLiteral("角度")));
	m_alignTitle->setText(tr(QStringLiteral("Alignment"), QStringLiteral("对齐")));
	m_antiAlignRadio->setText(tr(QStringLiteral("Anti-aligned"), QStringLiteral("反对齐")));
	m_alignRadio->setText(tr(QStringLiteral("Aligned"), QStringLiteral("对齐")));
	m_pickFace1Btn->setText(tr(QStringLiteral("Face 1 (grounded)"), QStringLiteral("面 1（固定）")));
	m_pickFace2Btn->setText(tr(QStringLiteral("Face 2 (moving)"), QStringLiteral("面 2（动件）")));
	m_okBtn->setText(tr(QStringLiteral("OK"), QStringLiteral("确定")));
	m_cancelBtn->setText(tr(QStringLiteral("Cancel"), QStringLiteral("取消")));
	if (m_face1.backendId.empty())
	{
		m_face1Label->setText(tr(QStringLiteral("Not selected"), QStringLiteral("未选择")));
	}
	if (m_face2.backendId.empty())
	{
		m_face2Label->setText(tr(QStringLiteral("Not selected"), QStringLiteral("未选择")));
	}
}

void AssemblyMatePanel::beginSession()
{
	restoreMovingIfPreviewed();
	stopFacePick();
	m_face1 = {};
	m_face2 = {};
	m_face1Verts.clear();
	m_haveSnapshot = false;
	m_previewed = false;
	m_okBtn->setEnabled(false);
	applyLanguage();
	setStatus(QString(), false);
}

void AssemblyMatePanel::endSession()
{
	restoreMovingIfPreviewed();
	stopFacePick();
	if (m_mw)
	{
		if (DocumentPage* page = m_mw->currentPage())
		{
			if (OsgWidget* osg = page->osgWidget())
			{
				osg->hidePinnedMeshFaceHighlight();
				osg->hideMeshElementHighlight();
				osg->setCrossObjectMeshPick(false);
				osg->setMeshFacePickMode(false);
			}
		}
	}
}

void AssemblyMatePanel::interruptPicking()
{
	restoreMovingIfPreviewed();
	stopFacePick();
}

void AssemblyMatePanel::resetForNextMate()
{
	stopFacePick();
	m_face1 = {};
	m_face2 = {};
	m_face1Verts.clear();
	m_haveSnapshot = false;
	m_previewed = false;
	m_okBtn->setEnabled(false);
	applyLanguage();
	if (m_mw)
	{
		if (DocumentPage* page = m_mw->currentPage())
		{
			if (OsgWidget* osg = page->osgWidget())
			{
				osg->hidePinnedMeshFaceHighlight();
				osg->hideMeshElementHighlight();
			}
		}
	}
	startFacePick(0);
	if (m_mw)
	{
		setStatus(m_mw->i18n(QStringLiteral("Applied. Pick the next grounded face"),
							 QStringLiteral("已应用，继续点选固定面")),
				  false);
	}
}

void AssemblyMatePanel::updateValueEditors()
{
	const int id = m_kindGroup->checkedId();
	m_distanceSpin->setEnabled(id == static_cast<int>(geoalgo::AssemblyMateKind::Distance));
	m_angleSpin->setEnabled(id == static_cast<int>(geoalgo::AssemblyMateKind::Angle));
}

geoalgo::AssemblyMateParams AssemblyMatePanel::currentParams() const
{
	geoalgo::AssemblyMateParams p;
	p.kind = kindFromId(m_kindGroup->checkedId());
	p.alignment = m_antiAlignRadio->isChecked() ? geoalgo::AssemblyMateAlignment::AntiAligned
												: geoalgo::AssemblyMateAlignment::Aligned;
	p.distanceMm = m_distanceSpin->value();
	p.angleDeg = m_angleSpin->value();
	return p;
}

void AssemblyMatePanel::setStatus(const QString& text, const bool error)
{
	m_statusLabel->setText(text);
	if (error)
	{
		const ApplicationStyle::ThemeTokens t = ApplicationStyle::tokens(ApplicationStyle::loadSavedTheme());
		m_statusLabel->setStyleSheet(QStringLiteral("color:%1;").arg(t.danger.name()));
	}
	else
	{
		m_statusLabel->setStyleSheet(QString());
	}
}

void AssemblyMatePanel::startFacePick(const int slot)
{
	if (!m_mw)
	{
		return;
	}
	DocumentPage* page = m_mw->currentPage();
	OsgWidget* osg = page ? page->osgWidget() : nullptr;
	if (!osg || !m_mw->m_robotHost)
	{
		setStatus(m_mw->i18n(QStringLiteral("No 3D view"), QStringLiteral("没有三维视图")), true);
		return;
	}
	m_pickSlot = slot;
	osg->setObjectSelectionMode(false);
	osg->setMeshLinePickMode(false);
	osg->setCrossObjectMeshPick(true);
	osg->setMeshFacePickMode(true);
	osg->syncSelectionForBackendId(std::string());
	osg->setSelectionActive(false);
	m_mw->m_robotHost->setMeshPickCommittedHandler(
		[this](const PickResult& pick, const PickKind kind) { onPickCommitted(pick, kind); });
	setStatus(m_mw->i18n(QStringLiteral("Pick a B-rep face in the view"), QStringLiteral("在视口点选 B-rep 面")),
			  false);
}

void AssemblyMatePanel::stopFacePick()
{
	m_pickSlot = -1;
	if (m_mw && m_mw->m_robotHost)
	{
		m_mw->m_robotHost->clearMeshPickCommittedHandler();
	}
	if (m_mw)
	{
		if (DocumentPage* page = m_mw->currentPage())
		{
			if (OsgWidget* osg = page->osgWidget())
			{
				osg->setCrossObjectMeshPick(false);
				osg->setMeshFacePickMode(false);
			}
		}
	}
}

void AssemblyMatePanel::onPickCommitted(const PickResult& pick, const PickKind kind)
{
	if (m_pickSlot < 0 || kind != PickKind::MeshFace)
	{
		return;
	}
	if (!pick.hit || !pick.brepNativePick || pick.brepFaceIndex < 0 || pick.backendId.empty())
	{
		setStatus(m_mw->i18n(QStringLiteral("Need a B-rep face"), QStringLiteral("需要 B-rep 面")), true);
		return;
	}
	cloudsim::host::DocumentHost* host = m_mw->currentDocumentHost();
	if (!host)
	{
		return;
	}
	cloudsim::host::AssemblyMateFaceRef resolved;
	QString err;
	if (!cloudsim::host::resolveAssemblyMatePick(*host, pick.backendId, pick.brepFaceIndex, pick.worldPoint.x(),
												 pick.worldPoint.y(), pick.worldPoint.z(), resolved, &err))
	{
		setStatus(err, true);
		return;
	}

	const auto nameOf = [host](const std::string& id) -> QString
	{
		const auto obj = host->findObject(id);
		if (!obj)
		{
			return QString::fromStdString(id);
		}
		const QString n = QString::fromStdString(obj->name());
		return n.isEmpty() ? QString::fromStdString(id) : n;
	};

	if (m_pickSlot == 0)
	{
		restoreMovingIfPreviewed();
		m_face1 = resolved;
		m_face1Verts = pick.meshFaceVertsWorld;
		m_face1Label->setText(nameOf(m_face1.backendId));
		if (DocumentPage* page = m_mw->currentPage())
		{
			if (OsgWidget* osg = page->osgWidget())
			{
				osg->showPinnedMeshFaceHighlight(m_face1Verts);
			}
		}
	}
	else
	{
		if (resolved.backendId == m_face1.backendId)
		{
			setStatus(m_mw->i18n(QStringLiteral("The two faces must belong to different parts"),
								 QStringLiteral("两面必须来自不同对象")),
					  true);
			return;
		}
		restoreMovingIfPreviewed();
		m_face2 = resolved;
		m_face2Label->setText(nameOf(m_face2.backendId));
		m_haveSnapshot = cloudsim::host::snapshotBackendWorldMatrix(*host, m_face2.backendId, m_movingSnapshot);
	}

	stopFacePick();
	m_okBtn->setEnabled(!m_face1.backendId.empty() && !m_face2.backendId.empty());
	setStatus(QString(), false);
	previewFromSnapshot();
}

void AssemblyMatePanel::restoreMovingIfPreviewed()
{
	if (!m_previewed || !m_haveSnapshot || !m_mw || m_face2.backendId.empty())
	{
		m_previewed = false;
		return;
	}
	if (cloudsim::host::DocumentHost* host = m_mw->currentDocumentHost())
	{
		(void)cloudsim::host::restoreBackendWorldMatrix(*host, m_face2.backendId, m_movingSnapshot, nullptr);
	}
	m_previewed = false;
}

void AssemblyMatePanel::previewFromSnapshot()
{
	if (m_face1.backendId.empty() || m_face2.backendId.empty() || !m_mw)
	{
		return;
	}
	cloudsim::host::DocumentHost* host = m_mw->currentDocumentHost();
	if (!host)
	{
		return;
	}
	QString err;
	if (!cloudsim::host::applyAssemblyMate(*host, m_face1, m_face2, currentParams(),
										   m_haveSnapshot ? &m_movingSnapshot : nullptr, false, &err))
	{
		setStatus(err, true);
		return;
	}
	m_previewed = true;
	setStatus(QString(), false);
}
