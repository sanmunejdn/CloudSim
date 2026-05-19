#include "RobotFrameSettingsWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QListWidget>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace
{
QDoubleSpinBox* makeSpin(QWidget* parent, const double lo, const double hi)
{
	auto* s = new QDoubleSpinBox(parent);
	s->setRange(lo, hi);
	s->setDecimals(3);
	s->setSingleStep(0.1);
	return s;
}

void setRigidToSpins(const RobotCoordinate::RobotRigidFrame& f, QDoubleSpinBox* pos[3], QDoubleSpinBox* euler[3])
{
	for (int i = 0; i < 3; ++i)
	{
		pos[i]->setValue(f.positionMm[i]);
		euler[i]->setValue(f.eulerDeg[i]);
	}
}

void readRigidFromSpins(RobotCoordinate::RobotRigidFrame& f, QDoubleSpinBox* pos[3], QDoubleSpinBox* euler[3])
{
	for (int i = 0; i < 3; ++i)
	{
		f.positionMm[i] = pos[i]->value();
		f.eulerDeg[i] = euler[i]->value();
	}
}
} // namespace

RobotFrameSettingsWidget::RobotFrameSettingsWidget(QWidget* parent)
	: QWidget(parent)
{
	m_framesDebounceTimer = new QTimer(this);
	m_framesDebounceTimer->setSingleShot(true);
	m_framesDebounceTimer->setInterval(80);
	connect(m_framesDebounceTimer, &QTimer::timeout, this, &RobotFrameSettingsWidget::onFramesChangeDebounce);

	auto* root = new QVBoxLayout(this);

	auto* toolGroup = new QGroupBox(QStringLiteral("Tool frames (flange)"), this);
	auto* toolLayout = new QVBoxLayout(toolGroup);
	m_toolList = new QListWidget(toolGroup);
	toolLayout->addWidget(m_toolList);
	auto* toolForm = new QFormLayout;
	m_flangeLinkCombo = new QComboBox(toolGroup);
	toolForm->addRow(QStringLiteral("Flange link"), m_flangeLinkCombo);
	for (int i = 0; i < 3; ++i)
	{
		m_toolPos[i] = makeSpin(toolGroup, -5000.0, 5000.0);
		m_toolEuler[i] = makeSpin(toolGroup, -360.0, 360.0);
	}
	toolForm->addRow(QStringLiteral("X (mm)"), m_toolPos[0]);
	toolForm->addRow(QStringLiteral("Y (mm)"), m_toolPos[1]);
	toolForm->addRow(QStringLiteral("Z (mm)"), m_toolPos[2]);
	toolForm->addRow(QStringLiteral("Rx (deg)"), m_toolEuler[0]);
	toolForm->addRow(QStringLiteral("Ry (deg)"), m_toolEuler[1]);
	toolForm->addRow(QStringLiteral("Rz (deg)"), m_toolEuler[2]);
	toolLayout->addLayout(toolForm);
	auto* toolBtnRow = new QHBoxLayout;
	m_addToolBtn = new QPushButton(QStringLiteral("Add"), toolGroup);
	m_removeToolBtn = new QPushButton(QStringLiteral("Remove"), toolGroup);
	m_duplicateToolBtn = new QPushButton(QStringLiteral("Duplicate"), toolGroup);
	m_setActiveToolBtn = new QPushButton(QStringLiteral("Set active"), toolGroup);
	toolBtnRow->addWidget(m_addToolBtn);
	toolBtnRow->addWidget(m_removeToolBtn);
	toolBtnRow->addWidget(m_duplicateToolBtn);
	toolBtnRow->addWidget(m_setActiveToolBtn);
	toolLayout->addLayout(toolBtnRow);
	auto* toolCapRow = new QHBoxLayout;
	m_captureToolBtn = new QPushButton(QStringLiteral("Capture from TCP"), toolGroup);
	m_resetToolBtn = new QPushButton(QStringLiteral("Reset to flange"), toolGroup);
	toolCapRow->addWidget(m_captureToolBtn);
	toolCapRow->addWidget(m_resetToolBtn);
	toolLayout->addLayout(toolCapRow);
	root->addWidget(toolGroup);

	auto* userGroup = new QGroupBox(QStringLiteral("User frames (base)"), this);
	auto* userLayout = new QVBoxLayout(userGroup);
	m_userList = new QListWidget(userGroup);
	userLayout->addWidget(m_userList);
	auto* userForm = new QFormLayout;
	for (int i = 0; i < 3; ++i)
	{
		m_userPos[i] = makeSpin(userGroup, -100000.0, 100000.0);
		m_userEuler[i] = makeSpin(userGroup, -360.0, 360.0);
	}
	userForm->addRow(QStringLiteral("X (mm)"), m_userPos[0]);
	userForm->addRow(QStringLiteral("Y (mm)"), m_userPos[1]);
	userForm->addRow(QStringLiteral("Z (mm)"), m_userPos[2]);
	userForm->addRow(QStringLiteral("Rx (deg)"), m_userEuler[0]);
	userForm->addRow(QStringLiteral("Ry (deg)"), m_userEuler[1]);
	userForm->addRow(QStringLiteral("Rz (deg)"), m_userEuler[2]);
	userLayout->addLayout(userForm);
	auto* userBtnRow = new QHBoxLayout;
	m_addUserBtn = new QPushButton(QStringLiteral("Add"), userGroup);
	m_removeUserBtn = new QPushButton(QStringLiteral("Remove"), userGroup);
	m_duplicateUserBtn = new QPushButton(QStringLiteral("Duplicate"), userGroup);
	m_setActiveUserBtn = new QPushButton(QStringLiteral("Set active"), userGroup);
	userBtnRow->addWidget(m_addUserBtn);
	userBtnRow->addWidget(m_removeUserBtn);
	userBtnRow->addWidget(m_duplicateUserBtn);
	userBtnRow->addWidget(m_setActiveUserBtn);
	userLayout->addLayout(userBtnRow);
	userLayout->addWidget(new QPushButton(QStringLiteral("Capture origin+pose"), userGroup));
	m_captureUserBtn = qobject_cast<QPushButton*>(userLayout->itemAt(userLayout->count() - 1)->widget());
	root->addWidget(userGroup);

	auto* showGroup = new QGroupBox(QStringLiteral("3D display"), this);
	auto* showLayout = new QVBoxLayout(showGroup);
	m_showToolCheck = new QCheckBox(QStringLiteral("Show tool frames"), showGroup);
	m_showUserCheck = new QCheckBox(QStringLiteral("Show user frames"), showGroup);
	m_showToolCheck->setChecked(true);
	m_showUserCheck->setChecked(true);
	showLayout->addWidget(m_showToolCheck);
	showLayout->addWidget(m_showUserCheck);
	root->addWidget(showGroup);
	root->addStretch();

	connect(m_flangeLinkCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
		if (m_blockSignals)
		{
			return;
		}
		saveToolFieldsToSelection();
		const int row = m_toolList->currentRow();
		if (row >= 0 && row < static_cast<int>(m_frames.toolFrames.size()))
		{
			if (m_flangeLinkCombo->currentIndex() >= 0)
			{
				m_frames.toolFrames[static_cast<size_t>(row)].flangeLinkName =
					m_flangeLinkCombo->currentText().toStdString();
			}
		}
		scheduleFramesChanged();
	});
	for (int i = 0; i < 3; ++i)
	{
		connect(m_toolPos[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&RobotFrameSettingsWidget::onToolFieldChanged);
		connect(m_toolEuler[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&RobotFrameSettingsWidget::onToolFieldChanged);
		connect(m_userPos[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&RobotFrameSettingsWidget::onUserFieldChanged);
		connect(m_userEuler[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&RobotFrameSettingsWidget::onUserFieldChanged);
	}
	connect(m_toolList, &QListWidget::currentRowChanged, this, &RobotFrameSettingsWidget::onToolListSelectionChanged);
	connect(m_userList, &QListWidget::currentRowChanged, this, &RobotFrameSettingsWidget::onUserListSelectionChanged);
	connect(m_addToolBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::onAddToolFrame);
	connect(m_removeToolBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::onRemoveToolFrame);
	connect(m_duplicateToolBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::onDuplicateToolFrame);
	connect(m_setActiveToolBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::onSetActiveToolFrame);
	connect(m_addUserBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::onAddUserFrame);
	connect(m_removeUserBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::onRemoveUserFrame);
	connect(m_duplicateUserBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::onDuplicateUserFrame);
	connect(m_setActiveUserBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::onSetActiveUserFrame);
	connect(m_captureToolBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::captureToolFromTcpRequested);
	connect(m_resetToolBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::resetToolFrameRequested);
	connect(m_captureUserBtn, &QPushButton::clicked, this, &RobotFrameSettingsWidget::captureUserFrameFromTcpRequested);
	connect(m_showToolCheck, &QCheckBox::toggled, this, &RobotFrameSettingsWidget::onShowToggled);
	connect(m_showUserCheck, &QCheckBox::toggled, this, &RobotFrameSettingsWidget::onShowToggled);
}

void RobotFrameSettingsWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	for (QGroupBox* gb : findChildren<QGroupBox*>())
	{
		if (!gb)
		{
			continue;
		}
		const QString t = gb->title();
		if (t.startsWith(QStringLiteral("Tool")))
		{
			gb->setTitle(chinese ? QStringLiteral("工具坐标系（法兰）") : QStringLiteral("Tool frames (flange)"));
		}
		else if (t.startsWith(QStringLiteral("User")))
		{
			gb->setTitle(chinese ? QStringLiteral("用户坐标系（基座）") : QStringLiteral("User frames (base)"));
		}
		else if (t.startsWith(QStringLiteral("3D")))
		{
			gb->setTitle(chinese ? QStringLiteral("三维显示") : QStringLiteral("3D display"));
		}
	}
	m_addToolBtn->setText(chinese ? QStringLiteral("添加") : QStringLiteral("Add"));
	m_removeToolBtn->setText(chinese ? QStringLiteral("删除") : QStringLiteral("Remove"));
	m_duplicateToolBtn->setText(chinese ? QStringLiteral("复制") : QStringLiteral("Duplicate"));
	m_setActiveToolBtn->setText(chinese ? QStringLiteral("设为当前") : QStringLiteral("Set active"));
	m_captureToolBtn->setText(chinese ? QStringLiteral("从当前 TCP 捕获") : QStringLiteral("Capture from TCP"));
	m_resetToolBtn->setText(chinese ? QStringLiteral("与末端重合") : QStringLiteral("Reset to flange"));
	m_captureUserBtn->setText(chinese ? QStringLiteral("从 TCP 捕获用户系") : QStringLiteral("Capture origin+pose"));
	m_showToolCheck->setText(chinese ? QStringLiteral("显示工具坐标系") : QStringLiteral("Show tool frames"));
	m_showUserCheck->setText(chinese ? QStringLiteral("显示用户坐标系") : QStringLiteral("Show user frames"));
}

void RobotFrameSettingsWidget::setLinkNameOptions(const QStringList& linkNames)
{
	m_linkNames = linkNames;
	m_blockSignals = true;
	m_flangeLinkCombo->clear();
	m_flangeLinkCombo->addItem(QString());
	m_flangeLinkCombo->addItems(linkNames);
	m_blockSignals = false;
}

void RobotFrameSettingsWidget::setCoordinateFrames(const RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	m_blockSignals = true;
	m_frames = frames;
	m_showToolCheck->setChecked(m_frames.showToolFrameInScene);
	m_showUserCheck->setChecked(m_frames.showUserFramesInScene);
	rebuildToolFrameList();
	rebuildUserFrameList();
	loadToolFieldsFromSelection();
	loadUserFieldsFromSelection();
	m_blockSignals = false;
}

RobotCoordinate::RobotCoordinateFrameSet RobotFrameSettingsWidget::coordinateFrames() const
{
	RobotCoordinate::RobotCoordinateFrameSet out = m_frames;
	out.showToolFrameInScene = m_showToolCheck->isChecked();
	out.showUserFramesInScene = m_showUserCheck->isChecked();
	return out;
}

void RobotFrameSettingsWidget::rebuildToolFrameList()
{
	m_toolList->clear();
	for (const RobotCoordinate::RobotToolFrame& tf : m_frames.toolFrames)
	{
		QString label = QString::fromStdString(tf.name);
		if (tf.id == m_frames.activeToolFrameId)
		{
			label += QStringLiteral(" *");
		}
		m_toolList->addItem(label);
	}
	if (m_toolList->count() > 0 && m_toolList->currentRow() < 0)
	{
		int row = 0;
		for (int i = 0; i < m_toolList->count(); ++i)
		{
			if (m_frames.toolFrames[static_cast<size_t>(i)].id == m_frames.activeToolFrameId)
			{
				row = i;
				break;
			}
		}
		m_toolList->setCurrentRow(row);
	}
}

void RobotFrameSettingsWidget::rebuildUserFrameList()
{
	m_userList->clear();
	for (const RobotCoordinate::RobotUserFrame& uf : m_frames.userFrames)
	{
		QString label = QString::fromStdString(uf.name);
		if (uf.id == m_frames.activeUserFrameId)
		{
			label += QStringLiteral(" *");
		}
		m_userList->addItem(label);
	}
	if (m_userList->count() > 0 && m_userList->currentRow() < 0)
	{
		m_userList->setCurrentRow(0);
	}
}

void RobotFrameSettingsWidget::loadToolFieldsFromSelection()
{
	const int row = m_toolList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.toolFrames.size()))
	{
		return;
	}
	const RobotCoordinate::RobotToolFrame& tf = m_frames.toolFrames[static_cast<size_t>(row)];
	setRigidToSpins(tf.T_flange_tool, m_toolPos, m_toolEuler);
	const QString flange = QString::fromStdString(tf.flangeLinkName);
	int idx = m_flangeLinkCombo->findText(flange);
	if (idx < 0 && flange.isEmpty() && !QString::fromStdString(m_frames.flangeLinkName).isEmpty())
	{
		idx = m_flangeLinkCombo->findText(QString::fromStdString(m_frames.flangeLinkName));
	}
	if (idx >= 0)
	{
		m_flangeLinkCombo->setCurrentIndex(idx);
	}
	else
	{
		m_flangeLinkCombo->setCurrentIndex(0);
	}
}

void RobotFrameSettingsWidget::loadUserFieldsFromSelection()
{
	const int row = m_userList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.userFrames.size()))
	{
		return;
	}
	setRigidToSpins(m_frames.userFrames[static_cast<size_t>(row)].T_base_user, m_userPos, m_userEuler);
}

void RobotFrameSettingsWidget::saveToolFieldsToSelection()
{
	const int row = m_toolList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.toolFrames.size()))
	{
		return;
	}
	RobotCoordinate::RobotToolFrame& tf = m_frames.toolFrames[static_cast<size_t>(row)];
	readRigidFromSpins(tf.T_flange_tool, m_toolPos, m_toolEuler);
	if (m_flangeLinkCombo->currentIndex() > 0)
	{
		tf.flangeLinkName = m_flangeLinkCombo->currentText().toStdString();
	}
	else
	{
		tf.flangeLinkName.clear();
	}
}

void RobotFrameSettingsWidget::saveUserFieldsToSelection()
{
	const int row = m_userList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.userFrames.size()))
	{
		return;
	}
	readRigidFromSpins(m_frames.userFrames[static_cast<size_t>(row)].T_base_user, m_userPos, m_userEuler);
}

void RobotFrameSettingsWidget::scheduleFramesChanged()
{
	saveToolFieldsToSelection();
	saveUserFieldsToSelection();
	m_framesDebounceTimer->start();
}

void RobotFrameSettingsWidget::onFramesChangeDebounce()
{
	emit framesChanged();
}

void RobotFrameSettingsWidget::onToolFieldChanged()
{
	if (m_blockSignals)
	{
		return;
	}
	scheduleFramesChanged();
}

void RobotFrameSettingsWidget::onUserFieldChanged()
{
	if (m_blockSignals)
	{
		return;
	}
	scheduleFramesChanged();
}

void RobotFrameSettingsWidget::onToolListSelectionChanged()
{
	if (m_blockSignals)
	{
		return;
	}
	loadToolFieldsFromSelection();
}

void RobotFrameSettingsWidget::onUserListSelectionChanged()
{
	if (m_blockSignals)
	{
		return;
	}
	loadUserFieldsFromSelection();
}

void RobotFrameSettingsWidget::onAddToolFrame()
{
	RobotCoordinate::RobotToolFrame tf;
	tf.id = RobotCoordinate::makeToolFrameId();
	tf.name = QStringLiteral("TFrame%1").arg(m_frames.toolFrames.size() + 1).toStdString();
	tf.T_flange_tool = RobotCoordinate::identityRigidFrame();
	m_frames.toolFrames.push_back(std::move(tf));
	if (m_frames.activeToolFrameId.empty())
	{
		m_frames.activeToolFrameId = m_frames.toolFrames.back().id;
	}
	rebuildToolFrameList();
	m_toolList->setCurrentRow(m_toolList->count() - 1);
	emit framesChanged();
}

void RobotFrameSettingsWidget::onRemoveToolFrame()
{
	const int row = m_toolList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.toolFrames.size()))
	{
		return;
	}
	if (m_frames.toolFrames.size() <= 1)
	{
		return;
	}
	const std::string removedId = m_frames.toolFrames[static_cast<size_t>(row)].id;
	m_frames.toolFrames.erase(m_frames.toolFrames.begin() + row);
	if (m_frames.activeToolFrameId == removedId)
	{
		m_frames.activeToolFrameId = m_frames.toolFrames.empty() ? std::string() : m_frames.toolFrames.front().id;
	}
	rebuildToolFrameList();
	emit framesChanged();
}

void RobotFrameSettingsWidget::onDuplicateToolFrame()
{
	const int row = m_toolList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.toolFrames.size()))
	{
		return;
	}
	RobotCoordinate::RobotToolFrame copy = m_frames.toolFrames[static_cast<size_t>(row)];
	copy.id = RobotCoordinate::makeToolFrameId();
	copy.name += "_copy";
	m_frames.toolFrames.push_back(std::move(copy));
	rebuildToolFrameList();
	m_toolList->setCurrentRow(m_toolList->count() - 1);
	emit framesChanged();
}

void RobotFrameSettingsWidget::onSetActiveToolFrame()
{
	const int row = m_toolList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.toolFrames.size()))
	{
		return;
	}
	saveToolFieldsToSelection();
	m_frames.activeToolFrameId = m_frames.toolFrames[static_cast<size_t>(row)].id;
	rebuildToolFrameList();
	emit framesChanged();
}

void RobotFrameSettingsWidget::onAddUserFrame()
{
	RobotCoordinate::RobotUserFrame uf;
	uf.id = RobotCoordinate::makeUserFrameId();
	uf.name = QStringLiteral("UFrame%1").arg(m_frames.userFrames.size() + 1).toStdString();
	uf.T_base_user = RobotCoordinate::identityRigidFrame();
	m_frames.userFrames.push_back(std::move(uf));
	if (m_frames.activeUserFrameId.empty())
	{
		m_frames.activeUserFrameId = m_frames.userFrames.back().id;
	}
	rebuildUserFrameList();
	m_userList->setCurrentRow(m_userList->count() - 1);
	emit framesChanged();
}

void RobotFrameSettingsWidget::onRemoveUserFrame()
{
	const int row = m_userList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.userFrames.size()))
	{
		return;
	}
	const std::string removedId = m_frames.userFrames[static_cast<size_t>(row)].id;
	m_frames.userFrames.erase(m_frames.userFrames.begin() + row);
	if (m_frames.activeUserFrameId == removedId)
	{
		m_frames.activeUserFrameId =
			m_frames.userFrames.empty() ? std::string() : m_frames.userFrames.front().id;
	}
	rebuildUserFrameList();
	emit framesChanged();
}

void RobotFrameSettingsWidget::onDuplicateUserFrame()
{
	const int row = m_userList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.userFrames.size()))
	{
		return;
	}
	RobotCoordinate::RobotUserFrame copy = m_frames.userFrames[static_cast<size_t>(row)];
	copy.id = RobotCoordinate::makeUserFrameId();
	copy.name += "_copy";
	m_frames.userFrames.push_back(std::move(copy));
	rebuildUserFrameList();
	m_userList->setCurrentRow(m_userList->count() - 1);
	emit framesChanged();
}

void RobotFrameSettingsWidget::onSetActiveUserFrame()
{
	const int row = m_userList->currentRow();
	if (row < 0 || row >= static_cast<int>(m_frames.userFrames.size()))
	{
		return;
	}
	m_frames.activeUserFrameId = m_frames.userFrames[static_cast<size_t>(row)].id;
	rebuildUserFrameList();
	emit framesChanged();
}

void RobotFrameSettingsWidget::onShowToggled()
{
	if (m_blockSignals)
	{
		return;
	}
	m_frames.showToolFrameInScene = m_showToolCheck->isChecked();
	m_frames.showUserFramesInScene = m_showUserCheck->isChecked();
	emit framesChanged();
}
