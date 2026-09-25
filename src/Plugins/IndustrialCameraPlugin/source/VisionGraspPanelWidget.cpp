/// @file VisionGraspPanelWidget.cpp
/// @brief 视觉抓取面板：位姿合成后经 IPluginRobotHost 规划确认

#include "VisionGraspPanelWidget.h"

#include "CameraTypes.h"
#include "HandEyePanelWidget.h"
#include "HandEyeTypes.h"
#include "IPluginHostContext.h"
#include "IPluginRobotHost.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QTimer>
#include <QVector>
#include <QVBoxLayout>

using industrial_camera::HandEyeMountMode;
using industrial_camera::Mat4;
using industrial_camera::Pose6d;
using industrial_camera::mat4ToPose6d;
using industrial_camera::pose6dToMat4;

namespace
{
Mat4 mulMat4(const Mat4& a, const Mat4& b)
{
	Mat4 c{};
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			double s = 0.0;
			for (int k = 0; k < 4; ++k)
				s += a[static_cast<size_t>(k * 4 + row)] * b[static_cast<size_t>(col * 4 + k)];
			c[static_cast<size_t>(col * 4 + row)] = s;
		}
	}
	return c;
}

PluginPose6d fromIndustrial(const Pose6d& p)
{
	PluginPose6d o;
	o.xMm = p.x;
	o.yMm = p.y;
	o.zMm = p.z;
	o.rxDeg = p.rxDeg;
	o.ryDeg = p.ryDeg;
	o.rzDeg = p.rzDeg;
	return o;
}

Pose6d toIndustrial(const PluginPose6d& p)
{
	Pose6d o;
	o.x = p.xMm;
	o.y = p.yMm;
	o.z = p.zMm;
	o.rxDeg = p.rxDeg;
	o.ryDeg = p.ryDeg;
	o.rzDeg = p.rzDeg;
	return o;
}

QDoubleSpinBox* makePoseSpin(QWidget* parent, double minV, double maxV, int decimals)
{
	auto* s = new QDoubleSpinBox(parent);
	s->setRange(minV, maxV);
	s->setDecimals(decimals);
	s->setSingleStep(1.0);
	return s;
}
} // namespace

VisionGraspPanelWidget::VisionGraspPanelWidget(HandEyePanelWidget* handEye, IPluginHostContext* host, QWidget* parent)
	: QWidget(parent), host_(host), handEye_(handEye)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(12, 12, 12, 12);

	handEyeStatus_ = new QLabel(this);
	planHint_ = new QLabel(this);
	planHint_->setWordWrap(true);
	root->addWidget(handEyeStatus_);
	root->addWidget(planHint_);

	srcBox_ = new QGroupBox(this);
	auto* srcLay = new QVBoxLayout(srcBox_);
	srcManualBase_ = new QRadioButton(srcBox_);
	srcManualCam_ = new QRadioButton(srcBox_);
	srcScene_ = new QRadioButton(srcBox_);
	srcManualBase_->setChecked(true);
	srcLay->addWidget(srcManualBase_);
	srcLay->addWidget(srcManualCam_);
	srcLay->addWidget(srcScene_);
	root->addWidget(srcBox_);

	poseBox_ = new QGroupBox(this);
	auto* form = new QFormLayout(poseBox_);
	px_ = makePoseSpin(poseBox_, -1e6, 1e6, 3);
	py_ = makePoseSpin(poseBox_, -1e6, 1e6, 3);
	pz_ = makePoseSpin(poseBox_, -1e6, 1e6, 3);
	prx_ = makePoseSpin(poseBox_, -360, 360, 3);
	pry_ = makePoseSpin(poseBox_, -360, 360, 3);
	prz_ = makePoseSpin(poseBox_, -360, 360, 3);
	form->addRow(QStringLiteral("X"), px_);
	form->addRow(QStringLiteral("Y"), py_);
	form->addRow(QStringLiteral("Z"), pz_);
	form->addRow(QStringLiteral("Rx"), prx_);
	form->addRow(QStringLiteral("Ry"), pry_);
	form->addRow(QStringLiteral("Rz"), prz_);
	root->addWidget(poseBox_);

	auto* btnRow = new QHBoxLayout;
	readTcpBtn_ = new QPushButton(this);
	readObjBtn_ = new QPushButton(this);
	btnRow->addWidget(readTcpBtn_);
	btnRow->addWidget(readObjBtn_);
	root->addLayout(btnRow);

	graspBox_ = new QGroupBox(this);
	auto* gForm = new QFormLayout(graspBox_);
	approachZ_ = makePoseSpin(graspBox_, 0, 1e5, 1);
	approachZ_->setValue(50.0);
	retractZ_ = makePoseSpin(graspBox_, 0, 1e5, 1);
	retractZ_->setValue(80.0);
	lockEuler_ = new QCheckBox(graspBox_);
	lockEuler_->setChecked(true);
	gForm->addRow(QStringLiteral("Approach +Z (mm)"), approachZ_);
	gForm->addRow(QStringLiteral("Retract +Z (mm)"), retractZ_);
	gForm->addRow(QString(), lockEuler_);
	root->addWidget(graspBox_);

	previewBtn_ = new QPushButton(this);
	planBtn_ = new QPushButton(this);
	root->addWidget(previewBtn_);
	root->addWidget(planBtn_);
	status_ = new QLabel(this);
	status_->setWordWrap(true);
	root->addWidget(status_);
	root->addStretch(1);

	connect(readTcpBtn_, &QPushButton::clicked, this, &VisionGraspPanelWidget::onReadCurrentTcp);
	connect(readObjBtn_, &QPushButton::clicked, this, &VisionGraspPanelWidget::onReadSelectedObject);
	connect(previewBtn_, &QPushButton::clicked, this, &VisionGraspPanelWidget::onPreviewThreePoints);
	connect(planBtn_, &QPushButton::clicked, this, &VisionGraspPanelWidget::onPlanAndConfirm);

	auto* timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &VisionGraspPanelWidget::refreshHandEyeStatus);
	timer->start(1000);

	applyLanguage();
	refreshHandEyeStatus();
}

void VisionGraspPanelWidget::setUseChinese(bool zh)
{
	zh_ = zh;
	applyLanguage();
}

void VisionGraspPanelWidget::applyLanguage()
{
	if (srcBox_)
		srcBox_->setTitle(zh_ ? QStringLiteral("位姿源") : QStringLiteral("Pose source"));
	if (poseBox_)
		poseBox_->setTitle(zh_ ? QStringLiteral("目标位姿") : QStringLiteral("Target pose"));
	if (graspBox_)
		graspBox_->setTitle(zh_ ? QStringLiteral("抓取参数") : QStringLiteral("Grasp offsets"));
	if (approachZ_)
	{
		if (auto* form = qobject_cast<QFormLayout*>(graspBox_ ? graspBox_->layout() : nullptr))
		{
			if (auto* lb = qobject_cast<QLabel*>(form->labelForField(approachZ_)))
				lb->setText(zh_ ? QStringLiteral("接近抬高 +Z (mm)") : QStringLiteral("Approach +Z (mm)"));
			if (auto* lb = qobject_cast<QLabel*>(form->labelForField(retractZ_)))
				lb->setText(zh_ ? QStringLiteral("抬起抬高 +Z (mm)") : QStringLiteral("Retract +Z (mm)"));
		}
	}
	if (srcManualBase_)
		srcManualBase_->setText(zh_ ? QStringLiteral("手工基座系") : QStringLiteral("Manual (robot base)"));
	if (srcManualCam_)
		srcManualCam_->setText(zh_ ? QStringLiteral("手工相机系（×手眼）") : QStringLiteral("Manual (camera × hand-eye)"));
	if (srcScene_)
		srcScene_->setText(zh_ ? QStringLiteral("场景选中物体（世界→基座）")
							   : QStringLiteral("Selected object (world→base)"));
	if (planHint_)
		planHint_->setText(zh_ ? QStringLiteral("规划器/碰撞开关取仿真「碰撞与规划」页当前设置。")
							   : QStringLiteral("Planner/collision settings come from the Collision & Planning page."));
	if (lockEuler_)
		lockEuler_->setText(zh_ ? QStringLiteral("抓取姿态锁定为当前 TCP 欧拉")
								: QStringLiteral("Lock grasp Euler to current TCP"));
	if (readTcpBtn_)
		readTcpBtn_->setText(zh_ ? QStringLiteral("读取当前 TCP") : QStringLiteral("Read current TCP"));
	if (readObjBtn_)
		readObjBtn_->setText(zh_ ? QStringLiteral("从选中物体读取") : QStringLiteral("Read selected object"));
	if (previewBtn_)
		previewBtn_->setText(zh_ ? QStringLiteral("预览三点") : QStringLiteral("Preview 3 poses"));
	if (planBtn_)
		planBtn_->setText(zh_ ? QStringLiteral("规划并确认抓取轨迹") : QStringLiteral("Plan & confirm grasp path"));
	refreshHandEyeStatus();
}

void VisionGraspPanelWidget::loadOffsetsFromJson(const QJsonObject& obj)
{
	if (obj.contains(QStringLiteral("approachZMm")))
		approachZ_->setValue(obj.value(QStringLiteral("approachZMm")).toDouble(50.0));
	if (obj.contains(QStringLiteral("retractZMm")))
		retractZ_->setValue(obj.value(QStringLiteral("retractZMm")).toDouble(80.0));
	if (obj.contains(QStringLiteral("lockEuler")))
		lockEuler_->setChecked(obj.value(QStringLiteral("lockEuler")).toBool(true));
}

void VisionGraspPanelWidget::saveOffsetsToJson(QJsonObject& obj) const
{
	obj.insert(QStringLiteral("approachZMm"), approachZ_->value());
	obj.insert(QStringLiteral("retractZMm"), retractZ_->value());
	obj.insert(QStringLiteral("lockEuler"), lockEuler_->isChecked());
}

VisionGraspPanelWidget::PoseSource VisionGraspPanelWidget::currentSource() const
{
	if (srcManualCam_ && srcManualCam_->isChecked())
		return PoseSource::ManualCamera;
	if (srcScene_ && srcScene_->isChecked())
		return PoseSource::SceneObject;
	return PoseSource::ManualBase;
}

PluginPose6d VisionGraspPanelWidget::readPoseSpins() const
{
	PluginPose6d p;
	p.xMm = px_->value();
	p.yMm = py_->value();
	p.zMm = pz_->value();
	p.rxDeg = prx_->value();
	p.ryDeg = pry_->value();
	p.rzDeg = prz_->value();
	return p;
}

void VisionGraspPanelWidget::writePoseSpins(const PluginPose6d& p)
{
	px_->setValue(p.xMm);
	py_->setValue(p.yMm);
	pz_->setValue(p.zMm);
	prx_->setValue(p.rxDeg);
	pry_->setValue(p.ryDeg);
	prz_->setValue(p.rzDeg);
}

void VisionGraspPanelWidget::hostLog(const QString& s, bool isError)
{
	if (!host_)
		return;
	if (isError)
		host_->logError(s);
	else
		host_->logInfo(s);
}

void VisionGraspPanelWidget::refreshHandEyeStatus()
{
	const bool ok = handEye_ && handEye_->lastCalibrationResult().ok;
	if (handEyeStatus_)
	{
		if (ok)
		{
			handEyeStatus_->setStyleSheet(QStringLiteral("color:green"));
			handEyeStatus_->setText(zh_ ? QStringLiteral("手眼：已标定") : QStringLiteral("Hand-eye: calibrated"));
		}
		else
		{
			handEyeStatus_->setStyleSheet(QStringLiteral("color:#b22222"));
			handEyeStatus_->setText(zh_ ? QStringLiteral("手眼：未标定（相机系模式需先标定）")
										: QStringLiteral("Hand-eye: not calibrated"));
		}
	}
	const bool needCalib = currentSource() == PoseSource::ManualCamera;
	if (planBtn_)
		planBtn_->setEnabled(!needCalib || ok);
}

void VisionGraspPanelWidget::onReadCurrentTcp()
{
	IPluginRobotHost* rh = host_ ? host_->robotHost() : nullptr;
	if (!rh)
	{
		status_->setText(zh_ ? QStringLiteral("机器人宿主不可用") : QStringLiteral("Robot host unavailable"));
		return;
	}
	PluginPose6d p;
	QString err;
	if (!rh->getActiveRobotTcpPose(p, &err))
	{
		status_->setText(err);
		hostLog(err, true);
		return;
	}
	writePoseSpins(p);
	srcManualBase_->setChecked(true);
	status_->setText(zh_ ? QStringLiteral("已读入当前 TCP") : QStringLiteral("Current TCP loaded"));
}

void VisionGraspPanelWidget::onReadSelectedObject()
{
	IPluginRobotHost* rh = host_ ? host_->robotHost() : nullptr;
	if (!rh)
	{
		status_->setText(zh_ ? QStringLiteral("机器人宿主不可用") : QStringLiteral("Robot host unavailable"));
		return;
	}
	PluginPose6d p;
	QString err;
	if (!rh->getSelectedBackendWorldPose(p, &err))
	{
		status_->setText(err);
		hostLog(err, true);
		return;
	}
	writePoseSpins(p);
	srcScene_->setChecked(true);
	status_->setText(zh_ ? QStringLiteral("已读入选中物体（基座系）") : QStringLiteral("Selected object loaded (base)"));
}

bool VisionGraspPanelWidget::composeGraspPoses(PluginPose6d& approach, PluginPose6d& grasp, PluginPose6d& retract,
											  QString* err)
{
	PluginPose6d obj = readPoseSpins();
	const PoseSource src = currentSource();

	if (src == PoseSource::ManualCamera)
	{
		if (!handEye_ || !handEye_->lastCalibrationResult().ok)
		{
			if (err)
				*err = zh_ ? QStringLiteral("请先完成手眼标定") : QStringLiteral("Calibrate hand-eye first");
			return false;
		}
		const Mat4 T_cam_obj = pose6dToMat4(toIndustrial(obj));
		Mat4 T_base_cam = handEye_->lastCalibrationResult().T_best;
		// Eye-in-hand 的 T_best 是 T_flange_cam，需再乘当前法兰
		if (handEye_->lastMountMode() == HandEyeMountMode::EyeInHand)
		{
			IPluginRobotHost* rh = host_ ? host_->robotHost() : nullptr;
			PluginPose6d tcp;
			QString e2;
			if (!rh || !rh->getActiveRobotTcpPose(tcp, &e2))
			{
				if (err)
					*err = e2.isEmpty() ? (zh_ ? QStringLiteral("无法读当前 TCP") : QStringLiteral("Cannot read TCP"))
										: e2;
				return false;
			}
			T_base_cam = mulMat4(pose6dToMat4(toIndustrial(tcp)), T_base_cam);
		}
		obj = fromIndustrial(mat4ToPose6d(mulMat4(T_base_cam, T_cam_obj)));
	}
	// ManualBase / SceneObject：自旋框按基座系解释（场景读取时已变换）

	grasp = obj;
	if (lockEuler_ && lockEuler_->isChecked())
	{
		IPluginRobotHost* rh = host_ ? host_->robotHost() : nullptr;
		PluginPose6d tcp;
		QString e2;
		if (rh && rh->getActiveRobotTcpPose(tcp, &e2))
		{
			grasp.rxDeg = tcp.rxDeg;
			grasp.ryDeg = tcp.ryDeg;
			grasp.rzDeg = tcp.rzDeg;
		}
	}

	approach = grasp;
	approach.zMm += approachZ_->value();
	retract = grasp;
	retract.zMm += retractZ_->value();
	return true;
}

void VisionGraspPanelWidget::onPreviewThreePoints()
{
	PluginPose6d a, g, r;
	QString err;
	if (!composeGraspPoses(a, g, r, &err))
	{
		status_->setText(err);
		hostLog(err, true);
		return;
	}
	const QString msg =
		QStringLiteral("approach (%.1f,%.1f,%.1f) grasp (%.1f,%.1f,%.1f) retract (%.1f,%.1f,%.1f)")
			.arg(a.xMm)
			.arg(a.yMm)
			.arg(a.zMm)
			.arg(g.xMm)
			.arg(g.yMm)
			.arg(g.zMm)
			.arg(r.xMm)
			.arg(r.yMm)
			.arg(r.zMm);
	status_->setText(msg);
	hostLog(QStringLiteral("[VisionGrasp] ") + msg);
}

void VisionGraspPanelWidget::onPlanAndConfirm()
{
	refreshHandEyeStatus();
	PluginPose6d a, g, r;
	QString err;
	if (!composeGraspPoses(a, g, r, &err))
	{
		status_->setText(err);
		hostLog(err, true);
		return;
	}
	IPluginRobotHost* rh = host_ ? host_->robotHost() : nullptr;
	if (!rh)
	{
		status_->setText(zh_ ? QStringLiteral("机器人宿主不可用") : QStringLiteral("Robot host unavailable"));
		return;
	}
	QVector<PluginPose6d> goals;
	goals << a << g << r;
	status_->setText(zh_ ? QStringLiteral("正在规划…") : QStringLiteral("Planning…"));
	if (!rh->planAndConfirmTcpWaypoints(goals, &err))
	{
		status_->setText(err.isEmpty() ? (zh_ ? QStringLiteral("规划/确认失败或已取消")
											  : QStringLiteral("Plan/confirm failed or canceled"))
									   : err);
		hostLog(status_->text(), true);
		return;
	}
	status_->setText(zh_ ? QStringLiteral("已写入活动程序") : QStringLiteral("Inserted into active program"));
	hostLog(status_->text());
}
