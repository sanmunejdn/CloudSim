/// @file HandEyePanelWidget.cpp
/// @brief 手眼标定向导：配置→采集→多算法结果

#include "HandEyePanelWidget.h"

#include "BoardDetector.h"
#include "CameraPanelWidget.h"
#include "CameraResourceStore.h"
#include "CameraTypes.h"
#include "ICamera.h"
#include "IPluginHostContext.h"

#include <QComboBox>
#include <QDesktopServices>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTableWidget>
#include <QUrl>
#include <QVBoxLayout>

using industrial_camera::BoardDetectParams;
using industrial_camera::BoardType;
using industrial_camera::CameraFrame2D;
using industrial_camera::CameraFrame3D;
using industrial_camera::CameraIntrinsics;
using industrial_camera::HandEyeMountMode;
using industrial_camera::HandEyeSample;
using industrial_camera::HandEyeSolveParams;
using industrial_camera::Pose6d;
using industrial_camera::mat4ToPose6d;
using industrial_camera::pose6dToMat4;

namespace
{
QDoubleSpinBox* makeSpin(QWidget* p, double minV, double maxV, double step = 1.0)
{
	auto* s = new QDoubleSpinBox(p);
	s->setRange(minV, maxV);
	s->setDecimals(3);
	s->setSingleStep(step);
	return s;
}
} // namespace

HandEyePanelWidget::HandEyePanelWidget(CameraPanelWidget* cameraPanel, IPluginHostContext* host, QWidget* parent)
	: QWidget(parent)
	, host_(host)
	, cameraPanel_(cameraPanel)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(12, 12, 12, 12);
	stageLabel_ = new QLabel(this);
	root->addWidget(stageLabel_);

	auto* stack = new QStackedWidget(this);

	pageConfig_ = new QWidget(stack);
	{
		auto* lay = new QVBoxLayout(pageConfig_);
		eyeInHand_ = new QRadioButton(pageConfig_);
		eyeToHand_ = new QRadioButton(pageConfig_);
		eyeInHand_->setChecked(true);
		lay->addWidget(eyeInHand_);
		lay->addWidget(eyeToHand_);
		poseManual_ = new QRadioButton(pageConfig_);
		poseReal_ = new QRadioButton(pageConfig_);
		poseManual_->setChecked(true);
		lay->addWidget(poseManual_);
		lay->addWidget(poseReal_);
		auto* hostRow = new QHBoxLayout;
		robotHost_ = new QLineEdit(QStringLiteral("127.0.0.1"), pageConfig_);
		robotPort_ = new QSpinBox(pageConfig_);
		robotPort_->setRange(1, 65535);
		robotPort_->setValue(19600);
		hostRow->addWidget(robotHost_);
		hostRow->addWidget(robotPort_);
		lay->addLayout(hostRow);

		auto* boardForm = new QFormLayout;
		boardTypeCombo_ = new QComboBox(pageConfig_);
		boardTypeCombo_->addItem(QStringLiteral("Chessboard"), static_cast<int>(BoardType::Chessboard));
		boardTypeCombo_->addItem(QStringLiteral("ArUco"), static_cast<int>(BoardType::Aruco));
		boardCornersX_ = new QSpinBox(pageConfig_);
		boardCornersY_ = new QSpinBox(pageConfig_);
		boardCornersX_->setRange(2, 30);
		boardCornersY_->setRange(2, 30);
		boardCornersX_->setValue(9);
		boardCornersY_->setValue(6);
		boardSquareMm_ = makeSpin(pageConfig_, 1, 500, 1);
		boardSquareMm_->setValue(20);
		boardForm->addRow(QStringLiteral("Board"), boardTypeCombo_);
		boardForm->addRow(QStringLiteral("Corners X"), boardCornersX_);
		boardForm->addRow(QStringLiteral("Corners Y"), boardCornersY_);
		boardForm->addRow(QStringLiteral("Square mm"), boardSquareMm_);
		lay->addLayout(boardForm);

		auto* next = new QPushButton(pageConfig_);
		next->setObjectName(QStringLiteral("nextCfg"));
		lay->addWidget(next);
		lay->addStretch(1);
		connect(next, &QPushButton::clicked, this, &HandEyePanelWidget::onNextConfig);
	}
	stack->addWidget(pageConfig_);

	pageCollect_ = new QWidget(stack);
	{
		auto* lay = new QVBoxLayout(pageCollect_);
		auto* poseBox = new QGroupBox(pageCollect_);
		auto* g = new QGridLayout(poseBox);
		px_ = makeSpin(poseBox, -1e6, 1e6);
		py_ = makeSpin(poseBox, -1e6, 1e6);
		pz_ = makeSpin(poseBox, -1e6, 1e6);
		prx_ = makeSpin(poseBox, -360, 360, 1);
		pry_ = makeSpin(poseBox, -360, 360, 1);
		prz_ = makeSpin(poseBox, -360, 360, 1);
		g->addWidget(new QLabel(QStringLiteral("X")), 0, 0);
		g->addWidget(px_, 0, 1);
		g->addWidget(new QLabel(QStringLiteral("Y")), 0, 2);
		g->addWidget(py_, 0, 3);
		g->addWidget(new QLabel(QStringLiteral("Z")), 0, 4);
		g->addWidget(pz_, 0, 5);
		g->addWidget(new QLabel(QStringLiteral("Rx")), 1, 0);
		g->addWidget(prx_, 1, 1);
		g->addWidget(new QLabel(QStringLiteral("Ry")), 1, 2);
		g->addWidget(pry_, 1, 3);
		g->addWidget(new QLabel(QStringLiteral("Rz")), 1, 4);
		g->addWidget(prz_, 1, 5);
		lay->addWidget(poseBox);

		auto* boardBox = new QGroupBox(pageCollect_);
		auto* bg = new QGridLayout(boardBox);
		bx_ = makeSpin(boardBox, -1e6, 1e6);
		by_ = makeSpin(boardBox, -1e6, 1e6);
		bz_ = makeSpin(boardBox, -1e6, 1e6, 10);
		brx_ = makeSpin(boardBox, -360, 360, 1);
		bry_ = makeSpin(boardBox, -360, 360, 1);
		brz_ = makeSpin(boardBox, -360, 360, 1);
		bz_->setValue(500);
		bg->addWidget(new QLabel(QStringLiteral("board X")), 0, 0);
		bg->addWidget(bx_, 0, 1);
		bg->addWidget(new QLabel(QStringLiteral("Y")), 0, 2);
		bg->addWidget(by_, 0, 3);
		bg->addWidget(new QLabel(QStringLiteral("Z")), 0, 4);
		bg->addWidget(bz_, 0, 5);
		bg->addWidget(new QLabel(QStringLiteral("Rx")), 1, 0);
		bg->addWidget(brx_, 1, 1);
		bg->addWidget(new QLabel(QStringLiteral("Ry")), 1, 2);
		bg->addWidget(bry_, 1, 3);
		bg->addWidget(new QLabel(QStringLiteral("Rz")), 1, 4);
		bg->addWidget(brz_, 1, 5);
		lay->addWidget(boardBox);

		auto* btnRow = new QHBoxLayout;
		auto* readBtn = new QPushButton(pageCollect_);
		auto* detectBtn = new QPushButton(pageCollect_);
		auto* addBtn = new QPushButton(pageCollect_);
		auto* delBtn = new QPushButton(pageCollect_);
		auto* demoBtn = new QPushButton(pageCollect_);
		readBtn->setObjectName(QStringLiteral("readPose"));
		detectBtn->setObjectName(QStringLiteral("captureDetect"));
		addBtn->setObjectName(QStringLiteral("addSample"));
		delBtn->setObjectName(QStringLiteral("delSample"));
		demoBtn->setObjectName(QStringLiteral("demoBtn"));
		btnRow->addWidget(readBtn);
		btnRow->addWidget(detectBtn);
		btnRow->addWidget(addBtn);
		btnRow->addWidget(delBtn);
		btnRow->addWidget(demoBtn);
		lay->addLayout(btnRow);

		sampleTable_ = new QTableWidget(0, 3, pageCollect_);
		sampleTable_->setHorizontalHeaderLabels({QStringLiteral("#"), QStringLiteral("Robot"), QStringLiteral("Board")});
		sampleTable_->horizontalHeader()->setStretchLastSection(true);
		lay->addWidget(sampleTable_);

		auto* nav = new QHBoxLayout;
		auto* back = new QPushButton(pageCollect_);
		auto* solve = new QPushButton(pageCollect_);
		back->setObjectName(QStringLiteral("backCollect"));
		solve->setObjectName(QStringLiteral("solveBtn"));
		nav->addWidget(back);
		nav->addWidget(solve);
		lay->addLayout(nav);

		connect(readBtn, &QPushButton::clicked, this, &HandEyePanelWidget::onReadPose);
		connect(detectBtn, &QPushButton::clicked, this, &HandEyePanelWidget::onCaptureDetect);
		connect(addBtn, &QPushButton::clicked, this, &HandEyePanelWidget::onAddSample);
		connect(delBtn, &QPushButton::clicked, this, &HandEyePanelWidget::onDeleteSample);
		connect(demoBtn, &QPushButton::clicked, this, &HandEyePanelWidget::onFillDemo);
		connect(back, &QPushButton::clicked, this, &HandEyePanelWidget::onBackCollect);
		connect(solve, &QPushButton::clicked, this, &HandEyePanelWidget::onSolve);
	}
	stack->addWidget(pageCollect_);

	pageResult_ = new QWidget(stack);
	{
		auto* lay = new QVBoxLayout(pageResult_);
		scoreTable_ = new QTableWidget(0, 5, pageResult_);
		scoreTable_->setHorizontalHeaderLabels(
			{QStringLiteral("Method"), QStringLiteral("rot"), QStringLiteral("trans(mm)"), QStringLiteral("score"),
			 QStringLiteral("ok")});
		scoreTable_->horizontalHeader()->setStretchLastSection(true);
		lay->addWidget(scoreTable_);
		pathLabel_ = new QLabel(pageResult_);
		pathLabel_->setWordWrap(true);
		lay->addWidget(pathLabel_);
		auto* row = new QHBoxLayout;
		auto* exp = new QPushButton(pageResult_);
		auto* open = new QPushButton(pageResult_);
		auto* again = new QPushButton(pageResult_);
		exp->setObjectName(QStringLiteral("exportBtn"));
		open->setObjectName(QStringLiteral("openCalib"));
		again->setObjectName(QStringLiteral("againBtn"));
		row->addWidget(exp);
		row->addWidget(open);
		row->addWidget(again);
		lay->addLayout(row);
		connect(exp, &QPushButton::clicked, this, &HandEyePanelWidget::onExport);
		connect(open, &QPushButton::clicked, this, &HandEyePanelWidget::onOpenDir);
		connect(again, &QPushButton::clicked, this, [this]() { setStage(Stage::Collect); });
	}
	stack->addWidget(pageResult_);

	root->addWidget(stack, 1);
	status_ = new QLabel(this);
	root->addWidget(status_);

	stack->setObjectName(QStringLiteral("stageStack"));
	setStage(Stage::Config);
	applyLanguage();
}

void HandEyePanelWidget::hostLog(const QString& s, bool isError)
{
	if (!host_)
		return;
	const QString msg = QStringLiteral("[手眼标定] %1").arg(s);
	if (isError)
		host_->logError(msg);
	else
		host_->logInfo(msg);
}

void HandEyePanelWidget::setUseChinese(bool zh)
{
	zh_ = zh;
}

void HandEyePanelWidget::applyLanguage()
{
	refreshStageLabel();
	eyeInHand_->setText(zh_ ? QStringLiteral("眼在手上") : QStringLiteral("Eye-in-hand"));
	eyeToHand_->setText(zh_ ? QStringLiteral("眼在手外") : QStringLiteral("Eye-to-hand"));
	poseManual_->setText(zh_ ? QStringLiteral("手动输入末端位姿") : QStringLiteral("Manual TCP pose"));
	poseReal_->setText(zh_ ? QStringLiteral("真实机器人 TCP/JSON") : QStringLiteral("Real robot TCP/JSON"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("nextCfg")))
		b->setText(zh_ ? QStringLiteral("下一步：开始采集") : QStringLiteral("Next: collect"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("readPose")))
		b->setText(zh_ ? QStringLiteral("读取位姿") : QStringLiteral("Read pose"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("captureDetect")))
		b->setText(zh_ ? QStringLiteral("拍并检测") : QStringLiteral("Grab+Detect"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("addSample")))
		b->setText(zh_ ? QStringLiteral("手填添加样本") : QStringLiteral("Add (manual board)"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("delSample")))
		b->setText(zh_ ? QStringLiteral("删除选中") : QStringLiteral("Delete"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("demoBtn")))
		b->setText(zh_ ? QStringLiteral("填充演示样本") : QStringLiteral("Fill demo"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("backCollect")))
		b->setText(zh_ ? QStringLiteral("上一步") : QStringLiteral("Back"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("solveBtn")))
		b->setText(zh_ ? QStringLiteral("开始求解") : QStringLiteral("Solve"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("exportBtn")))
		b->setText(zh_ ? QStringLiteral("导出到 resource") : QStringLiteral("Export"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("openCalib")))
		b->setText(zh_ ? QStringLiteral("打开标定目录") : QStringLiteral("Open folder"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("againBtn")))
		b->setText(zh_ ? QStringLiteral("返回采集") : QStringLiteral("Back to collect"));
}

void HandEyePanelWidget::refreshStageLabel()
{
	const QString s = zh_ ? QStringLiteral("阶段: %1") : QStringLiteral("Stage: %1");
	QString name;
	switch (stage_)
	{
	case Stage::Config:
		name = zh_ ? QStringLiteral("●配置 — ○采集 — ○结果") : QStringLiteral("Config");
		break;
	case Stage::Collect:
		name = zh_ ? QStringLiteral("○配置 — ●采集 — ○结果") : QStringLiteral("Collect");
		break;
	case Stage::Result:
		name = zh_ ? QStringLiteral("○配置 — ○采集 — ●结果") : QStringLiteral("Result");
		break;
	}
	stageLabel_->setText(s.arg(name));
}

void HandEyePanelWidget::setStage(Stage s)
{
	stage_ = s;
	if (auto* stack = findChild<QStackedWidget*>(QStringLiteral("stageStack")))
		stack->setCurrentIndex(static_cast<int>(s));
	refreshStageLabel();
}

Pose6d HandEyePanelWidget::readPoseSpins() const
{
	Pose6d p;
	p.x = px_->value();
	p.y = py_->value();
	p.z = pz_->value();
	p.rxDeg = prx_->value();
	p.ryDeg = pry_->value();
	p.rzDeg = prz_->value();
	return p;
}

void HandEyePanelWidget::writePoseSpins(const Pose6d& p)
{
	px_->setValue(p.x);
	py_->setValue(p.y);
	pz_->setValue(p.z);
	prx_->setValue(p.rxDeg);
	pry_->setValue(p.ryDeg);
	prz_->setValue(p.rzDeg);
}

void HandEyePanelWidget::onNextConfig()
{
	samples_.clear();
	sampleTable_->setRowCount(0);
	mechSession_.reset();
	if (poseReal_->isChecked())
	{
		industrial_camera::RealRobotPoseConfig cfg;
		cfg.host = robotHost_->text().trimmed().toStdString();
		cfg.port = robotPort_->value();
		poseSrc_ = industrial_camera::createRealRobotPoseSource(cfg);
	}
	else
	{
		poseSrc_ = industrial_camera::createManualPoseSource(readPoseSpins());
	}

	// 梅卡真机时并行开官方会话，失败不阻塞 OpenCV/手填路径
	if (cameraPanel_ && cameraPanel_->camera()
		&& cameraPanel_->camera()->brand() == industrial_camera::CameraBrand::MechMind
		&& industrial_camera::mechEyeSdkAvailable())
	{
		mechSession_ = std::make_unique<industrial_camera::MechOfficialHandEyeSession>();
		std::string err;
		const auto mode = eyeInHand_->isChecked() ? HandEyeMountMode::EyeInHand : HandEyeMountMode::EyeToHand;
		if (!mechSession_->begin(cameraPanel_->camera(), mode, &err))
		{
			hostLog(QString::fromStdString(err), true);
			mechSession_.reset();
		}
	}
	setStage(Stage::Collect);
}

void HandEyePanelWidget::onBackCollect()
{
	setStage(Stage::Config);
}

void HandEyePanelWidget::onReadPose()
{
	if (!poseSrc_)
		return;
	if (poseManual_->isChecked())
	{
		industrial_camera::setManualPose(poseSrc_.get(), readPoseSpins());
	}
	Pose6d p;
	if (!poseSrc_->getCurrentPose(p))
	{
		status_->setStyleSheet(QStringLiteral("color:#b22222"));
		status_->setText(QString::fromStdString(poseSrc_->lastError()));
		hostLog(status_->text(), true);
		return;
	}
	writePoseSpins(p);
	status_->setStyleSheet(QStringLiteral("color:green"));
	status_->setText(zh_ ? QStringLiteral("已读取位姿") : QStringLiteral("Pose read"));
	hostLog(status_->text());
}

void HandEyePanelWidget::appendSampleRow(const Pose6d& robot, const Pose6d& board)
{
	HandEyeSample s;
	s.T_base_flange = pose6dToMat4(robot);
	s.T_cam_board = pose6dToMat4(board);
	samples_.push_back(s);

	const int row = sampleTable_->rowCount();
	sampleTable_->insertRow(row);
	sampleTable_->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1)));
	sampleTable_->setItem(row, 1,
						  new QTableWidgetItem(QStringLiteral("%1,%2,%3")
												   .arg(robot.x, 0, 'f', 1)
												   .arg(robot.y, 0, 'f', 1)
												   .arg(robot.z, 0, 'f', 1)));
	sampleTable_->setItem(row, 2,
						  new QTableWidgetItem(QStringLiteral("%1,%2,%3")
												   .arg(board.x, 0, 'f', 1)
												   .arg(board.y, 0, 'f', 1)
												   .arg(board.z, 0, 'f', 1)));
}

void HandEyePanelWidget::onAddSample()
{
	if (poseManual_->isChecked() && poseSrc_)
		industrial_camera::setManualPose(poseSrc_.get(), readPoseSpins());

	Pose6d robot = readPoseSpins();
	if (poseReal_->isChecked() && poseSrc_)
	{
		if (!poseSrc_->getCurrentPose(robot))
		{
			status_->setStyleSheet(QStringLiteral("color:#b22222"));
			status_->setText(QString::fromStdString(poseSrc_->lastError()));
			hostLog(status_->text(), true);
			return;
		}
		writePoseSpins(robot);
	}

	Pose6d board;
	board.x = bx_->value();
	board.y = by_->value();
	board.z = bz_->value();
	board.rxDeg = brx_->value();
	board.ryDeg = bry_->value();
	board.rzDeg = brz_->value();
	brz_->setValue(brz_->value() + 8.0);
	prz_->setValue(prz_->value() + 10.0);

	appendSampleRow(robot, board);
	status_->setStyleSheet(QStringLiteral("color:green"));
	status_->setText(zh_ ? QStringLiteral("已采集 %1 / 建议≥6").arg(samples_.size())
						 : QStringLiteral("Samples %1").arg(samples_.size()));
	hostLog(status_->text());
}

void HandEyePanelWidget::onCaptureDetect()
{
	if (!cameraPanel_ || !cameraPanel_->camera() || !cameraPanel_->camera()->isConnected())
	{
		status_->setStyleSheet(QStringLiteral("color:#b22222"));
		status_->setText(zh_ ? QStringLiteral("请先在相机页连接设备") : QStringLiteral("Connect camera first"));
		hostLog(status_->text(), true);
		return;
	}

	if (poseManual_->isChecked() && poseSrc_)
		industrial_camera::setManualPose(poseSrc_.get(), readPoseSpins());
	Pose6d robot = readPoseSpins();
	if (poseReal_->isChecked() && poseSrc_)
	{
		if (!poseSrc_->getCurrentPose(robot))
		{
			status_->setStyleSheet(QStringLiteral("color:#b22222"));
			status_->setText(QString::fromStdString(poseSrc_->lastError()));
			hostLog(status_->text(), true);
			return;
		}
		writePoseSpins(robot);
	}

	CameraFrame2D frame2d;
	CameraFrame3D frame3d;
	if (!cameraPanel_->camera()->grabOne(frame2d, &frame3d, 3000))
	{
		status_->setStyleSheet(QStringLiteral("color:#b22222"));
		status_->setText(QString::fromStdString(cameraPanel_->camera()->lastError()));
		hostLog(status_->text(), true);
		return;
	}

	CameraIntrinsics K;
	cameraPanel_->camera()->getIntrinsics(K);
	BoardDetectParams bp;
	bp.type = static_cast<BoardType>(boardTypeCombo_->currentData().toInt());
	bp.cornersX = boardCornersX_->value();
	bp.cornersY = boardCornersY_->value();
	bp.squareSizeMm = boardSquareMm_->value();
	const auto det = industrial_camera::detectBoardPose(frame2d, K, bp);
	if (!det.ok)
	{
		status_->setStyleSheet(QStringLiteral("color:#b22222"));
		status_->setText(QString::fromStdString(det.error));
		hostLog(status_->text(), true);
		return;
	}

	const Pose6d board = mat4ToPose6d(det.T_cam_board);
	bx_->setValue(board.x);
	by_->setValue(board.y);
	bz_->setValue(board.z);
	brx_->setValue(board.rxDeg);
	bry_->setValue(board.ryDeg);
	brz_->setValue(board.rzDeg);
	appendSampleRow(robot, board);

	if (mechSession_)
	{
		std::string err;
		if (!mechSession_->addPoseAndDetect(robot, &err))
			hostLog(QString::fromStdString(err), true);
	}

	status_->setStyleSheet(QStringLiteral("color:green"));
	status_->setText(zh_ ? QStringLiteral("检测成功，样本 %1").arg(samples_.size())
						 : QStringLiteral("Detected, samples %1").arg(samples_.size()));
	hostLog(status_->text());
}

void HandEyePanelWidget::onFillDemo()
{
	samples_.clear();
	sampleTable_->setRowCount(0);
	// 合成一致样本：B = inv(X)*A*X，保证 Ensemble 可解
	Pose6d xPose;
	xPose.x = 50;
	xPose.y = -20;
	xPose.z = 80;
	xPose.rxDeg = 5;
	xPose.ryDeg = -3;
	xPose.rzDeg = 12;
	const auto X = pose6dToMat4(xPose);
	auto matMul = [](const industrial_camera::Mat4& a, const industrial_camera::Mat4& b) {
		industrial_camera::Mat4 c{};
		for (int ccol = 0; ccol < 4; ++ccol)
			for (int row = 0; row < 4; ++row)
			{
				double s = 0;
				for (int k = 0; k < 4; ++k)
					s += a[row + k * 4] * b[k + ccol * 4];
				c[row + ccol * 4] = s;
			}
		return c;
	};
	auto matInv = [](const industrial_camera::Mat4& m) {
		// 刚体逆
		industrial_camera::Mat4 inv{};
		inv[0] = m[0];
		inv[1] = m[4];
		inv[2] = m[8];
		inv[4] = m[1];
		inv[5] = m[5];
		inv[6] = m[9];
		inv[8] = m[2];
		inv[9] = m[6];
		inv[10] = m[10];
		inv[12] = -(inv[0] * m[12] + inv[4] * m[13] + inv[8] * m[14]);
		inv[13] = -(inv[1] * m[12] + inv[5] * m[13] + inv[9] * m[14]);
		inv[14] = -(inv[2] * m[12] + inv[6] * m[13] + inv[10] * m[14]);
		inv[15] = 1;
		return inv;
	};

	for (int i = 0; i < 8; ++i)
	{
		Pose6d robot;
		robot.x = 200 + 30 * i;
		robot.y = 100 + 20 * ((i % 2) ? 1 : -1);
		robot.z = 300;
		robot.rxDeg = 10 * i;
		robot.ryDeg = 5 * (i % 3);
		robot.rzDeg = 15 * i;
		const auto A = pose6dToMat4(robot);
		const auto B = matMul(matMul(matInv(X), A), X);
		HandEyeSample s;
		s.T_base_flange = A;
		s.T_cam_board = B;
		samples_.push_back(s);
		const int row = sampleTable_->rowCount();
		sampleTable_->insertRow(row);
		sampleTable_->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1)));
		sampleTable_->setItem(row, 1, new QTableWidgetItem(QStringLiteral("demo A%1").arg(i)));
		sampleTable_->setItem(row, 2, new QTableWidgetItem(QStringLiteral("demo B%1").arg(i)));
	}
	status_->setStyleSheet(QStringLiteral("color:green"));
	status_->setText(zh_ ? QStringLiteral("已填充 8 组演示样本") : QStringLiteral("Filled 8 demo samples"));
	hostLog(status_->text());
}

void HandEyePanelWidget::onDeleteSample()
{
	const int row = sampleTable_->currentRow();
	if (row < 0 || row >= static_cast<int>(samples_.size()))
		return;
	samples_.erase(samples_.begin() + row);
	sampleTable_->removeRow(row);
	for (int i = 0; i < sampleTable_->rowCount(); ++i)
		sampleTable_->setItem(i, 0, new QTableWidgetItem(QString::number(i + 1)));
}

void HandEyePanelWidget::fillScoreTable(const industrial_camera::HandEyeResult& r)
{
	scoreTable_->setRowCount(0);
	int bestRow = -1;
	for (const auto& sc : r.scores)
	{
		const int row = scoreTable_->rowCount();
		scoreTable_->insertRow(row);
		scoreTable_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(sc.name)));
		scoreTable_->setItem(row, 1, new QTableWidgetItem(QString::number(sc.rotErrRad, 'f', 6)));
		scoreTable_->setItem(row, 2, new QTableWidgetItem(QString::number(sc.transErrMm, 'f', 3)));
		scoreTable_->setItem(row, 3, new QTableWidgetItem(QString::number(sc.score, 'f', 6)));
		scoreTable_->setItem(row, 4, new QTableWidgetItem(sc.ok ? QStringLiteral("OK") : QStringLiteral("FAIL")));
		if (sc.ok && sc.name == r.bestMethodName)
			bestRow = row;
	}
	if (bestRow >= 0)
		scoreTable_->selectRow(bestRow);
}

void HandEyePanelWidget::onSolve()
{
	HandEyeSolveParams params;
	params.mode = eyeInHand_->isChecked() ? HandEyeMountMode::EyeInHand : HandEyeMountMode::EyeToHand;
	lastResult_ = industrial_camera::solveHandEyeEnsemble(samples_, params);

	if (mechSession_ && mechSession_->sampleCount() >= 3)
	{
		std::string err;
		auto mechScore = mechSession_->calculate(&err);
		if (!mechScore.ok && !err.empty())
			hostLog(QString::fromStdString(err), true);
		industrial_camera::mergeHandEyeCandidate(lastResult_, mechScore, samples_, params);
	}

	fillScoreTable(lastResult_);
	setStage(Stage::Result);
	if (!lastResult_.ok)
	{
		status_->setStyleSheet(QStringLiteral("color:#b22222"));
		status_->setText(QString::fromStdString(lastResult_.error));
		hostLog(status_->text(), true);
	}
	else
	{
		status_->setStyleSheet(QStringLiteral("color:green"));
		status_->setText(zh_ ? QStringLiteral("最优: %1").arg(QString::fromStdString(lastResult_.bestMethodName))
							 : QStringLiteral("Best: %1").arg(QString::fromStdString(lastResult_.bestMethodName)));
		hostLog(status_->text());
		onExport();
	}
}

void HandEyePanelWidget::onExport()
{
	if (!lastResult_.ok && lastResult_.scores.empty())
		return;
	QString err;
	const auto mode = eyeInHand_->isChecked() ? HandEyeMountMode::EyeInHand : HandEyeMountMode::EyeToHand;
	lastCalibDir_ = industrial_camera_ui::saveCalibrationSession(mode, lastResult_, nullptr, &err);
	pathLabel_->setText(lastCalibDir_.isEmpty() ? err : lastCalibDir_);
}

void HandEyePanelWidget::onOpenDir()
{
	const QString root = lastCalibDir_.isEmpty() ? industrial_camera_ui::industrialCameraRoot() : lastCalibDir_;
	industrial_camera_ui::ensureIndustrialCameraRoot(nullptr);
	QDesktopServices::openUrl(QUrl::fromLocalFile(root));
}
