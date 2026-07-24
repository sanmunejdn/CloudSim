/// @file CameraPanelWidget.cpp
/// @brief 相机侧栏实现

#include "CameraPanelWidget.h"

#include "CameraResourceStore.h"
#include "ICamera.h"
#include "IPluginHostContext.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>

using industrial_camera::CameraBrand;
using industrial_camera::CameraConnectParams;
using industrial_camera::CameraFrame2D;
using industrial_camera::CameraFrame3D;
using industrial_camera::CapDepth;
using industrial_camera::CapImage2D;
using industrial_camera::CapPointCloud;
using industrial_camera::PixelFormat;

CameraPanelWidget::CameraPanelWidget(IPluginHostContext* host, QWidget* parent)
	: QWidget(parent)
	, host_(host)
	, factory_(industrial_camera::createCameraFactory())
	, last2d_(std::make_unique<CameraFrame2D>())
	, last3d_(std::make_unique<CameraFrame3D>())
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(12, 12, 12, 12);

	auto* deviceBox = new QGroupBox(this);
	auto* grid = new QFormLayout(deviceBox);
	brandCombo_ = new QComboBox(deviceBox);
	brandCombo_->addItem(QStringLiteral("Hikvision"), static_cast<int>(CameraBrand::Hikvision));
	brandCombo_->addItem(QStringLiteral("MechMind"), static_cast<int>(CameraBrand::MechMind));
	brandCombo_->addItem(QStringLiteral("Simulated"), static_cast<int>(CameraBrand::Simulated));
	typeCombo_ = new QComboBox(deviceBox);
	typeCombo_->addItem(QStringLiteral("2D MVS"), 0);
	typeCombo_->addItem(QStringLiteral("3D Mv3dRgbd"), 1);
	deviceCombo_ = new QComboBox(deviceBox);
	ipEdit_ = new QLineEdit(deviceBox);
	ipEdit_->setPlaceholderText(QStringLiteral("192.168.1.100"));
	auto* enumBtn = new QPushButton(deviceBox);
	auto* connBtn = new QPushButton(deviceBox);
	auto* discBtn = new QPushButton(deviceBox);
	enumBtn->setObjectName(QStringLiteral("enumBtn"));
	connBtn->setObjectName(QStringLiteral("connBtn"));
	discBtn->setObjectName(QStringLiteral("discBtn"));
	grid->addRow(QStringLiteral("Brand"), brandCombo_);
	grid->addRow(QStringLiteral("Type"), typeCombo_);
	auto* enumRow = new QHBoxLayout;
	enumRow->addWidget(enumBtn);
	enumRow->addWidget(deviceCombo_, 1);
	grid->addRow(QStringLiteral("Device"), enumRow);
	grid->addRow(QStringLiteral("IP"), ipEdit_);
	auto* connRow = new QHBoxLayout;
	connRow->addWidget(connBtn);
	connRow->addWidget(discBtn);
	status_ = new QLabel(deviceBox);
	connRow->addWidget(status_, 1);
	grid->addRow(QString(), connRow);
	root->addWidget(deviceBox);

	auto* previewBox = new QGroupBox(this);
	auto* pvLay = new QVBoxLayout(previewBox);
	preview_ = new QLabel(previewBox);
	preview_->setMinimumHeight(180);
	preview_->setAlignment(Qt::AlignCenter);
	preview_->setStyleSheet(QStringLiteral("background:#222;color:#aaa;"));
	pvLay->addWidget(preview_);
	auto* grabRow = new QHBoxLayout;
	auto* grabBtn = new QPushButton(previewBox);
	grabBtn->setObjectName(QStringLiteral("grabBtn"));
	live_ = new QCheckBox(previewBox);
	timeoutSpin_ = new QSpinBox(previewBox);
	timeoutSpin_->setRange(100, 30000);
	timeoutSpin_->setValue(2000);
	timeoutSpin_->setSuffix(QStringLiteral(" ms"));
	grabRow->addWidget(grabBtn);
	grabRow->addWidget(live_);
	grabRow->addWidget(timeoutSpin_);
	pvLay->addLayout(grabRow);
	root->addWidget(previewBox);

	auto* dataBox = new QGroupBox(this);
	auto* dataLay = new QVBoxLayout(dataBox);
	saveColor_ = new QCheckBox(dataBox);
	saveDepth_ = new QCheckBox(dataBox);
	saveCloud_ = new QCheckBox(dataBox);
	saveColor_->setChecked(true);
	saveCloud_->setChecked(true);
	dataLay->addWidget(saveColor_);
	dataLay->addWidget(saveDepth_);
	dataLay->addWidget(saveCloud_);
	auto* dataBtns = new QHBoxLayout;
	auto* saveBtn = new QPushButton(dataBox);
	auto* importBtn = new QPushButton(dataBox);
	auto* openBtn = new QPushButton(dataBox);
	saveBtn->setObjectName(QStringLiteral("saveBtn"));
	importBtn->setObjectName(QStringLiteral("importBtn"));
	openBtn->setObjectName(QStringLiteral("openBtn"));
	dataBtns->addWidget(saveBtn);
	dataBtns->addWidget(importBtn);
	dataBtns->addWidget(openBtn);
	dataLay->addLayout(dataBtns);
	pathLabel_ = new QLabel(dataBox);
	pathLabel_->setWordWrap(true);
	dataLay->addWidget(pathLabel_);
	root->addWidget(dataBox);

	root->addStretch(1);

	liveTimer_ = new QTimer(this);
	liveTimer_->setInterval(200);

	connect(enumBtn, &QPushButton::clicked, this, &CameraPanelWidget::onEnumerate);
	connect(brandCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraPanelWidget::onBrandChanged);
	connect(deviceCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraPanelWidget::onDevicePicked);
	connect(connBtn, &QPushButton::clicked, this, &CameraPanelWidget::onConnect);
	connect(discBtn, &QPushButton::clicked, this, &CameraPanelWidget::onDisconnect);
	connect(grabBtn, &QPushButton::clicked, this, &CameraPanelWidget::onGrab);
	connect(live_, &QCheckBox::toggled, this, &CameraPanelWidget::onLiveToggled);
	connect(liveTimer_, &QTimer::timeout, this, &CameraPanelWidget::onGrab);
	connect(saveBtn, &QPushButton::clicked, this, &CameraPanelWidget::onSave);
	connect(importBtn, &QPushButton::clicked, this, &CameraPanelWidget::onImportCloud);
	connect(openBtn, &QPushButton::clicked, this, &CameraPanelWidget::onOpenDir);

	setConnectedUi(false);
	onBrandChanged(brandCombo_->currentIndex());
	applyLanguage();
}

CameraPanelWidget::~CameraPanelWidget()
{
	onDisconnect();
}

void CameraPanelWidget::setUseChinese(bool zh)
{
	zh_ = zh;
}

void CameraPanelWidget::applyLanguage()
{
	auto title = [this](QGroupBox* b, const char* zh, const char* en) {
		if (b)
			b->setTitle(zh_ ? QString::fromUtf8(zh) : QString::fromUtf8(en));
	};
	const auto boxes = findChildren<QGroupBox*>();
	if (boxes.size() >= 1)
		title(boxes[0], "设备", "Device");
	if (boxes.size() >= 2)
		title(boxes[1], "预览", "Preview");
	if (boxes.size() >= 3)
		title(boxes[2], "数据", "Data");

	if (auto* b = findChild<QPushButton*>(QStringLiteral("enumBtn")))
		b->setText(zh_ ? QStringLiteral("枚举设备") : QStringLiteral("Enumerate"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("connBtn")))
		b->setText(zh_ ? QStringLiteral("连接") : QStringLiteral("Connect"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("discBtn")))
		b->setText(zh_ ? QStringLiteral("断开") : QStringLiteral("Disconnect"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("grabBtn")))
		b->setText(zh_ ? QStringLiteral("拍照") : QStringLiteral("Grab"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("saveBtn")))
		b->setText(zh_ ? QStringLiteral("保存到 resource") : QStringLiteral("Save"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("importBtn")))
		b->setText(zh_ ? QStringLiteral("导入点云") : QStringLiteral("Import cloud"));
	if (auto* b = findChild<QPushButton*>(QStringLiteral("openBtn")))
		b->setText(zh_ ? QStringLiteral("打开目录") : QStringLiteral("Open folder"));
	live_->setText(zh_ ? QStringLiteral("连续预览") : QStringLiteral("Live"));
	saveColor_->setText(zh_ ? QStringLiteral("彩色") : QStringLiteral("Color"));
	saveDepth_->setText(zh_ ? QStringLiteral("深度") : QStringLiteral("Depth"));
	saveCloud_->setText(zh_ ? QStringLiteral("点云") : QStringLiteral("Point cloud"));
	preview_->setText(zh_ ? QStringLiteral("无图像") : QStringLiteral("No image"));
}

void CameraPanelWidget::appendLog(const QString& s)
{
	if (!host_)
		return;
	// 错误类文案走 logError，其余进宿主日志页
	const bool isErr = s.contains(QStringLiteral("失败"), Qt::CaseInsensitive)
					   || s.contains(QStringLiteral("fail"), Qt::CaseInsensitive)
					   || s.contains(QStringLiteral("错误"), Qt::CaseInsensitive)
					   || s.contains(QStringLiteral("无法"), Qt::CaseInsensitive)
					   || s.contains(QStringLiteral("未发现"), Qt::CaseInsensitive)
					   || s.contains(QStringLiteral("No "), Qt::CaseInsensitive);
	const QString msg = QStringLiteral("[工业相机] %1").arg(s);
	if (isErr)
		host_->logError(msg);
	else
		host_->logInfo(msg);
}

void CameraPanelWidget::setConnectedUi(bool on)
{
	if (auto* b = findChild<QPushButton*>(QStringLiteral("grabBtn")))
		b->setEnabled(on);
	if (auto* b = findChild<QPushButton*>(QStringLiteral("saveBtn")))
		b->setEnabled(on);
	live_->setEnabled(on);
	if (!on)
	{
		live_->setChecked(false);
		status_->setText(zh_ ? QStringLiteral("未连接") : QStringLiteral("Disconnected"));
	}
}

void CameraPanelWidget::refreshCapabilityChecks()
{
	unsigned caps = CapImage2D;
	if (camera_)
		caps = camera_->deviceInfo().capabilities;
	saveColor_->setEnabled(caps & CapImage2D);
	saveDepth_->setEnabled(caps & CapDepth);
	saveCloud_->setEnabled(caps & CapPointCloud);
	if (!(caps & CapDepth))
		saveDepth_->setChecked(false);
	if (!(caps & CapPointCloud))
		saveCloud_->setChecked(false);
}

void CameraPanelWidget::onBrandChanged(int)
{
	const auto brand = static_cast<CameraBrand>(brandCombo_->currentData().toInt());
	if (brand == CameraBrand::MechMind)
	{
		// 梅卡仅 3D 面阵；灰显 2D MVS 避免误选
		typeCombo_->setCurrentIndex(1);
		typeCombo_->setEnabled(false);
	}
	else if (brand == CameraBrand::Simulated)
	{
		typeCombo_->setEnabled(true);
	}
	else
	{
		typeCombo_->setEnabled(true);
	}
}

void CameraPanelWidget::onEnumerate()
{
	deviceCombo_->clear();
	const auto brand = static_cast<CameraBrand>(brandCombo_->currentData().toInt());
	const auto list = factory_->enumerate(brand);
	if (list.empty())
	{
		appendLog(factory_->lastError().empty()
					  ? (zh_ ? QStringLiteral("未发现设备") : QStringLiteral("No devices"))
					  : QString::fromStdString(factory_->lastError()));
		return;
	}
	for (const auto& d : list)
	{
		const QString text = QStringLiteral("%1 | %2 | %3")
								 .arg(QString::fromStdString(d.ip.empty() ? "-" : d.ip),
									  QString::fromStdString(d.serial),
									  QString::fromStdString(d.model));
		deviceCombo_->addItem(text, QVariant::fromValue(QString::fromStdString(d.serial + "\n" + d.ip)));
	}
	appendLog(zh_ ? QStringLiteral("枚举到 %1 台").arg(list.size()) : QStringLiteral("Found %1").arg(list.size()));
}

void CameraPanelWidget::onDevicePicked(int index)
{
	if (index < 0)
		return;
	const QString payload = deviceCombo_->itemData(index).toString();
	const auto parts = payload.split(QLatin1Char('\n'));
	if (parts.size() >= 2 && !parts[1].isEmpty() && parts[1] != QLatin1String("-"))
		ipEdit_->setText(parts[1]);
}

void CameraPanelWidget::onConnect()
{
	onDisconnect();
	const auto brand = static_cast<CameraBrand>(brandCombo_->currentData().toInt());
	const bool want3d = typeCombo_->currentData().toInt() == 1;
	if (brand == CameraBrand::Hikvision)
		camera_ = want3d ? industrial_camera::createHikMv3dCamera() : industrial_camera::createHikMvsCamera();
	else if (brand == CameraBrand::MechMind)
		camera_ = industrial_camera::createMechEyeCamera();
	else
		camera_ = factory_->create(brand);
	if (!camera_)
	{
		appendLog(QString::fromStdString(factory_->lastError()));
		return;
	}
	CameraConnectParams p;
	p.brand = brand;
	p.ip = ipEdit_->text().trimmed().toStdString();
	const QString payload = deviceCombo_->currentData().toString();
	const auto parts = payload.split(QLatin1Char('\n'));
	if (!parts.isEmpty())
		p.serial = parts[0].toStdString();
	p.timeoutMs = timeoutSpin_->value();
	if (!camera_->connect(p))
	{
		appendLog(QString::fromStdString(camera_->lastError()));
		camera_.reset();
		return;
	}
	camera_->startGrab();
	const auto info = camera_->deviceInfo();
	status_->setText(zh_ ? QStringLiteral("已连接 %1").arg(QString::fromStdString(info.ip))
						 : QStringLiteral("Connected %1").arg(QString::fromStdString(info.ip)));
	setConnectedUi(true);
	refreshCapabilityChecks();
	appendLog(status_->text());
}

void CameraPanelWidget::onDisconnect()
{
	liveTimer_->stop();
	if (camera_)
	{
		camera_->disconnect();
		camera_.reset();
	}
	setConnectedUi(false);
}

void CameraPanelWidget::onGrab()
{
	if (!camera_)
		return;
	if (!camera_->grabOne(*last2d_, last3d_.get(), timeoutSpin_->value()))
	{
		appendLog(QString::fromStdString(camera_->lastError()));
		return;
	}
	updatePreview(*last2d_);
	emit frameCaptured();
}

void CameraPanelWidget::onLiveToggled(bool on)
{
	if (on)
		liveTimer_->start();
	else
		liveTimer_->stop();
}

void CameraPanelWidget::updatePreview(const CameraFrame2D& f)
{
	if (f.bytes.empty() || f.width <= 0)
		return;
	QImage img;
	if (f.pixelFormat == PixelFormat::Bgr8)
		img = QImage(f.bytes.data(), f.width, f.height, f.width * 3, QImage::Format_RGB888).rgbSwapped().copy();
	else
		img = QImage(f.bytes.data(), f.width, f.height, f.width, QImage::Format_Grayscale8).copy();
	preview_->setPixmap(QPixmap::fromImage(img).scaled(preview_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void CameraPanelWidget::onSave()
{
	if (!camera_)
		return;
	industrial_camera::CameraIntrinsics K;
	camera_->getIntrinsics(K);
	const CameraFrame2D* c = saveColor_->isChecked() ? last2d_.get() : nullptr;
	const CameraFrame3D* cloud = saveCloud_->isChecked() ? last3d_.get() : nullptr;
	QString err;
	lastCaptureDir_ = industrial_camera_ui::saveCaptureSession(camera_->deviceInfo(), c, cloud, &K, &err);
	if (lastCaptureDir_.isEmpty())
	{
		appendLog(err);
		return;
	}
	pathLabel_->setText(lastCaptureDir_);
	appendLog(zh_ ? QStringLiteral("已保存") : QStringLiteral("Saved"));
}

void CameraPanelWidget::onImportCloud()
{
	if (!host_ || lastCaptureDir_.isEmpty())
	{
		appendLog(zh_ ? QStringLiteral("请先保存点云") : QStringLiteral("Save cloud first"));
		return;
	}
	const QString ply = QDir(lastCaptureDir_).filePath(QStringLiteral("cloud.ply"));
	if (!QFileInfo::exists(ply))
	{
		appendLog(zh_ ? QStringLiteral("无 cloud.ply") : QStringLiteral("No cloud.ply"));
		return;
	}
	const std::string id = host_->importFileIntoActiveDocument(ply.toStdString(), true);
	appendLog(id.empty() ? (zh_ ? QStringLiteral("导入失败") : QStringLiteral("Import failed"))
						 : (zh_ ? QStringLiteral("已导入 ") + QString::fromStdString(id)
								: QStringLiteral("Imported ") + QString::fromStdString(id)));
}

void CameraPanelWidget::onOpenDir()
{
	const QString root = lastCaptureDir_.isEmpty() ? industrial_camera_ui::industrialCameraRoot() : lastCaptureDir_;
	industrial_camera_ui::ensureIndustrialCameraRoot(nullptr);
	QDesktopServices::openUrl(QUrl::fromLocalFile(root));
}
