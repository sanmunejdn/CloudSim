/// @file ExternalControllerDialog.cpp
/// @brief 外置控制器设置对话框：选语言、看源码、启监听、跑 Python

#include "ExternalControllerDialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTextStream>
#include <QUrl>
#include <QVBoxLayout>

namespace
{
const int kDefaultPort = 19620;
}

ExternalControllerDialog::ExternalControllerDialog(QWidget* parent) : QDialog(parent)
{
	setModal(false);
	resize(720, 560);

	auto* root = new QVBoxLayout(this);

	auto* top = new QHBoxLayout;
	m_langCombo = new QComboBox(this);
	m_langCombo->addItem(QStringLiteral("Python"), static_cast<int>(Lang::Python));
	m_langCombo->addItem(QStringLiteral("C++"), static_cast<int>(Lang::Cpp));
	m_scriptCombo = new QComboBox(this);
	m_scriptCombo->addItem(QStringLiteral("example_sine_joints.py"),
						   QStringLiteral("example_sine_joints.py"));
	m_scriptCombo->addItem(QStringLiteral("cloudsim_controller.py"),
						   QStringLiteral("cloudsim_controller.py"));
	m_portLabel = new QLabel(this);
	top->addWidget(m_langCombo);
	top->addWidget(m_scriptCombo, 1);
	top->addWidget(m_portLabel);
	root->addLayout(top);

	auto* listenRow = new QHBoxLayout;
	m_listenCheck = new QCheckBox(this);
	m_statusLabel = new QLabel(this);
	listenRow->addWidget(m_listenCheck);
	listenRow->addWidget(m_statusLabel, 1);
	root->addLayout(listenRow);

	m_sourceEdit = new QPlainTextEdit(this);
	QFont mono = m_sourceEdit->font();
	mono.setFamily(QStringLiteral("Consolas"));
	mono.setStyleHint(QFont::Monospace);
	m_sourceEdit->setFont(mono);
	root->addWidget(m_sourceEdit, 1);

	auto* btnRow = new QHBoxLayout;
	m_reloadBtn = new QPushButton(this);
	m_runBtn = new QPushButton(this);
	m_stopBtn = new QPushButton(this);
	m_openControllersBtn = new QPushButton(this);
	m_closeBtn = new QPushButton(this);
	btnRow->addWidget(m_reloadBtn);
	btnRow->addWidget(m_runBtn);
	btnRow->addWidget(m_stopBtn);
	btnRow->addWidget(m_openControllersBtn);
	btnRow->addStretch(1);
	btnRow->addWidget(m_closeBtn);
	root->addLayout(btnRow);

	m_process = new QProcess(this);
	connect(m_langCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&ExternalControllerDialog::onLanguageChanged);
	connect(m_scriptCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&ExternalControllerDialog::onScriptFileChanged);
	connect(m_listenCheck, &QCheckBox::toggled, this, &ExternalControllerDialog::onListenCheckToggled);
	connect(m_reloadBtn, &QPushButton::clicked, this, &ExternalControllerDialog::onReloadClicked);
	connect(m_runBtn, &QPushButton::clicked, this, &ExternalControllerDialog::onRunClicked);
	connect(m_stopBtn, &QPushButton::clicked, this, &ExternalControllerDialog::onStopClicked);
	connect(m_openControllersBtn, &QPushButton::clicked, this,
			&ExternalControllerDialog::onOpenControllersClicked);
	connect(m_closeBtn, &QPushButton::clicked, this, &QDialog::close);
	connect(m_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
			&ExternalControllerDialog::onProcessFinished);

	retranslate();
	refreshSourceView();
	updateStatusLabel();
	updateRunStopEnabled();
}

ExternalControllerDialog::~ExternalControllerDialog() = default;

void ExternalControllerDialog::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	retranslate();
	updateStatusLabel();
}

void ExternalControllerDialog::setListening(bool listening)
{
	m_listening = listening;
	{
		const QSignalBlocker blocker(m_listenCheck);
		m_listenCheck->setChecked(listening);
	}
	updateStatusLabel();
	updateRunStopEnabled();
}

void ExternalControllerDialog::setClientConnected(bool connected)
{
	m_clientConnected = connected;
	updateStatusLabel();
}

bool ExternalControllerDialog::isListeningChecked() const
{
	return m_listenCheck && m_listenCheck->isChecked();
}

ExternalControllerDialog::Lang ExternalControllerDialog::currentLang() const
{
	return static_cast<Lang>(m_langCombo->currentData().toInt());
}

void ExternalControllerDialog::retranslate()
{
	setWindowTitle(m_useChinese ? QStringLiteral("外置控制器设置")
								: QStringLiteral("External Controller Settings"));
	m_portLabel->setText(m_useChinese
							 ? QStringLiteral("协议 JSON Lines · 127.0.0.1:%1").arg(kDefaultPort)
							 : QStringLiteral("JSON Lines · 127.0.0.1:%1").arg(kDefaultPort));
	m_listenCheck->setText(m_useChinese ? QStringLiteral("启用 Host 监听")
										: QStringLiteral("Enable Host listen"));
	m_reloadBtn->setText(m_useChinese ? QStringLiteral("重新加载") : QStringLiteral("Reload"));
	m_runBtn->setText(m_useChinese ? QStringLiteral("运行控制器") : QStringLiteral("Run Controller"));
	m_stopBtn->setText(m_useChinese ? QStringLiteral("停止控制器") : QStringLiteral("Stop Controller"));
	m_openControllersBtn->setText(m_useChinese ? QStringLiteral("打开控制器目录")
											   : QStringLiteral("Open controllers/"));
	m_closeBtn->setText(m_useChinese ? QStringLiteral("关闭") : QStringLiteral("Close"));
	m_scriptCombo->setEnabled(currentLang() == Lang::Python);
}

void ExternalControllerDialog::updateStatusLabel()
{
	QString text;
	if (!m_listening)
		text = m_useChinese ? QStringLiteral("状态：未监听") : QStringLiteral("Status: idle");
	else if (m_clientConnected)
		text = m_useChinese ? QStringLiteral("状态：监听中 · 已连接客户端")
							: QStringLiteral("Status: listening · client connected");
	else
		text = m_useChinese ? QStringLiteral("状态：监听中") : QStringLiteral("Status: listening");

	if (m_process && m_process->state() != QProcess::NotRunning)
	{
		text += m_useChinese ? QStringLiteral(" · 控制器进程运行中")
							 : QStringLiteral(" · controller process running");
	}
	m_statusLabel->setText(text);
}

void ExternalControllerDialog::updateRunStopEnabled()
{
	const bool running = m_process && m_process->state() != QProcess::NotRunning;
	m_runBtn->setEnabled(!running);
	m_stopBtn->setEnabled(running);
	m_langCombo->setEnabled(!running);
	m_scriptCombo->setEnabled(!running && currentLang() == Lang::Python);
}

QString ExternalControllerDialog::controllerPythonDir() const
{
	// 优先 OutDir；开发态回退仓库 CloudSim/resource
	const QDir app(QCoreApplication::applicationDirPath());
	const QString outDir = app.filePath(QStringLiteral("resource/Python/ControllerPython"));
	if (QFileInfo::exists(QDir(outDir).filePath(QStringLiteral("example_sine_joints.py"))) ||
		QFileInfo::exists(QDir(outDir).filePath(QStringLiteral("cloudsim_controller.py"))))
		return outDir;

	const QString candidates[] = {
		app.absoluteFilePath(QStringLiteral("../../CloudSim/resource/Python/ControllerPython")),
		app.absoluteFilePath(QStringLiteral("../../../CloudSim/resource/Python/ControllerPython")),
	};
	for (const QString& c : candidates)
	{
		if (QFileInfo::exists(QDir(c).filePath(QStringLiteral("example_sine_joints.py"))))
			return QDir(c).absolutePath();
	}
	return outDir;
}

QString ExternalControllerDialog::controllersRootDir() const
{
	const QDir app(QCoreApplication::applicationDirPath());
	const QString outDir = app.filePath(QStringLiteral("resource/controllers"));
	if (QDir(outDir).exists())
		return outDir;
	const QString fallback =
		app.absoluteFilePath(QStringLiteral("../../CloudSim/resource/controllers"));
	if (QDir(fallback).exists())
		return QDir(fallback).absolutePath();
	return outDir;
}

QString ExternalControllerDialog::pythonScriptPath(const QString& fileName) const
{
	return QDir(controllerPythonDir()).filePath(fileName);
}

QString ExternalControllerDialog::loadPythonSource(const QString& fileName) const
{
	QFile f(pythonScriptPath(fileName));
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return QString();
	QTextStream in(&f);
	in.setCodec("UTF-8");
	return in.readAll();
}

QString ExternalControllerDialog::embeddedCppSample() const
{
	return QStringLiteral(
		R"cpp(// CloudSimControllerSDK 最小示例（MVP：请自行编译为 exe 后运行）
#include "IControllerClient.h"
#include <cmath>
#include <thread>
#include <chrono>

int main()
{
	auto client = createControllerClient();
	ControllerEndpoint ep;
	ep.host = "127.0.0.1";
	ep.port = 19620;
	if (!client->connectHost(ep))
		return 1;
	ControllerHelloAck ack;
	if (!client->hello(0, ack))
		return 2;
	const int n = ack.jointCount;
	const int dt = ack.simDtMs > 0 ? ack.simDtMs : 16;
	for (int k = 0; k < 1000; ++k)
	{
		std::vector<double> q(static_cast<size_t>(n), 0.0);
		for (int i = 0; i < n; ++i)
			q[static_cast<size_t>(i)] = 0.3 * std::sin(0.05 * k + 0.4 * i);
		ControllerStepReply reply;
		if (!client->step(dt, q, reply))
			break;
		std::this_thread::sleep_for(std::chrono::milliseconds(dt));
	}
	client->goodbye();
	return 0;
}
)cpp");
}

void ExternalControllerDialog::refreshSourceView()
{
	if (currentLang() == Lang::Cpp)
	{
		m_sourceEdit->setReadOnly(true);
		m_sourceEdit->setPlainText(embeddedCppSample());
		return;
	}

	m_sourceEdit->setReadOnly(false);
	const QString file = m_scriptCombo->currentData().toString();
	QString src = loadPythonSource(file);
	if (src.isEmpty())
	{
		src = m_useChinese
				  ? QStringLiteral("# 未找到源文件：%1\n# 请确认已拷贝到 OutDir/resource/Python/ControllerPython/")
						.arg(pythonScriptPath(file))
				  : QStringLiteral("# Source not found: %1\n# Copy ControllerPython under OutDir/resource/")
						.arg(pythonScriptPath(file));
	}
	m_sourceEdit->setPlainText(src);
}

void ExternalControllerDialog::onLanguageChanged(int)
{
	retranslate();
	refreshSourceView();
	updateRunStopEnabled();
}

void ExternalControllerDialog::onScriptFileChanged(int)
{
	if (currentLang() == Lang::Python)
		refreshSourceView();
}

void ExternalControllerDialog::onListenCheckToggled(bool on)
{
	emit listenToggled(on);
}

void ExternalControllerDialog::onReloadClicked()
{
	refreshSourceView();
}

void ExternalControllerDialog::onRunClicked()
{
	if (!m_listening)
	{
		QMessageBox::warning(
			this, windowTitle(),
			m_useChinese ? QStringLiteral("请先勾选「启用 Host 监听」。")
						 : QStringLiteral("Enable Host listen first."));
		return;
	}

	if (currentLang() == Lang::Cpp)
	{
		const QString exe =
			QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("CloudSimControllerSample.exe"));
		if (!QFileInfo::exists(exe))
		{
			QMessageBox::warning(
				this, windowTitle(),
				m_useChinese
					? QStringLiteral("未找到 %1\n请先编译 CloudSimControllerSample。").arg(exe)
					: QStringLiteral("Missing %1\nBuild CloudSimControllerSample first.").arg(exe));
			return;
		}
		if (m_process->state() != QProcess::NotRunning)
			m_process->kill();
		m_process->setWorkingDirectory(QCoreApplication::applicationDirPath());
		m_process->setProgram(exe);
		m_process->setArguments({});
		m_process->start();
		if (!m_process->waitForStarted(3000))
		{
			QMessageBox::warning(this, windowTitle(), m_process->errorString());
			updateRunStopEnabled();
			return;
		}
		updateStatusLabel();
		updateRunStopEnabled();
		return;
	}

	const QString dir = controllerPythonDir();
	if (!QDir(dir).exists())
	{
		QMessageBox::warning(
			this, windowTitle(),
			m_useChinese ? QStringLiteral("找不到目录：%1").arg(dir)
						 : QStringLiteral("Directory missing: %1").arg(dir));
		return;
	}

	const QString runFile = QDir(dir).filePath(QStringLiteral("_dialog_run.py"));
	{
		QFile out(runFile);
		if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
		{
			QMessageBox::warning(this, windowTitle(),
								 m_useChinese ? QStringLiteral("无法写入临时脚本")
											  : QStringLiteral("Cannot write temp script"));
			return;
		}
		QTextStream ts(&out);
		ts.setCodec("UTF-8");
		ts << m_sourceEdit->toPlainText();
	}

	if (m_process->state() != QProcess::NotRunning)
		m_process->kill();

	m_process->setWorkingDirectory(dir);
	m_process->setProgram(QStringLiteral("python"));
	m_process->setArguments({runFile});
	m_process->start();
	if (!m_process->waitForStarted(3000))
	{
		m_process->setProgram(QStringLiteral("py"));
		m_process->setArguments({QStringLiteral("-3"), runFile});
		m_process->start();
		if (!m_process->waitForStarted(3000))
		{
			QMessageBox::warning(
				this, windowTitle(),
				m_useChinese ? QStringLiteral("无法启动 python/py：%1").arg(m_process->errorString())
							 : QStringLiteral("Failed to start python/py: %1").arg(m_process->errorString()));
			updateRunStopEnabled();
			return;
		}
	}
	updateStatusLabel();
	updateRunStopEnabled();
}

void ExternalControllerDialog::onOpenControllersClicked()
{
	const QString dir = controllersRootDir();
	QDir().mkpath(dir);
	QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
}

void ExternalControllerDialog::onStopClicked()
{
	if (!m_process || m_process->state() == QProcess::NotRunning)
		return;
	m_process->terminate();
	if (!m_process->waitForFinished(1500))
		m_process->kill();
	updateStatusLabel();
	updateRunStopEnabled();
}

void ExternalControllerDialog::onProcessFinished(int, QProcess::ExitStatus)
{
	updateStatusLabel();
	updateRunStopEnabled();
}
