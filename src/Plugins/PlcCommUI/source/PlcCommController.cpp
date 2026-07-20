/// @file PlcCommController.cpp
/// @brief PlcCommController 实现

#include "PlcCommController.h"

#include "PlcCommWorker.h"

#include <QMetaType>
#include <QThread>

Q_DECLARE_METATYPE(PlcConnectionConfig)
Q_DECLARE_METATYPE(PlcTagSpec)
Q_DECLARE_METATYPE(QList<int>)

PlcCommController::PlcCommController(QObject* parent) : QObject(parent)
{
	qRegisterMetaType<PlcConnectionConfig>("PlcConnectionConfig");
	qRegisterMetaType<PlcTagSpec>("PlcTagSpec");
	qRegisterMetaType<QList<int>>("QList<int>");
}

PlcCommController::~PlcCommController()
{
	stopWorker();
}

void PlcCommController::startWorker()
{
	if (thread_)
	{
		return;
	}
	thread_ = new QThread(this);
	worker_ = new PlcCommWorker();
	worker_->moveToThread(thread_);

	connect(thread_, &QThread::finished, worker_, &QObject::deleteLater);
	connect(worker_, &PlcCommWorker::connectedChanged, this, &PlcCommController::connectedChanged);
	connect(worker_, &PlcCommWorker::tagAdded, this, &PlcCommController::tagAdded);
	connect(worker_, &PlcCommWorker::tagRead, this, &PlcCommController::tagRead);
	connect(worker_, &PlcCommWorker::tagWriteFinished, this, &PlcCommController::tagWriteFinished);
	connect(worker_, &PlcCommWorker::logMessage, this, &PlcCommController::logMessage);
	connect(worker_, &PlcCommWorker::errorOccurred, this, &PlcCommController::errorOccurred);

	thread_->start();
}

void PlcCommController::setUseChinese(bool chinese)
{
	useChinese_ = chinese;
	if (!worker_)
	{
		return;
	}
	QMetaObject::invokeMethod(worker_, "setUseChinese", Qt::QueuedConnection, Q_ARG(bool, chinese));
}

void PlcCommController::stopWorker()
{
	if (!thread_)
	{
		return;
	}
	thread_->quit();
	if (!thread_->wait(3000))
	{
		thread_->terminate();
		thread_->wait(1000);
	}
	worker_ = nullptr;
	thread_ = nullptr;
}

void PlcCommController::connectPlc(const PlcConnectionConfig& config)
{
	QMetaObject::invokeMethod(worker_, "connectPlc", Qt::QueuedConnection, Q_ARG(PlcConnectionConfig, config));
}

void PlcCommController::disconnectPlc()
{
	QMetaObject::invokeMethod(worker_, "disconnectPlc", Qt::QueuedConnection);
}

void PlcCommController::addTag(const PlcTagSpec& spec)
{
	QMetaObject::invokeMethod(worker_, "addTag", Qt::QueuedConnection, Q_ARG(PlcTagSpec, spec));
}

void PlcCommController::removeTag(int handle)
{
	QMetaObject::invokeMethod(worker_, "removeTag", Qt::QueuedConnection, Q_ARG(int, handle));
}

void PlcCommController::readTag(int handle)
{
	QMetaObject::invokeMethod(worker_, "readTag", Qt::QueuedConnection, Q_ARG(int, handle));
}

void PlcCommController::writeTag(int handle, const QByteArray& data)
{
	QMetaObject::invokeMethod(worker_, "writeTag", Qt::QueuedConnection, Q_ARG(int, handle), Q_ARG(QByteArray, data));
}

void PlcCommController::pollTags(const QList<int>& handles)
{
	QMetaObject::invokeMethod(worker_, "pollTags", Qt::QueuedConnection, Q_ARG(QList<int>, handles));
}
