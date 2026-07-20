/// @file PlcCommWorker.cpp
/// @brief PlcCommWorker 实现

#include "PlcCommWorker.h"

#include "IPlcCommClient.h"

PlcCommWorker::PlcCommWorker(QObject* parent) : QObject(parent), client_(createPlcCommClient()) {}

PlcCommWorker::~PlcCommWorker() = default;

void PlcCommWorker::setUseChinese(bool chinese)
{
	useChinese_ = chinese;
}

QString PlcCommWorker::i18n(const QString& en, const QString& zh) const
{
	return useChinese_ ? zh : en;
}

void PlcCommWorker::connectPlc(const PlcConnectionConfig& config)
{
	const bool ok = client_->connect(config);
	if (!ok)
	{
		emit errorOccurred(QString::fromStdString(client_->lastError()));
	}
	else if (config.protocol == PlcProtocol::ModbusTcp)
	{
		emit logMessage(i18n(QStringLiteral("Modbus TCP connected"), QStringLiteral("Modbus TCP 已连通")));
	}
	else
	{
		emit logMessage(i18n(QStringLiteral("EIP settings saved; session starts when a tag is added"),
							 QStringLiteral("EIP 参数已就绪（添加标签后建立会话，非立即连通）")));
	}
	emit connectedChanged(client_->isConnected());
}

void PlcCommWorker::disconnectPlc()
{
	client_->disconnect();
	emit logMessage(i18n(QStringLiteral("Disconnected"), QStringLiteral("已断开")));
	emit connectedChanged(false);
}

void PlcCommWorker::addTag(const PlcTagSpec& spec)
{
	const int handle = client_->addTag(spec);
	if (handle < 0)
	{
		emit errorOccurred(QString::fromStdString(client_->lastError()));
		return;
	}
	emit tagAdded(handle, QString::fromStdString(spec.name));
	emit logMessage(i18n(QStringLiteral("Added tag %1 -> #%2").arg(QString::fromStdString(spec.name)).arg(handle),
						 QStringLiteral("添加标签 %1 -> #%2").arg(QString::fromStdString(spec.name)).arg(handle)));
}

void PlcCommWorker::removeTag(int handle)
{
	client_->removeTag(handle);
	emit logMessage(i18n(QStringLiteral("Removed tag #%1").arg(handle), QStringLiteral("移除标签 #%1").arg(handle)));
}

void PlcCommWorker::readTag(int handle)
{
	PlcTagValue value;
	const bool ok = client_->readTag(handle, value);
	if (!ok)
	{
		emit errorOccurred(QString::fromStdString(client_->lastError()));
		emit tagRead(handle, QByteArray(), false);
		return;
	}
	QByteArray bytes(reinterpret_cast<const char*>(value.data.data()), static_cast<int>(value.data.size()));
	emit tagRead(handle, bytes, true);
}

void PlcCommWorker::writeTag(int handle, const QByteArray& data)
{
	PlcTagValue value;
	value.data.assign(reinterpret_cast<const uint8_t*>(data.constData()),
					  reinterpret_cast<const uint8_t*>(data.constData() + data.size()));
	const bool ok = client_->writeTag(handle, value);
	if (!ok)
	{
		emit errorOccurred(QString::fromStdString(client_->lastError()));
	}
	emit tagWriteFinished(handle, ok);
}

void PlcCommWorker::pollTags(const QList<int>& handles)
{
	for (int handle : handles)
	{
		readTag(handle);
	}
}
