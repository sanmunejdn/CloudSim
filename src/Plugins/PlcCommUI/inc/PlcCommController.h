#ifndef PLCCOMMUI_PLCCOMMCONTROLLER_H
#define PLCCOMMUI_PLCCOMMCONTROLLER_H

/// @file PlcCommController.h
/// @brief UI 与 Worker 线程桥接

#include "PlcCommTypes.h"

#include <QList>
#include <QObject>

class PlcCommWorker;
class QThread;

/// UI 与 Worker 线程桥接
class PlcCommController : public QObject
{
	Q_OBJECT

public:
	explicit PlcCommController(QObject* parent = nullptr);
	~PlcCommController() override;

	PlcCommWorker* worker() const { return worker_; }

	void startWorker();
	void stopWorker();
	void setUseChinese(bool chinese);

public slots:
	void connectPlc(const PlcConnectionConfig& config);
	void disconnectPlc();
	void addTag(const PlcTagSpec& spec);
	void removeTag(int handle);
	void readTag(int handle);
	void writeTag(int handle, const QByteArray& data);
	void pollTags(const QList<int>& handles);

signals:
	void connectedChanged(bool connected);
	void tagAdded(int handle, const QString& name);
	void tagRead(int handle, const QByteArray& data, bool ok);
	void tagWriteFinished(int handle, bool ok);
	void logMessage(const QString& text);
	void errorOccurred(const QString& text);

private:
	bool useChinese_ = true;
	QThread* thread_ = nullptr;
	PlcCommWorker* worker_ = nullptr;
};

#endif // PLCCOMMUI_PLCCOMMCONTROLLER_H
