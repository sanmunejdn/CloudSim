#pragma once

#include "PlcCommTypes.h"

#include <QByteArray>
#include <QList>
#include <QObject>
#include <QString>

#include <memory>

class IPlcCommClient;

/// 独占 IPlcCommClient；所有 PLC I/O 在此线程执行
class PlcCommWorker : public QObject
{
    Q_OBJECT

public:
    explicit PlcCommWorker(QObject* parent = nullptr);
    ~PlcCommWorker() override;

public slots:
    void setUseChinese(bool chinese);
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
    QString i18n(const QString& en, const QString& zh) const;

    bool useChinese_ = true;
    std::unique_ptr<IPlcCommClient> client_;
};
