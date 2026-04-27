#pragma once
#include <QObject>
#include <QTcpSocket>

class PmacController : public QObject
{
    Q_OBJECT

public:
    explicit PmacController(QObject* parent = nullptr);
    ~PmacController();

    void connectToController(const QString& ip, quint16 port);
    void disconnectController();
    void sendCommand(const QString& cmd);
    bool isConnected() const;

signals:
    void connected();
    void disconnected();
    void logMessage(const QString& msg);

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void onError(QTcpSocket::SocketError socketError);

private:
    QTcpSocket* m_socket;
};
