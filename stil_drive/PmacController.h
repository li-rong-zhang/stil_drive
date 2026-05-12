// ============================================================================
// PmacController.h
// Delta Tau Power PMAC 以太网通讯控制器
// ============================================================================
// 封装 QTcpSocket，处理 Telnet 协议和自动登录，
// 通过 signal 向外传递连接状态和日志，不直接操作 UI。
//
// 使用方式：
//   PmacController* pm = new PmacController(this);
//   connect(pm, &PmacController::logMessage, logBrowser, &QTextBrowser::append);
//   pm->connectToController("192.168.0.200", 5002);
// ============================================================================

#pragma once
#include <QObject>
#include <QTcpSocket>

class PmacController : public QObject
{
    Q_OBJECT

public:
    explicit PmacController(QObject* parent = nullptr);
    ~PmacController();

    // 连接 / 断开 PMAC
    void connectToController(const QString& ip, quint16 port);
    void disconnectController();

    // 发送命令（自动追加 \n）
    void sendCommand(const QString& cmd);

    // 查询连接状态
    bool isConnected() const;

signals:
    void connected();                     // 连接成功
    void disconnected();                  // 连接断开
    void logMessage(const QString& msg); // 日志消息（供 UI 显示）
    void progFinished();                  // PROG 程序运行完成

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void onError(QTcpSocket::SocketError socketError);

private:
    QTcpSocket* m_socket;  // TCP 套接字
};
