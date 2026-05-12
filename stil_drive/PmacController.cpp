// ============================================================================
// PmacController.cpp
// Delta Tau Power PMAC 以太网通讯控制器
// ============================================================================
// 通过 TCP/Telnet 连接到 Power PMAC 运动控制器：
//   - 自动处理 Telnet IAC 选项协商（WILL/WONT/DO/DONT）
//   - 自动完成 login → password → gpascii 登录流程
//   - 向上层 UI 通过 signal 传递日志，不直接操作 UI
//
// 使用方式：
//   PmacController* pm = new PmacController(this);
//   connect(pm, &PmacController::connected, this, ...);
//   connect(pm, &PmacController::logMessage, logBrowser, ...);
//   pm->connectToController("192.168.0.200", 5002);
//   pm->sendCommand("Motor[1].JogSpeed=128000");
// ============================================================================

#include "PmacController.h"
#include <QHostAddress>
#include <QRegularExpression>

PmacController::PmacController(QObject* parent)
    : QObject(parent)
    , m_socket(new QTcpSocket(this))
{
    // 将 socket 信号连接到本对象槽函数，实现自封装
    connect(m_socket, &QTcpSocket::connected, this, &PmacController::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &PmacController::onDisconnected);
    connect(m_socket, &QTcpSocket::readyRead, this, &PmacController::onReadyRead);
    connect(m_socket, &QTcpSocket::errorOccurred, this, &PmacController::onError);
}

PmacController::~PmacController()
{
    if (m_socket->state() == QAbstractSocket::ConnectedState) {
        m_socket->disconnectFromHost();
    }
}

void PmacController::connectToController(const QString& ip, quint16 port)
{
    m_socket->connectToHost(ip, port);
}

void PmacController::disconnectController()
{
    m_socket->disconnectFromHost();
}

// ============================================================================
// sendCommand
// 发送 PMAC 命令，自动追加换行符，确保命令被正确识别
// ============================================================================
void PmacController::sendCommand(const QString& cmd)
{
    if (m_socket->state() != QAbstractSocket::ConnectedState) return;
    QString cmdWithNewline = cmd;
    if (!cmdWithNewline.endsWith("\n")) cmdWithNewline += "\n";
    m_socket->write(cmdWithNewline.toUtf8());
}

bool PmacController::isConnected() const
{
    return m_socket->state() == QAbstractSocket::ConnectedState;
}

void PmacController::onConnected()
{
    emit connected();
}

void PmacController::onDisconnected()
{
    emit disconnected();
}

// ============================================================================
// onReadyRead
// Telnet 协议处理：过滤 IAC 字节流，执行自动登录状态机
//
// Telnet 选项协商字节序列（0xFF 开头）：
//   IAC WILL X  → 对方请求我同意执行选项 X
//   IAC WONT X  → 对方保证不再请求选项 X
//   IAC DO X    → 我应执行选项 X
//   IAC DONT X  → 我不应执行选项 X
// 标准回应：WILL→发送 DONT，DO→发送 WONT，拒绝对方改变现状
// ============================================================================
void PmacController::onReadyRead()
{
    QByteArray data = m_socket->readAll();
    QByteArray telnetReply;   // 待发送回 PMAC 的选项协商响应
    QString cleanText;        // 去除 IAC 控制码后的净文本

    // --- 过滤 Telnet IAC 字节流 ---
    for (int i = 0; i < data.size(); ) {
        if ((unsigned char)data[i] == 0xFF) {
            if (i + 2 < data.size()) {
                unsigned char cmd = data[i + 1], opt = data[i + 2];
                if (cmd >= 251 && cmd <= 254) {
                    // 选项协商：发送反向应答，让 PMAC 停止询问
                    telnetReply.append((char)0xFF);
                    if (cmd == 253) telnetReply.append((char)252); // DONT ← WILL
                    else if (cmd == 251) telnetReply.append((char)254); // WONT ← DO
                    else telnetReply.append(cmd);
                    telnetReply.append(opt);
                    i += 3;
                    continue;
                }
            }
            ++i;
        } else {
            cleanText.append(data[i++]);
        }
    }

    // 立即回送协商应答（不等待，防止 PMAC 超时）
    if (!telnetReply.isEmpty()) {
        m_socket->write(telnetReply);
    }

    cleanText = cleanText.trimmed();
    if (cleanText.isEmpty()) return;

    emit logMessage("收到: " + cleanText);

    // --- 自动登录状态机 ---
    // PMAC Telnet 连接后按以下顺序输出提示符：
    //   1. "login:" → 发送 "root"
    //   2. "Password:" → 发送 "deltatau"
    //   3. "~#" 或 "~$" 或 "root@" → 发送 "gpascii -2" 进入 ASCII 命令通道
    //   4. "STDIN Open for ASCII" → 运动引擎就绪，可开始发送指令
    //   5. "\x06" (ACK) → 上条指令执行成功

    if (cleanText.contains("login", Qt::CaseInsensitive)) {
        m_socket->write("root\n");
    } else if (cleanText.contains("Password", Qt::CaseInsensitive)) {
        m_socket->write("deltatau\n");
    } else if (cleanText.contains("~$") || cleanText.contains("~#") || cleanText.contains("root@")) {
        m_socket->write("gpascii -2\n");
    } else if (cleanText.contains("STDIN Open for ASCII", Qt::CaseInsensitive)) {
        emit logMessage("<font color='magenta'><b>-> 运动引擎就绪！可点击【开始采集】</b></font>");
    } else if (cleanText.contains("\x06")) {
        emit logMessage("<font color='blue'>[指令执行成功 (ACK)]</font>");
    }

    // 检测 PROG 运行状态查询响应
    // 当查询 Coord[1].ProgRunning 时，PMAC 会返回 "0" 或 "1"
    // 返回 1 表示程序正在运行，返回 0 表示程序完成
    // 我们需要检测到从 "1" 到 "0" 的变化，表示程序刚完成
    static bool lastProgRunning = false;
    static QRegularExpression progStatusRegex("^[\\s]*([01])[\\s]*$");
    QRegularExpressionMatch match = progStatusRegex.match(cleanText);
    if (match.hasMatch()) {
        int status = match.captured(1).toInt();
        bool currentProgRunning = (status == 1);

        // 检测到程序从运行状态变为停止状态
        if (lastProgRunning && !currentProgRunning) {
            emit progFinished();
        }

        lastProgRunning = currentProgRunning;
    }
}

void PmacController::onError(QTcpSocket::SocketError socketError)
{
    Q_UNUSED(socketError);
    emit logMessage("<font color='red'>网络报错: " + m_socket->errorString() + "</font>");
}
