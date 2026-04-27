#include "PmacController.h"
#include <QHostAddress>

PmacController::PmacController(QObject* parent)
    : QObject(parent)
    , m_socket(new QTcpSocket(this))
{
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

void PmacController::onReadyRead()
{
    QByteArray data = m_socket->readAll();
    QByteArray telnetReply;
    QString cleanText;

    for (int i = 0; i < data.size(); ) {
        if ((unsigned char)data[i] == 0xFF) {
            if (i + 2 < data.size()) {
                unsigned char cmd = data[i + 1], opt = data[i + 2];
                if (cmd >= 251 && cmd <= 254) {
                    telnetReply.append((char)0xFF);
                    if (cmd == 253) telnetReply.append((char)252);
                    else if (cmd == 251) telnetReply.append((char)254);
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

    if (!telnetReply.isEmpty()) {
        m_socket->write(telnetReply);
    }

    cleanText = cleanText.trimmed();
    if (cleanText.isEmpty()) return;

    emit logMessage("收到: " + cleanText);

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
}

void PmacController::onError(QTcpSocket::SocketError socketError)
{
    Q_UNUSED(socketError);
    emit logMessage("<font color='red'>网络报错: " + m_socket->errorString() + "</font>");
}
