// ============================================================================
// PmacConnectDialog.h
// Power PMAC 控制器连接配置对话框
// ============================================================================
// 收集 IP / 端口，由 stil_drive 主窗口在 accept 后调用
// PmacController::connectToController() 进行真正的 TCP 连接。
// ============================================================================

#pragma once
#include <QDialog>

class QLineEdit;

class PmacConnectDialog : public QDialog
{
    Q_OBJECT
public:
    PmacConnectDialog(const QString& initialIp = "192.168.0.200",
                      quint16 initialPort = 5002,
                      QWidget* parent = nullptr);

    QString ip() const;
    quint16 port() const;

private:
    QLineEdit* m_editIp;
    QLineEdit* m_editPort;
};
