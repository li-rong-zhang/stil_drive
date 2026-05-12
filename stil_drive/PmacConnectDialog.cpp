#include "PmacConnectDialog.h"

#include <QLineEdit>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QIntValidator>

PmacConnectDialog::PmacConnectDialog(const QString& initialIp, quint16 initialPort, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("连接 Power PMAC 控制器"));
    setMinimumWidth(320);

    auto* layout = new QVBoxLayout(this);
    auto* form   = new QFormLayout();

    m_editIp = new QLineEdit(initialIp, this);
    form->addRow(QStringLiteral("控制器 IP："), m_editIp);

    m_editPort = new QLineEdit(QString::number(initialPort), this);
    m_editPort->setValidator(new QIntValidator(1, 65535, m_editPort));
    form->addRow(QStringLiteral("端口："), m_editPort);

    layout->addLayout(form);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    btns->button(QDialogButtonBox::Ok)->setText(QStringLiteral("连接"));
    btns->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    connect(btns, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(btns);
}

QString PmacConnectDialog::ip() const
{
    return m_editIp->text().trimmed();
}

quint16 PmacConnectDialog::port() const
{
    return (quint16)m_editPort->text().toUShort();
}
