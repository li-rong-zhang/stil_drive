#include "SensorConnectDialog.h"

#include <QListWidget>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLabel>

SensorConnectDialog::SensorConnectDialog(const QStringList& deviceList,
                                         int initialFreqIndex,
                                         QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("连接 STIL 测头"));
    setMinimumWidth(380);

    auto* layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel(QStringLiteral("可用设备："), this));
    m_listDevices = new QListWidget(this);
    if (deviceList.isEmpty()) {
        auto* item = new QListWidgetItem(QStringLiteral("(未扫描到设备)"));
        item->setFlags(Qt::NoItemFlags);
        m_listDevices->addItem(item);
    } else {
        m_listDevices->addItems(deviceList);
        m_listDevices->setCurrentRow(0);
    }
    layout->addWidget(m_listDevices);

    auto* form = new QFormLayout();
    m_cmbFreq = new QComboBox(this);
    m_cmbFreq->addItem(QStringLiteral("100 Hz"));
    m_cmbFreq->addItem(QStringLiteral("200 Hz"));
    m_cmbFreq->addItem(QStringLiteral("400 Hz"));
    m_cmbFreq->addItem(QStringLiteral("1000 Hz"));
    m_cmbFreq->addItem(QStringLiteral("2000 Hz"));
    if (initialFreqIndex >= 0 && initialFreqIndex < m_cmbFreq->count())
        m_cmbFreq->setCurrentIndex(initialFreqIndex);
    else
        m_cmbFreq->setCurrentIndex(3);
    form->addRow(QStringLiteral("采样频率："), m_cmbFreq);
    layout->addLayout(form);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    btns->button(QDialogButtonBox::Ok)->setText(QStringLiteral("连接"));
    btns->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    btns->button(QDialogButtonBox::Ok)->setEnabled(!deviceList.isEmpty());
    connect(btns, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(btns);
}

int SensorConnectDialog::selectedDeviceIndex() const
{
    return m_listDevices->currentRow();
}

int SensorConnectDialog::frequencyIndex() const
{
    return m_cmbFreq->currentIndex();
}

QString SensorConnectDialog::frequencyLabel() const
{
    return m_cmbFreq->currentText();
}
