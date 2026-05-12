#include "AxisMappingDialog.h"

#include <QSpinBox>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QGroupBox>

AxisMappingDialog::AxisMappingDialog(const AxisConfig& initial, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("轴映射配置"));
    setMinimumWidth(320);

    auto* layout = new QVBoxLayout(this);

    // ---- 电机映射 ----
    auto* grpMotor = new QGroupBox(QStringLiteral("逻辑轴 → PMAC 电机号"), this);
    auto* formMotor = new QFormLayout(grpMotor);

    auto makeSpin = [&](int val) {
        auto* sp = new QSpinBox(this);
        sp->setRange(1, 32);
        sp->setValue(val);
        return sp;
    };

    m_spinX = makeSpin(initial.motorX);
    m_spinY = makeSpin(initial.motorY);
    m_spinZ = makeSpin(initial.motorZ);
    m_spinB = makeSpin(initial.motorB);
    m_spinC = makeSpin(initial.motorC);

    formMotor->addRow(QStringLiteral("X 轴 →  #"), m_spinX);
    formMotor->addRow(QStringLiteral("Y 轴 →  #"), m_spinY);
    formMotor->addRow(QStringLiteral("Z 轴 →  #"), m_spinZ);
    formMotor->addRow(QStringLiteral("B 轴 →  #"), m_spinB);
    formMotor->addRow(QStringLiteral("C 轴 →  #"), m_spinC);

    layout->addWidget(grpMotor);

    // ---- 触发通道 ----
    auto* grpTrig = new QGroupBox(QStringLiteral("触发通道 Gate3[gate].Chan[chan]"), this);
    auto* formTrig = new QFormLayout(grpTrig);

    m_spinGate = new QSpinBox(this);
    m_spinGate->setRange(0, 15);
    m_spinGate->setValue(initial.triggerGate);
    formTrig->addRow(QStringLiteral("Gate："), m_spinGate);

    m_spinChan = new QSpinBox(this);
    m_spinChan->setRange(0, 3);
    m_spinChan->setValue(initial.triggerChan);
    formTrig->addRow(QStringLiteral("Chan："), m_spinChan);

    layout->addWidget(grpTrig);

    // ---- 按钮 ----
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    btns->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
    btns->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    connect(btns, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(btns);
}

AxisConfig AxisMappingDialog::result() const
{
    AxisConfig cfg;
    cfg.motorX = m_spinX->value();
    cfg.motorY = m_spinY->value();
    cfg.motorZ = m_spinZ->value();
    cfg.motorB = m_spinB->value();
    cfg.motorC = m_spinC->value();
    cfg.triggerGate = m_spinGate->value();
    cfg.triggerChan = m_spinChan->value();
    return cfg;
}
