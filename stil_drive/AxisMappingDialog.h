// ============================================================================
// AxisMappingDialog.h
// 轴映射配置编辑对话框
// ============================================================================
#pragma once
#include <QDialog>
#include "AxisConfig.h"

class QSpinBox;

class AxisMappingDialog : public QDialog
{
    Q_OBJECT
public:
    explicit AxisMappingDialog(const AxisConfig& initial,
                               QWidget* parent = nullptr);

    AxisConfig result() const;

private:
    QSpinBox* m_spinX;
    QSpinBox* m_spinY;
    QSpinBox* m_spinZ;
    QSpinBox* m_spinB;
    QSpinBox* m_spinC;
    QSpinBox* m_spinGate;
    QSpinBox* m_spinChan;
};
