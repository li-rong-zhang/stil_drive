// ============================================================================
// SensorConnectDialog.h
// STIL 测头连接配置对话框
// ============================================================================
// 由 stil_drive 主窗口在调用 MCHR_GetUsbDeviceList 后弹出，
// 用户选择目标设备序号与采样频率档位。Dialog 不直接调 MCHR_OpenUsbChr，
// 设备打开由主窗口完成（保持 SDK 调用集中）。
// ============================================================================

#pragma once
#include <QDialog>
#include <QStringList>

class QListWidget;
class QComboBox;

class SensorConnectDialog : public QDialog
{
    Q_OBJECT
public:
    SensorConnectDialog(const QStringList& deviceList,
                        int initialFreqIndex = 3,
                        QWidget* parent = nullptr);

    int     selectedDeviceIndex() const;   // -1 = 未选
    int     frequencyIndex() const;        // 对应原 cmb_Freq 0..4
    QString frequencyLabel() const;

private:
    QListWidget* m_listDevices;
    QComboBox*   m_cmbFreq;
};
