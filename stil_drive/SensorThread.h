// ============================================================================
// SensorThread.h
// STIL CCS Prima 传感器后台采集线程
// ============================================================================
// 在独立 QThread 子类中运行，调用 STIL DLL 的 TRE 硬件触发采集接口，
// 每次触发收到数据后发射 Qt 信号，将数据传递给主窗口处理。
// 线程通过 stopAcquisition() 的标志位安全终止。
// ============================================================================

#pragma once
#include <QThread>
#include <QVector>
#include <windows.h>

#ifndef DLL_CHR_API
#define DLL_CHR_API __declspec(dllimport)
#endif

#include "MchrDefine.h"
#include "MchrType.h"
#include "MchrError.h"
#include "Mchr.h"

class SensorThread : public QThread
{
    Q_OBJECT

public:
    explicit SensorThread(MCHR_ID sensorID, QObject* parent = nullptr);
    ~SensorThread();

    // 请求线程停止采集（安全退出）
    void stopAcquisition();

signals:
    // 数据就绪信号（QVector 可安全跨线程传递）
    void dataReady(QVector<double> altitudes, QVector<double> intensities);

    // 错误通知信号
    void errorOccurred(QString errorMsg);

protected:
    // 线程主函数（QThread 虚函数，重写 run()）
    void run() override;

private:
    MCHR_ID m_sensorID;  // 传感器句柄
    bool m_isRunning;    // 线程退出标志
};
