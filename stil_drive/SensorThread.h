#pragma once
#include <QThread>
#include <QVector>
#include <windows.h>

// 引入测头底层头文件
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

    // 暴露给主线程调用的停止接口
    void stopAcquisition();

signals:
    // 【核心信号】把采集到的一批数据打包发给主界面 (QVector 特别适合画图和保存)
    void dataReady(QVector<double> altitudes, QVector<double> intensities);

    // 如果出错，通知主界面弹窗
    void errorOccurred(QString errorMsg);

protected:
    // 线程的“心脏”，启动线程后会一直在这个函数里运行
    void run() override;

private:
    MCHR_ID m_sensorID;  // 测头句柄
    bool m_isRunning;    // 控制线程循环的开关
};