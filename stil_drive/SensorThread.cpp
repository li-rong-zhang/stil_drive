#include "SensorThread.h"
#include <QDebug>

SensorThread::SensorThread(MCHR_ID sensorID, QObject* parent)
    : QThread(parent), m_sensorID(sensorID), m_isRunning(false)
{
}

SensorThread::~SensorThread()
{
    stopAcquisition();
    wait(); // 确保线程安全死亡后再销毁
}

void SensorThread::stopAcquisition()
{
    m_isRunning = false; // 改变标志位，让循环自然结束
}

void SensorThread::run()
{
    if (m_sensorID == 0) return;

    m_isRunning = true;

    // 1. 创建一个 Windows 同步事件 (无信号，手动复位)
    HANDLE hEventEndBuffer = CreateEvent(NULL, FALSE, FALSE, NULL);

    // 2. 配置采集参数
    MCHR_tyAcqParam acqParam;
    memset(&acqParam, 0, sizeof(MCHR_tyAcqParam));

    acqParam.NumberOfPoints = 0;              // 0 代表无限连续采集

    // 【核心修改 1】：改为 TRUE！意味着引擎启动后会被“冻结”，死等 PMAC 的硬件脉冲
    acqParam.TriggerFlag = TRUE;

    acqParam.NumberOfBuffers = 2;             // 2个缓冲区 (双缓冲无缝衔接)

    // 【核心修改 2】：改小 Buffer，提高刷新率，防止扫描末尾的数据凑不齐一帧被卡住
    acqParam.BufferLength = 50;

    acqParam.EventEndBuffer = hEventEndBuffer;

    // 3. 为 DLL 分配 2 个独立的数据接收内存池
    float* pAltitudes[2];
    float* pIntensities[2];
    for (int i = 0; i < 2; i++) {
        pAltitudes[i] = new float[acqParam.BufferLength];
        pIntensities[i] = new float[acqParam.BufferLength];
    }

    // 4. 通知 DLL 开始“全面”连续采集
    short ret = MCHR_GetAltitudeMeasurement(
        m_sensorID,
        acqParam,
        pAltitudes,       // 高度数据缓冲区指针数组
        pIntensities,     // 光强数据缓冲区指针数组
        NULL, NULL, NULL  // 其他数据不需要，传 NULL
    );

    if (ret == 0) {
        emit errorOccurred("启动底层连续采集失败！");
        m_isRunning = false;
    }

    // 5. 核心采集死循环 (只要没点停止，就一直在后台收数据)
    while (m_isRunning) {
        // 挂起线程，死死盯住事件。超时设为 1000 毫秒
        DWORD waitRes = WaitForSingleObject(hEventEndBuffer, 1000);

        if (waitRes == WAIT_OBJECT_0) { // 等到了！说明有一个 Buffer 填满了

            long bufIdx = -1, ptIdx = -1;
            // 问问 DLL：你刚才填满的是 0 号还是 1 号 Buffer？
            MCHR_GetLastWrittenBuffer(m_sensorID, MCHR_BUFFER_EVENT, &bufIdx, &ptIdx);

            if (bufIdx >= 0 && bufIdx < acqParam.NumberOfBuffers) {
                // 把刚填满的那一块 C++ 数组，拷贝到 Qt 的 QVector 里
                QVector<double> vAlts(acqParam.BufferLength);
                QVector<double> vInts(acqParam.BufferLength);

                for (int i = 0; i < acqParam.BufferLength; i++) {
                    vAlts[i] = (double)pAltitudes[bufIdx][i];
                    vInts[i] = (double)pIntensities[bufIdx][i];
                }

                // 【发射信号】数据打包好了，通过槽函数抛给主界面！
                emit dataReady(vAlts, vInts);
            }
        }
        else if (waitRes == WAIT_TIMEOUT) {
            // 超时了也没事，继续等下一次循环。或者在这里可以加断线检测。
        }
    }

    // 6. 退出循环了，命令测头立即停止采集
    MCHR_Abort(m_sensorID);

    // 7. 清理我们在堆上 new 出来的内存
    for (int i = 0; i < 2; i++) {
        delete[] pAltitudes[i];
        delete[] pIntensities[i];
    }
    CloseHandle(hEventEndBuffer);
}