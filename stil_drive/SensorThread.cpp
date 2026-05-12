// ============================================================================
// SensorThread.cpp
// STIL CCS Prima 传感器后台采集线程
// ============================================================================
// 在独立线程中运行，使用 Windows 事件同步机制（WaitForSingleObject）
// 接收 STIL 传感器的 TRE（Burst）模式硬件触发采集数据，
// 每次触发采集 1 点，封装为 Qt 信号传递给主线程 UI。
//
// TRE 模式（Time Resolved Equivalent）原理：
//   - 测头在外部触发脉冲上升沿采集 1 个数据点（高度 + 强度）
//   - 使用双缓冲（Buffer A/B）交替接收，防止数据丢失
//   - 每写入一个 Buffer，Windows 事件被 SetEvent 触发，主线程立即读取
//
// 数据流向：
//   STIL DLL (硬件 FIFO)
//     → MCHR_GetAltitudeMeasurement (双缓冲 DMA)
//       → SensorThread 线程 (WaitForSingleObject 等待)
//         → emit dataReady(QVector, QVector) 信号
//           → stil_drive 主窗口 (handleDataReady)
//             → DataReducer::reduceModeX()
//               → QCustomPlot 绘图 + CSV 写入
// ============================================================================

#include "SensorThread.h"
#include <QDebug>

SensorThread::SensorThread(MCHR_ID sensorID, QObject* parent)
    : QThread(parent)
    , m_sensorID(sensorID)
    , m_isRunning(false)
{
}

SensorThread::~SensorThread()
{
    stopAcquisition();  // 停止循环
    wait();            // 等待线程安全退出（join）
}

void SensorThread::stopAcquisition()
{
    m_isRunning = false;  // 改变标志位，让 run() 循环自然退出
}

void SensorThread::run()
{
    if (m_sensorID == 0) return;

    m_isRunning = true;

    // --- 步骤 1：创建 Windows 手动重置事件（内核对象）---
    // SetEvent(hEventEndBuffer) 由 STIL DLL 在每个 Buffer 写满时调用
    HANDLE hEventEndBuffer = CreateEvent(NULL, FALSE, FALSE, NULL);

    // --- 步骤 2：配置 TRE 硬件触发采集参数 ---
    MCHR_tyAcqParam acqParam;
    memset(&acqParam, 0, sizeof(MCHR_tyAcqParam));

    acqParam.NumberOfPoints = 0;        // 0 = 无限点采集（持续到手动停止）
    acqParam.TriggerFlag = TRUE;         // TRUE = 启用外部硬件触发
    acqParam.TriggerType = MCHR_TYPE_TRE;  // TRE Burst 模式（每次触发采 1 点）
    acqParam.NumberPointsTRE = 1;        // 每次触发采集 1 个数据点
    // 上升沿触发（0V→5V 跳变，兼容 PMAC EQU 脉冲输出电平）
    acqParam.HighLevelOrRisingEdgeActivated = MCHR_RISING_EDGE;
    acqParam.NumberOfBuffers = 2;        // 双缓冲（A/B 交替）
    acqParam.BufferLength = 1;           // 每 Buffer 长度 1 点（每点立即上传）
    acqParam.EventEndBuffer = hEventEndBuffer;

    // --- 步骤 3：为 DLL 分配双缓冲内存 ---
    float* pAltitudes[2];
    float* pIntensities[2];
    for (int i = 0; i < 2; i++) {
        pAltitudes[i] = new float[acqParam.BufferLength];
        pIntensities[i] = new float[acqParam.BufferLength];
    }

    // --- 步骤 4：通知 DLL 开始采集 ---
    short ret = MCHR_GetAltitudeMeasurement(
        m_sensorID,
        acqParam,
        pAltitudes,    // 高度数据缓冲区指针数组
        pIntensities,  // 强度数据缓冲区指针数组
        NULL, NULL, NULL);

    if (ret == 0) {
        QString errorMsg = QString("传感器底层采集启动失败！错误码: %1").arg(ret);
        emit errorOccurred(errorMsg);
        m_isRunning = false;

        // 清理已分配的资源
        for (int i = 0; i < 2; i++) {
            delete[] pAltitudes[i];
            delete[] pIntensities[i];
        }
        CloseHandle(hEventEndBuffer);
        return;
    }

    // --- 步骤 5：主采集循环（等待事件 → 读取 Buffer → 发送信号）---
    while (m_isRunning) {
        // 等待 DLL 在 Buffer 写满时 SetEvent，超时 1000 ms
        DWORD waitRes = WaitForSingleObject(hEventEndBuffer, 1000);

        if (waitRes == WAIT_OBJECT_0) {
            // 事件触发：查询最近写入的是哪个 Buffer（0 或 1）
            long bufIdx = -1, ptIdx = -1;
            MCHR_GetLastWrittenBuffer(m_sensorID, MCHR_BUFFER_EVENT, &bufIdx, &ptIdx);

            if (bufIdx >= 0 && bufIdx < acqParam.NumberOfBuffers) {
                // 将原始 float 数据复制到 Qt QVector（跨线程传递安全）
                QVector<double> vAlts(acqParam.BufferLength);
                QVector<double> vInts(acqParam.BufferLength);

                for (int i = 0; i < acqParam.BufferLength; i++) {
                    vAlts[i] = (double)pAltitudes[bufIdx][i];
                    vInts[i] = (double)pIntensities[bufIdx][i];
                }

                // 发射数据信号，主线程 handleDataReady 接收并处理
                emit dataReady(vAlts, vInts);
            }
        }
        // WAIT_TIMEOUT：正常情况（每 1000 ms 醒一次检查退出标志）
        // 不做额外处理，避免产生日志噪音
    }

    // --- 步骤 6：退出循环后清理资源 ---
    MCHR_Abort(m_sensorID);  // 通知 DLL 停止采集

    for (int i = 0; i < 2; i++) {
        delete[] pAltitudes[i];
        delete[] pIntensities[i];
    }
    CloseHandle(hEventEndBuffer);
}
