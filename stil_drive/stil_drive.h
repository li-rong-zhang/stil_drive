#pragma once
#include <QtWidgets/QMainWindow>
#include "ui_stil_drive.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#ifndef DLL_CHR_API
#define DLL_CHR_API __declspec(dllimport)
#endif

#include "MchrDefine.h"
#include "MchrType.h"
#include "MchrError.h"
#include "Mchr.h"

#include "SensorThread.h"
#include "PmacController.h"
#include "AxisConfig.h"

#include <QFile>
#include <QTextStream>
#include <QString>
#include <QTimer>

class TrajectoryPlannerDock;
class LinearParamPage;
class SpiralParamPage;
class SphericalParamPage;
class AsphericParamPage;
class QLabel;
class QTextEdit;
class QTextBrowser;
class QTabWidget;
class QDockWidget;
class QActionGroup;

class stil_drive : public QMainWindow
{
    Q_OBJECT

public:
    stil_drive(QWidget* parent = nullptr);
    ~stil_drive();

private slots:
    // 测量主流程
    void onStartTriggered();
    void onStopTriggered();

    // 数据流
    void handleDataReady(QVector<double> alts, QVector<double> ints);
    void handleError(QString msg);

    // 设备菜单
    void onConnectSensor();
    void onDisconnectSensor();
    void onConnectPmac();
    void onDisconnectPmac();
    void onSendManualCommand();
    void onAxisMapping();

    // 模式菜单
    void onModeLinear();
    void onModeSpiral();
    void onModeSpherical();
    void onModeAspheric();

    // 工具菜单
    void onGenerateScript();
    void onEmergencyStop();
    void onAnalyze();
    void onAbout();

    // PMAC 状态
    void onPmacConnectStatusChanged(bool connected);

    // 来自 dock 的请求
    void onValidateRequested();
    void onGenerateRequested();
    void onStartRequested();

private:
    void setupTrajectoryDock();
    void setupBottomDock();
    void setupStatusBar();
    void wireMenuActions();
    void switchMode(int modeId);
    void refreshScriptPreview();
    void startAcquisition();    // 共用启动流程：检查设备 + 起线程 + 下发预览脚本
    void promptNextLatitude();  // 球面纬线：提示 + 下发定位脚本 + 推进 index
    void startCsvFile();
    void stopCsvFile();

    Ui::stil_driveClass ui;

    // STIL 测头
    MCHR_ID       m_sensorID  = 0;
    SensorThread* m_thread    = nullptr;
    int           m_freqIndex = 3;   // 默认 1000 Hz

    // CSV 保存
    long long    m_totalPoints = 0;
    QFile        m_csvFile;
    QTextStream  m_csvStream;

    // PMAC 控制器
    PmacController* m_pmac = nullptr;
    QString         m_pmacIp   = "192.168.0.200";
    quint16         m_pmacPort = 5002;

    // 轴映射配置
    AxisConfig m_axisCfg;

    // 轨迹规划 dock + 4 页
    TrajectoryPlannerDock* m_planner    = nullptr;
    LinearParamPage*       m_pageLinear = nullptr;
    SpiralParamPage*       m_pageSpiral = nullptr;
    SphericalParamPage*    m_pageSpherical = nullptr;
    AsphericParamPage*     m_pageAspheric  = nullptr;

    // 底部 dock：脚本预览 + PMAC 日志
    QDockWidget*  m_dockBottom = nullptr;
    QTabWidget*   m_tabBottom  = nullptr;
    QTextEdit*    m_textScript = nullptr;
    QTextBrowser* m_textLog    = nullptr;

    // 状态栏
    QLabel* m_lblSensorLed = nullptr;
    QLabel* m_lblPmacLed   = nullptr;
    QLabel* m_lblMode      = nullptr;
    QLabel* m_lblStatus    = nullptr;

    // 模式菜单互斥
    QActionGroup* m_modeGroup = nullptr;

    // PROG 运行状态检测定时器
    QTimer*       m_progMonitorTimer = nullptr;
private slots:
    void checkProgStatus();
};
