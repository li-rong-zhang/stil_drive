#pragma once
#include <QtWidgets/QMainWindow>
#include "ui_stil_drive.h"

// 1. Windows 系统头文件
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

// 2. 声明 DLL 导入
#ifndef DLL_CHR_API
#define DLL_CHR_API __declspec(dllimport)
#endif

// 3. 测头头文件
#include "MchrDefine.h"
#include "MchrType.h"
#include "MchrError.h"
#include "Mchr.h"

// 4. 自定义线程类
#include "SensorThread.h"

// 5. Qt 文件操作和网络支持
#include <QFile>
#include <QTextStream>
#include <QTcpSocket>

class stil_drive : public QMainWindow
{
    Q_OBJECT

public:
    stil_drive(QWidget* parent = nullptr);
    ~stil_drive();

private slots:
    // 测头控制
    void on_btn_Connect_clicked();
    void on_btn_Disconnect_clicked();
    void on_btn_Start_clicked();
    void on_btn_Stop_clicked();

    // 数据处理
    void handleDataReady(QVector<double> alts, QVector<double> ints);
    void handleError(QString msg);

    // 数据后处理
    void on_btn_Analyze_clicked();

    // PMAC 网络通讯
    void on_btn_pmac_connect_clicked();
    void on_btn_Send_Cmd_clicked();
    void onPmacConnected();
    void onPmacDisconnected();
    void onPmacReadyRead();

private:
    Ui::stil_driveClass ui;
    MCHR_ID m_sensorID = 0;
    SensorThread* m_thread = nullptr;

    // 图表与保存
    long long m_totalPoints = 0;
    QFile m_csvFile;
    QTextStream m_csvStream;

    // PMAC 网络通讯
    QTcpSocket* pmacSocket = nullptr;
};
