#pragma once
#include <QtWidgets/QMainWindow>
#include <QTcpSocket>
#include "ui_stil_drive.h"

// 1. Windows 系统头文件
#define WIN32_LEAN_AND_MEAN 
#define NOMINMAX      
#include <windows.h>

// 2. 测头 DLL 导入宏
#ifndef DLL_CHR_API
#define DLL_CHR_API __declspec(dllimport)
#endif

// 3. 测头核心文件
#include "MchrDefine.h"
#include "MchrType.h"
#include "MchrError.h"
#include "Mchr.h"

// 4. 自定义线程类
#include "SensorThread.h"

// 5. Qt 文件保存支持
#include <QFile>
#include <QTextStream>

// 6.引入文件选择对话框
#include <QFileDialog>   

// 7.引入世界最强的 C++ 矩阵库
#include <Eigen/Dense>   

class stil_drive : public QMainWindow
{
    Q_OBJECT

public:
    stil_drive(QWidget* parent = nullptr);
    ~stil_drive();

private slots:
    void on_btn_Connect_clicked();
    void on_btn_Disconnect_clicked();
    void on_btn_Start_clicked();
    void on_btn_Stop_clicked();
    void on_btn_Analyze_clicked();

    // 接收线程数据的槽函数
    void handleDataReady(QVector<double> alts, QVector<double> ints);
    void handleError(QString msg);

    // PMAC 通讯相关的槽函数
    void on_btn_pmac_connect_clicked(); // 点击“连接控制器”按钮
    void onPmacConnected();             // PMAC 连接成功的回调
    void onPmacDisconnected();          // PMAC 断开连接的回调
    void onPmacReadyRead();             // 收到 PMAC 返回数据时的回调

private:
    Ui::stil_driveClass ui;
    MCHR_ID m_sensorID = 0;
    SensorThread* m_thread = nullptr;

    // 画图与保存变量
    long long m_totalPoints = 0;
    QFile m_csvFile;
    QTextStream m_csvStream;
    QTcpSocket* pmacSocket;
};