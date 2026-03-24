#pragma once
#include <QtWidgets/QMainWindow>
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

    // 接收线程数据的槽函数
    void handleDataReady(QVector<double> alts, QVector<double> ints);
    void handleError(QString msg);

private:
    Ui::stil_driveClass ui;
    MCHR_ID m_sensorID = 0;
    SensorThread* m_thread = nullptr;

    // 画图与保存变量
    long long m_totalPoints = 0;
    QFile m_csvFile;
    QTextStream m_csvStream;
};