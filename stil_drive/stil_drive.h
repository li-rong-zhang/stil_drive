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

#include <QFile>
#include <QTextStream>

class stil_drive : public QMainWindow
{
    Q_OBJECT

public:
    stil_drive(QWidget* parent = nullptr);
    ~stil_drive();

private slots:
    // STIL 测头控制
    void on_btn_Connect_clicked();
    void on_btn_Disconnect_clicked();
    void on_btn_Start_clicked();
    void on_btn_Stop_clicked();

    // 数据处理（来自 SensorThread）
    void handleDataReady(QVector<double> alts, QVector<double> ints);
    void handleError(QString msg);

    // PMAC 通讯
    void on_btn_pmac_connect_clicked();
    void on_btn_Send_Cmd_clicked();
    void onPmacConnectStatusChanged(bool connected);

    // 脚本生成（模式切换时更新 UI 文本框）
    void onSyncAxisChanged(int index);

    // 数据后处理
    void on_btn_Analyze_clicked();

private:
    void startCsvFile();
    void stopCsvFile();

    Ui::stil_driveClass ui;

    // STIL 测头
    MCHR_ID m_sensorID = 0;
    SensorThread* m_thread = nullptr;

    // CSV 保存
    long long m_totalPoints = 0;
    QFile m_csvFile;
    QTextStream m_csvStream;

    // PMAC 控制器（独立模块）
    PmacController* m_pmac = nullptr;
};
