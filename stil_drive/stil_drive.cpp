#include "stil_drive.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QDateTime>
#include "qcustomplot.h"
#include <QPen>

#include "AsphericMath.h"
#include "DataReducer.h"
#include "PmacScriptGen.h"
#include "SurfaceAnalyzer.h"

// =========================================================================
// 构造与初始化
// =========================================================================
stil_drive::stil_drive(QWidget* parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    // --- PMAC 控制器（独立模块）---
    m_pmac = new PmacController(this);
    connect(m_pmac, &PmacController::connected, this, [=]() { onPmacConnectStatusChanged(true); });
    connect(m_pmac, &PmacController::disconnected, this, [=]() { onPmacConnectStatusChanged(false); });
    connect(m_pmac, &PmacController::logMessage, ui.textBrowser_pmac_log, &QTextBrowser::append);

    // --- 测头频率下拉框 ---
    ui.cmb_Freq->addItem("100 Hz");
    ui.cmb_Freq->addItem("200 Hz");
    ui.cmb_Freq->addItem("400 Hz");
    ui.cmb_Freq->addItem("1000 Hz");
    ui.cmb_Freq->addItem("2000 Hz");
    ui.cmb_Freq->setCurrentIndex(3);

    // --- PMAC 同步轴下拉框 ---
    ui.cmb_SyncAxis->addItem("跟随 X 轴 (Chan 0)", 0);
    ui.cmb_SyncAxis->addItem("跟随 C 轴 (Chan 2)", 2);
    ui.cmb_SyncAxis->addItem("球面纬线扫描 (Chan 2)", 3);
    ui.cmb_SyncAxis->addItem("非球面母线扫描", 4);

    // 模式切换时自动生成脚本
    connect(ui.cmb_SyncAxis, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &stil_drive::onSyncAxisChanged);
    ui.cmb_SyncAxis->setCurrentIndex(0);

    // --- 界面初始状态 ---
    ui.btn_Disconnect->setEnabled(false);
    ui.btn_Start->setEnabled(false);
    ui.btn_Stop->setEnabled(false);

    // --- 图表初始化 ---
    ui.plot_Widget->addGraph();
    ui.plot_Widget->graph(0)->setPen(QPen(Qt::blue));
    ui.plot_Widget->xAxis->setLabel("采样点数 / 位置");
    ui.plot_Widget->yAxis->setLabel("高度 / Altitude (um)");
    ui.plot_Widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

stil_drive::~stil_drive()
{
    if (m_thread != nullptr) {
        m_thread->stopAcquisition();
        m_thread->wait();
    }
    if (m_sensorID != 0) MCHR_CloseChr(m_sensorID);
    MCHR_Release();

    if (m_csvFile.isOpen()) m_csvFile.close();
}

// =========================================================================
// STIL 测头控制
// =========================================================================
void stil_drive::on_btn_Connect_clicked()
{
    if (m_sensorID != 0) return;
    ui.btn_Connect->setEnabled(false);
    ui.btn_Connect->setText("连接中...");

    static bool isDllInitialized = false;
    if (!isDllInitialized) {
        if (!MCHR_Init()) {
            QMessageBox::critical(this, "错误", "DLL 初始化失败！请检查USB连线或重启测头。");
            ui.btn_Connect->setText("连接测头");
            ui.btn_Connect->setEnabled(true);
            return;
        }
        isDllInitialized = true;
    }

    char* deviceList[MCHR_MAX_SENSOR];
    for (int i = 0; i < MCHR_MAX_SENSOR; i++) {
        deviceList[i] = new char[MAX_PATH];
        memset(deviceList[i], 0, MAX_PATH);
    }

    short deviceNumber = 0;
    if ((MCHR_GetUsbDeviceList(deviceList, &deviceNumber) != MCHR_ERROR) && deviceNumber > 0) {
        m_sensorID = MCHR_OpenUsbChr(deviceList[0], MCHR_CCS_PRIMA, deviceList[0], NULL, NULL);
        if (m_sensorID != 0) {
            ui.btn_Connect->setText("已连接");
            ui.btn_Disconnect->setEnabled(true);
            ui.btn_Start->setEnabled(true);
            QMessageBox::information(this, "成功", QString("成功连接到测头！\n设备名: %1").arg(deviceList[0]));
        } else {
            ui.btn_Connect->setText("连接测头");
            ui.btn_Connect->setEnabled(true);
            QMessageBox::critical(this, "失败", "找到设备但连接失败！");
        }
    } else {
        ui.btn_Connect->setText("连接测头");
        ui.btn_Connect->setEnabled(true);
        QMessageBox::warning(this, "警告", "未扫描到测头！请拔插USB线重试。");
    }

    for (int i = 0; i < MCHR_MAX_SENSOR; i++) {
        delete[] deviceList[i];
    }
}

void stil_drive::on_btn_Disconnect_clicked()
{
    if (m_sensorID == 0) return;
    on_btn_Stop_clicked();
    MCHR_CloseChr(m_sensorID);
    m_sensorID = 0;

    ui.btn_Connect->setEnabled(true);
    ui.btn_Connect->setText("连接测头");
    ui.btn_Disconnect->setEnabled(false);
    ui.btn_Start->setEnabled(false);
    ui.lbl_Status->setText("状态：已断开连接");
}

void stil_drive::on_btn_Start_clicked()
{
    if (m_sensorID == 0) {
        QMessageBox::warning(this, "警告", "请先连接 STIL 测头！");
        return;
    }
    if (!m_pmac->isConnected()) {
        QMessageBox::warning(this, "警告", "请先连接 Power PMAC 控制器！");
        return;
    }
    if (m_thread != nullptr && m_thread->isRunning()) return;

    // 设置测头频率
    WORD rateCode = MCHR_SCAN_RATE_CCS_PRIMA_1000HZ;
    switch (ui.cmb_Freq->currentIndex()) {
    case 0: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_100HZ; break;
    case 1: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_200HZ; break;
    case 2: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_400HZ; break;
    case 3: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_1000HZ; break;
    case 4: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_2000HZ; break;
    }
    MCHR_SetScanRate(m_sensorID, rateCode);

    m_totalPoints = 0;
    ui.plot_Widget->graph(0)->data()->clear();

    // 启动 CSV 文件
    startCsvFile();

    // 启动测头数据线程
    m_thread = new SensorThread(m_sensorID, this);
    connect(m_thread, &SensorThread::dataReady, this, &stil_drive::handleDataReady);
    connect(m_thread, &SensorThread::errorOccurred, this, &stil_drive::handleError);
    m_thread->start();

    ui.lbl_Status->setText("状态：等待硬件触发脉冲...");
    ui.btn_Start->setEnabled(false);
    ui.btn_Stop->setEnabled(true);

    // 下发 PMAC 运动及触发脚本
    ui.textBrowser_pmac_log->append("正在下发硬件脉冲同步指令...");
    QString cmd = ui.textEdit_pmac_script->toPlainText().trimmed();
    if (!cmd.endsWith("\n")) cmd += "\n";
    m_pmac->sendCommand(cmd);
}

void stil_drive::on_btn_Stop_clicked()
{
    if (m_thread != nullptr) {
        m_thread->stopAcquisition();
        m_thread->wait();
        m_thread->deleteLater();
        m_thread = nullptr;
    }

    stopCsvFile();

    ui.btn_Start->setEnabled(true);
    ui.btn_Stop->setEnabled(false);
    ui.lbl_Status->setText("状态：已停止");

    // 急停并关闭 PMAC 触发
    if (m_pmac->isConnected()) {
        m_pmac->sendCommand(PmacScriptGen::emergencyStop());
        ui.textBrowser_pmac_log->append("<font color='red'>已安全制动全轴并撤销触发使能。</font>");
    }
}

// =========================================================================
// 数据处理
// =========================================================================
void stil_drive::handleDataReady(QVector<double> alts, QVector<double> ints)
{
    int dataSize = alts.size();
    if (dataSize == 0) return;

    int syncMode = ui.cmb_SyncAxis->currentData().toInt();
    ReductionResult r;

    if (syncMode == 0) {
        r = DataReducer::reduceMode0(alts, ints, m_totalPoints, &m_csvStream);
    } else if (syncMode == 2) {
        double rStartMm = ui.lineEdit_Radius->text().toDouble();
        double pitch = ui.lineEdit_Pitch->text().toDouble();
        if (pitch <= 0) pitch = 10.0;
        r = DataReducer::reduceMode2(alts, ints, m_totalPoints,
            rStartMm * 1000.0, pitch, &m_csvStream);
    } else if (syncMode == 3) {
        r = DataReducer::reduceMode3(alts, ints, m_totalPoints, &m_csvStream);
    } else if (syncMode == 4) {
        double startRadiusUm = ui.lineEdit_Radius->text().toDouble();
        double scanRangeUm = ui.lineEdit_Aspheric_Range->text().toDouble();
        if (scanRangeUm <= 0) scanRangeUm = 50000.0;
        double R_mm = ui.lineEdit_Aspheric_R->text().toDouble();
        double k = ui.lineEdit_Aspheric_K->text().toDouble();
        double A4 = ui.lineEdit_Aspheric_A4->text().toDouble();
        double A6 = ui.lineEdit_Aspheric_A6->text().toDouble();
        double A8 = ui.lineEdit_Aspheric_A8->text().toDouble();
        double A10 = ui.lineEdit_Aspheric_A10->text().toDouble();
        double A12 = ui.lineEdit_Aspheric_A12->text().toDouble();
        if (R_mm <= 0) R_mm = 100.0;
        r = DataReducer::reduceMode4(alts, ints, m_totalPoints,
            startRadiusUm, scanRangeUm, R_mm, k, A4, A6, A8, A10, A12, &m_csvStream);
    }

    // 刷新图表
    ui.plot_Widget->graph(0)->addData(r.xData, alts);
    m_totalPoints += dataSize;
    ui.plot_Widget->xAxis->setRange(r.currentX - r.windowWidth, r.currentX, Qt::AlignRight);
    ui.plot_Widget->graph(0)->rescaleValueAxis(true);
    ui.plot_Widget->replot();

    // 更新状态栏
    ui.lbl_Status->setText(QString("状态：飞拍中 | 位置: %1 %2 | 高度: %3 um")
        .arg(r.currentX, 0, 'f', 2).arg(r.unit).arg(alts.last(), 0, 'f', 2));
}

void stil_drive::handleError(QString msg)
{
    QMessageBox::critical(this, "底层错误", msg);
    on_btn_Stop_clicked();
}

// =========================================================================
// CSV 文件管理
// =========================================================================
void stil_drive::startCsvFile()
{
    if (!ui.chk_SaveData->isChecked()) return;

    QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + "_SyncData.csv";
    m_csvFile.setFileName(fileName);
    if (!m_csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) return;

    m_csvStream.setDevice(&m_csvFile);
    int syncMode = ui.cmb_SyncAxis->currentData().toInt();
    if (syncMode == 4) {
        m_csvStream << "X_Axis(um),Y_Axis(um),Z_Measured(um),Z_Theory(um),Z_Deviation(um),Intensity(%)\n";
    } else {
        m_csvStream << "X_Axis(um/deg),Y_Axis(um),Z_Altitude(um),Intensity(%)\n";
    }
}

void stil_drive::stopCsvFile()
{
    if (m_csvFile.isOpen()) m_csvFile.close();
}

// =========================================================================
// PMAC 通讯
// =========================================================================
void stil_drive::on_btn_pmac_connect_clicked()
{
    if (!m_pmac->isConnected()) {
        m_pmac->connectToController(ui.lineEdit_pmac_ip->text(), ui.lineEdit_pmac_port->text().toUShort());
    } else {
        m_pmac->disconnectController();
    }
}

void stil_drive::on_btn_Send_Cmd_clicked()
{
    if (!m_pmac->isConnected()) return;
    QString cmd = ui.textEdit_pmac_script->toPlainText().trimmed();
    if (cmd.isEmpty()) return;
    if (!cmd.endsWith("\n")) cmd += "\n";

    m_pmac->sendCommand(cmd);
    QString logMsg = cmd;
    logMsg.replace("\n", " ");
    ui.textBrowser_pmac_log->append("<font color='purple'>[手动测试]: " + logMsg + "</font>");
}

void stil_drive::onPmacConnectStatusChanged(bool connected)
{
    if (connected) {
        ui.btn_pmac_connect->setText("断开控制器");
        ui.textBrowser_pmac_log->append("<font color='green'>成功连接到 Power PMAC!</font>");
    } else {
        ui.btn_pmac_connect->setText("连接控制器");
        ui.textBrowser_pmac_log->append("<font color='red'>已断开与 PMAC 的连接。</font>");
    }
}

// =========================================================================
// 模式切换：生成 PMAC 脚本
// =========================================================================
void stil_drive::onSyncAxisChanged(int index)
{
    int mode = ui.cmb_SyncAxis->itemData(index).toInt();
    QString script;

    // 统一使用 Gate1 Chan0（第二块 24E3 板）
    int targetGate = 1;
    int targetChan = 0;

    if (mode == 0) {
        ui.lineEdit_Radius->setEnabled(false);
        ui.lineEdit_Pitch->setEnabled(false);
        script = PmacScriptGen::generateLinearScript(targetGate, targetChan);
    } else if (mode == 2) {
        ui.lineEdit_Radius->setEnabled(true);
        ui.lineEdit_Pitch->setEnabled(true);
        double pitch = ui.lineEdit_Pitch->text().toDouble();
        if (pitch <= 0) pitch = 10.0;
        script = PmacScriptGen::generateSpiralScript(pitch, targetGate, targetChan);
    } else if (mode == 3) {
        ui.lineEdit_Radius->setEnabled(true);
        ui.lineEdit_Pitch->setEnabled(false);
        script = PmacScriptGen::generateSphericalLatScript(targetGate, targetChan);
    } else if (mode == 4) {
        ui.lineEdit_Radius->setEnabled(true);
        ui.lineEdit_Pitch->setEnabled(false);

        double startRadiusUm = ui.lineEdit_Radius->text().toDouble();
        double scanRangeUm = ui.lineEdit_Aspheric_Range->text().toDouble();
        if (scanRangeUm <= 0) scanRangeUm = 50000.0;
        double R_mm = ui.lineEdit_Aspheric_R->text().toDouble();
        double k = ui.lineEdit_Aspheric_K->text().toDouble();
        double A4 = ui.lineEdit_Aspheric_A4->text().toDouble();
        double A6 = ui.lineEdit_Aspheric_A6->text().toDouble();
        double A8 = ui.lineEdit_Aspheric_A8->text().toDouble();
        double A10 = ui.lineEdit_Aspheric_A10->text().toDouble();
        double A12 = ui.lineEdit_Aspheric_A12->text().toDouble();
        if (R_mm <= 0) R_mm = 100.0;

        bool followZ = ui.chk_FollowZ->isChecked();
        script = PmacScriptGen::generateAsphericMeridianScript(
            startRadiusUm, scanRangeUm, R_mm, k, A4, A6, A8, A10, A12, followZ, targetGate, targetChan);

        double stepUm = AsphericMath::calcSafeStepSize(
            startRadiusUm / 1000.0, R_mm, k, A4, A6, A8, A10, A12);
        int numPoints = (int)(scanRangeUm / stepUm) + 1;
        if (numPoints > 5000) numPoints = 5000;
        ui.textBrowser_pmac_log->append(QString("<font color='blue'>[Z轴跟随] 步距: %1 µm, 轨迹点数: %2</font>")
            .arg(stepUm, 0, 'f', 2).arg(numPoints));
    }

    ui.textEdit_pmac_script->setPlainText(script);
}

// =========================================================================
// 数据后处理（Eigen 面形分析）
// =========================================================================
void stil_drive::on_btn_Analyze_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this, "选择测头原始数据", "", "CSV 文件 (*.csv)");
    if (filePath.isEmpty()) return;

    AnalyzeResult res = SurfaceAnalyzer::analyzeFromCSV(filePath);
    if (res.validPoints < 3) {
        QMessageBox::warning(this, "警告", "有效数据点不足！");
        return;
    }

    QString report = QString("有效点数: %1\nX倾角: %2度\nY倾角: %3度\nPV: %4um\nRMS: %5um")
        .arg(res.validPoints)
        .arg(res.angleX, 0, 'f', 4)
        .arg(res.angleY, 0, 'f', 4)
        .arg(res.pv, 0, 'f', 4)
        .arg(res.rms, 0, 'f', 4);
    QMessageBox::information(this, "分析报告", report);
}
