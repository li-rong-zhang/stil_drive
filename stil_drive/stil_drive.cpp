`   12QA43WS54ED6RFT5GYUHIJOK89LP0[-;]'=\784'#include "stil_drive.h"
#include <QMessageBox>+
#include <QDebug>
#include <QDir>
#include <QDateTime>
#include "qcustomplot.h" 
#include <QPen>
#include <Eigen/Dense>

// 必须加此宏以使用 M_PI，且必须放在 cmath 前面
#define _USE_MATH_DEFINES
#include <cmath>

// =========================================================================
// 辅助函数：计算非球面 Z 值
// =========================================================================
static double calcAsphericZ(double r_mm, double R_mm, double k,
    double A4, double A6, double A8, double A10, double A12)
{
    if (R_mm <= 0) return 0;

    double c = 1.0 / R_mm;  // 曲率
    double r2 = r_mm * r_mm;
    double c2 = c * c;

    // 圆锥面部分: Z_conic = c*r² / (1 + sqrt(1 - (1+k)*c²*r²))
    double sqrt_term = 1.0 - (1.0 + k) * c2 * r2;
    double z_conic = 0.0;

    if (sqrt_term > 1e-10) {
        z_conic = (c * r2) / (1.0 + std::sqrt(sqrt_term));
    }

    // 高次项部分 (单位: mm，需要转换为 µm)
    double z_high = A4 * std::pow(r_mm, 4)
        + A6 * std::pow(r_mm, 6)
        + A8 * std::pow(r_mm, 8)
        + A10 * std::pow(r_mm, 10)
        + A12 * std::po
        (r_mm, 12);

    // 总 Z 值 (圆锥面部分转换为 µm)
    return z_conic * 1000.0 + z_high;
}

// =========================================================================
// 辅助函数：根据斜率约束计算安全步距
// =========================================================================
static double calcSafeStepSize(double r_mm, double R_mm, double k,
    double A4, double A6, double A8, double A10, double A12,
    double max_z_change_um = 100.0)
{
    if (R_mm <= 0) return 10.0; // 默认 10µm

    double c = 1.0 / R_mm;
    double c2 = c * c;
    double r2 = r_mm * r_mm;
    double r3 = r2 * r_mm;
    double r5 = r3 * r2;
    double r7 = r5 * r2;
    double r9 = r7 * r2;

    // 计算圆锥面部分的斜率 dZ_conic/dr
    double sqrt_term = 1.0 - (1.0 + k) * c2 * r2;
    double dZ_conic_dr = 0.0;

    if (sqrt_term > 1e-10) {
        double S = std::sqrt(sqrt_term);
        // d/dr [c*r²/(1+S)] 的解析导数
        dZ_conic_dr = (2.0 * c * r_mm * (1.0 + S) - c * r3 * (1.0 + k) * c2 * r_mm / S)
            / ((1.0 + S) * (1.0 + S));
    }

    // 高次项斜率
    double dZ_high_dr = 4.0 * A4 * r3
        + 6.0 * A6 * r5
        + 8.0 * A8 * r7
        + 10.0 * A10 * r9
        + 12.0 * A12 * r2 * r9;

    // 总斜率 (dZ/dr 的单位: µm/mm = µm/µm * 1000)
    double total_slope = std::abs((dZ_conic_dr * 1000.0 + dZ_high_dr));

    if (total_slope < 0.001) total_slope = 0.001; // 避免除零

    // 安全步距: max_z_change_um / |dZ/dr|
    double safe_step_um = max_z_change_um / total_slope;

    // 限制在合理范围内
    if (safe_step_um < 0.5) safe_step_um = 0.5;   // 最小 0.5µm
    if (safe_step_um > 50.0) safe_step_um = 50.0; // 最大 50µm

    return safe_step_um;
}

// =========================================================================
// 1. 构造函数与初始化
// =========================================================================
stil_drive::stil_drive(QWidget* parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    // --- 1.1 初始化测头频率下拉框 ---
    ui.cmb_Freq->addItem("100 Hz");
    ui.cmb_Freq->addItem("200 Hz");
    ui.cmb_Freq->addItem("400 Hz");
    ui.cmb_Freq->addItem("1000 Hz");
    ui.cmb_Freq->addItem("2000 Hz");
    ui.cmb_Freq->setCurrentIndex(3); // 默认 1000 Hz

    // --- 1.2 初始化 PMAC 同步轴下拉框及动态脚本生成 ---
    ui.cmb_SyncAxis->addItem("跟随 X 轴 (Chan 0)", 0);
    ui.cmb_SyncAxis->addItem("跟随 C 轴 (Chan 2)", 2);
    ui.cmb_SyncAxis->addItem("球面纬线扫描 (Chan 2)", 3);
    ui.cmb_SyncAxis->addItem("非球面母线扫描", 4);

    // =========================================================================
    // 绑定下拉框改变信号：自动生成对应的 PMAC 底层 EQU 脉冲配置代码
    // =========================================================================
    connect(ui.cmb_SyncAxis, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
        int mode = ui.cmb_SyncAxis->itemData(index).toInt();
        QString scriptTemplate;

        // 【核心拓扑映射】
        // 模式 0 (X轴): 第 1 块 24E3 板 (Gate 0) 的第 1 个通道 (Chan 0)
        // 模式 2/3 (C轴): 第 2 块 24E3 板 (Gate 1) 的第 1 个通道 (Chan 0)
        //int targetGate = (mode == 0) ? 0 : 1;
        //int targetChan = 0;
        int targetGate = 1;
        int targetChan = 0;

        // -----------------------------------
        // 【模式 1】：跟随 X 轴 (直线扫描模式)
        // -----------------------------------
        if (mode == 0) {
            ui.lineEdit_Radius->setEnabled(false);
            ui.lineEdit_Pitch->setEnabled(false);

            scriptTemplate = QString(
                "abort\n"
                "#1 j/\n"
                "Motor[1].ServoCtrl=1\n"
                "undefine all\n"
                "&1 #1->X\n"
                "Motor[1].JogSpeed=128000\n" // 1mm/s

                // 注意这里的动态参数：Gate3[%1].Chan[%2]
                "Gate3[%1].Chan[%2].Equ1Ena=0\n"
                "Gate3[%1].Chan[%2].EquOutPol=0\n"
                "Gate3[%1].Chan[%2].EquOutMask=1\n"
                "Gate3[%1].Chan[%2].EquWrite=1\n"

                "Gate3[%1].Chan[%2].CompAdd=100000\n" // 1微米
                "Gate3[%1].Chan[%2].CompA=Gate3[%1].Chan[%2].ServoCapt+100000\n"
                "Gate3[%1].Chan[%2].CompB=Gate3[%1].Chan[%2].CompA+50000\n"

                "Gate3[%1].Chan[%2].Equ1Ena=1\n"
                "#1 j+\n"
            ).arg(targetGate).arg(targetChan);
        }
        // -----------------------------------
        // 【模式 2】：跟随 C 轴 (螺旋线扫描)
        // -----------------------------------
        else if (mode == 2) {
            ui.lineEdit_Radius->setEnabled(true);
            ui.lineEdit_Pitch->setEnabled(true);

            double pitch_um = ui.lineEdit_Pitch->text().toDouble();
            if (pitch_um <= 0) pitch_um = 10.0;
            long x_jog_speed = (long)((pitch_um / 0.36) * 128.0);

            scriptTemplate = QString(
                "abort\n"
                "#1 j/ #5 j/\n"
                "Motor[1].ServoCtrl=1\n"
                "Motor[5].ServoCtrl=1\n"
                "&1 #1->X #5->C\n"

                "Motor[5].JogSpeed=1638400\n" // C轴 1000度/s
                "Motor[1].JogSpeed=%3\n"     // X轴进给速度

                "Gate3[%1].Chan[%2].Equ1Ena=0\n"
                "Gate3[%1].Chan[%2].EquOutPol=0\n"
                "Gate3[%1].Chan[%2].EquOutMask=1\n"
                "Gate3[%1].Chan[%2].EquWrite=1\n"

                "Gate3[%1].Chan[%2].CompAdd=204800\n" // 0.5度
                "Gate3[%1].Chan[%2].CompA=Gate3[%1].Chan[%2].ServoCapt+204800\n"
                "Gate3[%1].Chan[%2].CompB=Gate3[%1].Chan[%2].CompA+102400\n"

                "Gate3[%1].Chan[%2].Equ1Ena=1\n"
                "#1 j- #5 j+\n"
            ).arg(targetGate).arg(targetChan).arg(x_jog_speed);
        }
        // -----------------------------------
        // 【模式 3】：球面纬线扫描 (单纯旋转)
        // -----------------------------------
        else if (mode == 3) {
            ui.lineEdit_Radius->setEnabled(true);
            ui.lineEdit_Pitch->setEnabled(false);

            scriptTemplate = QString(
                "#5 j/\n"
                "Motor[5].ServoCtrl=1\n"
                "undefine all\n"
                "&1 #5->C\n"
                "Motor[5].JogSpeed=163840\n"

                "Gate3[%1].Chan[%2].Equ1Ena=0\n"
                "Gate3[%1].Chan[%2].EquOutPol=0\n"
                "Gate3[%1].Chan[%2].EquOutMask=1\n"
                "Gate3[%1].Chan[%2].EquWrite=1\n"

                "Gate3[%1].Chan[%2].CompAdd=40960\n" // 0.1度
                "Gate3[%1].Chan[%2].CompA=Gate3[%1].Chan[%2].ServoCapt+40960\n"
                "Gate3[%1].Chan[%2].CompB=Gate3[%1].Chan[%2].CompA+20480\n"

                "Gate3[%1].Chan[%2].Equ1Ena=1\n"
                "#5 j+\n"
            ).arg(targetGate).arg(targetChan);
        }
        // -----------------------------------
        // 【模式 4】：非球面母线扫描 (带Z轴轨迹跟随)
        // -----------------------------------
        else if (mode == 4) {
            ui.lineEdit_Radius->setEnabled(true);
            ui.lineEdit_Pitch->setEnabled(false);

            bool followZ = ui.chk_FollowZ->isChecked();

            // 读取非球面参数
            double R_mm = ui.lineEdit_Aspheric_R->text().toDouble();
            double k = ui.lineEdit_Aspheric_K->text().toDouble();
            double A4 = ui.lineEdit_Aspheric_A4->text().toDouble();
            double A6 = ui.lineEdit_Aspheric_A6->text().toDouble();
            double A8 = ui.lineEdit_Aspheric_A8->text().toDouble();
            double A10 = ui.lineEdit_Aspheric_A10->text().toDouble();
            double A12 = ui.lineEdit_Aspheric_A12->text().toDouble();

            if (R_mm <= 0) R_mm = 100.0;

            // 起始半径 (µm) 和扫描范围 (µm)
            double start_radius_um = ui.lineEdit_Radius->text().toDouble();
            double scan_range_um = ui.lineEdit_Aspheric_Range->text().toDouble();
            if (scan_range_um <= 0) scan_range_um = 50000.0;

            // 根据斜率计算安全步距
            double step_um = calcSafeStepSize(start_radius_um / 1000.0, R_mm, k, A4, A6, A8, A10, A12);

            if (followZ) {
                // ========== Z轴轨迹跟随模式 ==========
                // 生成带Z轴联动的PMAC脚本
                // 使用IMOV增量移动命令

                // 计算轨迹点数量
                int num_points = (int)(scan_range_um / step_um) + 1;
                if (num_points > 5000) num_points = 5000; // 限制最大点数

                double prev_z_um = calcAsphericZ(start_radius_um / 1000.0, R_mm, k, A4, A6, A8, A10, A12);

                QString trajectoryScript = "abort\n";
                trajectoryScript += "#1j/ #3j/\n";  // 停止X和Z轴
                trajectoryScript += "Motor[1].ServoCtrl=1\n";
                trajectoryScript += "Motor[3].ServoCtrl=1\n";  // Z轴
                trajectoryScript += "undefine all\n";
                trajectoryScript += "&1 #1->X #3->Z\n";
                trajectoryScript += "Motor[1].JogSpeed=128000\n";
                trajectoryScript += "Motor[3].JogSpeed=128000\n";

                // 初始位置归零
                trajectoryScript += "X1=0 Z3=0\n";
                trajectoryScript += "IMOV\n";
                trajectoryScript += "Dwell=200\n";

                // 生成轨迹
                for (int i = 1; i < num_points; i++) {
                    double r_um = start_radius_um + i * step_um;
                    double r_mm = r_um / 1000.0;
                    double z_um = calcAsphericZ(r_mm, R_mm, k, A4, A6, A8, A10, A12);
                    double dz_um = z_um - prev_z_um;

                    // X增量 (µm), Z增量 (µm)
                    // PMAC中单位通常是 counts，需要转换
                    // 假设 1µm = 100000 counts
                    long dx_counts = (long)(step_um * 100000.0);
                    long dz_counts = (long)(dz_um * 100000.0);  // Z轴同样的分辨率

                    trajectoryScript += QString("X1=%1 Z3=%2 IMOV Dwell=50\n").arg(dx_counts).arg(dz_counts);

                    prev_z_um = z_um;
                }

                // 触发急停
                trajectoryScript += "abort\n";
                trajectoryScript += "Gate3[1].Chan[0].Equ1Ena=0\n";

                scriptTemplate = trajectoryScript;

                ui.textBrowser_pmac_log->append(QString("<font color='blue'>[Z轴跟随] 步距: %1 µm, 轨迹点数: %2</font>")
                    .arg(step_um, 0, 'f', 2).arg(num_points));
            } else {
                // ========== 纯X轴扫描模式 ==========
                // 不带Z轴跟随，直接用X轴触发
                scriptTemplate = QString(
                    "abort\n"
                    "#1 j/\n"
                    "Motor[1].ServoCtrl=1\n"
                    "undefine all\n"
                    "&1 #1->X\n"
                    "Motor[1].JogSpeed=128000\n"

                    "Gate3[%1].Chan[%2].Equ1Ena=0\n"
                    "Gate3[%1].Chan[%2].EquOutPol=0\n"
                    "Gate3[%1].Chan[%2].EquOutMask=1\n"
                    "Gate3[%1].Chan[%2].EquWrite=1\n"

                    "Gate3[%1].Chan[%2].CompAdd=%3\n"
                    "Gate3[%1].Chan[%2].CompA=Gate3[%1].Chan[%2].ServoCapt+%3\n"
                    "Gate3[%1].Chan[%2].CompB=Gate3[%1].Chan[%2].CompA+%4\n"

                    "Gate3[%1].Chan[%2].Equ1Ena=1\n"
                    "#1 j+\n"
                ).arg(targetGate).arg(targetChan).arg((long)(step_um * 100000.0)).arg((long)(step_um * 50000.0));
            }
        }

        ui.textEdit_pmac_script->setPlainText(scriptTemplate);
        });

    // 初始化触发一次，填入默认的 X 轴代码
    ui.cmb_SyncAxis->setCurrentIndex(0);

    // --- 1.3 初始化界面状态 ---
    ui.btn_Disconnect->setEnabled(false);
    ui.btn_Start->setEnabled(false);
    ui.btn_Stop->setEnabled(false);

    // --- 1.4 图表初始化 (QCustomPlot) ---
    ui.plot_Widget->addGraph();
    ui.plot_Widget->graph(0)->setPen(QPen(Qt::blue));
    ui.plot_Widget->xAxis->setLabel("采样点数 / 位置");
    ui.plot_Widget->yAxis->setLabel("高度 / Altitude (um)");
    ui.plot_Widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    // --- 1.5 PMAC TCP 网络通讯初始化 ---
    pmacSocket = new QTcpSocket(this);
    connect(pmacSocket, &QTcpSocket::connected, this, &stil_drive::onPmacConnected);
    connect(pmacSocket, &QTcpSocket::disconnected, this, &stil_drive::onPmacDisconnected);
    connect(pmacSocket, &QTcpSocket::readyRead, this, &stil_drive::onPmacReadyRead);
    connect(pmacSocket, &QTcpSocket::errorOccurred, this, [=](QTcpSocket::SocketError socketError) {
        ui.textBrowser_pmac_log->append("<font color='red'>网络报错: " + pmacSocket->errorString() + "</font>");
        });
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

    if (pmacSocket && pmacSocket->state() == QAbstractSocket::ConnectedState) {
        pmacSocket->disconnectFromHost();
    }
}

// =========================================================================
// 2. STIL 测头控制逻辑
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

    char* UsbCCSDeviceList[MCHR_MAX_SENSOR];
    for (int i = 0; i < MCHR_MAX_SENSOR; i++) {
        UsbCCSDeviceList[i] = new char[MAX_PATH];
        memset(UsbCCSDeviceList[i], 0, MAX_PATH);
    }

    short deviceNumber = 0;
    if ((MCHR_GetUsbDeviceList(UsbCCSDeviceList, &deviceNumber) != MCHR_ERROR) && (deviceNumber > 0)) {
        m_sensorID = MCHR_OpenUsbChr(UsbCCSDeviceList[0], MCHR_CCS_PRIMA, UsbCCSDeviceList[0], NULL, NULL);
        if (m_sensorID != 0) {
            ui.btn_Connect->setText("已连接");
            ui.btn_Disconnect->setEnabled(true);
            ui.btn_Start->setEnabled(true);
            QMessageBox::information(this, "成功", QString("成功连接到测头！\n设备名: %1").arg(UsbCCSDeviceList[0]));
        }
        else {
            ui.btn_Connect->setText("连接测头");
            ui.btn_Connect->setEnabled(true);
            QMessageBox::critical(this, "失败", "找到设备但连接失败！");
        }
    }
    else {
        ui.btn_Connect->setText("连接测头");
        ui.btn_Connect->setEnabled(true);
        QMessageBox::warning(this, "警告", "未扫描到测头！请拔插USB线重试。");
    }

    for (int i = 0; i < MCHR_MAX_SENSOR; i++) {
        delete[] UsbCCSDeviceList[i];
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
    if (pmacSocket->state() != QAbstractSocket::ConnectedState) {
        QMessageBox::warning(this, "警告", "请先连接 Power PMAC 控制器！");
        return;
    }
    if (m_thread != nullptr && m_thread->isRunning()) return;

    // 获取并设置测头频率限制 (防呆)
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

    if (ui.chk_SaveData->isChecked()) {
        QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + "_SyncData.csv";
        m_csvFile.setFileName(fileName);
        if (m_csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            m_csvStream.setDevice(&m_csvFile);
            int syncMode = ui.cmb_SyncAxis->currentData().toInt();
            if (syncMode == 4) {
                // 非球面母线扫描：额外输出理论Z值和偏差
                m_csvStream << "X_Axis(um),Y_Axis(um),Z_Measured(um),Z_Theory(um),Z_Deviation(um),Intensity(%)\n";
            } else {
                m_csvStream << "X_Axis(um/deg),Y_Axis(um),Z_Altitude(um),Intensity(%)\n";
            }
        }
    }

    // 启动数据接收线程
    m_thread = new SensorThread(m_sensorID, this);
    connect(m_thread, &SensorThread::dataReady, this, &stil_drive::handleDataReady);
    connect(m_thread, &SensorThread::errorOccurred, this, &stil_drive::handleError);
    m_thread->start();

    ui.lbl_Status->setText("状态：等待硬件触发脉冲...");
    ui.btn_Start->setEnabled(false);
    ui.btn_Stop->setEnabled(true);

    // ==========================================
    // 【核心联动】：发送UI文本框中的运动及触发代码给 PMAC
    // ==========================================
    ui.textBrowser_pmac_log->append("正在下发硬件脉冲同步指令...");
    QString cmd = ui.textEdit_pmac_script->toPlainText().trimmed();
    if (!cmd.endsWith("\n")) {
        cmd += "\n";
    }
    pmacSocket->write(cmd.toUtf8());
}

void stil_drive::on_btn_Stop_clicked()
{
    if (m_thread != nullptr) {
        m_thread->stopAcquisition();
        m_thread->wait();
        m_thread->deleteLater();
        m_thread = nullptr;
    }
    if (m_csvFile.isOpen()) m_csvFile.close();

    ui.btn_Start->setEnabled(true);
    ui.btn_Stop->setEnabled(false);
    ui.lbl_Status->setText("状态：已停止");

    // 急停机床并关闭硬件脉冲，防止误触发
    if (pmacSocket && pmacSocket->state() == QAbstractSocket::ConnectedState) {
        QString stopCmd =
            "#1j/ #2j/ #3j/ #4j/ #5j/\n"
            "Gate3[0].Chan[0].Equ1Ena=0\n"   // 关 X 轴触发 (第一块板)
            "Gate3[1].Chan[0].Equ1Ena=0\n";  // 关 C 轴触发 (第二块板)
        pmacSocket->write(stopCmd.toUtf8());
        ui.textBrowser_pmac_log->append("<font color='red'>已安全制动全轴并撤销触发使能。</font>");
    }
}

// =========================================================================
// 3. 核心解算：根据脉冲还原三维坐标
// =========================================================================
void stil_drive::handleDataReady(QVector<double> alts, QVector<double> ints)
{
    int dataSize = alts.size();
    if (dataSize == 0) return;

    QVector<double> xData(dataSize);
    int syncMode = ui.cmb_SyncAxis->currentData().toInt();

    double current_display_X = 0.0;
    double window_width = 1000.0;
    QString unit_str = "um";

    // =========================================================================
    // 【模式 1】：直线跟随解算 (跟随 X 轴)
    // =========================================================================
    if (syncMode == 0) {
        double pulse_pitch_um = 1.0;  // 严格对应底层设置的 1um 触发间距
        double start_X = 0.0;

        for (int i = 0; i < dataSize; i++) {
            double simulated_X = start_X + (m_totalPoints + i) * pulse_pitch_um;
            xData[i] = simulated_X;

            if (m_csvFile.isOpen()) {
                m_csvStream << simulated_X << ",0.0," << alts[i] << "," << ints[i] << "\n";
            }
            if (i == dataSize - 1) current_display_X = simulated_X;
        }
        window_width = 1000 * pulse_pitch_um; // 图表显示最近 1000 个点的范围
        unit_str = "um";
    }
    // =========================================================================
    // 【模式 2】：螺旋线跟随解算 (跟随 C 轴 + X 轴联动)
    // =========================================================================
    else if (syncMode == 2) {
        // 【关键修改】：必须和底层的 CompAdd 严格对齐！
        // 因为底层现在是每 0.5 度打一枪，软件这里必须按 0.5 度来推算当前转到了哪里
        double pulse_angle_deg = 0.5;

        // 1. 提取并统一单位 (将界面的 mm 转换为 um)
        double r_start_mm = ui.lineEdit_Radius->text().toDouble();
        double start_radius_um = r_start_mm * 1000.0;

        double pitch_per_rev_um = ui.lineEdit_Pitch->text().toDouble();
        if (pitch_per_rev_um <= 0) pitch_per_rev_um = 10.0;  // 默认每转进给 10um

        double r_over_um = 1000.0; // 过圆心 1mm (1000um)

        // 2. 自动停止逻辑校验 (这里的逻辑不需要改，它会自动根据 0.5 度算对圈数)
        double total_angle_needed = ((start_radius_um + r_over_um) / pitch_per_rev_um) * 360.0;
        double current_total_angle = m_totalPoints * pulse_angle_deg;

        if (current_total_angle >= total_angle_needed) {
            on_btn_Stop_clicked();
            ui.lbl_Status->setText(QString("状态：极速螺旋扫描完成！已自动停止。"));
            return;
        }

        // 3. 数据极坐标转直角坐标解算
        for (int i = 0; i < dataSize; i++) {
            double current_angle_deg = (m_totalPoints + i) * pulse_angle_deg;
            double revolutions = current_angle_deg / 360.0;

            // 实时半径 = 初始半径 - 圈数 * 螺距 
            // (注意：这里允许半径变为负数，以实现完美的过圆心计算)
            double current_radius = start_radius_um - (revolutions * pitch_per_rev_um);

            // 极坐标转直角坐标 (X, Y)
            double angle_rad = current_angle_deg * M_PI / 180.0;
            double simulated_X = current_radius * std::cos(angle_rad);
            double simulated_Y = current_radius * std::sin(angle_rad);

            xData[i] = current_angle_deg; // 画图横坐标依然以“度数”展开显示，方便观察形貌

            if (m_csvFile.isOpen()) {
                // CSV 导出真实的 X、Y 直角坐标用于后期 Eigen 面形拟合
                m_csvStream << simulated_X << "," << simulated_Y << "," << alts[i] << "," << ints[i] << "\n";
            }
            if (i == dataSize - 1) current_display_X = current_angle_deg;
        }
        window_width = 360.0; // 螺旋线图表默认显示最近一圈 (360度) 的波形
        unit_str = "度";
    }
    // =========================================================================
      // 【模式 3】：球面纬线扫描 (跟随 C 轴单纯旋转)
      // =========================================================================
    else if (syncMode == 3) {
        double pulse_angle_deg = 0.1; // 0.1 度触发一次

        // 【核心设定】：设定目标角度为 370 度 (360度主体 + 10度重叠区)
        double target_angle_deg = 370.0;

        // 1. 自动停止逻辑校验
        double current_total_angle = m_totalPoints * pulse_angle_deg;
        if (current_total_angle >= target_angle_deg) {
            on_btn_Stop_clicked(); // 达到 370 度，自动急停
            ui.lbl_Status->setText(QString("状态：纬线扫描完成！总计旋转 %1 度，已自动停止。").arg(target_angle_deg));
            return; // 丢弃多余的尾部数据，防止数组越界
        }

        // 2. 数据处理与导出
        for (int i = 0; i < dataSize; i++) {
            double current_angle_deg = (m_totalPoints + i) * pulse_angle_deg;
            xData[i] = current_angle_deg;

            if (m_csvFile.isOpen()) {
                // 纬线扫描只需记录角度和对应的高度
                m_csvStream << current_angle_deg << ",0.0," << alts[i] << "," << ints[i] << "\n";
            }
            if (i == dataSize - 1) current_display_X = current_angle_deg;
        }

        // 图表显示设置：为了能一眼看清重叠效果，把视窗拉宽到 370 度
        window_width = target_angle_deg;
        unit_str = "度";
    }
    // =========================================================================
    // 【模式 4】：非球面母线扫描
    // =========================================================================
    else if (syncMode == 4) {
        // 起始半径 (µm) 和扫描范围 (µm)
        double start_radius_um = ui.lineEdit_Radius->text().toDouble();
        double scan_range_um = ui.lineEdit_Aspheric_Range->text().toDouble();
        if (scan_range_um <= 0) scan_range_um = 50000.0;

        // 读取非球面参数
        double R_mm = ui.lineEdit_Aspheric_R->text().toDouble();        // 曲率半径 (mm)
        double k = ui.lineEdit_Aspheric_K->text().toDouble();           // 圆锥系数
        double A4 = ui.lineEdit_Aspheric_A4->text().toDouble();          // 4次项系数
        double A6 = ui.lineEdit_Aspheric_A6->text().toDouble();          // 6次项系数
        double A8 = ui.lineEdit_Aspheric_A8->text().toDouble();          // 8次项系数
        double A10 = ui.lineEdit_Aspheric_A10->text().toDouble();        // 10次项系数
        double A12 = ui.lineEdit_Aspheric_A12->text().toDouble();        // 12次项系数

        if (R_mm <= 0) R_mm = 100.0; // 默认 100mm

        // 计算安全步距
        double step_um = calcSafeStepSize(start_radius_um / 1000.0, R_mm, k, A4, A6, A8, A10, A12);

        // 自动停止校验
        double current_radius = start_radius_um + m_totalPoints * step_um;
        if (current_radius >= start_radius_um + scan_range_um) {
            on_btn_Stop_clicked();
            ui.lbl_Status->setText(QString("状态：非球面母线扫描完成！已自动停止。"));
            return;
        }

        // 计算理论非球面 Z 值
        for (int i = 0; i < dataSize; i++) {
            double current_r_um = start_radius_um + (m_totalPoints + i) * step_um;
            double current_r_mm = current_r_um / 1000.0;

            // 使用辅助函数计算理论 Z 值
            double z_theory = calcAsphericZ(current_r_mm, R_mm, k, A4, A6, A8, A10, A12);

            // 计算实测偏差
            double z_deviation = alts[i] - z_theory;

            xData[i] = current_r_um;

            if (m_csvFile.isOpen()) {
                // CSV: 半径(µm), 0, 实测Z(µm), 理论Z(µm), 偏差(µm), 强度
                m_csvStream << current_r_um << ",0.0," << alts[i] << "," << z_theory << "," << z_deviation << "," << ints[i] << "\n";
            }
            if (i == dataSize - 1) current_display_X = current_r_um;
        }

        window_width = scan_range_um;
        unit_str = "µm";
    }

    // =========================================================================
    // 刷新 QCustomPlot 波形图
    // =========================================================================
    ui.plot_Widget->graph(0)->addData(xData, alts);
    m_totalPoints += dataSize;

    // 动态移动 X 轴视窗，实现心电图般的滚动效果
    ui.plot_Widget->xAxis->setRange(current_display_X - window_width, current_display_X, Qt::AlignRight);
    ui.plot_Widget->graph(0)->rescaleValueAxis(true); // Y 轴高度自动缩放
    ui.plot_Widget->replot();

    // 更新底部状态栏
    ui.lbl_Status->setText(QString("状态：飞拍中 | 位置: %1 %2 | 高度: %3 um")
        .arg(current_display_X, 0, 'f', 2).arg(unit_str).arg(alts.last(), 0, 'f', 2));
}

void stil_drive::handleError(QString msg)
{
    QMessageBox::critical(this, "底层错误", msg);
    on_btn_Stop_clicked();
}

// =========================================================================
// 4. 数据后处理与面形分析 (Eigen)
// =========================================================================
void stil_drive::on_btn_Analyze_clicked()
{
    // ...[保持你原有的 on_btn_Analyze_clicked Eigen 矩阵运算不变] ...
    // (因篇幅限制，保留你原先完美可运行的代码，无需任何修改)
    QString filePath = QFileDialog::getOpenFileName(this, "选择测头原始数据", "", "CSV 文件 (*.csv)");
    if (filePath.isEmpty()) return;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "错误", "无法打开数据文件！");
        return;
    }

    QTextStream in(&file);
    in.readLine(); // 跳过表头

    std::vector<double> valid_X, valid_Y, valid_Z;

    while (!in.atEnd()) {
        QStringList parts = in.readLine().split(',');
        if (parts.size() >= 4) {
            double x = parts[0].toDouble(), y = parts[1].toDouble(), z = parts[2].toDouble(), intensity = parts[3].toDouble();
            if (intensity > 5.0 && intensity < 95.0 && !std::isnan(z)) {
                valid_X.push_back(x); valid_Y.push_back(y); valid_Z.push_back(z);
            }
        }
    }
    file.close();

    int n = valid_X.size();
    if (n < 3) return;

    Eigen::MatrixXd M(n, 3);
    Eigen::VectorXd Z_vec(n);
    for (int i = 0; i < n; i++) {
        M(i, 0) = valid_X[i]; M(i, 1) = valid_Y[i]; M(i, 2) = 1.0; Z_vec(i) = valid_Z[i];
    }

    Eigen::Vector3d v = M.colPivHouseholderQr().solve(Z_vec);
    double A = v(0), B = v(1), C = v(2);
    double max_res = -1e9, min_res = 1e9, sq_sum = 0.0;

    for (int i = 0; i < n; i++) {
        double residual = valid_Z[i] - (A * valid_X[i] + B * valid_Y[i] + C);
        if (residual > max_res) max_res = residual;
        if (residual < min_res) min_res = residual;
        sq_sum += residual * residual;
    }

    double angleX = std::atan(A) * 180.0 / M_PI;
    double angleY = std::atan(B) * 180.0 / M_PI;

    QString report = QString("有效点数: %1\nX倾角: %2度\nY倾角: %3度\nPV: %4um\nRMS: %5um")
        .arg(n).arg(angleX, 0, 'f', 4).arg(angleY, 0, 'f', 4).arg(max_res - min_res, 0, 'f', 4).arg(std::sqrt(sq_sum / n), 0, 'f', 4);
    QMessageBox::information(this, "分析报告", report);
}

// =========================================================================
// 5. Power PMAC 网络通讯 (全自动 Telnet 破防)
// =========================================================================
void stil_drive::on_btn_pmac_connect_clicked()
{
    if (pmacSocket->state() != QAbstractSocket::ConnectedState) {
        pmacSocket->connectToHost(ui.lineEdit_pmac_ip->text(), ui.lineEdit_pmac_port->text().toUShort());
    }
    else {
        pmacSocket->disconnectFromHost();
    }
}

void stil_drive::onPmacConnected() {
    ui.btn_pmac_connect->setText("断开控制器");
    ui.textBrowser_pmac_log->append("<font color='green'>成功连接到 Power PMAC!</font>");
}

void stil_drive::onPmacDisconnected() {
    ui.btn_pmac_connect->setText("连接控制器");
    ui.textBrowser_pmac_log->append("<font color='red'>已断开与 PMAC 的连接。</font>");
}

void stil_drive::onPmacReadyRead()
{
    QByteArray data = pmacSocket->readAll();
    QByteArray telnetReply;
    QString cleanText;

    // Telnet 控制码过滤
    for (int i = 0; i < data.size(); ) {
        if ((unsigned char)data[i] == 0xFF) {
            if (i + 2 < data.size()) {
                unsigned char cmd = data[i + 1], opt = data[i + 2];
                if (cmd >= 251 && cmd <= 254) {
                    telnetReply.append((char)0xFF);
                    if (cmd == 253) telnetReply.append((char)252);
                    else if (cmd == 251) telnetReply.append((char)254);
                    else telnetReply.append(cmd);
                    telnetReply.append(opt);
                    i += 3; continue;
                }
            }
            i++;
        }
        else {
            cleanText.append(data[i++]);
        }
    }

    if (!telnetReply.isEmpty()) pmacSocket->write(telnetReply);
    cleanText = cleanText.trimmed();
    if (cleanText.isEmpty()) return;

    ui.textBrowser_pmac_log->append("收到: " + cleanText);

    // 自动握手登录
    if (cleanText.contains("login", Qt::CaseInsensitive)) pmacSocket->write("root\n");
    else if (cleanText.contains("Password", Qt::CaseInsensitive)) pmacSocket->write("deltatau\n");
    else if (cleanText.contains("~$") || cleanText.contains("~#") || cleanText.contains("root@")) pmacSocket->write("gpascii -2\n");
    else if (cleanText.contains("STDIN Open for ASCII", Qt::CaseInsensitive)) ui.textBrowser_pmac_log->append("<font color='magenta'><b>-> 运动引擎就绪！可点击【开始采集】</b></font>");
    else if (cleanText.contains("\x06")) ui.textBrowser_pmac_log->append("<font color='blue'>[指令执行成功 (ACK)]</font>");
}

// =========================================================================
// 6. 手动测试发送指令保持不变
// =========================================================================
void stil_drive::on_btn_Send_Cmd_clicked()
{
    if (pmacSocket->state() != QAbstractSocket::ConnectedState) return;
    QString cmd = ui.textEdit_pmac_script->toPlainText().trimmed();
    if (cmd.isEmpty()) return;
    if (!cmd.endsWith("\n")) cmd += "\n";

    pmacSocket->write(cmd.toUtf8());

    QString logMsg = cmd;
    logMsg.replace("\n", " ");
    ui.textBrowser_pmac_log->append("<font color='purple'>[手动测试]: " + logMsg + "</font>");
}