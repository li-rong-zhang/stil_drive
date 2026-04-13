#include "stil_drive.h"
#include <QMessageBox>
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

    // =========================================================================
    // 绑定下拉框改变信号：自动生成对应的 PMAC 底层 EQU 脉冲配置代码
    // =========================================================================
    connect(ui.cmb_SyncAxis, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
        int chan = ui.cmb_SyncAxis->itemData(index).toInt();
        QString scriptTemplate;

        // -----------------------------------
        // 【模式 1】：跟随 X 轴 (直线扫描模式)
        // -----------------------------------
        if (chan == 0) {
            ui.lineEdit_Radius->setEnabled(false);
            ui.lineEdit_Pitch->setEnabled(false);

            scriptTemplate = QString(
                "#1 j/\n"                  // 温和停车
                "Motor[1].ServoCtrl=1\n"   // 明确 X 轴是 1 号电机
                "undefine all\n"
                "&1 #1->X\n"
                // 速度: 1 毫米/秒 (1000Hz 触发频率) = 128000 jog/ms
                "Motor[1].JogSpeed=128000\n"

                "Gate3[0].Chan[%1].Equ1Ena=0\n"
                "Gate3[0].Chan[%1].EquOutPol=0\n"
                "Gate3[0].Chan[%1].EquOutMask=1\n"
                "Gate3[0].Chan[%1].EquWrite=1\n"

                // 【精准触发】: 1微米 = 32000 counts
                "Gate3[0].Chan[%1].CompAdd=32000\n"
                "Gate3[0].Chan[%1].CompA=Gate3[0].Chan[%1].ServoCapt+32000\n"
                "Gate3[0].Chan[%1].CompB=Gate3[0].Chan[%1].CompA+16000\n" 

                "Gate3[0].Chan[%1].Equ1Ena=1\n"
                "#1 j+\n"
            ).arg(chan);
        }
        // -----------------------------------
        // 【模式 2】：跟随 C 轴 (主轴旋转扫描)
        // -----------------------------------
        else if (chan == 2) {
            ui.lineEdit_Radius->setEnabled(true);
            ui.lineEdit_Pitch->setEnabled(true);

            scriptTemplate = QString(
                "#5 j/\n" 
                "Motor[5].ServoCtrl=1\n"
                "undefine all\n"
                "&1 #5->C\n"
                // 速度: 100 度/秒 (1000Hz 触发频率) = 163840 jog/ms
                "Motor[5].JogSpeed=163840\n"

                "Gate3[0].Chan[%1].Equ1Ena=0\n"
                "Gate3[0].Chan[%1].EquOutPol=0\n"
                "Gate3[0].Chan[%1].EquOutMask=1\n"
                "Gate3[0].Chan[%1].EquWrite=1\n"

                // 【精准触发】: 0.1度 = 40960 counts
                "Gate3[0].Chan[%1].CompAdd=40960\n"
                "Gate3[0].Chan[%1].CompA=Gate3[0].Chan[%1].ServoCapt+40960\n"
                "Gate3[0].Chan[%1].CompB=Gate3[0].Chan[%1].CompA+20480\n" // 50% 占空比

                "Gate3[0].Chan[%1].Equ1Ena=1\n"
                "#5 j+\n"
            ).arg(chan);
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
            m_csvStream << "X_Axis(um/deg),Y_Axis(um),Z_Altitude(um),Intensity(%)\n";
        }
    }

    // 启动数据接收线程
    m_thread = new SensorThread(m_sensorID, this);
    connect(m_thread, &SensorThread::dataReady, this, &stil_drive::handleDataReady);
    connect(m_thread, &SensorThread::errorOccurred, this, &stil_drive::handleError);
    m_thread->start();

    ui.lbl_Status->setText("状态：等待硬件触发脉冲 (运动由NC软件控制)...");
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
            "Gate3[0].Chan[0].Equ1Ena=0\n"   
            "Gate3[0].Chan[2].Equ1Ena=0\n";  
        pmacSocket->write(stopCmd.toUtf8());
        ui.textBrowser_pmac_log->append("<font color='red'>已发送停止运动及关闭脉冲指令。</font>");
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

    // 【模式 A：直线跟随解算】
    if (syncMode == 0) {
        double pulse_pitch_um = 1.0;  // 假设设置的 CompAdd=1000 对应物理 1um
        int direction = 1;
        double start_X = 0.0;

        for (int i = 0; i < dataSize; i++) {
            double simulated_X = start_X + (m_totalPoints + i) * pulse_pitch_um * direction;
            xData[i] = simulated_X;

            if (m_csvFile.isOpen()) {
                m_csvStream << simulated_X << ",0.0," << alts[i] << "," << ints[i] << "\n";
            }
            if (i == dataSize - 1) current_display_X = simulated_X;
        }
        window_width = 1000 * pulse_pitch_um;
        unit_str = "um";
    }
    // 【模式 B：螺旋线跟随解算】
    else if (syncMode == 2) {
        double pulse_angle_deg = 0.1; // 假设 CompAdd=100 对应 0.1度

        // 从 UI 提取对应的半径和螺距，需与发给 PMAC 的代码逻辑相吻合
        double start_radius_um = ui.lineEdit_Radius->text().toDouble();
        if (start_radius_um <= 0) start_radius_um = 10000.0; // 默认 10mm

        double pitch_per_rev_um = ui.lineEdit_Pitch->text().toDouble();
        if (pitch_per_rev_um <= 0) pitch_per_rev_um = 10.0;  // 默认每转进给 10um

        for (int i = 0; i < dataSize; i++) {
            double current_angle_deg = (m_totalPoints + i) * pulse_angle_deg;
            double revolutions = current_angle_deg / 360.0;

            // 实时半径 = 初始半径 - 圈数 * 螺距
            double current_radius = start_radius_um - (revolutions * pitch_per_rev_um);
            if (current_radius < 0) current_radius = 0.0;

            // 极坐标转直角坐标
            double angle_rad = current_angle_deg * M_PI / 180.0;
            double simulated_X = current_radius * std::cos(angle_rad);
            double simulated_Y = current_radius * std::sin(angle_rad);

            xData[i] = current_angle_deg; // 画图横坐标以“度数”展开显示

            if (m_csvFile.isOpen()) {
                m_csvStream << simulated_X << "," << simulated_Y << "," << alts[i] << "," << ints[i] << "\n";
            }
            if (i == dataSize - 1) current_display_X = current_angle_deg;
        }
        window_width = 1000 * pulse_angle_deg;
        unit_str = "度";
    }

    // 刷新波形图
    ui.plot_Widget->graph(0)->addData(xData, alts);
    m_totalPoints += dataSize;

    ui.plot_Widget->xAxis->setRange(current_display_X - window_width, current_display_X, Qt::AlignRight);
    ui.plot_Widget->graph(0)->rescaleValueAxis(true);
    ui.plot_Widget->replot();

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