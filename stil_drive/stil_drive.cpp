#include "stil_drive.h"
#include <QMessageBox>
#include <QDebug>
#include <QDir>
#include <QDateTime>
#include "qcustomplot.h" 
#include <QPen>
#include <Eigen/Dense>

// ==========================================
// 构造函数
// ==========================================
stil_drive::stil_drive(QWidget* parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    // ==========================================
    // 初始化频率下拉框 (QComboBox)
    // ==========================================
    ui.cmb_Freq->addItem("100 Hz");
    ui.cmb_Freq->addItem("200 Hz");
    ui.cmb_Freq->addItem("400 Hz");
    ui.cmb_Freq->addItem("1000 Hz");
    ui.cmb_Freq->addItem("2000 Hz");
    ui.cmb_Freq->setCurrentIndex(3);

    // ==========================================
    // 初始化按钮状态 (QPushButton)
    // ==========================================
    ui.btn_Disconnect->setEnabled(false);
    ui.btn_Start->setEnabled(false);
    ui.btn_Stop->setEnabled(false);

    // ==========================================
    // 图表初始化配置 (QCustomPlot)
    // ==========================================
    ui.plot_Widget->addGraph();
    ui.plot_Widget->graph(0)->setPen(QPen(Qt::blue));
    ui.plot_Widget->xAxis->setLabel("采样点数 (Points)");
    ui.plot_Widget->yAxis->setLabel("高度 / Altitude (um)");
    ui.plot_Widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    // ==========================================
    // 初始化 PMAC TCP 通讯
    // ==========================================
    pmacSocket = new QTcpSocket(this);

    // 绑定 Socket 的状态信号到我们自定义的槽函数
    connect(pmacSocket, &QTcpSocket::connected, this, &stil_drive::onPmacConnected);
    connect(pmacSocket, &QTcpSocket::disconnected, this, &stil_drive::onPmacDisconnected);
    connect(pmacSocket, &QTcpSocket::readyRead, this, &stil_drive::onPmacReadyRead);

    // ==========================================
// 【新增】：捕获底层的网络报错，并用红字打印出来！
// ==========================================
    connect(pmacSocket, &QTcpSocket::errorOccurred, this, [=](QTcpSocket::SocketError socketError) {
        QString errStr = pmacSocket->errorString();
        ui.textBrowser_pmac_log->append("<font color='red'>网络报错: " + errStr + "</font>");
        });
}

// ==========================================
// 析构函数
// ==========================================
stil_drive::~stil_drive()
{
    // 1. 线程安全退出处理
    if (m_thread != nullptr) {
        m_thread->stopAcquisition();
        m_thread->wait();
    }

    // 2. 硬件设备断开
    if (m_sensorID != 0) {
        MCHR_CloseChr(m_sensorID);
    }
    MCHR_Release();

    // 3. 文件安全关闭
    if (m_csvFile.isOpen()) {
        m_csvFile.close();
    }

    // ==========================================
    // 安全断开 PMAC 的网络连接
    // ==========================================
    if (pmacSocket && pmacSocket->state() == QAbstractSocket::ConnectedState) {
        pmacSocket->disconnectFromHost();
    }
}

// ==========================================
// STIL 测头测量
// ==========================================
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

void stil_drive::on_btn_Start_clicked()
{
    // 1. 终极查岗：必须双端都连上！
    if (m_sensorID == 0) {
        QMessageBox::warning(this, "警告", "请先连接 STIL 测头！");
        return;
    }
    if (pmacSocket->state() != QAbstractSocket::ConnectedState) {
        QMessageBox::warning(this, "警告", "请先连接 Power PMAC 控制器！");
        return;
    }
    if (m_thread != nullptr && m_thread->isRunning()) return;


    ui.textBrowser_pmac_log->append("<font color='orange'>测头已切换至等待外部 EQU 触发模式...</font>");

    // 3. 准备数据文件和绘图区
    m_totalPoints = 0;
    ui.plot_Widget->graph(0)->data()->clear();

    if (ui.chk_SaveData->isChecked()) {
        QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + "_SyncData.csv";
        m_csvFile.setFileName(fileName);
        if (m_csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            m_csvStream.setDevice(&m_csvFile);
            m_csvStream << "Point_Index,Z_Altitude(um),Intensity(%)\n"; // 硬件同步下，X/Y由脉冲严格对齐
        }
    }

    // 4. 启动接收线程（设下埋伏）
    m_thread = new SensorThread(m_sensorID, this);
    connect(m_thread, &SensorThread::dataReady, this, &stil_drive::handleDataReady);
    connect(m_thread, &SensorThread::errorOccurred, this, &stil_drive::handleError);
    m_thread->start();

    ui.lbl_Status->setText("状态：等待硬件触发脉冲...");
    ui.btn_Start->setEnabled(false);
    ui.btn_Stop->setEnabled(true);

    // 5. 唤醒 PMAC，下发运动与脉冲配置指令（开火！）
    ui.textBrowser_pmac_log->append("正在下发底层运动与脉冲同步指令...");

    QString cmd =
        "Motor[1].ServoCtrl=1\n"
        "Motor[2].ServoCtrl=1\n"
        "Motor[3].ServoCtrl=1\n"
        "#1k #2k #3k\n"
        "#1j/ #2j/ #3j/\n"
        "&1\n"
        "undefine all\n"
        "#1->X\n"
        "#2->Z\n"
        "#3->C\n"
        // ==========================================
        // 【已解封】：配置 X 轴 (0号通道) 为发令枪
        // ==========================================
        "Gate3[0].Chan[0].Equ1Ena=0\n"
        "Gate3[0].Chan[0].EquOutPol=1\n"
        "Gate3[0].Chan[0].EquOutMask=1\n"

        // 假设光栅尺分辨率使得 1000 个 count = 1um
        "Gate3[0].Chan[0].CompAdd=1000\n"
        "Gate3[0].Chan[0].CompA=Gate3[0].Chan[0].ServoCapt+1000\n"
        "Gate3[0].Chan[0].CompB=Gate3[0].Chan[0].CompA+50\n"

        "Gate3[0].Chan[0].Equ1Ena=1\n"
        "b10 r\n";

    pmacSocket->write(cmd.toUtf8());
}

void stil_drive::on_btn_Stop_clicked()
{
    // 1. 停止 STIL 测头接收线程
    if (m_thread != nullptr) {
        m_thread->stopAcquisition();
        m_thread->wait();
        m_thread->deleteLater();
        m_thread = nullptr;
        ui.lbl_Status->setText("状态：已停止");
    }

    // 2. 关闭保存的文件
    if (m_csvFile.isOpen()) {
        m_csvFile.close();
    }

    ui.btn_Start->setEnabled(true);
    ui.btn_Stop->setEnabled(false);

    // ==========================================
    // 3. 【已解封】：紧急制动 PMAC 并关闭 X 轴的 EQU 脉冲
    // ==========================================
    if (pmacSocket && pmacSocket->state() == QAbstractSocket::ConnectedState) {
        QString stopCmd =
            "&1 a\n"                             // 强行中止 1 号坐标系的运动程序 (电机刹车)
            "Gate3[0].Chan[0].Equ1Ena=0\n";      // 彻底关闭 X 轴的 EQU 脉冲输出

        pmacSocket->write(stopCmd.toUtf8());
        ui.textBrowser_pmac_log->append("<font color='red'>已发送停止运动及关闭脉冲指令。</font>");
    }
}

void stil_drive::handleDataReady(QVector<double> alts, QVector<double> ints)
{
    int dataSize = alts.size();
    if (dataSize == 0) return;

    QVector<double> xData(dataSize);

    // ==========================================
    // 【核心重构】：纯硬件步长计算，彻底摒弃时间频率！
    // ==========================================
    // 假设您在 PMAC 里设置的 CompAdd 对应物理距离为 1.0 um
    double pulse_pitch_um = 1.0;
    // 扫描方向：正向走填 1，反向走填 -1
    int direction = 1;
    // 扫描起始位置（假设从 0 开始）
    double start_X = 0.0;

    for (int i = 0; i < dataSize; i++) {
        // 终极公式：当前坐标 = 起点 + (历史总脉冲数 + 当前批次索引) * 步长 * 方向
        double simulated_X = start_X + (m_totalPoints + i) * pulse_pitch_um * direction;
        double simulated_Y = 0.0; // 如果是拉直线，Y 就是 0

        xData[i] = simulated_X;

        if (m_csvFile.isOpen()) {
            m_csvStream << simulated_X << "," << simulated_Y << "," << alts[i] << "," << ints[i] << "\n";
        }
    }

    ui.plot_Widget->graph(0)->addData(xData, alts);
    m_totalPoints += dataSize;

    // 绘图窗口跟随最新点滚动
    double current_X = start_X + m_totalPoints * pulse_pitch_um * direction;
    double window_width_X = 1000 * pulse_pitch_um; // 界面显示最近 1000 个点的宽度
    ui.plot_Widget->xAxis->setRange(current_X - window_width_X, current_X, Qt::AlignRight);

    ui.plot_Widget->graph(0)->rescaleValueAxis(true);
    ui.plot_Widget->replot();

    ui.lbl_Status->setText(QString("状态：飞拍中 | 当前位置 X: %1 um | 最新高度 Z: %2 um")
        .arg(current_X, 0, 'f', 2).arg(alts.last(), 0, 'f', 2));
}

void stil_drive::on_btn_Analyze_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this, "选择测头原始数据", "", "CSV 文件 (*.csv)");
    if (filePath.isEmpty()) return;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "错误", "无法打开数据文件！");
        return;
    }

    QTextStream in(&file);
    QString header = in.readLine();

    std::vector<double> valid_X, valid_Y, valid_Z;

    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList parts = line.split(',');
        if (parts.size() >= 4) {
            double x = parts[0].toDouble();
            double y = parts[1].toDouble();
            double z = parts[2].toDouble();
            double intensity = parts[3].toDouble();

            if (intensity > 5.0 && intensity < 95.0 && !qIsNaN(z)) {
                valid_X.push_back(x);
                valid_Y.push_back(y);
                valid_Z.push_back(z);
            }
        }
    }
    file.close();

    int n = valid_X.size();
    if (n < 3) {
        QMessageBox::warning(this, "失败", "清洗后有效数据不足 3 个点，无法拟合 3D 平面！");
        return;
    }

    Eigen::MatrixXd M(n, 3);
    Eigen::VectorXd Z_vec(n);

    for (int i = 0; i < n; i++) {
        M(i, 0) = valid_X[i];
        M(i, 1) = valid_Y[i];
        M(i, 2) = 1.0;
        Z_vec(i) = valid_Z[i];
    }

    Eigen::Vector3d v = M.colPivHouseholderQr().solve(Z_vec);

    double A = v(0);
    double B = v(1);
    double C = v(2);

    double max_residual = -1e9;
    double min_residual = 1e9;
    double sq_sum = 0.0;

    for (int i = 0; i < n; i++) {
        double fit_Z = A * valid_X[i] + B * valid_Y[i] + C;
        double residual = valid_Z[i] - fit_Z;

        if (residual > max_residual) max_residual = residual;
        if (residual < min_residual) min_residual = residual;
        sq_sum += residual * residual;
    }

    double PV = max_residual - min_residual;
    double RMS = std::sqrt(sq_sum / n);

    double angleX = std::atan(A) * 180.0 / M_PI;
    double angleY = std::atan(B) * 180.0 / M_PI;

    QString report = QString(
        "【数据清洗报告】\n"
        "原始采集点数：海量\n"
        "有效优质点数：%1 个\n\n"
        "【装配倾角分析 (Leveling)】\n"
        "X 轴装配倾角： %2 度\n"
        "Y 轴装配倾角： %3 度\n"
        "Z 轴平均截距： %4 um\n\n"
        "【面形误差评估 (Form Error)】\n"
        "面形 PV 值 (最大起伏)： %5 um\n"
        "面形 RMS 值 (均方根)： %6 um"
    ).arg(n)
        .arg(angleX, 0, 'f', 4)
        .arg(angleY, 0, 'f', 4)
        .arg(C, 0, 'f', 3)
        .arg(PV, 0, 'f', 4)
        .arg(RMS, 0, 'f', 4);

    QMessageBox::information(this, "三维面形分析报告", report);
}

void stil_drive::handleError(QString msg)
{
    QMessageBox::critical(this, "底层错误", msg);
    on_btn_Stop_clicked();
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
    QMessageBox::information(this, "断开", "测头已安全断开！");
}

// ==========================================
// 下面是 PMAC 控制器的通讯逻辑
// ==========================================

// 点击“连接控制器”按钮
void stil_drive::on_btn_pmac_connect_clicked()
{
    if (pmacSocket->state() != QAbstractSocket::ConnectedState) {
        QString ip = ui.lineEdit_pmac_ip->text();
        quint16 port = ui.lineEdit_pmac_port->text().toUShort();

        ui.textBrowser_pmac_log->append(QString("正在连接 PMAC: %1:%2 ...").arg(ip).arg(port));
        pmacSocket->connectToHost(ip, port);
    }
    else {
        // 如果已经连上了，点击按钮就断开
        pmacSocket->disconnectFromHost();
    }
}

// 成功连上 PMAC 时的自动回调
void stil_drive::onPmacConnected()
{
    ui.btn_pmac_connect->setText("断开控制器");
    // 这里使用 HTML 语法让日志显示为绿色，更直观
    ui.textBrowser_pmac_log->append("<font color='green'>成功连接到 Power PMAC!</font>");
}

// 和 PMAC 断开连接时的自动回调
void stil_drive::onPmacDisconnected()
{
    ui.btn_pmac_connect->setText("连接控制器");
    ui.textBrowser_pmac_log->append("<font color='red'>已断开与 PMAC 的连接。</font>");
}

// 收到 PMAC 返回的数据时的自动回调
// 收到 PMAC 返回的数据时的自动回调
void stil_drive::onPmacReadyRead()
{
    // readAll() 会把缓冲区里收到的所有字节读出来
    QByteArray data = pmacSocket->readAll();

    // 转换为 QString，并使用 trimmed() 去掉末尾多余的回车换行符
    QString msg = QString::fromUtf8(data).trimmed();

    // Power PMAC 执行成功但不返回数据时，会发送一个 ACK 字符 (ASCII 0x06)
    // 为了不显示乱码，我们将其转换为易读的 "[指令执行成功]"
    if (msg == "\x06") {
        ui.textBrowser_pmac_log->append("<font color='blue'>[指令执行成功 (ACK)]</font>");
    }
    else {
        // 如果不是 ACK，就直接打印收到的原始内容（比如报错或者坐标值）
        ui.textBrowser_pmac_log->append("收到反馈: " + msg);
    }
}

