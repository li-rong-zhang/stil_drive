#include "stil_drive.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QDateTime>
#include <QPen>
#include <QSettings>
#include <QActionGroup>
#include <QDockWidget>
#include <QTabWidget>
#include <QTextEdit>
#include <QTextBrowser>
#include <QLabel>
#include <QStatusBar>
#include <QInputDialog>
#include <QRegularExpression>

#include "qcustomplot.h"

#include "AsphericMath.h"
#include "DataReducer.h"
#include "PmacScriptGen.h"
#include "SurfaceAnalyzer.h"

#include "TrajectoryPlannerDock.h"
#include "LinearParamPage.h"
#include "SpiralParamPage.h"
#include "SphericalParamPage.h"
#include "AsphericParamPage.h"
#include "SensorConnectDialog.h"
#include "PmacConnectDialog.h"
#include "AxisMappingDialog.h"

// =========================================================================
// 构造与初始化
// =========================================================================
stil_drive::stil_drive(QWidget* parent) : QMainWindow(parent)
{
    ui.setupUi(this);

    // PMAC 控制器
    m_pmac = new PmacController(this);
    connect(m_pmac, &PmacController::connected,    this, [=]() { onPmacConnectStatusChanged(true); });
    connect(m_pmac, &PmacController::disconnected, this, [=]() { onPmacConnectStatusChanged(false); });
    connect(m_pmac, &PmacController::progFinished, this, &stil_drive::onStopTriggered);

    // PROG 运行状态检测定时器
    m_progMonitorTimer = new QTimer(this);
    connect(m_progMonitorTimer, &QTimer::timeout, this, &stil_drive::checkProgStatus);
    m_progMonitorTimer->setInterval(500);  // 每 500ms 检查一次

    // 图表
    ui.plot_Widget->addGraph();
    ui.plot_Widget->graph(0)->setPen(QPen(Qt::blue));
    ui.plot_Widget->xAxis->setLabel("采样点数 / 位置");
    ui.plot_Widget->yAxis->setLabel("高度 / Altitude (um)");
    ui.plot_Widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    // 主控按钮
    ui.btn_Start->setEnabled(false);
    ui.btn_Stop->setEnabled(false);
    connect(ui.btn_Start, &QPushButton::clicked, this, &stil_drive::onStartTriggered);
    connect(ui.btn_Stop,  &QPushButton::clicked, this, &stil_drive::onStopTriggered);

    // 子组件
    setupTrajectoryDock();
    setupBottomDock();
    setupStatusBar();
    wireMenuActions();

    // 连接日志输出到底部 dock
    connect(m_pmac, &PmacController::logMessage, m_textLog, &QTextBrowser::append);

    // 加载 QSettings
    QSettings settings("stil_drive", "trajectory");
    m_pmacIp   = settings.value("pmac/ip",   m_pmacIp).toString();
    m_pmacPort = settings.value("pmac/port", m_pmacPort).toUInt();
    m_freqIndex = settings.value("sensor/freqIndex", m_freqIndex).toInt();
    m_axisCfg.load(settings);
    PmacScriptGen::setAxisConfig(m_axisCfg);
    for (auto* p : m_planner->pages()) p->loadSettings(settings);

    // 默认模式：直线
    switchMode(0);
}

stil_drive::~stil_drive()
{
    QSettings settings("stil_drive", "trajectory");
    settings.setValue("pmac/ip",   m_pmacIp);
    settings.setValue("pmac/port", m_pmacPort);
    settings.setValue("sensor/freqIndex", m_freqIndex);
    m_axisCfg.save(settings);
    if (m_planner) {
        for (auto* p : m_planner->pages()) p->saveSettings(settings);
    }

    if (m_thread != nullptr) {
        m_thread->stopAcquisition();
        m_thread->wait();
    }
    if (m_sensorID != 0) MCHR_CloseChr(m_sensorID);
    MCHR_Release();

    if (m_csvFile.isOpen()) m_csvFile.close();
}

// =========================================================================
// 子组件初始化
// =========================================================================
void stil_drive::setupTrajectoryDock()
{
    m_planner = new TrajectoryPlannerDock(this);

    m_pageLinear    = new LinearParamPage();
    m_pageSpiral    = new SpiralParamPage();
    m_pageSpherical = new SphericalParamPage();
    m_pageAspheric  = new AsphericParamPage();

    m_planner->addPage(m_pageLinear);
    m_planner->addPage(m_pageSpiral);
    m_planner->addPage(m_pageSpherical);
    m_planner->addPage(m_pageAspheric);

    addDockWidget(Qt::LeftDockWidgetArea, m_planner);

    connect(m_planner, &TrajectoryPlannerDock::validateRequested, this, &stil_drive::onValidateRequested);
    connect(m_planner, &TrajectoryPlannerDock::generateRequested, this, &stil_drive::onGenerateRequested);
    connect(m_planner, &TrajectoryPlannerDock::startRequested,    this, &stil_drive::onStartRequested);

    // 任意 page 参数变化 → 自动刷新脚本预览
    for (auto* p : m_planner->pages())
        connect(p, &ParamPageBase::paramChanged, this, &stil_drive::refreshScriptPreview);
}

void stil_drive::setupBottomDock()
{
    m_dockBottom = new QDockWidget(QStringLiteral("脚本 / 日志"), this);
    m_dockBottom->setObjectName("dock_Bottom");
    m_dockBottom->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);

    m_tabBottom = new QTabWidget(m_dockBottom);
    m_textScript = new QTextEdit(m_tabBottom);
    m_textScript->setPlaceholderText(QStringLiteral("PMAC 脚本预览 / 编辑（开始测量时下发）"));
    m_textLog = new QTextBrowser(m_tabBottom);

    m_tabBottom->addTab(m_textScript, QStringLiteral("脚本预览"));
    m_tabBottom->addTab(m_textLog,    QStringLiteral("PMAC 日志"));

    m_dockBottom->setWidget(m_tabBottom);
    addDockWidget(Qt::BottomDockWidgetArea, m_dockBottom);
}

void stil_drive::setupStatusBar()
{
    m_lblSensorLed = new QLabel(QStringLiteral("● 测头：未连接"), this);
    m_lblPmacLed   = new QLabel(QStringLiteral("● PMAC：未连接"), this);
    m_lblMode      = new QLabel(QStringLiteral("模式：—"), this);
    m_lblStatus    = new QLabel(QStringLiteral("就绪"), this);
    m_lblSensorLed->setStyleSheet("color:#888;");
    m_lblPmacLed->setStyleSheet("color:#888;");

    statusBar()->addWidget(m_lblSensorLed);
    statusBar()->addWidget(m_lblPmacLed);
    statusBar()->addWidget(m_lblMode);
    statusBar()->addPermanentWidget(m_lblStatus, 1);
}

void stil_drive::wireMenuActions()
{
    // 文件
    connect(ui.act_Exit, &QAction::triggered, this, &stil_drive::close);

    // 设备
    connect(ui.act_ConnectSensor,    &QAction::triggered, this, &stil_drive::onConnectSensor);
    connect(ui.act_DisconnectSensor, &QAction::triggered, this, &stil_drive::onDisconnectSensor);
    connect(ui.act_ConnectPmac,      &QAction::triggered, this, &stil_drive::onConnectPmac);
    connect(ui.act_DisconnectPmac,   &QAction::triggered, this, &stil_drive::onDisconnectPmac);
    connect(ui.act_SendCmd,          &QAction::triggered, this, &stil_drive::onSendManualCommand);
    connect(ui.act_AxisMapping,     &QAction::triggered, this, &stil_drive::onAxisMapping);

    // 模式（互斥）
    m_modeGroup = new QActionGroup(this);
    m_modeGroup->setExclusive(true);
    m_modeGroup->addAction(ui.act_ModeLinear);
    m_modeGroup->addAction(ui.act_ModeSpiral);
    m_modeGroup->addAction(ui.act_ModeSpherical);
    m_modeGroup->addAction(ui.act_ModeAspheric);
    ui.act_ModeLinear->setChecked(true);

    connect(ui.act_ModeLinear,    &QAction::triggered, this, &stil_drive::onModeLinear);
    connect(ui.act_ModeSpiral,    &QAction::triggered, this, &stil_drive::onModeSpiral);
    connect(ui.act_ModeSpherical, &QAction::triggered, this, &stil_drive::onModeSpherical);
    connect(ui.act_ModeAspheric,  &QAction::triggered, this, &stil_drive::onModeAspheric);

    // 视图
    connect(ui.act_ShowPlanner, &QAction::toggled, m_planner,    &QDockWidget::setVisible);
    connect(ui.act_ShowScript,  &QAction::toggled, this, [this](bool v) {
        m_dockBottom->setVisible(v || ui.act_ShowLog->isChecked());
        m_tabBottom->setTabVisible(0, v);
    });
    connect(ui.act_ShowLog,     &QAction::toggled, this, [this](bool v) {
        m_dockBottom->setVisible(v || ui.act_ShowScript->isChecked());
        m_tabBottom->setTabVisible(1, v);
    });

    // 工具
    connect(ui.act_GenerateScript, &QAction::triggered, this, &stil_drive::onGenerateScript);
    connect(ui.act_EmergencyStop,  &QAction::triggered, this, &stil_drive::onEmergencyStop);
    connect(ui.act_Analyze,        &QAction::triggered, this, &stil_drive::onAnalyze);

    // 帮助
    connect(ui.act_About, &QAction::triggered, this, &stil_drive::onAbout);
}

// =========================================================================
// 模式切换
// =========================================================================
void stil_drive::switchMode(int modeId)
{
    m_planner->setMode(modeId);
    auto* p = m_planner->currentPage();
    if (p) m_lblMode->setText(QStringLiteral("模式：%1").arg(p->modeName()));
    // 切到球面纬线模式时重置纬度指针，从首条纬线开始
    if (modeId == 3 && m_pageSpherical) m_pageSpherical->resetIndex();
    refreshScriptPreview();
}

void stil_drive::onModeLinear()    { switchMode(0); }
void stil_drive::onModeSpiral()    { switchMode(2); }
void stil_drive::onModeSpherical() { switchMode(3); }
void stil_drive::onModeAspheric()  { switchMode(4); }

void stil_drive::refreshScriptPreview()
{
    auto* p = m_planner ? m_planner->currentPage() : nullptr;
    if (!p) return;
    m_textScript->setPlainText(p->buildScript());
}

// =========================================================================
// 测头连接（菜单）
// =========================================================================
void stil_drive::onConnectSensor()
{
    if (m_sensorID != 0) {
        QMessageBox::information(this, "提示", "测头已连接。如需更换设备，请先断开。");
        return;
    }

    static bool isDllInitialized = false;
    if (!isDllInitialized) {
        if (!MCHR_Init()) {
            QMessageBox::critical(this, "错误", "DLL 初始化失败！请检查USB连线或重启测头。");
            return;
        }
        isDllInitialized = true;
    }

    char* deviceList[MCHR_MAX_SENSOR];
    for (int i = 0; i < MCHR_MAX_SENSOR; i++) {
        deviceList[i] = new char[MAX_PATH];
        memset(deviceList[i], 0, MAX_PATH);
    }

    QStringList qList;
    short deviceNumber = 0;
    if ((MCHR_GetUsbDeviceList(deviceList, &deviceNumber) != MCHR_ERROR) && deviceNumber > 0) {
        for (int i = 0; i < deviceNumber; ++i) qList << QString::fromLocal8Bit(deviceList[i]);
    }

    SensorConnectDialog dlg(qList, m_freqIndex, this);
    if (dlg.exec() == QDialog::Accepted) {
        int idx = dlg.selectedDeviceIndex();
        m_freqIndex = dlg.frequencyIndex();
        if (idx >= 0 && idx < deviceNumber) {
            m_sensorID = MCHR_OpenUsbChr(deviceList[idx], MCHR_CCS_PRIMA, deviceList[idx], NULL, NULL);
            if (m_sensorID != 0) {
                m_lblSensorLed->setText(QStringLiteral("● 测头：%1").arg(dlg.frequencyLabel()));
                m_lblSensorLed->setStyleSheet("color:#2a7;");
                ui.act_ConnectSensor->setEnabled(false);
                ui.act_DisconnectSensor->setEnabled(true);
                ui.btn_Start->setEnabled(true);
            } else {
                QMessageBox::critical(this, "失败", "找到设备但连接失败！");
            }
        }
    }

    for (int i = 0; i < MCHR_MAX_SENSOR; i++) delete[] deviceList[i];
}

void stil_drive::onDisconnectSensor()
{
    if (m_sensorID == 0) return;
    onStopTriggered();
    MCHR_CloseChr(m_sensorID);
    m_sensorID = 0;

    m_lblSensorLed->setText(QStringLiteral("● 测头：未连接"));
    m_lblSensorLed->setStyleSheet("color:#888;");
    ui.act_ConnectSensor->setEnabled(true);
    ui.act_DisconnectSensor->setEnabled(false);
    ui.btn_Start->setEnabled(false);
}

// =========================================================================
// PMAC 连接（菜单）
// =========================================================================
void stil_drive::onConnectPmac()
{
    if (m_pmac->isConnected()) {
        QMessageBox::information(this, "提示", "PMAC 已连接。");
        return;
    }
    PmacConnectDialog dlg(m_pmacIp, m_pmacPort, this);
    if (dlg.exec() == QDialog::Accepted) {
        m_pmacIp   = dlg.ip();
        m_pmacPort = dlg.port();
        m_pmac->connectToController(m_pmacIp, m_pmacPort);
    }
}

void stil_drive::onDisconnectPmac()
{
    if (m_pmac->isConnected()) m_pmac->disconnectController();
}

void stil_drive::onPmacConnectStatusChanged(bool connected)
{
    if (connected) {
        m_lblPmacLed->setText(QStringLiteral("● PMAC：%1").arg(m_pmacIp));
        m_lblPmacLed->setStyleSheet("color:#2a7;");
        ui.act_ConnectPmac->setEnabled(false);
        ui.act_DisconnectPmac->setEnabled(true);
        m_textLog->append("<font color='green'>成功连接到 Power PMAC!</font>");
    } else {
        m_lblPmacLed->setText(QStringLiteral("● PMAC：未连接"));
        m_lblPmacLed->setStyleSheet("color:#888;");
        ui.act_ConnectPmac->setEnabled(true);
        ui.act_DisconnectPmac->setEnabled(false);
        m_textLog->append("<font color='red'>已断开与 PMAC 的连接。</font>");
    }
}

void stil_drive::onSendManualCommand()
{
    if (!m_pmac->isConnected()) {
        QMessageBox::warning(this, "警告", "PMAC 未连接！");
        return;
    }
    QString cmd = m_textScript->toPlainText().trimmed();
    if (cmd.isEmpty()) return;
    if (!cmd.endsWith("\n")) cmd += "\n";

    m_pmac->sendCommand(cmd);
    QString logMsg = cmd;
    logMsg.replace("\n", " ");
    m_textLog->append("<font color='purple'>[手动测试]: " + logMsg + "</font>");
}

void stil_drive::onAxisMapping()
{
    AxisMappingDialog dlg(m_axisCfg, this);
    if (dlg.exec() == QDialog::Accepted) {
        m_axisCfg = dlg.result();
        PmacScriptGen::setAxisConfig(m_axisCfg);
        refreshScriptPreview();
        m_textLog->append(QStringLiteral("[轴映射] X→#%1 Y→#%2 Z→#%3 B→#%4 C→#%5 触发=Gate3[%6].Chan[%7]")
            .arg(m_axisCfg.motorX).arg(m_axisCfg.motorY)
            .arg(m_axisCfg.motorZ).arg(m_axisCfg.motorB).arg(m_axisCfg.motorC)
            .arg(m_axisCfg.triggerGate).arg(m_axisCfg.triggerChan));
    }
}

// =========================================================================
// 测量主流程
// =========================================================================
//
// 两个启动入口的语义区分：
//   onStartRequested  ←「发送脚本」(dock 底部，调试用)
//       ：validate(参数页) → 重新生成脚本写入预览 → 仅下发 PMAC（不连测头/不采集）
//   onStartTriggered  ←「开始采集」(主窗口下方，完整测量)
//       ：直接以预览编辑器中的当前脚本 → startAcquisition()
// =========================================================================
void stil_drive::onStartRequested()
{
    auto* page = m_planner ? m_planner->currentPage() : nullptr;
    if (!page) {
        QMessageBox::warning(this, "警告", "请先选择测量模式！");
        return;
    }
    QString err;
    if (!page->validate(err)) {
        QMessageBox::warning(this, "参数校验失败", err);
        return;
    }
    if (!m_pmac->isConnected()) {
        QMessageBox::warning(this, "警告", "请先连接 Power PMAC 控制器！");
        return;
    }

    QString cmd = m_textScript->toPlainText().trimmed();
    if (cmd.isEmpty()) {
        QMessageBox::warning(this, "警告", "脚本预览为空，请先选择模式或在工具菜单中生成脚本。");
        return;
    }

    m_textLog->append(QStringLiteral("正在下发脚本至 PMAC..."));
    if (!cmd.endsWith("\n")) cmd += "\n";
    m_pmac->sendCommand(cmd);
    m_textLog->append(QStringLiteral("脚本已发送。"));
}

void stil_drive::onStartTriggered()
{
    // 快速重跑：信任脚本预览中的当前内容，不再校验/重生成
    startAcquisition();
}

void stil_drive::startAcquisition()
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

    QString cmd = m_textScript->toPlainText().trimmed();
    if (cmd.isEmpty()) {
        QMessageBox::warning(this, "警告", "脚本预览为空，请先选择模式或在工具菜单中生成脚本。");
        return;
    }

    // 测头采样频率
    WORD rateCode = MCHR_SCAN_RATE_CCS_PRIMA_1000HZ;
    switch (m_freqIndex) {
    case 0: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_100HZ; break;
    case 1: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_200HZ; break;
    case 2: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_400HZ; break;
    case 3: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_1000HZ; break;
    case 4: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_2000HZ; break;
    }
    MCHR_SetScanRate(m_sensorID, rateCode);

    // 重置传感器状态，防止前一次采集未正确停止导致的问题
    MCHR_Abort(m_sensorID);
    Sleep(100);  // 等待传感器状态恢复

    m_totalPoints = 0;
    ui.plot_Widget->graph(0)->data()->clear();
    startCsvFile();

    m_thread = new SensorThread(m_sensorID, this);
    connect(m_thread, &SensorThread::dataReady, this, &stil_drive::handleDataReady);
    connect(m_thread, &SensorThread::errorOccurred, this, &stil_drive::handleError);
    m_thread->start();

    m_lblStatus->setText(QStringLiteral("等待硬件触发脉冲..."));
    ui.btn_Start->setEnabled(false);
    ui.btn_Stop->setEnabled(true);

    m_textLog->append(QStringLiteral("正在下发硬件脉冲同步指令..."));
    if (!cmd.endsWith("\n")) cmd += "\n";
    m_pmac->sendCommand(cmd);

    // 启动 PROG 运行状态检测定时器
    m_progMonitorTimer->start();
}

void stil_drive::onStopTriggered()
{
    // 停止 PROG 运行状态检测定时器
    m_progMonitorTimer->stop();

    if (m_thread != nullptr) {
        m_thread->stopAcquisition();
        m_thread->wait();
        m_thread->deleteLater();
        m_thread = nullptr;
    }
    stopCsvFile();

    ui.btn_Start->setEnabled(m_sensorID != 0);
    ui.btn_Stop->setEnabled(false);
    m_lblStatus->setText(QStringLiteral("已停止"));

    if (m_pmac->isConnected()) {
        m_pmac->sendCommand(PmacScriptGen::emergencyStop());
        m_textLog->append(QStringLiteral("<font color='red'>已安全制动全轴并撤销触发使能。</font>"));
    }

    // 球面多纬线调度：若还有下一条纬线，提示用户确认 X/Z 移动
    auto* page = m_planner ? m_planner->currentPage() : nullptr;
    if (page && page->modeId() == 3 && m_pageSpherical && m_pageSpherical->hasNext()) {
        promptNextLatitude();
    }
}

// ---- 检测 PROG 运行状态，程序完成时自动停止采集 ----
void stil_drive::checkProgStatus()
{
    if (!m_pmac->isConnected()) return;

    // 查询 Coord[1] 的程序运行状态
    // Power PMAC 中，Coord[1].ProgRunning 表示程序是否正在运行
    // 返回 1 表示正在运行，返回 0 表示程序完成
    m_pmac->sendCommand("? Coord[1].ProgRunning\n");
}

// ---- 球面：提示并下发下一纬线的 X/Z 定位脚本 ----
void stil_drive::promptNextLatitude()
{
    if (!m_pageSpherical || !m_pageSpherical->hasNext()) return;
    if (!m_pmac->isConnected()) return;

    double curLat  = m_pageSpherical->currentLatitude();
    double nextLat = m_pageSpherical->peekNextLatitude();
    QString posScript = m_pageSpherical->buildPositioningScriptToNext();

    QMessageBox box(this);
    box.setIcon(QMessageBox::Question);
    box.setWindowTitle(QStringLiteral("下一条纬线确认"));
    box.setText(QStringLiteral("当前纬度：φ = %1°\n下一纬度：φ = %2°\n\n"
                               "确认前请检查测头是否会与待测件碰撞。\n"
                               "查看脚本预览中的 ΔX / ΔZ 值。")
                .arg(curLat, 0, 'f', 2).arg(nextLat, 0, 'f', 2));
    box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    box.button(QMessageBox::Ok)->setText(QStringLiteral("确认移动"));
    box.button(QMessageBox::Cancel)->setText(QStringLiteral("暂不移动（保持当前位置）"));

    // 把定位脚本预先填到预览区便于核对
    m_textScript->setPlainText(posScript);
    m_tabBottom->setCurrentIndex(0);

    if (box.exec() == QMessageBox::Ok) {
        m_textLog->append(QStringLiteral("<font color='blue'>下发纬线定位脚本：%1° → %2°</font>")
                            .arg(curLat, 0, 'f', 2).arg(nextLat, 0, 'f', 2));
        m_pmac->sendCommand(posScript.endsWith("\n") ? posScript : posScript + "\n");
        m_pageSpherical->advance();
        // 把扫描脚本（新纬线）刷回预览，供下一次"开始测量"
        refreshScriptPreview();
    } else {
        m_textLog->append(QStringLiteral("<font color='gray'>用户取消纬线切换。</font>"));
    }
}

// =========================================================================
// 数据处理
// =========================================================================
void stil_drive::handleDataReady(QVector<double> alts, QVector<double> ints)
{
    int dataSize = alts.size();
    if (dataSize == 0) return;

    auto* page = m_planner->currentPage();
    if (!page) return;
    int modeId = page->modeId();
    ReductionResult r;

    if (modeId == 0) {
        r = DataReducer::reduceMode0(alts, ints, m_totalPoints, &m_csvStream);
    } else if (modeId == 2) {
        double rStartUm = m_pageSpiral->radiusMm() * 1000.0;
        double pitch    = m_pageSpiral->pitchUmPerRev();
        double density  = m_pageSpiral->densityDegPerPt();
        double overCenter = m_pageSpiral->overCenterMm();
        if (pitch <= 0) pitch = 10.0;
        if (density <= 0) density = 0.5;
        r = DataReducer::reduceMode2(alts, ints, m_totalPoints, rStartUm, pitch, density, overCenter, &m_csvStream);
    } else if (modeId == 3) {
        r = DataReducer::reduceMode3(alts, ints, m_totalPoints, &m_csvStream);
    } else if (modeId == 4) {
        r = DataReducer::reduceMode4(alts, ints, m_totalPoints,
            m_pageAspheric->startRadiusUm(), m_pageAspheric->scanRangeUm(),
            m_pageAspheric->R_mm(), m_pageAspheric->K(),
            m_pageAspheric->A4(), m_pageAspheric->A6(), m_pageAspheric->A8(),
            m_pageAspheric->A10(), m_pageAspheric->A12(),
            &m_csvStream);
    }

    ui.plot_Widget->graph(0)->addData(r.xData, alts);
    m_totalPoints += dataSize;
    ui.plot_Widget->xAxis->setRange(r.currentX - r.windowWidth, r.currentX, Qt::AlignRight);
    ui.plot_Widget->graph(0)->rescaleValueAxis(true);
    ui.plot_Widget->replot();

    m_lblStatus->setText(QStringLiteral("飞拍中 | 位置: %1 %2 | 高度: %3 um")
        .arg(r.currentX, 0, 'f', 2).arg(r.unit).arg(alts.last(), 0, 'f', 2));
}

void stil_drive::handleError(QString msg)
{
    QMessageBox::critical(this, "底层错误", msg);
    onStopTriggered();
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
    int modeId = m_planner->currentPage() ? m_planner->currentPage()->modeId() : 0;
    if (modeId == 4) {
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
// dock 信号 → 槽
// =========================================================================
void stil_drive::onValidateRequested()
{
    auto* p = m_planner->currentPage();
    if (!p) return;
    QString err;
    if (p->validate(err)) {
        QMessageBox::information(this, "校验通过", p->summary().remove(QRegularExpression("<[^>]*>")));
    } else {
        QMessageBox::warning(this, "参数校验失败", err);
    }
}

void stil_drive::onGenerateRequested()
{
    refreshScriptPreview();
    m_tabBottom->setCurrentIndex(0);
}

// =========================================================================
// 工具菜单
// =========================================================================
void stil_drive::onGenerateScript()
{
    refreshScriptPreview();
    m_tabBottom->setCurrentIndex(0);
}

void stil_drive::onEmergencyStop()
{
    onStopTriggered();
}

void stil_drive::onAnalyze()
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
        .arg(res.angleX, 0, 'f', 4).arg(res.angleY, 0, 'f', 4)
        .arg(res.pv, 0, 'f', 4).arg(res.rms, 0, 'f', 4);
    QMessageBox::information(this, "分析报告", report);
}

void stil_drive::onAbout()
{
    QMessageBox::about(this, QStringLiteral("关于"),
        QStringLiteral("stil_drive — STIL 测头 + Power PMAC 同步采集\n\n"
                       "支持直线 / 平面螺旋 / 球面纬线 / 非球面母线 四种轨迹规划。"));
}
