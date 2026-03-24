#include "stil_drive.h"
#include <QMessageBox>
#include <QDebug>
#include <QDir>
#include <QDateTime>
#include "qcustomplot.h" 
#include <QPen>

// 构造函数：stil_drive 类的初始化，继承自 QMainWindow
stil_drive::stil_drive(QWidget* parent)
    : QMainWindow(parent) // 调用父类 QMainWindow 的构造函数
{
    // 将在 Qt Designer (UI设计器) 中画好的界面元素实例化并绑定到当前窗口
    ui.setupUi(this);

    // ==========================================
    // 初始化频率下拉框 (QComboBox)
    // ==========================================
    ui.cmb_Freq->addItem("100 Hz");   // 添加选项，对应的索引(Index)为 0
    ui.cmb_Freq->addItem("200 Hz");   // Index 1
    ui.cmb_Freq->addItem("400 Hz");   // Index 2
    ui.cmb_Freq->addItem("1000 Hz");  // Index 3
    ui.cmb_Freq->addItem("2000 Hz");  // Index 4

    // 设置下拉框默认显示的选项，索引 3 对应的是 "1000 Hz"
    ui.cmb_Freq->setCurrentIndex(3);

    // ==========================================
    // 初始化按钮状态 (QPushButton)
    // ==========================================
    // 软件刚启动时还没有建立连接，为了防止用户误触，将这些按钮设置为“禁用”状态（变灰）
    ui.btn_Disconnect->setEnabled(false); // 禁用“断开连接”按钮
    ui.btn_Start->setEnabled(false);      // 禁用“开始”按钮
    ui.btn_Stop->setEnabled(false);       // 禁用“停止”按钮

    // ==========================================
    // 图表初始化配置 (QCustomPlot)
    // ==========================================
    // 在图表控件中添加一条新的曲线（图层）
    ui.plot_Widget->addGraph();

    // 获取刚刚添加的第 0 条曲线，并设置它的画笔颜色为蓝色（用于绘制线条）
    ui.plot_Widget->graph(0)->setPen(QPen(Qt::blue));

    // 设置 X 轴的标签（名称）
    ui.plot_Widget->xAxis->setLabel("采样点数 (Points)");

    // 设置 Y 轴的标签（名称）
    ui.plot_Widget->yAxis->setLabel("高度 / Altitude (um)");

    // 开启图表的键鼠交互功能：允许鼠标拖拽平移图表 (iRangeDrag) 和 允许鼠标滚轮缩放图表 (iRangeZoom)
    ui.plot_Widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

// 析构函数：在 stil_drive 窗口对象被销毁时自动调用
// 注意函数名前面的波浪号 (~) 代表这是析构函数
stil_drive::~stil_drive()
{
    // ==========================================
    // 1. 线程安全退出处理
    // ==========================================
    // 防御性编程：检查线程指针是否为空，防止解引用空指针导致程序崩溃 (Segfault)
    if (m_thread != nullptr) {
        // 调用自定义方法，通知工作线程停止采集数据（通常是将线程内部的循环标志位设为 false）
        m_thread->stopAcquisition();

        // 阻塞当前主线程，等待工作线程彻底执行完毕并安全退出后，再继续往下执行
        m_thread->wait();
    }

    // ==========================================
    // 2. 硬件/传感器设备断开与资源释放
    // ==========================================
    // m_sensorID 可能是传感器的句柄/ID，如果不为 0，说明之前成功连接了设备
    if (m_sensorID != 0) {
        MCHR_CloseChr(m_sensorID);  // 调用厂商提供的 SDK 函数，断开与该传感器的连接
    }
    // 调用厂商提供的 SDK 函数，释放整个硬件库占用的系统资源
    MCHR_Release();

    // ==========================================
    // 3. 文件安全关闭处理
    // ==========================================
    // 检查用于记录数据的 CSV 文件 (可能是 QFile 对象) 是否处于打开状态
    if (m_csvFile.isOpen()) {
        m_csvFile.close(); // 关闭文件，确保还在内存缓冲区的数据被完整写入磁盘，防止数据丢失
    }
}

// 这是一个 Qt 的“槽函数”(Slot)，当 UI 上的 btn_Connect（连接按钮）被点击时，会自动执行这个函数
void stil_drive::on_btn_Connect_clicked()
{
    // 1. 【防抖/防呆】如果已经连接了（m_sensorID 不为 0），直接退出，防止重复连接
    if (m_sensorID != 0) return;

    // 2. 【状态更新】点击后立刻禁用连接按钮，并修改文字，防止用户在连接过程中狂点导致程序卡死
    ui.btn_Connect->setEnabled(false);
    ui.btn_Connect->setText("连接中...");

    // 3. 【终极防呆保险】：确保 DLL (动态链接库) 绝对被初始化了！
    // static (静态局部变量)：这个变量只会在第一次点击时被创建并设为 false。
    // 以后就算退出这个函数再次进来，它依然会保持上一次的值，不会重新变为 false。
    static bool isDllInitialized = false;

    if (!isDllInitialized) {
        // MCHR_Init() 是厂商提供的 SDK 函数，用于初始化整个库环境
        if (!MCHR_Init()) {
            // 如果初始化失败，弹出一个红色的“错误”对话框
            QMessageBox::critical(this, "错误", "DLL 初始化失败！请检查USB连线或重启测头。");

            // 恢复按钮状态，让用户可以重试
            ui.btn_Connect->setText("连接测头");
            ui.btn_Connect->setEnabled(true);
            return; // 结束当前函数，后面的代码不再执行
        }
        isDllInitialized = true; // 标记为已初始化，下次再点连接就不会再走一遍初始化了
    }

    // 4. 【分配内存】准备扫描设备所需的字符串数组空间
    // 创建一个指针数组，最多能存 MCHR_MAX_SENSOR 个设备名字
    char* UsbCCSDeviceList[MCHR_MAX_SENSOR];
    for (int i = 0; i < MCHR_MAX_SENSOR; i++) {
        // C++ 的动态内存分配：在“堆区”为每个设备名字分配最大长度 (MAX_PATH) 的内存
        UsbCCSDeviceList[i] = new char[MAX_PATH];

        // memset：将刚刚分配的内存全部清零，防止里面有乱码（脏数据）
        memset(UsbCCSDeviceList[i], 0, MAX_PATH);
    }

    short deviceNumber = 0; // 用来接收找到了几个设备

    // 5. 【扫描设备】调用 SDK 扫描 USB 列表，结果存入 UsbCCSDeviceList，数量存入 deviceNumber
    if ((MCHR_GetUsbDeviceList(UsbCCSDeviceList, &deviceNumber) != MCHR_ERROR) && (deviceNumber > 0)) {

        // 6. 【打开设备】如果找到了设备（数量 > 0），默认尝试打开第 1 个（索引为 0）
        m_sensorID = MCHR_OpenUsbChr(UsbCCSDeviceList[0], MCHR_CCS_PRIMA, UsbCCSDeviceList[0], NULL, NULL);

        if (m_sensorID != 0) { // 连接成功，拿到了有效的句柄/ID
            ui.btn_Connect->setText("已连接");

            // 解除我们在构造函数里禁用的按钮，现在可以“断开”和“开始”了
            ui.btn_Disconnect->setEnabled(true);
            ui.btn_Start->setEnabled(true);

            // 弹出提示框，QString("%1").arg(...) 是一种格式化字符串的方法，把设备名填到 %1 的位置
            QMessageBox::information(this, "成功", QString("成功连接到测头！\n设备名: %1").arg(UsbCCSDeviceList[0]));
        }
        else { // 虽然扫描到了，但是打开失败（可能被别的软件占用了）
            ui.btn_Connect->setText("连接测头");
            ui.btn_Connect->setEnabled(true);
            QMessageBox::critical(this, "失败", "找到设备但连接失败！");
        }
    }
    else { // 根本没有扫描到任何设备
        ui.btn_Connect->setText("连接测头");
        ui.btn_Connect->setEnabled(true);
        QMessageBox::warning(this, "警告", "未扫描到测头！请拔插USB线重试。");
    }

    // 7. 【释放内存】（极其重要！！！）
    // 刚才用 new 借来的内存，用完必须用 delete[] 还回去，否则会导致“内存泄漏”！
    for (int i = 0; i < MCHR_MAX_SENSOR; i++) {
        delete[] UsbCCSDeviceList[i];
    }
}

// ---- 开始采集按钮 ----
void stil_drive::on_btn_Start_clicked()
{
    // 1. 如果没有连接测头，直接弹窗警告并退出
    if (m_sensorID == 0) {
        QMessageBox::warning(this, "警告", "请先连接测头！");
        return;
    }

    // 2. 【防重入拦截】如果后台采集线程已经存在，并且正在运行中，则什么也不做直接退出
    // 防止用户狂点“开始”按钮导致创建无数个线程把电脑卡死
    if (m_thread != nullptr && m_thread->isRunning()) return;

    // ==========================================
    // 3. 根据下拉框的选择，下发采样频率配置给硬件
    // ==========================================
    WORD rateCode = MCHR_SCAN_RATE_CCS_PRIMA_1000HZ; // 先设定一个默认的频率参数宏

    // switch 语句：根据下拉框当前的索引 (0~4)，切换对应的硬件命令代码
    switch (ui.cmb_Freq->currentIndex()) {
    case 0: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_100HZ; break;
    case 1: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_200HZ; break;
    case 2: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_400HZ; break;
    case 3: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_1000HZ; break;
    case 4: rateCode = MCHR_SCAN_RATE_CCS_PRIMA_2000HZ; break;
    }

    // 下发命令到测头 (调用硬件 SDK 函数)
    if (MCHR_SetScanRate(m_sensorID, rateCode) == MCHR_ERROR) {
        // 光谱共焦测头特性：如果频率太高而物体表面反光太弱（太暗），测头硬件会拒绝高频
        // 这里只是给个警告，但不 return 打断程序，让它使用上一次的默认安全频率继续运行
        QMessageBox::warning(this, "频率设置警告", "测头拒绝了该频率！可能是信号太弱(Dark限制)。将使用默认安全频率。");
    }

    // ==========================================
    // 4. 清空上一次采集的历史图表数据
    // ==========================================
    m_totalPoints = 0;                           // 内部计数的采样点数清零
    ui.plot_Widget->graph(0)->data()->clear();   // 把图表第 0 条曲线里的画线数据清空

    // （注：原代码这里不小心重复写了一遍上面两行，我已经帮你精简掉了，保留一遍即可）

    // ==========================================
    // 5. 数据保存（如果勾选了“保存数据”复选框）
    // ==========================================
    if (ui.chk_SaveData->isChecked()) {
        // 自动生成带时间戳的文件名：比如 "20260324_101930_SensorData.csv"
        QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + "_SensorData.csv";
        m_csvFile.setFileName(fileName); // 绑定文件名到 m_csvFile 文件对象

        // 尝试以“只写(WriteOnly)”和“文本格式(Text)”打开文件
        if (m_csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            // 将文本数据流对象 (m_csvStream) 绑定到这个文件上
            m_csvStream.setDevice(&m_csvFile);

            // 写入 CSV 表头（即第一行的列名），\n 代表换行
            m_csvStream << "Point_Index,Altitude(um),Intensity(%)\n";
        }
        else {
            QMessageBox::warning(this, "警告", "无法创建数据文件！"); // 可能没有文件夹写入权限
        }
    }

    // ==========================================
    // 6. 启动后台采集线程 (核心！)
    // ==========================================
    // 创建一个新线程专门用来接收硬件数据（因为接数据是死循环，如果在主程序做会导致界面卡死）
    m_thread = new SensorThread(m_sensorID, this);

    // 【跨线程通信魔法】：将子线程发出的信号，连接到主界面的接收函数（槽函数）上
    // 只要子线程采集到一波数据，发出 dataReady 信号，主界面就会执行 handleDataReady 来画图
    connect(m_thread, &SensorThread::dataReady, this, &stil_drive::handleDataReady);

    // 如果子线程里硬件报错了，发出 errorOccurred 信号，主界面执行 handleError 弹窗报错
    connect(m_thread, &SensorThread::errorOccurred, this, &stil_drive::handleError);

    // 正式启动子线程（这会触发 SensorThread 内部的 run() 函数）
    m_thread->start();

    // ==========================================
    // 7. 更新界面状态
    // ==========================================
    ui.lbl_Status->setText("状态：正在采集..."); // 更新左下角/状态栏的文字
    ui.btn_Start->setEnabled(false);           // 开始后，禁用“开始”按钮
    ui.btn_Stop->setEnabled(true);             // 开始后，启用“停止”按钮
}

// ---- 停止采集按钮 ----
// 这是一个 Qt 的槽函数，当 UI 上的 btn_Stop（停止按钮）被点击时自动执行
void stil_drive::on_btn_Stop_clicked()
{
    // ==========================================
    // 1. 安全结束并销毁后台采集线程
    // ==========================================
    // 【防呆检查】判断当前是否有正在运行的线程对象 (m_thread 不是空指针)
    if (m_thread != nullptr) {

        // 调用我们自定义的函数，通知线程内部的循环停止（比如把 isRunning 标志位设为 false）
        // 这样线程就不会再继续死循环去读硬件数据了
        m_thread->stopAcquisition();

        // 【极其重要】阻塞主线程，乖乖等待子线程把手头最后一次数据处理完，并彻底安全退出
        // 如果不等待，下面直接 delete 线程可能会导致程序崩溃 (Segfault)
        m_thread->wait();

        // 【Qt 专属内存管理】告诉 Qt 框架：“这个线程我不用了，等主线程空闲的时候，帮我把它从内存里删掉吧”
        // 不要直接用 delete m_thread; 这种粗暴的方式，deleteLater() 更加安全
        m_thread->deleteLater();

        // 既然线程被销毁了，必须把指针重新指向 nullptr (空指针)，
        // 防止以后误操作这个已经变成“野指针”的地址，造成二次崩溃
        m_thread = nullptr;

        // 更新界面左下角的状态栏文字
        ui.lbl_Status->setText("状态：已停止");
    }

    // ==========================================
    // 2. 安全关闭数据保存文件
    // ==========================================
    // 如果之前勾选了“保存数据”，文件就是打开状态 (isOpen() 为 true)
    if (m_csvFile.isOpen()) {
        // 调用 close() 关闭文件
        // 这一步非常关键：它会把内存中还没来得及写进硬盘的缓冲数据“刷(flush)”进磁盘，确保数据完整性
        m_csvFile.close();
    }

    // ==========================================
    // 3. 恢复界面按钮的初始状态
    // ==========================================
    ui.btn_Start->setEnabled(true);   // 既然停下来了，就可以重新允许点击“开始”
    ui.btn_Stop->setEnabled(false);   // 已经停下来了，就不需要再点“停止”了，将其变灰禁用
}

// ---- 接收数据的槽函数 (核心画图与保存逻辑) ----
// 这个函数在 btn_Start 中被 connect 绑定。子线程每采集到一批数据，就会自动触发它。
// 参数 alts (Altitudes) 存的是高度数组，ints (Intensities) 存的是光强数组
void stil_drive::handleDataReady(QVector<double> alts, QVector<double> ints)
{
    // 1. 获取这批传过来的数据量大小
    int dataSize = alts.size();

    // 【防呆】如果传过来的是空数组，直接退出，防止后面报错
    if (dataSize == 0) return;

    // 2. 准备 X 轴的数据
    // 因为测头传过来的只有 Y 轴（高度）和光强，我们要自己给它配上 X 轴（第几个采样点）
    // 提前分配好 dataSize 大小的内存，这样比一个一个追加 (append) 速度快很多
    QVector<double> xData(dataSize);

    // 3. 遍历处理这批新数据
    for (int i = 0; i < dataSize; i++) {
        // 计算当前点的 X 坐标：历史总点数 + 当前批次里的第 i 个
        xData[i] = m_totalPoints + i;

        // 如果用户勾选了“保存数据”（文件处于打开状态）
        if (m_csvFile.isOpen()) {
            // 将 序号、高度、光强 按照逗号分隔的 CSV 格式写入文件，并加上换行符 \n
            // QTextStream 内部有缓冲区，这样频繁使用 << 写入也是相对高效的
            m_csvStream << (m_totalPoints + i) << "," << alts[i] << "," << ints[i] << "\n";
        }
    }

    // ==========================================
    // 4. QCustomPlot 动态画图核心逻辑
    // ==========================================
    // 将刚刚算好的 X 轴数组和传过来的 Y 轴数组 (高度)，追加到第 0 条曲线的尾部
    // 注意：这里用的是 addData (追加)，而不是 setData (全部替换)
    ui.plot_Widget->graph(0)->addData(xData, alts);

    // 更新历史总点数记录（加上刚才处理的这批数据量）
    m_totalPoints += dataSize;

    // 【实现波形滚动的魔法】
    // 设定 X 轴的显示范围。比如永远只显示最新的 1000 个点，多出来的旧点会被挤到左边屏幕外
    // Qt::AlignRight 表示向右对齐，配合 m_totalPoints 实现波形从右向左“流动”的视觉效果
    ui.plot_Widget->xAxis->setRange(m_totalPoints - 1000, m_totalPoints, Qt::AlignRight);

    // 自动缩放 Y 轴：根据当前屏幕内能看到的波形最高点和最低点，自动拉伸 Y 轴的高度
    // 传入 true 表示允许它仅仅根据当前可见的范围去计算缩放比例
    ui.plot_Widget->graph(0)->rescaleValueAxis(true);

    // 【终极触发】前面只是设置了数据和坐标轴参数，调用 replot() 才会真正让显卡去把图画出来！
    ui.plot_Widget->replot();

    // ==========================================
    // 5. 更新状态栏 UI
    // ==========================================
    // 使用 QString::arg() 拼接出动态的文本。
    // alts.last() 是 QVector 的便捷函数，直接获取数组里的最后一个元素（也就是最新采集到的那个高度）
    ui.lbl_Status->setText(QString("状态：采集中 | 已采点数: %1 | 最新高度: %2 um")
        .arg(m_totalPoints)
        .arg(alts.last()));
}

// ==========================================
// ---- 接收底层错误的槽函数 ----
// ==========================================
// 这个函数在 btn_Start 中被 connect 绑定。当子线程(SensorThread)在后台读取硬件数据时
// 比如USB线突然被拔了，或者硬件死机了，子线程就会发出 errorOccurred 信号，触发这个函数
void stil_drive::handleError(QString msg)
{
    // 1. 弹出一个带有红叉图标的严重错误警告框，把子线程传过来的具体错误信息 (msg) 显示给用户看
    QMessageBox::critical(this, "底层错误", msg);
    
    // 2. 【核心复用】既然出错了，采集肯定进行不下去了。
    // 我们不需要在这里重新写一遍“停止线程、关闭文件”的代码，
    // 直接调用“停止按钮”被点击时的处理函数，相当于在代码里“帮用户点了一下停止按钮”！
    on_btn_Stop_clicked();
}

// ==========================================
// ---- 断开连接按钮 ----
// ==========================================
void stil_drive::on_btn_Disconnect_clicked()
{
    // 1. 【防呆拦截】如果压根就没有连接硬件 (句柄为 0)，直接退出，什么也不做
    if (m_sensorID == 0) return;

    // 2. 如果当前正在疯狂采集中，用户突然点了断开连接怎么办？
    // 为了防止底层崩溃，必须先安全地把采集线程停下来、文件关掉。
    // 同样，直接调用“停止按钮”的函数来实现逻辑复用
    on_btn_Stop_clicked();

    // 3. 调用硬件厂商提供的 API，正式向设备发送断开连接的指令
    MCHR_CloseChr(m_sensorID);
    
    // 4. 【极其重要】状态重置！
    // 既然断开了，之前拿到的“设备身份证号(句柄)”就失效了，必须立刻清零。
    // 否则以后代码里的 if (m_sensorID != 0) 就会误以为设备还连着，导致崩溃。
    m_sensorID = 0; 

    // 5. 恢复界面上所有按钮的初始状态
    ui.btn_Connect->setEnabled(true);     // 允许重新点击“连接”
    ui.btn_Connect->setText("连接测头");    // 把文字从“已连接”改回“连接测头”
    ui.btn_Disconnect->setEnabled(false); // 已经断开了，禁用“断开”按钮
    ui.btn_Start->setEnabled(false);      // 断开了，肯定不能点“开始”了

    // 6. 更新左下角状态栏的文字
    ui.lbl_Status->setText("状态：已断开连接");
    
    // 7. 弹窗告诉用户已经安全断开了
    QMessageBox::information(this, "断开", "测头已安全断开！");
}