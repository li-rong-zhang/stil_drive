# stil_drive

基于 Qt 的超精密光学表面轮廓仪驱动与采集软件，集成 STIL CCS Prima 色散共焦传感器与 Delta Tau Power PMAC 多轴运动控制器，实现硬件同步"飞拍"扫描与表面面形分析。

---

## 目录

- [项目简介](#项目简介)
- [功能特性](#功能特性)
- [硬件依赖](#硬件依赖)
- [软件依赖](#软件依赖)
- [项目结构](#项目结构)
- [模块说明](#模块说明)
- [构建方法](#构建方法)
- [使用说明](#使用说明)
- [扫描模式详解](#扫描模式详解)
- [数据格式](#数据格式)
- [面形分析算法](#面形分析算法)
- [PMAC 通信协议](#pmac-通信协议)
- [关键参数说明](#关键参数说明)
- [已知限制](#已知限制)

---

## 项目简介

`stil_drive` 是一款运行于 Windows 平台的桌面应用程序，用于驱动 **STIL CCS Prima 色散共焦传感器** 对光学/机械精密表面（如透镜、反射镜）进行非接触式高精度轮廓测量。

系统核心亮点是**硬件位置同步采集**（"飞拍"技术）：
- **Delta Tau Power PMAC** 运动控制器在预设位置间隔处通过 Gate3 EQU 寄存器输出硬件触发脉冲
- **STIL 传感器** 每收到一个触发脉冲精确采集一个高度测量值
- 软件通过脉冲计数还原每个数据点的绝对空间坐标

这种方式消除了软件定时误差，确保数据点的空间位置精度与运动控制器编码器分辨率一致。

---

## 功能特性

| 功能 | 描述 |
|------|------|
| **菜单驱动 UI** | 设备连接、模式切换、视图、工具集中在菜单栏；主界面只显示图表与采集控制 |
| **轨迹规划 Dock** | 左侧停靠面板按测量模式切换参数页（QStackedWidget），4 种模式 4 个独立页 |
| **多种扫描模式** | 直线 / 平面螺旋 / 球面纬线 / 非球面母线 — 每种模式参数独立、UI 互不干扰 |
| **硬件同步触发** | 通过 TCP/Telnet 向 PMAC 发送 Gate3 EQU 触发脚本，实现硬件级位置同步 |
| **参数化脚本生成** | 所有 PmacScriptGen 函数接受触发密度、速度、方向等用户参数（不再硬编码） |
| **UI 级安全校验** | 频率防溢出、球面 90° 极点拦截、|纬度|<50° 赤道死区、非球面陡坡步距比对 |
| **参数持久化** | QSettings 自动保存/恢复每个模式的参数与设备配置 |
| **实时波形显示** | QCustomPlot 滚动显示高度-位置波形 |
| **CSV 数据记录** | 可选自动保存带时间戳文件名的 CSV 数据文件 |
| **面形分析** | 基于 Eigen 最小二乘平面拟合，计算 PV、RMS、倾斜角 |
| **PMAC 远程控制** | 内置 Telnet 自动登录与 gpascii 通道，支持手动发送运动指令 |
| **多采样率** | 支持 100 / 200 / 400 / 1000 / 2000 Hz 采样频率可选 |

---

## 硬件依赖

### STIL CCS Prima 色散共焦传感器
- 接口：USB
- 采样率：最高 2000 Hz
- 输出：表面高度（µm）+ 反射光强度（归一化）
- 驱动 DLL：`Dll_chr.dll`（需单独安装 STIL 官方驱动，默认路径 `D:\STIL\DLL\`）

### Delta Tau Power PMAC 运动控制器
- 接口：以太网（TCP/Telnet，默认 IP `192.168.0.200`，端口 `5002`）
- 默认登录：用户名 `root`，密码 `deltatau`
- 使用 Gate3 EQU 硬件比较器输出触发脉冲

---

## 软件依赖

| 依赖 | 版本 | 说明 |
|------|------|------|
| **Qt** | 6.10.2 (MSVC 2022 x64) | GUI 框架，含 core/gui/widgets/network/printsupport 模块 |
| **Visual Studio** | 2022 (v143 工具集) | 编译器与构建环境 |
| **Qt VS Tools** | QtVS_v304 | VS 中的 Qt 集成插件 |
| **STIL DLL SDK** | — | `Dll_chr.lib` + 头文件，需安装至 `D:\STIL\DLL\` |
| **Eigen3** | 3.x（仅头文件） | 已打包在 `stil_drive/Eigen/` 目录中 |
| **QCustomPlot** | 2.x | 已打包为 `qcustomplot.h/.cpp` |
| **Windows SDK** | — | 使用 `WaitForSingleObject`/`CreateEvent` 等 Win32 API |

---

## 项目结构

```
stil_drive/
├── stil_drive.sln                  # Visual Studio 解决方案文件
├── README.md                       # 本文件
├── .gitignore
│
└── stil_drive/                     # 主项目目录
    ├── main.cpp                    # 程序入口（创建 QApplication 和主窗口）
    │
    ├── stil_drive.h/.cpp/.ui       # 主窗口（菜单栏 + dock 容器 + 状态栏）
    ├── stil_drive.qrc              # Qt 资源文件
    ├── stil_drive.vcxproj(.filters)# MSVC 项目文件
    │
    ├── 【轨迹规划模块】
    ├── ParamPageBase.h             # 参数页抽象基类（接口：validate/buildScript/summary）
    ├── LinearParamPage.h/.cpp/.ui  # 直线扫描参数页
    ├── SpiralParamPage.h/.cpp/.ui  # 平面螺旋参数页
    ├── SphericalParamPage.h/.cpp/.ui  # 球面纬线参数页
    ├── AsphericParamPage.h/.cpp/.ui   # 非球面母线参数页
    ├── TrajectoryPlannerDock.h/.cpp   # 左侧停靠面板（持有 QStackedWidget）
    │
    ├── 【设备连接对话框】
    ├── SensorConnectDialog.h/.cpp  # STIL 测头连接 Dialog（设备列表 + 频率）
    ├── PmacConnectDialog.h/.cpp    # PMAC 连接 Dialog（IP / Port）
    │
    ├── 【底层模块】
    ├── AsphericMath.h/.cpp         # 非球面数学工具（纯算法，无 UI 依赖）
    ├── PmacController.h/.cpp       # PMAC TCP 通讯与 Telnet 自动登录
    ├── PmacScriptGen.h/.cpp        # 4 种扫描模式的 PMAC 脚本生成（参数化）
    ├── DataReducer.h/.cpp          # 脉冲序列 → 空间坐标还原
    ├── SurfaceAnalyzer.h/.cpp      # CSV 面形分析（Eigen 平面拟合）
    ├── SensorThread.h/.cpp         # STIL 传感器后台采集线程
    │
    ├── qcustomplot.h/.cpp          # QCustomPlot 绘图库（第三方）
    │
    └── Eigen/                      # Eigen3 线性代数库（仅头文件）

外部依赖（需单独安装，路径硬编码）：
D:\STIL\DLL\
    ├── Headers/  (Mchr.h, MchrDefine.h, MchrType.h, MchrError.h)
    └── Lib_x64/  (Dll_chr.lib)
```

---

## 模块说明

### 轨迹规划层

#### ParamPageBase（参数页抽象基类）

所有扫描模式参数页面继承自此类。主窗口通过 `QStackedWidget` 切换显示。

**接口**：
| 方法 | 用途 |
|------|------|
| `validate(errMsg)` | UI 级安全校验（频率上限、极点/死区、陡坡） |
| `buildScript()`    | 调用 `PmacScriptGen` 对应函数生成脚本字符串 |
| `summary()`        | 返回信息条文本（频率/时间/点数） |
| `modeId()`         | 返回 DataReducer 模式编号（0/2/3/4） |
| `modeName()`       | UI 显示用中文名 |
| `saveSettings/loadSettings(QSettings&)` | 参数持久化 |

**派生类**：`LinearParamPage`、`SpiralParamPage`、`SphericalParamPage`、`AsphericParamPage`，每个对应一个 `.ui` 文件。

#### TrajectoryPlannerDock

QDockWidget 子类，内嵌 QStackedWidget + 顶部模式标签 + 底部按钮组（`[校验] [生成脚本] [开始测量]`）。

**信号**：`validateRequested` / `generateRequested` / `startRequested` / `modeSwitched`，由主窗口 `stil_drive` 接收并执行。

---

### 设备通讯层

#### PmacController

封装 `QTcpSocket`，处理 Telnet 协议和自动登录流程。

**自动登录状态机**：
```
PMAC Telnet → "login:" → "root" → "Password:" → "deltatau"
          → "~#" → "gpascii -2" → "STDIN Open for ASCII" → 就绪
```

**关键方法**：`connectToController(ip, port)` / `disconnectController()` / `sendCommand(cmd)` / `isConnected()`
**Signal**：`connected()` / `disconnected()` / `logMessage(QString)`

#### SensorConnectDialog / PmacConnectDialog

模态对话框，从菜单 `[设备]→连接测头… / 连接 PMAC…` 触发：
- **SensorConnectDialog**：显示 USB 设备列表 + 采样频率下拉
- **PmacConnectDialog**：填写 IP / Port

主窗口在 `accept()` 后调用底层 SDK 完成实际连接。

---

### 运动指令生成层 — PmacScriptGen

根据扫描模式生成 Power PMAC ASCII 命令脚本，由 `PmacController` 下发。**所有函数已参数化**：

| 函数 | 模式 | 关键入参 |
|------|------|---------|
| `generateLinearScript()` | 直线 | 方向、长度、密度（µm/pt）、速度（mm/s） |
| `generateSpiralScript()` | 平面螺旋 | 半径、螺距、密度（°/pt）、角速度、过心补偿 |
| `generateSphericalLatScript()` | 球面纬线 | 球面 R、纬度起/终/步、密度（次/°）、C 转速、过扫角 |
| `generateAsphericMeridianScript()` | 非球面母线 | R/K/A4..A12、X 起始/范围、密度、进给速度、followZ |
| `emergencyStop()` | 急停 | — |

**Gate3 EQU 工作原理**：
- `CompAdd` 设置触发间隔（编码器计数）
- `CompA/B` 设置比较基准（`ServoCapt + 偏移`）
- `Equ1Ena=1` 使能硬件触发输出

**编码器单位换算**：

| 轴 | 坐标定义（&1 缩放） | Gate3 触发（CompAdd） | JogSpeed |
|---|---|---|---|
| X / Y / Z | 1 mm = 128,000,000 jog | 1 µm = 100,000 enc counts | 1 mm/s = 128,000 jog/ms |
| B | 1° = 5,898,240 jog | 1° = 409,600 enc counts | — |
| C | 1° = 1,638,400 jog | 1° = 409,600 enc counts | 1°/s = 1,638.4 jog/ms |

- **坐标定义**（`#N->...X/C`）：jog 单位，决定 `F` 值与用户单位的换算关系
- **Gate3 触发**（`CompAdd`）：编码器原始计数，决定触发脉冲的空间间隔
- **JogSpeed**（`Motor[].JogSpeed`）：jog/ms，仅用于纯 Jog 模式（球面纬线）

**各函数参数单位**：

| 函数 | 参数 | 单位 |
|------|------|------|
| `generateLinearScript()` | scanLengthMm | mm |
| | densityUmPerPt | µm/pt |
| | feedrateMmPerSec | mm/s |
| `generateSpiralScript()` | startRadiusMm | mm |
| | pitchUmPerRev | µm/rev |
| | densityDegPerPt | °/pt |
| | angularSpeedDegPerSec | °/s |
| | overCenterMm | mm |
| `generateSphericalLatScript()` | sphereRadiusMm | mm |
| | densityPerDeg | 次/° |
| | cSpeedDegPerSec | °/s |
| | overScanDeg | ° |
| `generateAsphericMeridianScript()` | startRadiusUm | µm |
| | scanRangeUm | µm |
| | R_mm | mm |
| | densityUmPerPt | µm/pt |
| | feedrateMmPerSec | mm/s |

---

### 数据处理层 — DataReducer

将"高度+强度"脉冲序列还原为空间坐标，实时写入 CSV。

| 函数 | 模式 | 坐标还原 |
|------|------|----------|
| `reduceMode0()` | 直线 | `X = n × density_µm, Y = 0` |
| `reduceMode2()` | 螺旋 | 极坐标 → 直角坐标，需起始半径与螺距 |
| `reduceMode3()` | 纬线 | `X=R·cos(θ), Y=R·sin(θ)` |
| `reduceMode4()` | 非球面母线 | `X=半径µm`，对比理论 Z 与实测 Z 偏差 |

主窗口 `handleDataReady()` 通过 `currentPage()->modeId()` 选择分支，并从对应 ParamPage 的 getter 读取参数。

---

### 数据后处理层 — SurfaceAnalyzer

从 CSV 文件读取扫描数据，执行强度过滤与平面最小二乘拟合：

1. 强度过滤：保留 5%~95% 范围点
2. 构建设计矩阵 `M ∈ R^(n×3)`，每行 `[x, y, 1]`
3. QR 分解求解 `Z = aX + bY + c`
4. 计算残差，输出 PV / RMS / X 倾角 / Y 倾角

---

### 设备驱动层 — SensorThread

独立线程运行 STIL DLL 的 TRE 硬件触发采集接口，封装双缓冲读取。

**数据流向**：
```
STIL DLL (双缓冲 DMA)
  → MCHR_GetAltitudeMeasurement
    → WaitForSingleObject (事件等待)
      → emit dataReady(QVector, QVector)
        → stil_drive::handleDataReady
          → DataReducer::reduceModeX()
            → QCustomPlot + CSV
```

---

### 主窗口 — stil_drive

只做信号槽连接和 UI 调度，不含业务逻辑。

**职责**：
- 构造时实例化各模块、挂载左侧 `TrajectoryPlannerDock` 与底部 `QTabWidget(脚本预览|PMAC日志)`
- 菜单 action ↔ slot 绑定（设备/模式/视图/工具）
- `switchMode()`：QActionGroup 互斥选择 → dock 切页 → 状态栏更新 → 重新生成脚本预览
- `startAcquisition()`：共用启动流程（设备检查 + 起线程 + 下发预览脚本）
- `handleDataReady()`：根据 `currentPage()->modeId()` 走 DataReducer 分支
- 析构与启动前调用各 ParamPage 的 `saveSettings`，构造时统一 `loadSettings`

---

## 构建方法

### 前置条件

1. 安装 **Visual Studio 2022**（含 C++ 桌面开发工作负载）
2. 安装 **Qt 6.10.2 MSVC 2022 64-bit**，并安装 **Qt VS Tools** 插件
3. 安装 **STIL CCS Prima 驱动程序**，确保以下文件存在：
   - `D:\STIL\DLL\Headers\Mchr.h`（及同目录其他头文件）
   - `D:\STIL\DLL\Lib_x64\Dll_chr.lib`
4. 确保运行时 `Dll_chr.dll` 在系统 PATH 中或与可执行文件同目录

### 编译步骤

1. 用 Visual Studio 2022 打开 `stil_drive.sln`
2. 选择配置：`Debug|x64` 或 `Release|x64`
3. 生成解决方案（Ctrl+Shift+B）

> **注**：本仓库新增多个 `.h/.cpp/.ui` 文件后，VS 首次打开请 **卸载 → 重新加载项目**，让 Qt VS Tools 重新解析 `QtUic` / `QtMoc` 项，否则 IntelliSense 可能误报 Qt 头文件找不到。

### 运行时部署（Release）

```bat
windeployqt.exe x64\Release\stil_drive.exe
```

---

## 使用说明

### 主界面布局

```
┌───────────────────────────────────────────────────────────┐
│ 文件  设备  测量模式  视图  工具  帮助           ← 菜单栏 │
├──────────────────┬────────────────────────────────────────┤
│ 轨迹规划 Dock    │  实时波形图 (QCustomPlot)              │
│  ┌────────────┐  │                                         │
│  │ 当前模式   │  │                                         │
│  ├────────────┤  │                                         │
│  │            │  │                                         │
│  │ 参数面板   │  │                                         │
│  │ (4 选 1)   │  │                                         │
│  │            │  ├────────────────────────────────────────┤
│  │            │  │  [开始采集] [停止采集] □同时保存 CSV   │
│  ├────────────┤  ├────────────────────────────────────────┤
│  │校验│生成│开始测量│ 脚本预览 / PMAC 日志 (Tab)           │
│  └────────────┘  │                                         │
├──────────────────┴────────────────────────────────────────┤
│ ●测头  ●PMAC  模式：…  位置/点数：…             ← 状态栏 │
└───────────────────────────────────────────────────────────┘
```

### 操作流程

1. **连接 PMAC**：菜单 `[设备] → 连接 PMAC…` → 弹出 Dialog 填 IP/端口 → 等待 Telnet 自动登录
2. **连接测头**：菜单 `[设备] → 连接测头…` → 弹出 Dialog 选设备 + 采样频率
3. **选模式**：菜单 `[测量模式] → 选一种` → 左侧 dock 切到对应参数页
4. **填参数**：在参数页输入扫描参数；信息条实时显示频率/点数/时间，超频或越界处会红字标记
5. **校验**（可选）：dock 底部 `[校验]` 仅检查参数合理性，不下发
6. **生成脚本**（可选）：`[生成脚本]` 重新生成脚本写入预览（用户也可手动微调脚本）
7. **启动测量**：见下文「两个启动按钮」
8. **停止 / 急停**：主窗口 `[停止采集]` 或菜单 `[工具] → 急停 (Esc)` —— 都会下发 `emergencyStop()` 脚本
9. **数据分析**：菜单 `[工具] → 数据分析（加载 CSV）`，输出 PV/RMS/倾斜角

### 两个启动按钮的语义

软件有两处可触发采集启动，**职责明确分工**：

| 按钮 | 位置 | 行为 | 适用场景 |
|------|------|------|---------|
| **「开始测量」** | dock 底部 | 完整工作流：① `validate()` 安全校验 → ② `buildScript()` 重新生成脚本写入预览 → ③ 下发并启动采集 | **首次扫描 / 改完参数后**：保证下发的脚本与参数面板一致 |
| **「开始采集」** | 主窗口下方 | 快速重跑：跳过校验，**直接以脚本预览编辑器中的当前脚本** 下发并启动 | **手动微调脚本后重跑 / 已校验过的同参数重测**：避免参数页覆盖已编辑的脚本 |

> **典型用法**：第一次进入某模式后用「开始测量」（走完整流程），后续如果你在脚本预览里手改了某行常量、或想用同一参数连测几次，用「开始采集」直接重跑。
>
> 警告：「开始采集」会**信任**脚本预览的当前内容，不再校验是否超频/越界，请确保你知道脚本含义。

---

## 扫描模式详解

### 1. 直线扫描（modeId = 0）

- **运动轴**：X 轴单轴
- **用户参数**：方向、长度（mm）、触发密度（µm/pt）、速度（mm/s）
- **触发器**：X 轴 Gate3 EQU，CompAdd = `density(µm) × 100,000` encoder counts
- **UI 校验**：`f = (v × 1000) / d ≤ 2000 Hz`
- **坐标还原**：`x = n × density_µm, y = 0`

### 2. 平面螺旋扫描（modeId = 2）

- **运动轴**：C 轴（旋转）+ X 轴（径向进给）联动
- **用户参数**：圆面半径、螺距（µm/圈）、密度（°/pt）、角速度、过心补偿
- **触发器**：C 轴 Gate3 EQU，CompAdd = `density_deg × 409,600` encoder counts
- **UI 校验**：`f = w / d ≤ 2000 Hz`
- **坐标还原**：极坐标 → 直角坐标
- **用途**：旋转光学元件全口径面形测量（从外向内）

### 3. 球面纬线扫描（modeId = 3）

- **运动轴**：C 轴单轴匀速
- **用户参数**：球面 R、纬度起/终/步、触发密度（次/°）、C 转速、过扫角
- **触发器**：C 轴 Gate3 EQU，CompAdd = `409,600 / density_per_deg` encoder counts
- **UI 校验**：拒绝 90°（极点）、拒绝 |纬度| < 50°（赤道死区，测头最大可测倾角约 40°）；`f = w × d ≤ 2000 Hz`
- **过扫**：单条纬线物理旋转 360° + 2 × overScan，去头去尾
- **用途**：球面元件单条/多条纬度圆弧轮廓

### 4. 非球面母线扫描（modeId = 4）

- **运动轴**：X 轴径向（可选 Z 轴 IMOV 跟随）
- **用户参数**：方程参数（R/K/A4..A12）+ 扫描参数（X 起始/范围/密度/进给）+ followZ
- **触发器**：X 轴 Gate3 EQU，密度 = 用户值或 `AsphericMath::calcSafeStepSize`（自动安全步距）
- **UI 校验**：用户密度 > 推荐安全步距 × 1.5 时拒绝（陡坡风险）；频率防溢出

---

## 数据格式

CSV 文件以时间戳命名：`YYYYMMDD_HHMMSS_SyncData.csv`。

### 列格式（模式 0/2/3）

| 列 | 内容 | 单位 |
|----|------|------|
| 1 | X 坐标 | µm 或 ° |
| 2 | Y 坐标 | µm |
| 3 | 表面高度 | µm |
| 4 | 反射光强度 | 归一化 |

### 列格式（模式 4 非球面）

| 列 | 内容 | 单位 |
|----|------|------|
| 1 | X 坐标 | µm |
| 2 | Y 坐标 | µm |
| 3 | 实测 Z | µm |
| 4 | 理论 Z | µm |
| 5 | 偏差 | µm |
| 6 | 强度 | 归一化 |

---

## 面形分析算法

菜单 `[工具] → 数据分析` 后：

1. **读取 CSV**：加载所有数据点
2. **强度过滤**：仅保留强度在 5%~95% 范围内的点
3. **最小二乘平面拟合**：构建 `A`（每行 `[x,y,1]`）+ `b`（高度），用 Eigen `colPivHouseholderQr()` 求解 `Z = ax + by + c`
4. **去除倾斜**：`residual = z - (ax + by + c)`
5. **输出**：PV（最大-最小残差，µm）、RMS、X 倾角 `atan(a)·180/π`、Y 倾角 `atan(b)·180/π`

---

## PMAC 通信协议

### 自动登录流程

1. 连接到 `IP:5002`
2. 自动应答 Telnet IAC 选项协商
3. 发送 `root\r\n`，等待密码提示
4. 发送 `deltatau\r\n`，等待 shell 提示符
5. 发送 `gpascii -2\r\n`，打开 PMAC ASCII 命令通道

### Gate3 EQU 触发脚本片段（直线扫描示例）

```
abort
delete all lookahead
undefine all
&1 #1->128000000X                      // 1 mm = 128,000,000 jog
&1 define lookahead 2048
#1 j/
Coord[1].FeedTime=1
Gate3[1].Chan[0].Equ1Ena=0
Gate3[1].Chan[0].EquOutPol=0
Gate3[1].Chan[0].EquOutMask=1
Gate3[1].Chan[0].EquWrite=1
Gate3[1].Chan[0].CompAdd=100000        // = density(µm) × 100,000 enc counts
Gate3[1].Chan[0].CompA=Gate3[1].Chan[0].ServoCapt+100000
Gate3[1].Chan[0].CompB=Gate3[1].Chan[0].CompA+50000
open prog 1000
LINEAR inc
ta 100 ts 50
FRAX(X)
F0.000500                              // = feedrate(mm/s) / 1000
Gate3[1].Chan[0].Equ1Ena == 1          // 同步开启触发
X50.0000                               // 精确走完 50mm
Gate3[1].Chan[0].Equ1Ena == 0          // 同步关闭触发
dwell 100
close
&1 b1000 r
```

---

## 关键参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| PMAC IP | `192.168.0.200` | 控制器以太网地址（菜单 `设备 → 连接 PMAC…` 修改） |
| PMAC Port | `5002` | Telnet 服务端口 |
| 测头采样频率 | 1000 Hz | 100/200/400/1000/2000 Hz 五档 |
| 测头物理采样上限 | `f_max = 2000 Hz` | UI 校验依据，超过即拒绝执行 |
| 球面赤道死区 | `|纬度| < 50°` | 测头最大可测倾角约 40°，对应纬度需 ≥ 50° |
| 非球面 followZ 点数上限 | 5000 | 超过自动截断 |
| QSettings 路径 | `org=stil_drive, app=trajectory` | Windows 注册表 / Linux ini |

---

## 已知限制

1. **STIL DLL 路径硬编码**：路径固定为 `D:\STIL\DLL\`，迁移到其他机器需改 `.vcxproj` 中的 include/lib 路径
2. **PMAC Gate3 通道硬编码**：默认使用 `Gate3[1].Chan[0]`，不同机器配置需改 `PmacScriptGen` 默认参数
3. **仅支持 x64 Windows**：依赖 Win32 API 和 STIL x64 DLL
4. **单传感器**：每次只能连接并使用一个 STIL 传感器
5. **本次重构暂未包含**：引入弧线 (Lead-in)、过心补偿轨迹脚本、球面纬线步进调度、非球面陡坡 Feedrate 控制（详见 `论文思路.md` 与轨迹规划说明书的"后续"章节）
6. **「开始采集」不做参数校验**：信任脚本预览中当前内容，请确保已通过「开始测量」校验过，或手动确认脚本合理后再使用

---

## 开发信息

- **开发语言**：C++17
- **UI 框架**：Qt 6.10.2
- **构建工具**：Visual Studio 2022 + Qt VS Tools
- **代码注释语言**：中文
- **版本管理**：Git
