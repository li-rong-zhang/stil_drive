// ============================================================================
// PmacScriptGen.cpp
// PMAC 运动脚本生成模块（重构版 — 连续 LINEAR 插补 + 同步触发）
// ============================================================================
// 根据不同扫描模式生成 Power PMAC ASCII 命令脚本，通过 TCP 下发。
//
// 设计原则：
//   1. 运动程序（PROG）内部使用 LINEAR inc 连续插补，严禁轨迹循环内加 Dwell
//   2. 硬件触发（Gate3 EQU）通过同步赋值 == 在 PROG 内部精确启停
//   3. 坐标轴定义带缩放因子，使 F 值落在安全范围（≈ 1~100000），
//      并以 double 小数格式输出，防止 PMAC 整数溢出
//   4. PROG 收尾动作（关闭触发、停轴）写在 close 之前，
//      &1 b1000 r 之后不跟任何 abort 或触发关闭指令
//
// 轴缩放约定（jog 单位，用于 &1 坐标系定义）：
//   X / Y / Z 轴：1 mm = 128,000,000  jog（#1->128000000X）
//   B 轴：       1°   = 5,898,240     jog（#4->5898240B）
//   C 轴：       1°   = 1,638,400     jog（#5->1638400C）
// Gate3 编码器计数（CompAdd/CompA/B 专用，不受轴缩放影响）：
//   1 µm = 100,000 encoder counts ；1° = 409,600 encoder counts
//
// Gate3 EQU 工作原理（不变）：
//   CompAdd   设置触发间隔（编码器计数，不受用户单位缩放影响）
//   CompA/B   设置比较基准（ServoCapt + 引入段偏移 + CompAdd）
//   EquWrite  写入式比较（每次触发后自动递增基准）
//   Equ1Ena   由 PROG 内同步赋值 == 1 / == 0 精确控制
// ============================================================================

#include "PmacScriptGen.h"
#include "AsphericMath.h"
#include <QString>
#include <QStringBuilder>
#include <cmath>

AxisConfig PmacScriptGen::s_cfg;

namespace {
// =========================================================================
// Gate3 编码器计数（CompAdd/CompA/B 触发间隔专用）
// =========================================================================
constexpr double UM_TO_COUNT  = 100000.0;     // 1 µm = 100,000 encoder counts
constexpr double DEG_TO_COUNT = 409600.0;     // 1°   = 409,600 encoder counts
constexpr double PI_D         = 3.141592653589793;

// =========================================================================
// JogSpeed 换算（Motor[].JogSpeed 单位 = jog/ms）
// =========================================================================
constexpr double MMS_TO_JOG   = 128000.0;     // 1 mm/s = 128,000 jog/ms
constexpr double DEGS_TO_JOG  = 1638.4;       // 1°/s   = 1,638.4 jog/ms

// =========================================================================
// 坐标轴缩放因子（jog 单位，用于 &1 坐标系定义）
// =========================================================================
constexpr double X_SCALE = 128000000.0;  // 1 mm  = 128,000,000 jog
constexpr double Y_SCALE = 128000000.0;  // 1 mm  = 128,000,000 jog
constexpr double Z_SCALE = 128000000.0;  // 1 mm  = 128,000,000 jog
constexpr double B_SCALE = 5898240.0;    // 1°    = 5,898,240   jog
constexpr double C_SCALE = 1638400.0;    // 1°    = 1,638,400   jog

// 引入段（空走加速段）长度，保证电机在触发开启前达到稳态速度
constexpr double PROG_NUM       = 1000;  // 固定使用的运动程序号

// =========================================================================
// 生成 Gate3 EQU 配置块（不包含 Equ1Ena，由 PROG 内部控制）
// compAdd/compB 单位均为编码器原始计数
// =========================================================================
QString gate3EquSetup(int gate, int chan, long compAdd, long compB)
{
    // CompA = 当前位置 + 触发间隔
    // Equ1Ena=1 后，首个触发正好落在当前位置 + CompAdd 处
    return QString(
        "Gate3[%1].Chan[%2].Equ1Ena=0\n"
        "Gate3[%1].Chan[%2].EquOutPol=0\n"
        "Gate3[%1].Chan[%2].EquOutMask=1\n"
        "Gate3[%1].Chan[%2].EquWrite=1\n"
        "Gate3[%1].Chan[%2].CompAdd=%3\n"
        "Gate3[%1].Chan[%2].CompA=Gate3[%1].Chan[%2].ServoCapt+%3\n"
        "Gate3[%1].Chan[%2].CompB=Gate3[%1].Chan[%2].CompA+%4\n"
    ).arg(gate).arg(chan).arg(compAdd).arg(compB);
}

// =========================================================================
// PROG 内部用：同步启停触发宏
// =========================================================================
QString equEnaSync(int gate, int chan, int onOff)
{
    return QString("Gate3[%1].Chan[%2].Equ1Ena == %3\n")
        .arg(gate).arg(chan).arg(onOff);
}

// =========================================================================
// 拼接急停用全轴 jog release
// =========================================================================
QString jogReleaseAll(const AxisConfig& c)
{
    return QString("#%1j/ #%2j/ #%3j/ #%4j/ #%5j/\n")
        .arg(c.motorX).arg(c.motorY).arg(c.motorZ).arg(c.motorB).arg(c.motorC);
}

// =========================================================================
// 格式化 F 指令，double 小数格式，防止 PMAC 整数溢出
// =========================================================================
QString formatF(double feedrate)
{
    return QString("F%1\n").arg(feedrate, 0, 'f', 6);
}
} // namespace

// ============================================================================
// generateLinearScript - X 轴直线扫描（LINEAR PROG，精确距离自动停止）
// ============================================================================
QString PmacScriptGen::generateLinearScript(
    double scanLengthMm,
    double densityUmPerPt,
    double feedrateMmPerSec)
{
    if (densityUmPerPt <= 0)   densityUmPerPt   = 1.0;
    if (feedrateMmPerSec <= 0) feedrateMmPerSec = 1.0;

    long compAdd  = (long)(densityUmPerPt * UM_TO_COUNT);
    long compB    = compAdd / 2;
    double scanFeed = feedrateMmPerSec / 1000.0;  // mm/s → mm/ms（user/ms）

    QString script;
    script += QString("// 直线扫描 长度=%1 mm 密度=%2 µm/pt 速度=%3 mm/s\n")
                .arg(scanLengthMm).arg(densityUmPerPt).arg(feedrateMmPerSec);
    script += "abort\n";
    script += "delete all lookahead\n";
    script += "undefine all\n";
    script += QString("&1 #%1->%2X\n")
        .arg(s_cfg.motorX).arg((long)X_SCALE);
    script += "&1 define lookahead 2048\n";
    script += QString("#%1j/\n").arg(s_cfg.motorX);
    script += "Coord[1].FeedTime=1\n";

    // EQU 配置（无引入段）
    script += gate3EquSetup(s_cfg.triggerGate, s_cfg.triggerChan, compAdd, compB);

    // LINEAR 运动程序：定位 → 扫描 → 自动停止
    script += QString("open prog %1\n").arg((int)PROG_NUM);
    script += "LINEAR inc\n";
    script += "ta 100 ts 50\n";
    script += formatF(0.01);  // 定位速度 10 mm/s
    script += "X0\n";         // 原地刷新（确保 F 生效）
    script += "FRAX(X)\n";
    script += formatF(scanFeed);
    script += equEnaSync(s_cfg.triggerGate, s_cfg.triggerChan, 1);
    script += QString("X%1\n").arg(scanLengthMm, 0, 'f', 4);
    script += equEnaSync(s_cfg.triggerGate, s_cfg.triggerChan, 0);
    script += "dwell 100\n";
    script += "close\n";
    script += QString("&1 b%1 r\n").arg((int)PROG_NUM);

    return script;
}

// ============================================================================
// generateSpiralScript - 平面螺旋连续 LINEAR 插补（单条指令走完全程）
//
// 螺旋在 (r, θ) 空间为严格直线：X(r) = R_start - (pitch/360)×θ
// 使用 LINEAR inc + FRAX(C) 单段完成，Gate3 EQU 在 PROG 内同步启停。
// 无引入段：LINEAR inc 模式下 X 和 C 严格一一对应，无需额外加速段。
// ============================================================================
QString PmacScriptGen::generateSpiralScript(
    double startRadiusMm,
    double pitchUmPerRev,
    double densityDegPerPt,
    double angularSpeedDegPerSec,
    double overCenterMm)
{
    if (startRadiusMm <= 0)         startRadiusMm = 10.0;
    if (pitchUmPerRev <= 0)         pitchUmPerRev = 10.0;
    if (densityDegPerPt <= 0)       densityDegPerPt = 0.5;
    if (angularSpeedDegPerSec <= 0) angularSpeedDegPerSec = 1000.0;

    // ---- 计数体系计算 ----
    long compAdd = (long)(densityDegPerPt * DEG_TO_COUNT);
    long compB   = compAdd / 2;

    double totalRangeMm  = startRadiusMm + overCenterMm;
    double totalAngleDeg = totalRangeMm * 1000.0 / pitchUmPerRev * 360.0;

    // 全程轨迹：X 从起始半径移动到中心（负方向），C 旋转 totalAngleDeg
    double total_dX_mm   = -totalRangeMm;
    double total_dC_deg  = totalAngleDeg;

    // F 值：以 C 轴为基准（FRAX(C)），单位 user/ms
    double feedrate = angularSpeedDegPerSec / 1000.0;

    // ---- 在线前处理 ----
    QString script;
    script += QString("// 平面螺旋 LINEAR 单段 R=%1 mm 螺距=%2 µm/rev "
                      "密度=%3 °/pt 角速度=%4 °/s 过心=%5 mm 总角=%6°\n")
        .arg(startRadiusMm).arg(pitchUmPerRev).arg(densityDegPerPt)
        .arg(angularSpeedDegPerSec).arg(overCenterMm).arg(totalAngleDeg);
    script += "abort\n";
    script += "delete all lookahead\n";
    script += "undefine all\n";

    // 带缩放因子的坐标系定义
    script += QString("&1 #%1->%2X #%3->%4C\n")
        .arg(s_cfg.motorX).arg((long)X_SCALE)
        .arg(s_cfg.motorC).arg((long)C_SCALE);
    script += "&1 define lookahead 2048\n";
    script += QString("#%1j/ #%2j/\n").arg(s_cfg.motorX).arg(s_cfg.motorC);
    script += "Coord[1].FeedTime=1\n";

    // EQU 配置（无引入段）
    script += gate3EquSetup(s_cfg.triggerGate, s_cfg.triggerChan, compAdd, compB);

    // ---- 运动程序 ----
    script += QString("open prog %1\n").arg((int)PROG_NUM);
    script += "LINEAR inc\n";
    script += "ta 100 ts 50\n";

    // 定位段：X 从圆心走到起始半径（C 不动），使用矢量速度
    script += formatF(0.5);
    script += QString("X%1 C0\n").arg(startRadiusMm, 0, 'f', 4);

    // 切换到 C 轴速度模式用于螺旋扫描
    script += "FRAX(C)\n";
    script += formatF(feedrate);

    // 同步开启触发
    script += equEnaSync(s_cfg.triggerGate, s_cfg.triggerChan, 1);

    // 主轨迹 — 单条 LINEAR 走完全程（X 和 C 严格一一对应）
    script += QString("X%1 C%2\n")
        .arg(total_dX_mm, 0, 'f', 4)
        .arg(total_dC_deg, 0, 'f', 2);

    // 同步关闭触发
    script += equEnaSync(s_cfg.triggerGate, s_cfg.triggerChan, 0);

    // 收尾：减速停车
    script += "dwell 100\n";
    script += "close\n";
    script += QString("&1 b%1 r\n").arg((int)PROG_NUM);

    return script;
}

// ============================================================================
// generateSphericalLatScript - 单条纬线扫描（纯 Jog，无运动程序）
// 注：调用前必须已通过 generateSphericalPositioningScript 将 X/Z 定位到该纬线
// ============================================================================
QString PmacScriptGen::generateSphericalLatScript(
    double sphereRadiusMm,
    double currentLatDeg,
    double densityPerDeg,
    double cSpeedDegPerSec,
    double overScanDeg)
{
    if (densityPerDeg <= 0)   densityPerDeg = 10.0;
    if (cSpeedDegPerSec <= 0) cSpeedDegPerSec = 100.0;
    if (overScanDeg < 0)      overScanDeg = 5.0;

    long cJogSpeed = (long)(cSpeedDegPerSec * DEGS_TO_JOG);
    long compAdd   = (long)(DEG_TO_COUNT / densityPerDeg);
    long compB     = compAdd / 2;

    double totalAngleDeg     = 360.0 + 2.0 * overScanDeg;
    long   targetAngleCounts = (long)(totalAngleDeg * 1638400.0);

    QString script;
    script += QString("// 球面纬线扫描 R=%1 mm 当前纬度=%2° 密度=%3 /° 转速=%4 °/s 过扫=%5°\n")
                .arg(sphereRadiusMm).arg(currentLatDeg).arg(densityPerDeg)
                .arg(cSpeedDegPerSec).arg(overScanDeg);
    script += "abort\n";
    script += "delete all lookahead\n";
    script += "undefine all\n";
    script += QString("&1 #%1->C\n").arg(s_cfg.motorC);
    script += "&1 define lookahead 2048\n";
    script += QString("#%1j/\n").arg(s_cfg.motorC);
    script += "Coord[1].FeedTime=1\n";
    script += QString("Motor[%1].JogSpeed=%2\n").arg(s_cfg.motorC).arg(cJogSpeed);
    script += gate3EquSetup(s_cfg.triggerGate, s_cfg.triggerChan, compAdd, compB);
    script += QString("Gate3[%1].Chan[%2].Equ1Ena=1\n")
                .arg(s_cfg.triggerGate).arg(s_cfg.triggerChan);
    script += QString("#%1 j^%2\n").arg(s_cfg.motorC).arg(targetAngleCounts);
    return script;
}

// ============================================================================
// generateSphericalPositioningScript - 多纬线之间的 X/Z 定位
// 凸面：球心在工件下方，探头位于球外侧
// 凹面：探头由球内观察，方向相反
// ============================================================================
QString PmacScriptGen::generateSphericalPositioningScript(
    double sphereRadiusMm,
    double fromLatDeg, double toLatDeg,
    bool isConvex)
{
    double fromRad = fromLatDeg * PI_D / 180.0;
    double toRad   = toLatDeg   * PI_D / 180.0;
    double signXZ  = isConvex ? 1.0 : -1.0;

    double dx_um = signXZ * sphereRadiusMm * 1000.0 * (std::cos(toRad) - std::cos(fromRad));
    double dz_um = signXZ * sphereRadiusMm * 1000.0 * (std::sin(toRad) - std::sin(fromRad));

    // 用户单位 mm
    double dx_user = dx_um / 1000.0;
    double dz_user = dz_um / 1000.0;

    // 定位速度固定 1 mm/s
    double feedrate = 1.0 / 1000.0;  // 1 mm/s → user/ms

    QString script;
    script += QString("// 球面纬线定位 %1° → %2° (%3) ΔX=%4 µm ΔZ=%5 µm\n")
                .arg(fromLatDeg).arg(toLatDeg)
                .arg(isConvex ? "凸" : "凹")
                .arg(dx_um, 0, 'f', 2).arg(dz_um, 0, 'f', 2);
    script += "abort\n";
    script += "delete all lookahead\n";
    script += "undefine all\n";
    script += QString("&1 #%1->%2X #%3->%4Z\n")
        .arg(s_cfg.motorX).arg((long)X_SCALE)
        .arg(s_cfg.motorZ).arg((long)Z_SCALE);
    script += "&1 define lookahead 2048\n";
    script += QString("#%1j/ #%2j/\n").arg(s_cfg.motorX).arg(s_cfg.motorZ);
    script += "Coord[1].FeedTime=1\n";

    // 纯定位，无触发 —— 不需要 EQU
    script += QString("open prog %1\n").arg((int)PROG_NUM);
    script += "LINEAR inc\n";
    script += "ta 100 ts 50\n";
    script += formatF(feedrate);
    script += QString("X%1 Z%2\n")
        .arg(dx_user, 0, 'f', 4)
        .arg(dz_user, 0, 'f', 4);
    script += "dwell 100\n";
    script += "close\n";
    script += QString("&1 b%1 r\n").arg((int)PROG_NUM);

    return script;
}

// ============================================================================
// generateAsphericMeridianScript - 非球面母线连续 LINEAR 微线段扫描
//
// Z 轴严格跟随非球面方程 Z = f(R, X)。微线段以固定 X 步距切分，
// 连续 LINEAR inc 输出，不在轨迹循环内加 Dwell。
// Gate3 EQU 在定位段后同步开启，主轨迹走完同步关闭。
// ============================================================================
QString PmacScriptGen::generateAsphericMeridianScript(
    double startRadiusUm, double scanRangeUm,
    double R_mm, double k,
    double A4, double A6, double A8, double A10, double A12,
    double densityUmPerPt,
    double feedrateMmPerSec)
{
    if (R_mm == 0.0)           R_mm = 100.0;
    if (scanRangeUm <= 0)      scanRangeUm = 50000.0;
    if (feedrateMmPerSec <= 0) feedrateMmPerSec = 1.0;

    // 步距：用户给定密度时直接采用，否则按斜率求安全步距
    double step_um = (densityUmPerPt > 0)
        ? densityUmPerPt
        : AsphericMath::calcSafeStepSize(
              startRadiusUm / 1000.0, R_mm, k, A4, A6, A8, A10, A12);

    int numPoints = (int)(scanRangeUm / step_um) + 1;

    long compAdd = (long)(step_um * UM_TO_COUNT);
    long compB   = compAdd / 2;

    // F 值：以 X 轴为基准（FRAX(X)），单位 user/ms
    double feedrate = feedrateMmPerSec / 1000.0;

    // 起始点 Z 值
    double prev_z_um = AsphericMath::calcAsphericZ(
        startRadiusUm / 1000.0, R_mm, k, A4, A6, A8, A10, A12);

    // ---- 在线前处理 ----
    QString script;
    script += QString("// 非球面母线 LINEAR 连续 R=%1 mm K=%2 步距=%3 µm 进给=%4 mm/s 点数=%5\n")
                .arg(R_mm).arg(k).arg(step_um, 0, 'f', 3)
                .arg(feedrateMmPerSec).arg(numPoints);
    script += "abort\n";
    script += "delete all lookahead\n";
    script += "undefine all\n";
    script += QString("&1 #%1->%2X #%3->%4Z\n")
        .arg(s_cfg.motorX).arg((long)X_SCALE)
        .arg(s_cfg.motorZ).arg((long)Z_SCALE);
    script += "&1 define lookahead 2048\n";
    script += QString("#%1j/ #%2j/\n").arg(s_cfg.motorX).arg(s_cfg.motorZ);
    script += "Coord[1].FeedTime=1\n";

    // EQU 配置（无引入段）
    script += gate3EquSetup(s_cfg.triggerGate, s_cfg.triggerChan, compAdd, compB);

    // ---- 运动程序（包含定位 + 主轨迹 + 收尾）----
    double xStart_mm = startRadiusUm / 1000.0;
    double zStart_mm = prev_z_um / 1000.0;

    // ---- 运动程序 ----
    script += QString("open prog %1\n").arg((int)PROG_NUM);
    script += "LINEAR inc\n";
    script += "ta 100 ts 50\n";
    script += formatF(0.05);  // 定位段矢量速度（无 FRAX，默认矢量）

    // 从中心定位到扫描起点
    if (R_mm > 0.0) {
        // 凸面：先抬 Z 越过最高点再平移
        double retract_mm = (std::abs(prev_z_um) + 2000.0) / 1000.0;
        double drop2mm   = -2.0;  // Z 降回 2mm 安全高度
        script += QString("Z%1\n").arg(retract_mm, 0, 'f', 4);
        script += QString("X%1 Z%2\n")
            .arg(xStart_mm, 0, 'f', 4)
            .arg(drop2mm, 0, 'f', 4);
    } else {
        // 凹面：直连路径在曲面之上，安全联动
        script += QString("X%1 Z%2\n")
            .arg(xStart_mm, 0, 'f', 4)
            .arg(zStart_mm, 0, 'f', 4);
    }

    // 切换到 X 轴速度模式用于主轨迹
    script += "FRAX(X)\n";
    script += formatF(feedrate);

    // 同步开启触发
    script += equEnaSync(s_cfg.triggerGate, s_cfg.triggerChan, 1);

    // ---- 主轨迹：连续微线段 LINEAR inc，无 Dwell ----
    for (int i = 1; i < numPoints; ++i) {
        double r_um = startRadiusUm + i * step_um;
        double r_mm = r_um / 1000.0;
        double z_um = AsphericMath::calcAsphericZ(
            r_mm, R_mm, k, A4, A6, A8, A10, A12);
        double dz_um = z_um - prev_z_um;

        double dx_user = step_um / 1000.0;
        double dz_user = dz_um   / 1000.0;
        script += QString("X%1 Z%2\n")
            .arg(dx_user, 0, 'f', 6)
            .arg(dz_user, 0, 'f', 6);
        prev_z_um = z_um;
    }

    // 同步关闭触发
    script += equEnaSync(s_cfg.triggerGate, s_cfg.triggerChan, 0);

    // 收尾：减速停车
    script += "dwell 100\n";
    script += "close\n";
    script += QString("&1 b%1 r\n").arg((int)PROG_NUM);

    return script;
}

// ============================================================================
// emergencyStop - 急停脚本（同时斩断 Gate3[0] 和 Gate3[1] 两组 EQU）
// ============================================================================
QString PmacScriptGen::emergencyStop()
{
    return jogReleaseAll(s_cfg) +
        QString(
            "Gate3[0].Chan[0].Equ1Ena=0\n"
            "Gate3[0].Chan[1].Equ1Ena=0\n"
            "Gate3[0].Chan[2].Equ1Ena=0\n"
            "Gate3[0].Chan[3].Equ1Ena=0\n"
            "Gate3[1].Chan[0].Equ1Ena=0\n"
            "Gate3[1].Chan[1].Equ1Ena=0\n"
            "Gate3[1].Chan[2].Equ1Ena=0\n"
            "Gate3[1].Chan[3].Equ1Ena=0\n"
        );
}
