// ============================================================================
// PmacScriptGen.h
// PMAC 运动脚本生成模块
// ============================================================================
// 根据扫描模式生成 Power PMAC ASCII 命令脚本字符串，
// 由 PmacController::sendCommand() 下发到控制器执行。
//
// 轴缩放（jog 单位，用于 &1 坐标系定义）：
//   X/Y/Z：1 mm = 128,000,000 jog ；B：1° = 5,898,240 jog ；C：1° = 1,638,400 jog
// Gate3 编码器计数（CompAdd/CompA/B 专用）：
//   1 µm = 100,000 encoder counts ；1° = 409,600 encoder counts
// JogSpeed 单位：jog/ms（1 mm/s = 128,000 jog/ms ；1°/s = 1,638.4 jog/ms）
//
// 螺旋与非球面统一采用 inc 增量联动，避免独立 JogSpeed 长时间走形坐标系漂移；
// 球面纬线扫描每次只生成单条纬线脚本，多纬线由主窗口手动确认调度。
//
// 轴映射与触发通道通过静态 AxisConfig 配置，主窗口在启动时加载用户设置。
// ============================================================================

#pragma once
#include <QString>
#include "AxisConfig.h"

class PmacScriptGen
{
public:
    static void setAxisConfig(const AxisConfig& cfg) { s_cfg = cfg; }
    static const AxisConfig& axisConfig()            { return s_cfg; }

    // 模式 0: X 轴直线扫描（仅正向 j+）
    static QString generateLinearScript(
        double scanLengthMm,         // 有效扫描长度（注释中体现）
        double densityUmPerPt,       // 触发密度 µm/次
        double feedrateMmPerSec);    // 目标速度 mm/s

    // 模式 2: C+X 轴平面螺旋扫描（inc 联动）
    // C 轴每步增量 dC = density_deg；X 轴每步增量 dX = -pitch × density / 360
    // 由 G 代码联动绑定 X-C 运动学，C 轴 Gate3 EQU 在每个 dC 步末触发
    static QString generateSpiralScript(
        double startRadiusMm,        // 圆面半径
        double pitchUmPerRev,        // 螺距 µm/圈
        double densityDegPerPt,      // 触发密度 °/次
        double angularSpeedDegPerSec,// C 轴角速度 °/s
        double overCenterMm);        // 过心补偿 mm

    // 模式 3: 单条纬线扫描
    // C 轴匀速旋转 (360° + 2 × overScan)，C 轴 Gate3 EQU 触发
    // X/Z 必须已位于该纬线对应位置（由 generateSphericalPositioningScript 提前完成）
    static QString generateSphericalLatScript(
        double sphereRadiusMm,
        double currentLatDeg,        // 当前要扫的纬度（仅用于注释/记录）
        double densityPerDeg,        // 触发密度 次/°
        double cSpeedDegPerSec,      // C 转速 °/s
        double overScanDeg);         // 过扫角（默认 5°）

    // 模式 3 辅助：计算并生成"从 fromLat 移动到 toLat"的 X/Z 定位脚本
    // 用于多纬线扫描中两条纬线之间的位置切换，用户确认后下发
    // 凸面：r_xy = R·cos(φ), z = R·sin(φ)
    // 凹面：方向相反（dir = -1）
    static QString generateSphericalPositioningScript(
        double sphereRadiusMm,
        double fromLatDeg, double toLatDeg,
        bool isConvex);

    // 模式 4: 非球面母线扫描（强制 Z 轴 inc 跟随 + X 轴硬件触发）
    // 不再支持 followZ=false 模式
    static QString generateAsphericMeridianScript(
        double startRadiusUm, double scanRangeUm,
        double R_mm, double k,
        double A4, double A6, double A8, double A10, double A12,
        double densityUmPerPt,       // 触发密度 µm/次（≤0 时回退安全步距）
        double feedrateMmPerSec);    // X 轴进给 mm/s

    // 急停脚本（停止全轴+关闭 EQU 触发使能）
    static QString emergencyStop();

private:
    static AxisConfig s_cfg;
};
