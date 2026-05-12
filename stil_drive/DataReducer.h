// ============================================================================
// DataReducer.h
// 扫描数据坐标还原模块
// ============================================================================
// 根据扫描模式，将脉冲序列（高度+强度）还原为空间坐标。
// ReductionResult 结构体携带横坐标数据、当前值、窗口宽度和单位信息，
// 供主窗口刷新 QCustomPlot 图表和写入 CSV 文件。
// ============================================================================

#pragma once
#include <QString>
#include <QVector>
#include <QTextStream>

// 数据还原结果（供 UI 层绘制图表和导出 CSV）
struct ReductionResult {
    QVector<double> xData;      // 横坐标数据
    double currentX = 0.0;     // 当前显示的横坐标值
    double windowWidth = 1000.0;// 图表窗口宽度
    QString unit;               // 单位描述（"um" / "度" / "µm"）
};

class DataReducer
{
public:
    // 模式 0: X 轴直线扫描
    static ReductionResult reduceMode0(
        const QVector<double>& alts,
        const QVector<double>& ints,
        qint64 startPoint,
        QTextStream* csvStream);

    // 模式 2: C+X 轴螺旋扫描
    static ReductionResult reduceMode2(
        const QVector<double>& alts,
        const QVector<double>& ints,
        qint64 startPoint,
        double startRadiusUm,
        double pitchPerRevUm,
        double densityDegPerPt,
        double overCenterMm,
        QTextStream* csvStream);

    // 模式 3: C 轴球面纬度扫描
    static ReductionResult reduceMode3(
        const QVector<double>& alts,
        const QVector<double>& ints,
        qint64 startPoint,
        QTextStream* csvStream);

    // 模式 4: 非球面母线扫描
    static ReductionResult reduceMode4(
        const QVector<double>& alts,
        const QVector<double>& ints,
        qint64 startPoint,
        double startRadiusUm,
        double scanRangeUm,
        double R_mm, double k,
        double A4, double A6, double A8, double A10, double A12,
        QTextStream* csvStream);
};
