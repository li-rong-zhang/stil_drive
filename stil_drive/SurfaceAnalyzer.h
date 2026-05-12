// ============================================================================
// SurfaceAnalyzer.h
// 表面面形分析模块
// ============================================================================
// 从 CSV 文件读取扫描数据，执行强度过滤和平面最小二乘拟合，
// 返回 AnalyzeResult 结构体（有效点数、倾斜角、PV、RMS）。
// ============================================================================

#pragma once
#include <QString>

struct AnalyzeResult {
    int validPoints = 0;    // 强度过滤后的有效点数
    double angleX = 0.0;    // X 方向倾角 (°)
    double angleY = 0.0;    // Y 方向倾角 (°)
    double pv = 0.0;       // 峰谷值 (µm)
    double rms = 0.0;      // RMS 均方根 (µm)
};

class SurfaceAnalyzer
{
public:
    static AnalyzeResult analyzeFromCSV(const QString& filePath);
};
