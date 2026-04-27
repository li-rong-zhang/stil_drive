#pragma once
#include <QString>

struct AnalyzeResult {
    int validPoints = 0;
    double angleX = 0.0;   // X 倾角 (度)
    double angleY = 0.0;  // Y 倾角 (度)
    double pv = 0.0;      // 峰谷值 (um)
    double rms = 0.0;     // RMS (um)
};

class SurfaceAnalyzer
{
public:
    static AnalyzeResult analyzeFromCSV(const QString& filePath);
};
