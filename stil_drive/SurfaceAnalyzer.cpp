// ============================================================================
// SurfaceAnalyzer.cpp
// 表面面形分析模块
// ============================================================================
// 从 CSV 文件读取扫描数据，执行以下分析：
//   1. 强度过滤：剔除强度异常（饱和/无效）的数据点
//   2. 最小二乘平面拟合：Z = aX + bY + c
//   3. 残差计算：去除平移和倾斜后计算面形 PV/RMS 指标
//   4. 倾斜角计算：atan(a)、atan(b) 得 X/Y 倾角（度）
//
// 算法：
//   构建设计矩阵 M ∈ R^(n×3)，每行 [x, y, 1]
//   使用 QR 分解（Eigen colPivHouseholderQr）求解最小二乘解
//   计算残差后统计 PV（峰谷值）和 RMS（均方根）
// ============================================================================

#include "SurfaceAnalyzer.h"
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <cmath>
#include <Eigen/Dense>

// 用 std::acos(-1.0) 在运行时计算 π，结果即为 double 能表示的最精确 π 值
static const double kPi = std::acos(-1.0);

AnalyzeResult SurfaceAnalyzer::analyzeFromCSV(const QString& filePath)
{
    AnalyzeResult res;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return res;
    }

    QTextStream in(&file);
    in.readLine(); // 跳过 CSV 表头

    std::vector<double> valid_X, valid_Y, valid_Z;

    while (!in.atEnd()) {
        QStringList parts = in.readLine().split(',');
        if (parts.size() >= 4) {
            double x = parts[0].toDouble();
            double y = parts[1].toDouble();
            double z = parts[2].toDouble();
            double intensity = parts[3].toDouble();

            // 强度过滤：仅保留 5%~95% 范围（去除无效/饱和点）
            if (intensity > 5.0 && intensity < 95.0 && !std::isnan(z)) {
                valid_X.push_back(x);
                valid_Y.push_back(y);
                valid_Z.push_back(z);
            }
        }
    }
    file.close();

    int n = (int)valid_X.size();
    if (n < 3) return res;  // 至少需要 3 个点才能拟合平面

    // --- 构建设计矩阵 M ∈ R^(n×3) ---
    Eigen::MatrixXd M(n, 3);
    Eigen::VectorXd Z_vec(n);
    for (int i = 0; i < n; i++) {
        M(i, 0) = valid_X[i];
        M(i, 1) = valid_Y[i];
        M(i, 2) = 1.0;  // 常数项系数
        Z_vec(i) = valid_Z[i];
    }

    // --- QR 分解求解最小二乘：Z = aX + bY + c ---
    Eigen::Vector3d v = M.colPivHouseholderQr().solve(Z_vec);
    double A = v(0), B = v(1), C = v(2);

    // --- 计算残差并统计 ---
    double max_res = -1e9, min_res = 1e9, sq_sum = 0.0;
    for (int i = 0; i < n; i++) {
        double residual = valid_Z[i] - (A * valid_X[i] + B * valid_Y[i] + C);
        if (residual > max_res) max_res = residual;
        if (residual < min_res) min_res = residual;
        sq_sum += residual * residual;
    }

    res.validPoints = n;
    res.angleX = std::atan(A) * 180.0 / kPi;
    res.angleY = std::atan(B) * 180.0 / kPi;
    res.pv = max_res - min_res;
    res.rms = std::sqrt(sq_sum / n);

    return res;
}
