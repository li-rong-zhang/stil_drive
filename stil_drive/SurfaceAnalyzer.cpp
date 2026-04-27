#include "SurfaceAnalyzer.h"
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <cmath>
#include <Eigen/Dense>

AnalyzeResult SurfaceAnalyzer::analyzeFromCSV(const QString& filePath)
{
    AnalyzeResult res;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return res;
    }

    QTextStream in(&file);
    in.readLine(); // 跳过表头

    std::vector<double> valid_X, valid_Y, valid_Z;

    while (!in.atEnd()) {
        QStringList parts = in.readLine().split(',');
        if (parts.size() >= 4) {
            double x = parts[0].toDouble();
            double y = parts[1].toDouble();
            double z = parts[2].toDouble();
            double intensity = parts[3].toDouble();
            if (intensity > 5.0 && intensity < 95.0 && !std::isnan(z)) {
                valid_X.push_back(x);
                valid_Y.push_back(y);
                valid_Z.push_back(z);
            }
        }
    }
    file.close();

    int n = (int)valid_X.size();
    if (n < 3) return res;

    Eigen::MatrixXd M(n, 3);
    Eigen::VectorXd Z_vec(n);
    for (int i = 0; i < n; i++) {
        M(i, 0) = valid_X[i];
        M(i, 1) = valid_Y[i];
        M(i, 2) = 1.0;
        Z_vec(i) = valid_Z[i];
    }

    Eigen::Vector3d v = M.colPivHouseholderQr().solve(Z_vec);
    double A = v(0), B = v(1), C = v(2);

    double max_res = -1e9, min_res = 1e9, sq_sum = 0.0;
    for (int i = 0; i < n; i++) {
        double residual = valid_Z[i] - (A * valid_X[i] + B * valid_Y[i] + C);
        if (residual > max_res) max_res = residual;
        if (residual < min_res) min_res = residual;
        sq_sum += residual * residual;
    }

    res.validPoints = n;
    res.angleX = std::atan(A) * 180.0 / M_PI;
    res.angleY = std::atan(B) * 180.0 / M_PI;
    res.pv = max_res - min_res;
    res.rms = std::sqrt(sq_sum / n);

    return res;
}
