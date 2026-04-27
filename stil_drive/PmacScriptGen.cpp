#include "PmacScriptGen.h"
#include "AsphericMath.h"
#include <QString>

QString PmacScriptGen::generateLinearScript(int targetGate, int targetChan)
{
    return QString(
        "abort\n"
        "#1 j/\n"
        "Motor[1].ServoCtrl=1\n"
        "undefine all\n"
        "&1 #1->X\n"
        "Motor[1].JogSpeed=128000\n"
        "Gate3[%1].Chan[%2].Equ1Ena=0\n"
        "Gate3[%1].Chan[%2].EquOutPol=0\n"
        "Gate3[%1].Chan[%2].EquOutMask=1\n"
        "Gate3[%1].Chan[%2].EquWrite=1\n"
        "Gate3[%1].Chan[%2].CompAdd=100000\n"
        "Gate3[%1].Chan[%2].CompA=Gate3[%1].Chan[%2].ServoCapt+100000\n"
        "Gate3[%1].Chan[%2].CompB=Gate3[%1].Chan[%2].CompA+50000\n"
        "Gate3[%1].Chan[%2].Equ1Ena=1\n"
        "#1 j+\n"
    ).arg(targetGate).arg(targetChan);
}

QString PmacScriptGen::generateSpiralScript(double pitchUm, int targetGate, int targetChan)
{
    if (pitchUm <= 0) pitchUm = 10.0;
    long xJogSpeed = (long)((pitchUm / 0.36) * 128.0);

    return QString(
        "abort\n"
        "#1 j/ #5 j/\n"
        "Motor[1].ServoCtrl=1\n"
        "Motor[5].ServoCtrl=1\n"
        "&1 #1->X #5->C\n"
        "Motor[5].JogSpeed=1638400\n"
        "Motor[1].JogSpeed=%3\n"
        "Gate3[%1].Chan[%2].Equ1Ena=0\n"
        "Gate3[%1].Chan[%2].EquOutPol=0\n"
        "Gate3[%1].Chan[%2].EquOutMask=1\n"
        "Gate3[%1].Chan[%2].EquWrite=1\n"
        "Gate3[%1].Chan[%2].CompAdd=204800\n"
        "Gate3[%1].Chan[%2].CompA=Gate3[%1].Chan[%2].ServoCapt+204800\n"
        "Gate3[%1].Chan[%2].CompB=Gate3[%1].Chan[%2].CompA+102400\n"
        "Gate3[%1].Chan[%2].Equ1Ena=1\n"
        "#1 j- #5 j+\n"
    ).arg(targetGate).arg(targetChan).arg(xJogSpeed);
}

QString PmacScriptGen::generateSphericalLatScript(int targetGate, int targetChan)
{
    return QString(
        "#5 j/\n"
        "Motor[5].ServoCtrl=1\n"
        "undefine all\n"
        "&1 #5->C\n"
        "Motor[5].JogSpeed=163840\n"
        "Gate3[%1].Chan[%2].Equ1Ena=0\n"
        "Gate3[%1].Chan[%2].EquOutPol=0\n"
        "Gate3[%1].Chan[%2].EquOutMask=1\n"
        "Gate3[%1].Chan[%2].EquWrite=1\n"
        "Gate3[%1].Chan[%2].CompAdd=40960\n"
        "Gate3[%1].Chan[%2].CompA=Gate3[%1].Chan[%2].ServoCapt+40960\n"
        "Gate3[%1].Chan[%2].CompB=Gate3[%1].Chan[%2].CompA+20480\n"
        "Gate3[%1].Chan[%2].Equ1Ena=1\n"
        "#5 j+\n"
    ).arg(targetGate).arg(targetChan);
}

QString PmacScriptGen::generateAsphericMeridianScript(
    double startRadiusUm, double scanRangeUm,
    double R_mm, double k,
    double A4, double A6, double A8, double A10, double A12,
    bool followZ,
    int targetGate, int targetChan)
{
    if (R_mm <= 0) R_mm = 100.0;
    if (scanRangeUm <= 0) scanRangeUm = 50000.0;

    double step_um = AsphericMath::calcSafeStepSize(
        startRadiusUm / 1000.0, R_mm, k, A4, A6, A8, A10, A12);

    if (followZ) {
        int num_points = (int)(scanRangeUm / step_um) + 1;
        if (num_points > 5000) num_points = 5000;

        double prev_z_um = AsphericMath::calcAsphericZ(
            startRadiusUm / 1000.0, R_mm, k, A4, A6, A8, A10, A12);

        QString script = "abort\n";
        script += "#1j/ #3j/\n";
        script += "Motor[1].ServoCtrl=1\n";
        script += "Motor[3].ServoCtrl=1\n";
        script += "undefine all\n";
        script += "&1 #1->X #3->Z\n";
        script += "Motor[1].JogSpeed=128000\n";
        script += "Motor[3].JogSpeed=128000\n";
        script += "X1=0 Z3=0\n";
        script += "IMOV\n";
        script += "Dwell=200\n";

        for (int i = 1; i < num_points; i++) {
            double r_um = startRadiusUm + i * step_um;
            double r_mm = r_um / 1000.0;
            double z_um = AsphericMath::calcAsphericZ(r_mm, R_mm, k, A4, A6, A8, A10, A12);
            double dz_um = z_um - prev_z_um;

            long dx_counts = (long)(step_um * 100000.0);
            long dz_counts = (long)(dz_um * 100000.0);

            script += QString("X1=%1 Z3=%2 IMOV Dwell=50\n").arg(dx_counts).arg(dz_counts);
            prev_z_um = z_um;
        }

        script += "abort\n";
        script += QString("Gate3[%1].Chan[%2].Equ1Ena=0\n").arg(targetGate).arg(targetChan);
        return script;
    } else {
        long compAdd = (long)(step_um * 100000.0);
        long compB = (long)(step_um * 50000.0);
        return QString(
            "abort\n"
            "#1 j/\n"
            "Motor[1].ServoCtrl=1\n"
            "undefine all\n"
            "&1 #1->X\n"
            "Motor[1].JogSpeed=128000\n"
            "Gate3[%1].Chan[%2].Equ1Ena=0\n"
            "Gate3[%1].Chan[%2].EquOutPol=0\n"
            "Gate3[%1].Chan[%2].EquOutMask=1\n"
            "Gate3[%1].Chan[%2].EquWrite=1\n"
            "Gate3[%1].Chan[%2].CompAdd=%3\n"
            "Gate3[%1].Chan[%2].CompA=Gate3[%1].Chan[%2].ServoCapt+%3\n"
            "Gate3[%1].Chan[%2].CompB=Gate3[%1].Chan[%2].CompA+%4\n"
            "Gate3[%1].Chan[%2].Equ1Ena=1\n"
            "#1 j+\n"
        ).arg(targetGate).arg(targetChan).arg(compAdd).arg(compB);
    }
}

QString PmacScriptGen::emergencyStop()
{
    return QString(
        "#1j/ #2j/ #3j/ #4j/ #5j/\n"
        "Gate3[0].Chan[0].Equ1Ena=0\n"
        "Gate3[1].Chan[0].Equ1Ena=0\n"
    );
}
