// ============================================================================
// AxisConfig.h
// 轴映射配置 — 逻辑轴 → PMAC 电机号 + 触发通道
// ============================================================================
#pragma once
#include <QSettings>

struct AxisConfig {
    int motorX = 1;
    int motorY = 2;
    int motorZ = 3;
    int motorB = 4;
    int motorC = 5;
    int triggerGate = 1;
    int triggerChan = 0;

    void save(QSettings& s) const
    {
        s.beginGroup("Axis");
        s.setValue("motorX", motorX);
        s.setValue("motorY", motorY);
        s.setValue("motorZ", motorZ);
        s.setValue("motorB", motorB);
        s.setValue("motorC", motorC);
        s.setValue("triggerGate", triggerGate);
        s.setValue("triggerChan", triggerChan);
        s.endGroup();
    }

    void load(QSettings& s)
    {
        s.beginGroup("Axis");
        motorX = s.value("motorX", 1).toInt();
        motorY = s.value("motorY", 2).toInt();
        motorZ = s.value("motorZ", 3).toInt();
        motorB = s.value("motorB", 4).toInt();
        motorC = s.value("motorC", 5).toInt();
        triggerGate = s.value("triggerGate", 1).toInt();
        triggerChan = s.value("triggerChan", 0).toInt();
        s.endGroup();
    }
};
