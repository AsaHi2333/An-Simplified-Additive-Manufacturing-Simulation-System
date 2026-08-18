#pragma once
#include <QWidget>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QSpinBox>

struct DiskParams {
    float lineWidth = 0.01f;
    int layerNum = 10;
    float heightIncrement = 0.02f;
    float baseRadius = 3.0f;
    float radiusDelta = -0.3f;
    float fillStep = 0.05f;
};

class ParamEditDialog : public QDialog {
    Q_OBJECT
public:
    ParamEditDialog(const DiskParams& params, QWidget *parent = nullptr);
    DiskParams getParams() const;

private:
    QDoubleSpinBox *lineWidthSpin;
    QSpinBox *layerNumSpin;
    QDoubleSpinBox *heightIncSpin;
    QDoubleSpinBox *baseRadiusSpin;
    QDoubleSpinBox *radiusDeltaSpin;
    QDoubleSpinBox *fillStepSpin;

signals:
    void paramsChanged(DiskParams params);
};

class ParamWidget : public QWidget {
    Q_OBJECT
public:
    ParamWidget(QWidget *parent = nullptr);
    void updateParamDisplay(const DiskParams& params);

private:
    QFormLayout *formLayout;
    DiskParams currentParams;
    QLabel *lineWidthLabel, *layerNumLabel, *heightIncLabel;
    QLabel *baseRadiusLabel, *radiusDeltaLabel, *fillStepLabel;

signals:
    void paramsChanged(DiskParams params);
    void enterSimRequested();
}; 