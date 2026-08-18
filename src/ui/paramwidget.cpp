#include "paramwidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>
#include <QTimer> // Added for QTimer

ParamEditDialog::ParamEditDialog(const DiskParams& params, QWidget *parent) : QDialog(parent) {
    setWindowTitle("修改参数");
    QFormLayout *form = new QFormLayout(this);
    
    // 使用QDoubleSpinBox和QSpinBox替代QLineEdit，提供更好的输入验证
    // 线宽 - 范围0.001-0.5，放宽上限
    lineWidthSpin = new QDoubleSpinBox();
    lineWidthSpin->setRange(0.001, 0.5);
    lineWidthSpin->setSingleStep(0.001);
    lineWidthSpin->setDecimals(3);
    lineWidthSpin->setValue(params.lineWidth);
    
    // 层数 - 范围1-200，大幅放宽上限
    layerNumSpin = new QSpinBox();
    layerNumSpin->setRange(1, 200);
    layerNumSpin->setValue(params.layerNum);
    
    // 层高增量 - 范围0.001-0.5，放宽上限
    heightIncSpin = new QDoubleSpinBox();
    heightIncSpin->setRange(0.001, 0.5);
    heightIncSpin->setSingleStep(0.001);
    heightIncSpin->setDecimals(3);
    heightIncSpin->setValue(params.heightIncrement);
    
    // 基础半径 - 范围0.1-20，放宽上限
    baseRadiusSpin = new QDoubleSpinBox();
    baseRadiusSpin->setRange(0.1, 20.0);
    baseRadiusSpin->setSingleStep(0.1);
    baseRadiusSpin->setDecimals(2);
    baseRadiusSpin->setValue(params.baseRadius);
    
    // 半径增量 - 范围-2.0-2.0，放宽范围
    radiusDeltaSpin = new QDoubleSpinBox();
    radiusDeltaSpin->setRange(-2.0, 2.0);
    radiusDeltaSpin->setSingleStep(0.01);
    radiusDeltaSpin->setDecimals(3);
    radiusDeltaSpin->setValue(params.radiusDelta);
    
    // 填充步长 - 范围0.01-2.0，放宽上限
    fillStepSpin = new QDoubleSpinBox();
    fillStepSpin->setRange(0.01, 2.0);
    fillStepSpin->setSingleStep(0.01);
    fillStepSpin->setDecimals(2);
    fillStepSpin->setValue(params.fillStep);
    
    form->addRow("线宽", lineWidthSpin);
    form->addRow("层数", layerNumSpin);
    form->addRow("层高增量", heightIncSpin);
    form->addRow("基础半径", baseRadiusSpin);
    form->addRow("半径增量", radiusDeltaSpin);
    form->addRow("填充步长", fillStepSpin);
    
    // 添加提示信息
    QLabel *infoLabel = new QLabel("请注意：参数值过大可能导致渲染问题");
    infoLabel->setStyleSheet("color: red;");
    form->addRow(infoLabel);
    
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Cancel);
    QPushButton *applyBtn = new QPushButton("应用参数");
    buttonBox->addButton(applyBtn, QDialogButtonBox::AcceptRole);
    
    // 修改：应用按钮同时发出paramsChanged信号
    connect(applyBtn, &QPushButton::clicked, [this]() {
        // 获取当前参数并发出信号
        DiskParams params = getParams();
        emit paramsChanged(params);
        accept(); // 关闭对话框
    });
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    
    form->addRow(buttonBox);
    
    // 实时检查参数组合的有效性
    auto validateParams = [this]() {
        // 检查半径增量，确保最后一层半径大于0
        float lastLayerRadius = baseRadiusSpin->value() + 
                               (layerNumSpin->value() - 1) * radiusDeltaSpin->value();
        if (lastLayerRadius <= 0) {
            radiusDeltaSpin->setStyleSheet("background-color: #ffcccc;");
        } else {
            radiusDeltaSpin->setStyleSheet("");
        }
    };
    
    // 连接信号以实时验证
    connect(baseRadiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), validateParams);
    connect(layerNumSpin, QOverload<int>::of(&QSpinBox::valueChanged), validateParams);
    connect(radiusDeltaSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), validateParams);
    
    // 初始验证
    validateParams();
}

DiskParams ParamEditDialog::getParams() const {
    DiskParams p;
    
    p.lineWidth = lineWidthSpin->value();
    p.layerNum = layerNumSpin->value();
    p.heightIncrement = heightIncSpin->value();
    p.baseRadius = baseRadiusSpin->value();
    p.radiusDelta = radiusDeltaSpin->value();
    p.fillStep = fillStepSpin->value();
    
    // 确保最后一层半径大于0
    float lastLayerRadius = p.baseRadius + (p.layerNum - 1) * p.radiusDelta;
    if (p.radiusDelta < 0 && lastLayerRadius <= 0) {
        p.radiusDelta = -p.baseRadius / p.layerNum + 0.001f;
    }
    
    return p;
}

ParamWidget::ParamWidget(QWidget *parent) : QWidget(parent) {
    formLayout = new QFormLayout(this);
    
    // 初始化当前参数
    currentParams = DiskParams();
    
    // 创建只读标签显示参数
    lineWidthLabel = new QLabel(QString::number(currentParams.lineWidth));
    layerNumLabel = new QLabel(QString::number(currentParams.layerNum));
    heightIncLabel = new QLabel(QString::number(currentParams.heightIncrement));
    baseRadiusLabel = new QLabel(QString::number(currentParams.baseRadius));
    radiusDeltaLabel = new QLabel(QString::number(currentParams.radiusDelta));
    fillStepLabel = new QLabel(QString::number(currentParams.fillStep));
    
    // 添加到表单布局
    formLayout->addRow("线宽:", lineWidthLabel);
    formLayout->addRow("层数:", layerNumLabel);
    formLayout->addRow("层高增量:", heightIncLabel);
    formLayout->addRow("基础半径:", baseRadiusLabel);
    formLayout->addRow("半径增量:", radiusDeltaLabel);
    formLayout->addRow("填充步长:", fillStepLabel);
    
    // 添加操作按钮
    QPushButton *editBtn = new QPushButton("修改参数");
    QPushButton *enterSimBtn = new QPushButton("进入模拟");
    
    QHBoxLayout *btnLayout = new QHBoxLayout();
    btnLayout->addWidget(editBtn);
    btnLayout->addWidget(enterSimBtn);
    formLayout->addRow(btnLayout);
    
    // 连接信号
    connect(editBtn, &QPushButton::clicked, [this]() {
        ParamEditDialog dialog(currentParams, this);
        
        // 只需要连接一次信号，这里是为了立即更新界面显示
        connect(&dialog, &ParamEditDialog::paramsChanged, [this](DiskParams params) {
            currentParams = params;
            updateParamDisplay(params);
            // 延迟发送参数变更信号，避免对话框关闭过程中的并发问题
            QTimer::singleShot(100, this, [this, params]() {
                emit paramsChanged(params);
            });
        });
        
        // 执行对话框
        dialog.exec();
        
        // 不需要在这里再次更新参数，因为已经在paramsChanged信号处理中完成了
    });
    
    connect(enterSimBtn, &QPushButton::clicked, [this]() {
        emit enterSimRequested();
    });
}

void ParamWidget::updateParamDisplay(const DiskParams& params) {
    currentParams = params;
    lineWidthLabel->setText(QString::number(params.lineWidth));
    layerNumLabel->setText(QString::number(params.layerNum));
    heightIncLabel->setText(QString::number(params.heightIncrement));
    baseRadiusLabel->setText(QString::number(params.baseRadius));
    radiusDeltaLabel->setText(QString::number(params.radiusDelta));
    fillStepLabel->setText(QString::number(params.fillStep));
} 