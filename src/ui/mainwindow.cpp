#include "mainwindow.h"

#include <QCheckBox>
#include <QCoreApplication>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <functional>
#include <QLabel>
#include <QComboBox>
#include <QProgressDialog>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    QWidget* root = new QWidget(this);
    QHBoxLayout* rootLayout = new QHBoxLayout(root);

    ogl_ = new OpenGLWidget(root);
    rootLayout->addWidget(ogl_, 5);

    QScrollArea* sideScroll = new QScrollArea(root);
    sideScroll->setWidgetResizable(true);
    sideScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    rootLayout->addWidget(sideScroll, 2);

    QWidget* side = new QWidget(sideScroll);
    QVBoxLayout* sideLayout = new QVBoxLayout(side);
    sideScroll->setWidget(side);

    QLabel* title = new QLabel("工艺栏", side);
    QFont f = title->font();
    f.setPointSize(14);
    title->setFont(f);
    sideLayout->addWidget(title);

    QGroupBox* preGroup = new QGroupBox("模型预处理", side);
    QHBoxLayout* preHBox = new QHBoxLayout(preGroup);
    QPushButton* preRotX = new QPushButton("绕 X 逆时针 90°", preGroup);
    QPushButton* preRotY = new QPushButton("绕 Y 逆时针 90°", preGroup);
    QPushButton* preRotZ = new QPushButton("绕 Z 逆时针 90°", preGroup);
    preRotX->setToolTip("右手系：拇指指向轴正方向，四指弯曲为旋转正方向。绕世界坐标 X 轴旋转 90°，并重新切片/规划。");
    preRotY->setToolTip("绕世界坐标 Y 轴旋转 90°，并重新切片/规划。");
    preRotZ->setToolTip("绕世界坐标 Z 轴旋转 90°，并重新切片/规划。");
    preHBox->addWidget(preRotX);
    preHBox->addWidget(preRotY);
    preHBox->addWidget(preRotZ);
    sideLayout->addWidget(preGroup);

    QGroupBox* modelPickGroup = new QGroupBox("当前加载模型", side);
    QFormLayout* modelPickForm = new QFormLayout(modelPickGroup);
    QComboBox* modelPresetCombo = new QComboBox(side);
    modelPresetCombo->addItem(QStringLiteral("hollowCylinder"),
                              QStringLiteral("../assets/models/hollowCylinder/HOLLOW_CYLINDER.STL"));
    modelPresetCombo->addItem(QStringLiteral("showerDoorPart"),
                              QStringLiteral("../assets/models/showerDoorPar/shower_part_b.stl"));
    modelPresetCombo->addItem(QStringLiteral("backpack"),
                              QStringLiteral("../assets/models/backpack/backpack.obj"));
    modelPresetCombo->addItem(QStringLiteral("dog"),
                              QStringLiteral("../assets/models/dog/dog.obj"));
    modelPresetCombo->setCurrentIndex(0);
    modelPickForm->addRow(QStringLiteral("预设模型"), modelPresetCombo);
    sideLayout->addWidget(modelPickGroup);

    QCheckBox* showContours = new QCheckBox("显示切片轮廓", side);
    showContours->setChecked(true);
    QCheckBox* showModel = new QCheckBox("显示模型实体", side);
    showModel->setChecked(true);
    QCheckBox* showModelTriWire = new QCheckBox("显示原始模型三角面片（线框）", side);
    showModelTriWire->setChecked(false);
    showModelTriWire->setToolTip("在三角网格上以线框方式显示每条边，可与实体着色同时开启；仅线框时可关闭「显示模型实体」。");
    QCheckBox* showOverhang = new QCheckBox("显示悬垂高亮", side);
    showOverhang->setChecked(true);
    QCheckBox* showSupport = new QCheckBox("显示支撑结构", side);
    showSupport->setChecked(true);

    QFormLayout* form = new QFormLayout();
    UiProcessParams p = ogl_->params();
    QSpinBox* layerCount = new QSpinBox(side);
    layerCount->setRange(10, 1000);
    layerCount->setValue(p.layerCount);
    QCheckBox* useAdaptiveLayering = new QCheckBox("使用自适应分层", side);
    useAdaptiveLayering->setChecked(p.useAdaptiveLayering);
    QDoubleSpinBox* adaptiveMinLayerHeight = new QDoubleSpinBox(side);
    adaptiveMinLayerHeight->setRange(0.001, 1.0);
    adaptiveMinLayerHeight->setDecimals(3);
    adaptiveMinLayerHeight->setValue(p.adaptiveMinLayerHeight);
    QDoubleSpinBox* adaptiveMaxLayerHeight = new QDoubleSpinBox(side);
    adaptiveMaxLayerHeight->setRange(0.001, 2.0);
    adaptiveMaxLayerHeight->setDecimals(3);
    adaptiveMaxLayerHeight->setValue(p.adaptiveMaxLayerHeight);
    QDoubleSpinBox* adaptiveLowCurv = new QDoubleSpinBox(side);
    adaptiveLowCurv->setRange(0.0, 89.0);
    adaptiveLowCurv->setDecimals(1);
    adaptiveLowCurv->setValue(p.adaptiveLowCurvatureDeg);
    QDoubleSpinBox* adaptiveHighCurv = new QDoubleSpinBox(side);
    adaptiveHighCurv->setRange(0.0, 89.0);
    adaptiveHighCurv->setDecimals(1);
    adaptiveHighCurv->setValue(p.adaptiveHighCurvatureDeg);
    QDoubleSpinBox* adaptiveSmoothAlpha = new QDoubleSpinBox(side);
    adaptiveSmoothAlpha->setRange(0.0, 1.0);
    adaptiveSmoothAlpha->setDecimals(2);
    adaptiveSmoothAlpha->setSingleStep(0.05);
    adaptiveSmoothAlpha->setValue(p.adaptiveSmoothAlpha);
    QCheckBox* useOctreePruneSlicing = new QCheckBox("使用八叉树剪枝切片", side);
    useOctreePruneSlicing->setChecked(p.useOctreePruneSlicing);
    QCheckBox* useTopologyStitch = new QCheckBox("使用拓扑加速拼环", side);
    useTopologyStitch->setChecked(p.useTopologyStitch);
    QDoubleSpinBox* overhangAngle = new QDoubleSpinBox(side);
    overhangAngle->setRange(1.0, 89.0);
    overhangAngle->setValue(p.overhangAngleDeg);
    QDoubleSpinBox* overhangVerticalWallSlack = new QDoubleSpinBox(side);
    overhangVerticalWallSlack->setRange(0.0, 45.0);
    overhangVerticalWallSlack->setDecimals(1);
    overhangVerticalWallSlack->setToolTip(
        "竖壁：法向几乎垂直堆积方向时不判悬垂。侧面竖棱因网格噪声略朝下时被误标时可增大；0 关闭。");
    overhangVerticalWallSlack->setValue(p.overhangVerticalWallSlackDeg);
    QCheckBox* enableOverhangSecondPass = new QCheckBox("启用悬垂二次验证", side);
    enableOverhangSecondPass->setChecked(p.enableOverhangSecondPass);
    QDoubleSpinBox* supportRadius = new QDoubleSpinBox(side);
    supportRadius->setRange(0.0005, 1.0);
    supportRadius->setDecimals(4);
    supportRadius->setValue(p.supportRadius);
    QDoubleSpinBox* supportSpacing = new QDoubleSpinBox(side);
    supportSpacing->setRange(0.005, 10.0);
    supportSpacing->setDecimals(3);
    supportSpacing->setValue(p.supportSpacing);
    QDoubleSpinBox* supportTipInset = new QDoubleSpinBox(side);
    supportTipInset->setRange(0.0, 1.0);
    supportTipInset->setDecimals(4);
    supportTipInset->setValue(p.supportTipInset);
    QSpinBox* supportMax = new QSpinBox(side);
    supportMax->setRange(1, 20000);
    supportMax->setValue(p.supportMax);
    QDoubleSpinBox* infillSpacing = new QDoubleSpinBox(side);
    infillSpacing->setRange(0.002, 5.0);
    infillSpacing->setDecimals(3);
    infillSpacing->setValue(p.infillSpacing);
    QComboBox* infillPattern = new QComboBox(side);
    infillPattern->addItem("平行线填充", 0);
    infillPattern->addItem("Z型填充", 1);
    infillPattern->setCurrentIndex((p.infillPattern == 1) ? 1 : 0);
    QDoubleSpinBox* playbackSpeed = new QDoubleSpinBox(side);
    playbackSpeed->setRange(0.1, 20.0);
    playbackSpeed->setDecimals(2);
    playbackSpeed->setValue(p.playbackSpeed);
    QDoubleSpinBox* cameraFlySpeed = new QDoubleSpinBox(side);
    cameraFlySpeed->setRange(0.1, 50.0);
    cameraFlySpeed->setDecimals(2);
    cameraFlySpeed->setSingleStep(0.1);
    cameraFlySpeed->setValue(p.cameraFlySpeed);
    cameraFlySpeed->setToolTip("按住右键飞行视口时，WASD 与滚轮相对默认速度的倍率；默认 1.0 与改前一致。");
    QDoubleSpinBox* nozzleDiameter = new QDoubleSpinBox(side);
    nozzleDiameter->setRange(0.05, 2.0);
    nozzleDiameter->setDecimals(3);
    nozzleDiameter->setValue(p.nozzleDiameter);
    QDoubleSpinBox* physicalLayerHeight = new QDoubleSpinBox(side);
    physicalLayerHeight->setRange(0.01, 1.0);
    physicalLayerHeight->setDecimals(3);
    physicalLayerHeight->setValue(p.physicalLayerHeight);
    QDoubleSpinBox* extrusionFlowRate = new QDoubleSpinBox(side);
    extrusionFlowRate->setRange(0.01, 30.0);
    extrusionFlowRate->setDecimals(3);
    extrusionFlowRate->setValue(p.extrusionFlowRate);
    QDoubleSpinBox* travelSpeed = new QDoubleSpinBox(side);
    travelSpeed->setRange(0.1, 300.0);
    travelSpeed->setDecimals(2);
    travelSpeed->setValue(p.travelSpeed);
    QDoubleSpinBox* beadWidthScale = new QDoubleSpinBox(side);
    beadWidthScale->setRange(0.01, 5.0);
    beadWidthScale->setDecimals(3);
    beadWidthScale->setValue(p.beadWidthScale);
    QDoubleSpinBox* beadRenderScale = new QDoubleSpinBox(side);
    beadRenderScale->setRange(0.0001, 1.0);
    beadRenderScale->setDecimals(4);
    beadRenderScale->setSingleStep(0.005);
    beadRenderScale->setValue(p.beadRenderScale);
    QDoubleSpinBox* minBeadWidth = new QDoubleSpinBox(side);
    minBeadWidth->setRange(0.0001, 1.0);
    minBeadWidth->setDecimals(4);
    minBeadWidth->setValue(p.minBeadWidth);
    QDoubleSpinBox* maxBeadWidth = new QDoubleSpinBox(side);
    maxBeadWidth->setRange(0.0001, 2.0);
    maxBeadWidth->setDecimals(4);
    maxBeadWidth->setValue(p.maxBeadWidth);
    QCheckBox* enableLayerSink = new QCheckBox("启用物理沉积", side);
    enableLayerSink->setChecked(p.enableLayerSink);
    QDoubleSpinBox* layerSinkRatio = new QDoubleSpinBox(side);
    layerSinkRatio->setRange(0.0, 0.50);
    layerSinkRatio->setDecimals(3);
    layerSinkRatio->setSingleStep(0.01);
    layerSinkRatio->setValue(p.layerSinkRatio);

    QGroupBox* sliceGroup = new QGroupBox("切片设置", side);
    QVBoxLayout* sliceLayout = new QVBoxLayout(sliceGroup);
    QFormLayout* sliceForm = new QFormLayout();
    sliceForm->addRow("分层数", layerCount);
    sliceForm->addRow("最小层高", adaptiveMinLayerHeight);
    sliceForm->addRow("最大层高", adaptiveMaxLayerHeight);
    sliceForm->addRow("低曲率阈值(°)", adaptiveLowCurv);
    sliceForm->addRow("高曲率阈值(°)", adaptiveHighCurv);
    sliceForm->addRow("层高平滑α", adaptiveSmoothAlpha);
    sliceLayout->addLayout(sliceForm);
    sliceLayout->addWidget(showContours);
    sliceLayout->addWidget(useAdaptiveLayering);
    sliceLayout->addWidget(useOctreePruneSlicing);
    sliceLayout->addWidget(useTopologyStitch);
    sideLayout->addWidget(sliceGroup);

    QGroupBox* processGroup = new QGroupBox("工艺参数", side);
    QFormLayout* processForm = new QFormLayout(processGroup);
    processForm->addRow("悬垂角阈值", overhangAngle);
    processForm->addRow("竖壁容差(°)", overhangVerticalWallSlack);
    processForm->addRow(enableOverhangSecondPass);
    processForm->addRow("支撑半径", supportRadius);
    processForm->addRow("支撑间距", supportSpacing);
    processForm->addRow("支撑顶端内缩", supportTipInset);
    processForm->addRow("支撑数量上限", supportMax);
    processForm->addRow("填充方式", infillPattern);
    processForm->addRow("填充间距", infillSpacing);
    sideLayout->addWidget(processGroup);

    QGroupBox* displayGroup = new QGroupBox("显示与视口", side);
    QVBoxLayout* displayLayout = new QVBoxLayout(displayGroup);
    displayLayout->addWidget(showModel);
    displayLayout->addWidget(showModelTriWire);
    displayLayout->addWidget(showOverhang);
    displayLayout->addWidget(showSupport);
    QFormLayout* displayForm = new QFormLayout();
    displayForm->addRow("飞行相机速度(倍率)", cameraFlySpeed);
    displayLayout->addLayout(displayForm);
    sideLayout->addWidget(displayGroup);

    QGroupBox* simGroup = new QGroupBox("模拟设置", side);
    QFormLayout* simForm = new QFormLayout(simGroup);
    simForm->addRow("模拟速度", playbackSpeed);
    simForm->addRow("喷嘴直径", nozzleDiameter);
    simForm->addRow("物理层高", physicalLayerHeight);
    simForm->addRow("体积流率Q", extrusionFlowRate);
    simForm->addRow("喷头速度v", travelSpeed);
    simForm->addRow("宽度系数", beadWidthScale);
    simForm->addRow("物理显示缩放", beadRenderScale);
    simForm->addRow("最小线宽", minBeadWidth);
    simForm->addRow("最大线宽", maxBeadWidth);
    simForm->addRow(enableLayerSink);
    simForm->addRow("物理沉积下陷比例", layerSinkRatio);
    sideLayout->addWidget(simGroup);

    QGroupBox* statsGroup = new QGroupBox("统计信息", side);
    QFormLayout* statsForm = new QFormLayout(statsGroup);
    QLabel* modelFaceCountValue = new QLabel("-", statsGroup);
    QLabel* sliceLayerCountValue = new QLabel("-", statsGroup);
    QLabel* processTimeValue = new QLabel("-", statsGroup);
    QLabel* modelLoadTimeValue = new QLabel("-", statsGroup);
    QLabel* sliceTimeValue = new QLabel("-", statsGroup);
    QLabel* sliceIntersectTimeValue = new QLabel("-", statsGroup);
    QLabel* sliceContourTimeValue = new QLabel("-", statsGroup);
    QLabel* overhangTimeValue = new QLabel("-", statsGroup);
    QLabel* supportTimeValue = new QLabel("-", statsGroup);
    QLabel* planningTimeValue = new QLabel("-", statsGroup);
    QLabel* uploadTimeValue = new QLabel("-", statsGroup);
    statsForm->addRow("模型面数", modelFaceCountValue);
    statsForm->addRow("实际分层数", sliceLayerCountValue);
    statsForm->addRow("总耗时(ms)", processTimeValue);
    statsForm->addRow("模型加载(ms)", modelLoadTimeValue);
    statsForm->addRow("切片(ms)", sliceTimeValue);
    statsForm->addRow("切片·求交(ms)", sliceIntersectTimeValue);
    statsForm->addRow("切片·轮廓提取(ms)", sliceContourTimeValue);
    statsForm->addRow("悬垂检测(ms)", overhangTimeValue);
    statsForm->addRow("支撑生成/切片(ms)", supportTimeValue);
    statsForm->addRow("填充/路径规划(ms)", planningTimeValue);
    statsForm->addRow("几何上传(ms)", uploadTimeValue);
    sideLayout->addWidget(statsGroup);

    QPushButton* applyBtn = new QPushButton("应用参数并重算", side);
    QPushButton* startBtn = new QPushButton("开始模拟", side);
    QPushButton* startPhysicalBtn = new QPushButton("进行物理模拟", side);
    QPushButton* finalPhysicalBtn = new QPushButton("查看物理模拟最终结果", side);
    QPushButton* finalSimBtn = new QPushButton("显示最终模拟结果", side);
    QPushButton* pauseBtn = new QPushButton("暂停/开始模拟", side);
    QPushButton* exitBtn = new QPushButton("退出打印", side);
    pauseBtn->setEnabled(false);
    exitBtn->setEnabled(false);
    sideLayout->addWidget(applyBtn);
    sideLayout->addWidget(startBtn);
    sideLayout->addWidget(startPhysicalBtn);
    sideLayout->addWidget(finalPhysicalBtn);
    sideLayout->addWidget(finalSimBtn);
    sideLayout->addWidget(pauseBtn);
    sideLayout->addWidget(exitBtn);
    sideLayout->addStretch();

    setCentralWidget(root);
    setWindowTitle("增材可视化系统");
    resize(1400, 900);

    auto applyAndProcess = [=]() {
        UiProcessParams pp = ogl_->params();
        pp.layerCount = layerCount->value();
        pp.useAdaptiveLayering = useAdaptiveLayering->isChecked();
        pp.adaptiveMinLayerHeight = static_cast<float>(adaptiveMinLayerHeight->value());
        pp.adaptiveMaxLayerHeight = static_cast<float>(adaptiveMaxLayerHeight->value());
        pp.adaptiveLowCurvatureDeg = static_cast<float>(adaptiveLowCurv->value());
        pp.adaptiveHighCurvatureDeg = static_cast<float>(adaptiveHighCurv->value());
        pp.adaptiveSmoothAlpha = static_cast<float>(adaptiveSmoothAlpha->value());
        pp.useOctreePruneSlicing = useOctreePruneSlicing->isChecked();
        pp.useTopologyStitch = useTopologyStitch->isChecked();
        pp.enablePhysicalDeposition = false;
        pp.overhangAngleDeg = static_cast<float>(overhangAngle->value());
        pp.overhangVerticalWallSlackDeg = static_cast<float>(overhangVerticalWallSlack->value());
        pp.enableOverhangSecondPass = enableOverhangSecondPass->isChecked();
        pp.supportRadius = static_cast<float>(supportRadius->value());
        pp.supportSpacing = static_cast<float>(supportSpacing->value());
        pp.supportTipInset = static_cast<float>(supportTipInset->value());
        pp.supportMax = supportMax->value();
        pp.infillPattern = infillPattern->currentData().toInt();
        pp.infillSpacing = static_cast<float>(infillSpacing->value());
        pp.playbackSpeed = static_cast<float>(playbackSpeed->value());
        pp.cameraFlySpeed = static_cast<float>(cameraFlySpeed->value());
        pp.nozzleDiameter = static_cast<float>(nozzleDiameter->value());
        pp.physicalLayerHeight = static_cast<float>(physicalLayerHeight->value());
        pp.extrusionFlowRate = static_cast<float>(extrusionFlowRate->value());
        pp.travelSpeed = static_cast<float>(travelSpeed->value());
        pp.beadWidthScale = static_cast<float>(beadWidthScale->value());
        pp.beadRenderScale = static_cast<float>(beadRenderScale->value());
        pp.minBeadWidth = static_cast<float>(minBeadWidth->value());
        pp.maxBeadWidth = static_cast<float>(maxBeadWidth->value());
        pp.enableLayerSink = enableLayerSink->isChecked();
        pp.layerSinkRatio = static_cast<float>(layerSinkRatio->value());
        ogl_->setParams(pp);

        QProgressDialog dlg("模型处理中...", QString(), 0, 0, this);
        dlg.setWindowModality(Qt::ApplicationModal);
        dlg.setCancelButton(nullptr);
        dlg.show();
        QCoreApplication::processEvents();
        ogl_->processPipeline();
        dlg.close();
    };

    auto runPreprocessWithProgress = [=](const std::function<void()>& fn) {
        QProgressDialog dlg("模型处理中...", QString(), 0, 0, this);
        dlg.setWindowModality(Qt::ApplicationModal);
        dlg.setCancelButton(nullptr);
        dlg.show();
        QCoreApplication::processEvents();
        fn();
        dlg.close();
    };

    auto refreshStats = [=]() {
        modelFaceCountValue->setText(QString::number(static_cast<qulonglong>(ogl_->modelFaceCount())));
        sliceLayerCountValue->setText(QString::number(ogl_->lastSliceLayerCount()));
        const ProcessTimingStats ts = ogl_->processTimingStats();
        processTimeValue->setText(QString::number(ts.totalMs, 'f', 2));
        modelLoadTimeValue->setText(QString::number(ts.modelLoadMs, 'f', 2));
        sliceTimeValue->setText(QString::number(ts.slicingMs, 'f', 2));
        sliceIntersectTimeValue->setText(QString::number(ts.slicingIntersectMs, 'f', 2));
        sliceContourTimeValue->setText(QString::number(ts.slicingContourMs, 'f', 2));
        overhangTimeValue->setText(QString::number(ts.overhangMs, 'f', 2));
        supportTimeValue->setText(QString::number(ts.supportMs, 'f', 2));
        planningTimeValue->setText(QString::number(ts.planningMs, 'f', 2));
        uploadTimeValue->setText(QString::number(ts.uploadMs, 'f', 2));
    };

    connect(showContours, &QCheckBox::toggled, ogl_, &OpenGLWidget::setShowContours);
    connect(showModel, &QCheckBox::toggled, ogl_, &OpenGLWidget::setShowModel);
    connect(showModelTriWire, &QCheckBox::toggled, ogl_, &OpenGLWidget::setShowModelTriangleWire);
    connect(showOverhang, &QCheckBox::toggled, ogl_, &OpenGLWidget::setShowOverhang);
    connect(showSupport, &QCheckBox::toggled, ogl_, &OpenGLWidget::setShowSupport);
    connect(cameraFlySpeed, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double v) {
        UiProcessParams pp = ogl_->params();
        pp.cameraFlySpeed = static_cast<float>(v);
        ogl_->setParams(pp);
    });

    connect(preRotX, &QPushButton::clicked, this, [=]() {
        runPreprocessWithProgress([&]() { ogl_->rotateModelPreprocessWorldCCW90(0); });
    });
    connect(preRotY, &QPushButton::clicked, this, [=]() {
        runPreprocessWithProgress([&]() { ogl_->rotateModelPreprocessWorldCCW90(1); });
    });
    connect(preRotZ, &QPushButton::clicked, this, [=]() {
        runPreprocessWithProgress([&]() { ogl_->rotateModelPreprocessWorldCCW90(2); });
    });
    connect(modelPresetCombo, QOverload<int>::of(&QComboBox::activated), this, [=](int index) {
        const QString path = modelPresetCombo->itemData(index).toString();
        if (path.isEmpty())
            return;
        QProgressDialog dlg(QStringLiteral("模型加载中"), QString(), 0, 0, this);
        dlg.setWindowModality(Qt::ApplicationModal);
        dlg.setCancelButton(nullptr);
        dlg.show();
        QCoreApplication::processEvents();
        ogl_->loadModelFromAssetPath(path.toStdString());
        dlg.close();
    });
    connect(applyBtn, &QPushButton::clicked, this, applyAndProcess);
    connect(ogl_, &OpenGLWidget::processingFinished, this, refreshStats);
    connect(useAdaptiveLayering, &QCheckBox::toggled, this, [=](bool on) {
        layerCount->setEnabled(!on);
        adaptiveMinLayerHeight->setEnabled(on);
        adaptiveMaxLayerHeight->setEnabled(on);
        adaptiveLowCurv->setEnabled(on);
        adaptiveHighCurv->setEnabled(on);
        adaptiveSmoothAlpha->setEnabled(on);
    });
    {
        const bool on = useAdaptiveLayering->isChecked();
        layerCount->setEnabled(!on);
        adaptiveMinLayerHeight->setEnabled(on);
        adaptiveMaxLayerHeight->setEnabled(on);
        adaptiveLowCurv->setEnabled(on);
        adaptiveHighCurv->setEnabled(on);
        adaptiveSmoothAlpha->setEnabled(on);
    }

    connect(startBtn, &QPushButton::clicked, this, [=]() {
        UiProcessParams pp = ogl_->params();
        pp.enablePhysicalDeposition = false;
        ogl_->setParams(pp);
        ogl_->startSimulation();
        pauseBtn->setEnabled(true);
        exitBtn->setEnabled(true);
    });
    connect(startPhysicalBtn, &QPushButton::clicked, this, [=]() {
        UiProcessParams pp = ogl_->params();
        pp.enablePhysicalDeposition = true;
        ogl_->setParams(pp);
        ogl_->startSimulation();
        pauseBtn->setEnabled(true);
        exitBtn->setEnabled(true);
    });
    connect(finalPhysicalBtn, &QPushButton::clicked, this, [=]() {
        UiProcessParams pp = ogl_->params();
        pp.enablePhysicalDeposition = true;
        ogl_->setParams(pp);
        ogl_->showPhysicalSimulationFinalResult();
        pauseBtn->setEnabled(false);
        exitBtn->setEnabled(true);
    });
    connect(finalSimBtn, &QPushButton::clicked, this, [=]() {
        UiProcessParams pp = ogl_->params();
        pp.enablePhysicalDeposition = false;
        ogl_->setParams(pp);
        ogl_->showNonPhysicalSimulationFinalResult();
        pauseBtn->setEnabled(false);
        exitBtn->setEnabled(true);
    });
    connect(pauseBtn, &QPushButton::clicked, this, [=]() { ogl_->pauseSimulation(); });
    connect(exitBtn, &QPushButton::clicked, this, [=]() {
        ogl_->stopSimulation();
        pauseBtn->setEnabled(false);
        exitBtn->setEnabled(false);
    });

    // 启动后直接处理当前模型
    applyAndProcess();
}

