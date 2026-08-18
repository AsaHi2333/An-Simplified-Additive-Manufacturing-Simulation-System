#pragma once

#include <QFocusEvent>
#include <QKeyEvent>
#include <QOpenGLWidget>
#include <QPoint>
#include <QTimer>
#include <chrono>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "dataStructure/Layer.h"
#include "process/PathOrderPlanner.h"

class Mesh;
class Model;
class Shader;
struct OverhangResult;

struct UiProcessParams
{
    int layerCount = 200;
    bool useAdaptiveLayering = false;
    float adaptiveMinLayerHeight = 0.02f;
    float adaptiveMaxLayerHeight = 0.10f;
    float adaptiveLowCurvatureDeg = 10.0f;
    float adaptiveHighCurvatureDeg = 30.0f;
    float adaptiveSmoothAlpha = 0.35f;
    bool useOctreePruneSlicing = false;
    bool useTopologyStitch = false;
    bool enablePhysicalDeposition = false;
    bool enableLayerSink = true;
    float contourLineWidth = 0.1f;
    float overhangAngleDeg = 45.0f;
    /// 竖壁容差(°)：越大越倾向把侧面竖结构排除在悬垂之外；0 关闭
    float overhangVerticalWallSlackDeg = 15.0f;
    bool enableOverhangSecondPass = true;
    float supportRadius = 0.005f;
    float supportSpacing = 0.20f;
    float supportTipInset = 0.0025f;
    int supportMax = 350;
    int infillPattern = 0; // 0: 平行线, 1: Z型
    float infillSpacing = 0.06f;
    float infillAngleDeg = 45.0f;
    float playbackSpeed = 1.2f;
    float nozzleDiameter = 0.4f;
    float physicalLayerHeight = 0.2f;
    float extrusionFlowRate = 2.4f;
    float travelSpeed = 40.0f;
    float beadWidthScale = 1.0f;
    float beadRenderScale = 0.02f;
    float minBeadWidth = 0.005f;
    float maxBeadWidth = 0.05f;
    float layerSinkRatio = 0.08f;
    /// 物理沉积：每段挤出圆柱的圆周离散段数（越大越圆滑，三角形越多）
    int physCylinderSegments = 12;
    /// 按住右键飞行视口时，WASD/滚轮移动速度倍率（在「与视距相关的基础速度」上乘以该值；默认 1 与旧版一致）
    float cameraFlySpeed = 1.0f;
};

struct ProcessTimingStats
{
    double totalMs = 0.0;
    double modelLoadMs = 0.0;
    double slicingMs = 0.0;
    /// 切片中求交（三角与切平面求线段）；八叉树剪枝主要影响此项
    double slicingIntersectMs = 0.0;
    /// 切片中轮廓提取（拼环 + 内外洞分类 + 转 Path）；拓扑拼环主要影响此项
    double slicingContourMs = 0.0;
    double overhangMs = 0.0;
    double supportMs = 0.0;
    double planningMs = 0.0;
    double uploadMs = 0.0;
};

class OpenGLWidget : public QOpenGLWidget
{
    Q_OBJECT
public:
    struct BeamElement
    {
        glm::vec3 start = glm::vec3(0.0f);
        glm::vec3 end = glm::vec3(0.0f);
        float radius = 0.2f;
        float layerHeight = 0.2f;
    };

    struct LineBatch
    {
        unsigned int vao = 0;
        unsigned int vbo = 0;
        std::vector<float> verts;
        std::vector<int> first;
        std::vector<int> count;
    };

    explicit OpenGLWidget(QWidget* parent = nullptr);
    ~OpenGLWidget() override;

    UiProcessParams params() const { return params_; }
    size_t modelFaceCount() const { return modelFaceCount_; }
    int lastSliceLayerCount() const { return lastSliceLayerCount_; }
    ProcessTimingStats processTimingStats() const { return timingStats_; }
    void setParams(const UiProcessParams& p);

    void setShowContours(bool on);
    void setShowOverhang(bool on) { showOverhang_ = on; update(); }
    void setShowSupport(bool on) { showSupport_ = on; update(); }
    void setShowModel(bool on) { showModel_ = on; update(); }
    /// 以线框方式叠加绘制（或单独绘制）原始网格三角面片边
    void setShowModelTriangleWire(bool on) { showModelTriangleWire_ = on; update(); }

    void processPipeline();
    /// 设置模型文件路径（相对工作目录，与其它处 modelSourcePath_ 一致）并重新执行完整处理管线。
    void loadModelFromAssetPath(const std::string& path);
    /// 在世界坐标系下绕 X/Y/Z 轴逆时针（右手系正方向）旋转 90°，累积到预处理烘焙矩阵并重新跑切片等管线。axisIndex：0=X，1=Y，2=Z。
    void rotateModelPreprocessWorldCCW90(int axisIndex);

    void startSimulation();
    /// 非物理沉积：一次性生成与逐步模拟结束时相同的折线累积结果（黄线），不播放动画。
    void showNonPhysicalSimulationFinalResult();
    void showPhysicalSimulationFinalResult();
    void pauseSimulation();
    void stopSimulation();

signals:
    void processingStarted();
    void processingFinished();
    void simulationStateChanged(bool running);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void keyPressEvent(QKeyEvent* e) override;
    void keyReleaseEvent(QKeyEvent* e) override;
    void focusOutEvent(QFocusEvent* e) override;

private:
    bool glReady_ = false;
    bool pendingProcess_ = false;
    UiProcessParams params_;

    bool showContours_ = true;
    bool showInfill_ = true;
    bool showOverhang_ = true;
    bool showSupport_ = true;
    bool showModel_ = true;
    bool showModelTriangleWire_ = false;

    // Camera orbit（左键旋转 + 滚轮距离）；按住右键为类虚幻视口飞行（鼠标环视 + WASD）
    float yaw_ = -90.0f;
    float pitch_ = 15.0f;
    float distance_ = 6.0f;
    glm::vec3 cameraTarget_ = glm::vec3(0.0f, 0.5f, 0.0f);
    QPoint lastMouse_;

    bool flyRmbActive_ = false;
    glm::vec3 flyEye_{0.0f};
    float flyYaw_ = -90.0f;
    float flyPitch_ = 15.0f;
    bool flyKeyW_ = false;
    bool flyKeyA_ = false;
    bool flyKeyS_ = false;
    bool flyKeyD_ = false;
    std::chrono::steady_clock::time_point flyLastStep_{};

    glm::vec3 cameraDirFromYawPitch(float yawDeg, float pitchDeg) const;
    void beginRmbFlyCamera();
    void endRmbFlyCameraSyncOrbit();
    void applyFlyCameraKeys(float dt);

    QTimer simTimer_;
    bool simRunning_ = false;
    bool simModeActive_ = false;
    double simLastT_ = 0.0;
    size_t simLayer_ = 0;
    size_t simPlannedIdx_ = 0;
    size_t simSegIdx_ = 0;
    float simSegU_ = 0.0f;
    std::vector<glm::vec3> simCurPts_;
    std::vector<std::vector<PlannedPath>> plannedOrder_;
    LineBatch simBatch_;
    bool simBatchDirty_ = false;
    LineBatch simPhysBatch_;
    bool simPhysBatchDirty_ = false;
    std::vector<BeamElement> simPrintedBeams_;

    std::unique_ptr<Model> model_;
    /// 从文件加载后累加的顶点烘焙变换（再导入时乘到顶点上）
    glm::mat4 modelBakeTransform_{1.0f};
    std::string modelSourcePath_;
    size_t modelFaceCount_ = 0;
    int lastSliceLayerCount_ = 0;
    std::vector<Layer> plannedLayers_;
    ProcessTimingStats timingStats_;

    Shader* modelShader_ = nullptr;
    Shader* lineShader_ = nullptr;
    Shader* skyboxShader_ = nullptr;

    unsigned int supportVAO_ = 0;
    unsigned int supportVBO_ = 0;
    unsigned int supportEBO_ = 0;
    int supportIndexCount_ = 0;
    unsigned int skyboxVAO_ = 0;
    unsigned int skyboxVBO_ = 0;
    unsigned int skyboxCubemapTex_ = 0;

    LineBatch contourOuter_;
    LineBatch contourInner_;
    LineBatch infill_;
    LineBatch overhang_;

    void ensureLineBatch(LineBatch& b);
    void updateLineBatch(LineBatch& b);
    void releaseLineBatch(LineBatch& b);

    void uploadIndexedMesh(const Mesh& mesh);
    void releaseIndexedMesh();

    void appendLineStrip(const std::vector<Point3D>& pts, LineBatch& b);
    void buildContourGeometry(const std::vector<Layer>& layers, LineBatch& outer, LineBatch& inner);
    void buildInfillGeometry(const std::vector<Layer>& layers, LineBatch& infill);
    void buildOverhangGeometry(const Model& model, const OverhangResult& overhang, LineBatch& triBatch);

    glm::mat4 projectionMat() const;
    glm::mat4 viewMat() const;

    void advanceSimulation(float dt);
    std::vector<glm::vec3> orderedPathPoints(const PlannedPath& pp) const;
    float computeBeadWidth() const;
    void appendBeamElement(const glm::vec3& a, const glm::vec3& b);
    void rebuildPhysBatchFromBeams();
    /// 沿 a→b 追加封闭圆柱（轴线为路径方向，直径≈线宽），用于物理沉积可视化
    static void appendPhysCylinderSegmentRaw(
        const glm::vec3& a,
        const glm::vec3& b,
        float beadWidth,
        float layerHeight,
        int radialSegments,
        LineBatch& bch);
};