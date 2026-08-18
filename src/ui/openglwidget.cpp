#include <glad/glad.h>
#include "openglwidget.h"

#include <QCursor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QTimer>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <array>
#include <string>
#include <limits>

#include "application/stb_image.h"
#include "process/OverhangDetector.h"
#include "process/PathPlanner.h"
#include "process/Slicer.h"
#include "process/SupportCylinderGenerator.h"

namespace
{
unsigned int loadSkyboxCubemap(const std::array<std::string, 6>& faces)
{
    unsigned int textureID = 0;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width = 0;
    int height = 0;
    int nrChannels = 0;
    stbi_set_flip_vertically_on_load(false);
    for (size_t i = 0; i < faces.size(); ++i)
    {
        unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            GLenum format = GL_RGB;
            if (nrChannels == 1) format = GL_RED;
            else if (nrChannels == 4) format = GL_RGBA;
            glTexImage2D(
                GL_TEXTURE_CUBE_MAP_POSITIVE_X + static_cast<GLenum>(i),
                0,
                format,
                width,
                height,
                0,
                format,
                GL_UNSIGNED_BYTE,
                data);
            stbi_image_free(data);
        }
        else
        {
            stbi_image_free(data);
        }
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    return textureID;
}

void appendLineStripRaw(const std::vector<glm::vec3>& pts, OpenGLWidget::LineBatch& b)
{
    if (pts.size() < 2)
        return;
    const GLint base = static_cast<GLint>(b.verts.size() / 3);
    for (const auto& p : pts)
    {
        b.verts.push_back(p.x);
        b.verts.push_back(p.y);
        b.verts.push_back(p.z);
    }
    b.first.push_back(base);
    b.count.push_back(static_cast<GLsizei>(pts.size()));
}
} // namespace

float OpenGLWidget::computeBeadWidth() const
{
    const float h = std::max(1e-4f, params_.physicalLayerHeight);
    const float v = std::max(1e-4f, params_.travelSpeed);
    const float q = std::max(1e-6f, params_.extrusionFlowRate);
    const float physicalW = params_.beadWidthScale * (q / (v * h));
    float w = params_.beadRenderScale * physicalW;
    w = std::max(w, params_.beadRenderScale * params_.nozzleDiameter * 0.5f);
    if (params_.maxBeadWidth > params_.minBeadWidth)
        w = glm::clamp(w, params_.minBeadWidth, params_.maxBeadWidth);
    return w;
}

void OpenGLWidget::appendBeamElement(const glm::vec3& a, const glm::vec3& b)
{
    if (glm::length(b - a) <= 1e-6f)
        return;

    const float beadWidth = computeBeadWidth();
    const float radius = 0.5f * beadWidth;
    const float layerHeight = std::max(1e-4f, params_.physicalLayerHeight);
    const float sink = params_.enableLayerSink
        ? glm::clamp(params_.layerSinkRatio, 0.0f, 0.5f) * layerHeight
        : 0.0f;

    BeamElement one;
    one.start = a;
    one.end = b;
    one.start.y -= sink;
    one.end.y -= sink;
    one.radius = radius;
    one.layerHeight = layerHeight;
    simPrintedBeams_.push_back(one);
}

void OpenGLWidget::rebuildPhysBatchFromBeams()
{
    simPhysBatch_.verts.clear();
    simPhysBatch_.first.clear();
    simPhysBatch_.count.clear();
    int segs = params_.physCylinderSegments;
    if (segs < 4) segs = 4;
    if (segs > 64) segs = 64;
    for (const BeamElement& b : simPrintedBeams_)
    {
        appendPhysCylinderSegmentRaw(
            b.start,
            b.end,
            b.radius * 2.0f,
            b.layerHeight,
            segs,
            simPhysBatch_);
    }
    updateLineBatch(simPhysBatch_);
    simPhysBatchDirty_ = false;
}

void OpenGLWidget::appendPhysCylinderSegmentRaw(
    const glm::vec3& a,
    const glm::vec3& b,
    float beadWidth,
    float layerHeight,
    int radialSegments,
    LineBatch& bch)
{
    (void)layerHeight;
    const glm::vec3 pa(a.x, a.y, a.z);
    const glm::vec3 pb(b.x, b.y, b.z);
    glm::vec3 axis = pb - pa;
    const float len = glm::length(axis);
    if (len <= 1e-6f)
        return;
    const glm::vec3 axisN = axis / len;
    const float r = 0.5f * std::max(1e-6f, beadWidth);
    int seg = radialSegments;
    if (seg < 4)
        seg = 4;
    if (seg > 64)
        seg = 64;

    const glm::vec3 arb = (std::fabs(axisN.y) < 0.95f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 u = glm::cross(arb, axisN);
    if (glm::length(u) <= 1e-6f)
        u = glm::vec3(0.0f, 0.0f, 1.0f);
    u = glm::normalize(u);
    const glm::vec3 v = glm::normalize(glm::cross(axisN, u));

    constexpr float kTwoPi = 6.28318530717958647692f;
    auto pushTri = [&bch](const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2) {
        bch.verts.insert(bch.verts.end(), {
            p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z
        });
    };

    for (int i = 0; i < seg; ++i)
    {
        const float t1 = kTwoPi * static_cast<float>(i) / static_cast<float>(seg);
        const float t2 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(seg);
        const glm::vec3 o1 = (std::cos(t1) * u + std::sin(t1) * v) * r;
        const glm::vec3 o2 = (std::cos(t2) * u + std::sin(t2) * v) * r;
        const glm::vec3 b1 = pa + o1;
        const glm::vec3 b2 = pa + o2;
        const glm::vec3 t1v = pb + o1;
        const glm::vec3 t2v = pb + o2;
        pushTri(b1, b2, t2v);
        pushTri(b1, t2v, t1v);
        pushTri(pa, b2, b1);
        pushTri(pb, t1v, t2v);
    }
}

OpenGLWidget::OpenGLWidget(QWidget* parent)
    : QOpenGLWidget(parent)
{
    modelSourcePath_ = "../assets/models/hollowCylinder/HOLLOW_CYLINDER.STL";

    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setContextMenuPolicy(Qt::NoContextMenu);
    connect(&simTimer_, &QTimer::timeout, this, [this]() {
        const float dt = static_cast<float>(simTimer_.interval()) / 1000.0f;
        if (simRunning_)
            advanceSimulation(dt);
        update();
    });
    simTimer_.start(16);
}

OpenGLWidget::~OpenGLWidget()
{
    if (flyRmbActive_)
        endRmbFlyCameraSyncOrbit();
    makeCurrent();
    releaseIndexedMesh();
    releaseLineBatch(contourOuter_);
    releaseLineBatch(contourInner_);
    releaseLineBatch(infill_);
    releaseLineBatch(overhang_);
    releaseLineBatch(simBatch_);
    releaseLineBatch(simPhysBatch_);
    if (skyboxVBO_ != 0) glDeleteBuffers(1, &skyboxVBO_);
    if (skyboxVAO_ != 0) glDeleteVertexArrays(1, &skyboxVAO_);
    if (skyboxCubemapTex_ != 0) glDeleteTextures(1, &skyboxCubemapTex_);
    delete modelShader_;
    delete lineShader_;
    delete skyboxShader_;
    doneCurrent();
}

void OpenGLWidget::setParams(const UiProcessParams& p)
{
    params_ = p;
}

void OpenGLWidget::loadModelFromAssetPath(const std::string& path)
{
    modelSourcePath_ = path;
    processPipeline();
}

void OpenGLWidget::rotateModelPreprocessWorldCCW90(int axisIndex)
{
    glm::vec3 ax(0.0f);
    if (axisIndex == 0)
        ax = glm::vec3(1.0f, 0.0f, 0.0f);
    else if (axisIndex == 1)
        ax = glm::vec3(0.0f, 1.0f, 0.0f);
    else
        ax = glm::vec3(0.0f, 0.0f, 1.0f);
    const glm::mat4 R = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), ax);
    modelBakeTransform_ = R * modelBakeTransform_;
    processPipeline();
}

void OpenGLWidget::setShowContours(bool on)
{
    showContours_ = on;
    showInfill_ = on; // 轮廓开关联动填充线显示
    update();
}

void OpenGLWidget::initializeGL()
{
    auto loader = [](const char* name) -> void* {
        QOpenGLContext* ctx = QOpenGLContext::currentContext();
        if (!ctx)
            return nullptr;
        return reinterpret_cast<void*>(ctx->getProcAddress(name));
    };
    gladLoadGLLoader((GLADloadproc)loader);

    glEnable(GL_DEPTH_TEST);
    modelShader_ = new Shader("../assets/shaders/model_loading.vs", "../assets/shaders/model_loading.fs");
    lineShader_ = new Shader("../assets/shaders/color_line.vs", "../assets/shaders/color_line.fs");
    skyboxShader_ = new Shader("../assets/shaders/skybox_vertex.glsl", "../assets/shaders/skybox_fragment.glsl");

    constexpr float skyboxVertices[] = {
        -1.0f,  1.0f, -1.0f,  -1.0f, -1.0f, -1.0f,   1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,   1.0f,  1.0f, -1.0f,  -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,  -1.0f, -1.0f, -1.0f,  -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,  -1.0f,  1.0f,  1.0f,  -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,   1.0f, -1.0f,  1.0f,   1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,   1.0f,  1.0f, -1.0f,   1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,  -1.0f,  1.0f,  1.0f,   1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,   1.0f, -1.0f,  1.0f,  -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,   1.0f,  1.0f, -1.0f,   1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,  -1.0f,  1.0f,  1.0f,  -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,  -1.0f, -1.0f,  1.0f,   1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,  -1.0f, -1.0f,  1.0f,   1.0f, -1.0f,  1.0f
    };
    glGenVertexArrays(1, &skyboxVAO_);
    glGenBuffers(1, &skyboxVBO_);
    glBindVertexArray(skyboxVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glBindVertexArray(0);

    const std::array<std::string, 6> faces = {
        "../assets/textures/skybox/right.jpg",
        "../assets/textures/skybox/left.jpg",
        "../assets/textures/skybox/top.jpg",
        "../assets/textures/skybox/bottom.jpg",
        "../assets/textures/skybox/front.jpg",
        "../assets/textures/skybox/back.jpg"
    };
    skyboxCubemapTex_ = loadSkyboxCubemap(faces);
    skyboxShader_->begin();
    skyboxShader_->setInt("skybox", 0);
    skyboxShader_->end();
    glReady_ = true;

    if (pendingProcess_)
    {
        pendingProcess_ = false;
        QTimer::singleShot(0, this, [this]() { processPipeline(); });
    }
}

void OpenGLWidget::resizeGL(int, int) {}

glm::mat4 OpenGLWidget::projectionMat() const
{
    const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    return glm::perspective(glm::radians(45.0f), aspect, 0.1f, 200.0f);
}

glm::vec3 OpenGLWidget::cameraDirFromYawPitch(float yawDeg, float pitchDeg) const
{
    const float cy = glm::cos(glm::radians(yawDeg));
    const float sy = glm::sin(glm::radians(yawDeg));
    const float cp = glm::cos(glm::radians(pitchDeg));
    const float sp = glm::sin(glm::radians(pitchDeg));
    return glm::normalize(glm::vec3(cy * cp, sp, sy * cp));
}

void OpenGLWidget::beginRmbFlyCamera()
{
    flyYaw_ = yaw_;
    flyPitch_ = pitch_;
    const glm::vec3 dir = cameraDirFromYawPitch(yaw_, pitch_);
    flyEye_ = cameraTarget_ - dir * distance_;
    flyRmbActive_ = true;
    flyKeyW_ = flyKeyA_ = flyKeyS_ = flyKeyD_ = false;
    flyLastStep_ = std::chrono::steady_clock::now();
    grabMouse();
    setFocus(Qt::MouseFocusReason);
    setCursor(Qt::BlankCursor);
    if (width() > 2 && height() > 2)
        QCursor::setPos(mapToGlobal(QPoint(width() / 2, height() / 2)));
}

void OpenGLWidget::endRmbFlyCameraSyncOrbit()
{
    if (!flyRmbActive_)
        return;
    const glm::vec3 dir = cameraDirFromYawPitch(flyYaw_, flyPitch_);
    cameraTarget_ = flyEye_ + dir * distance_;
    yaw_ = flyYaw_;
    pitch_ = flyPitch_;
    flyRmbActive_ = false;
    flyKeyW_ = flyKeyA_ = flyKeyS_ = flyKeyD_ = false;
    releaseMouse();
    unsetCursor();
}

void OpenGLWidget::applyFlyCameraKeys(float dt)
{
    glm::vec3 front = cameraDirFromYawPitch(flyYaw_, flyPitch_);
    const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::cross(front, worldUp);
    const float rl = glm::length(right);
    if (rl > 1e-5f)
        right /= rl;
    else
        right = glm::vec3(1.0f, 0.0f, 0.0f);
    const float baseSpeed = std::max(0.35f, distance_ * 0.45f) * params_.cameraFlySpeed;
    const float s = baseSpeed * dt;
    if (flyKeyW_)
        flyEye_ += front * s;
    if (flyKeyS_)
        flyEye_ -= front * s;
    if (flyKeyD_)
        flyEye_ += right * s;
    if (flyKeyA_)
        flyEye_ -= right * s;
}

glm::mat4 OpenGLWidget::viewMat() const
{
    if (flyRmbActive_)
    {
        const glm::vec3 f = cameraDirFromYawPitch(flyYaw_, flyPitch_);
        return glm::lookAt(flyEye_, flyEye_ + f, glm::vec3(0, 1, 0));
    }
    const glm::vec3 dir = cameraDirFromYawPitch(yaw_, pitch_);
    const glm::vec3 eye = cameraTarget_ - dir * distance_;
    return glm::lookAt(eye, cameraTarget_, glm::vec3(0, 1, 0));
}

void OpenGLWidget::paintGL()
{
    glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (glReady_ && flyRmbActive_ && (flyKeyW_ || flyKeyA_ || flyKeyS_ || flyKeyD_))
    {
        const auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - flyLastStep_).count();
        flyLastStep_ = now;
        dt = std::min(0.08f, std::max(0.001f, dt));
        applyFlyCameraKeys(dt);
    }
    if (!glReady_ || !model_)
        return;

    const glm::mat4 proj = projectionMat();
    const glm::mat4 view = viewMat();
    const glm::mat4 modelMat(1.0f);

    if (showModel_ || showModelTriangleWire_)
    {
        modelShader_->begin();
        modelShader_->setMat4x4("projection", proj);
        modelShader_->setMat4x4("view", view);
        modelShader_->setMat4x4("model", modelMat);
        float modelRgb[3] = {0.2f, 0.5f, 0.8f};
        float supportRgb[3] = {0.92f, 0.48f, 0.10f}; // 暖橙，与模型蓝区分
        if (showModel_)
        {
            modelShader_->setFloat("uBaseColor", modelRgb);
            model_->Draw(*modelShader_);
        }
        if (showModelTriangleWire_)
        {
            modelShader_->setFloat("uBaseColor", modelRgb);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glEnable(GL_POLYGON_OFFSET_LINE);
            glPolygonOffset(-0.75f, -0.75f);
            model_->Draw(*modelShader_);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_POLYGON_OFFSET_LINE);
        }
        if (showModel_ && showSupport_ && supportVAO_ != 0 && supportIndexCount_ > 0)
        {
            modelShader_->setFloat("uBaseColor", supportRgb);
            glBindVertexArray(supportVAO_);
            glDrawElements(GL_TRIANGLES, supportIndexCount_, GL_UNSIGNED_INT, nullptr);
            glBindVertexArray(0);
        }
        modelShader_->end();
    }

    lineShader_->begin();
    lineShader_->setMat4x4("projection", proj);
    lineShader_->setMat4x4("view", view);
    lineShader_->setMat4x4("model", modelMat);
    glLineWidth(2.0f);

    const bool drawStaticPlan = !simModeActive_;
    if (drawStaticPlan && showContours_ && contourOuter_.vao != 0 && !contourOuter_.first.empty())
    {
        float rgb[3] = {0.25f, 1.0f, 0.45f};
        lineShader_->setFloat("uColor", rgb);
        glBindVertexArray(contourOuter_.vao);
        glMultiDrawArrays(GL_LINE_STRIP, contourOuter_.first.data(), contourOuter_.count.data(), static_cast<GLsizei>(contourOuter_.first.size()));
    }
    if (drawStaticPlan && showContours_ && contourInner_.vao != 0 && !contourInner_.first.empty())
    {
        float rgb[3] = {1.0f, 0.55f, 0.15f};
        lineShader_->setFloat("uColor", rgb);
        glBindVertexArray(contourInner_.vao);
        glMultiDrawArrays(GL_LINE_STRIP, contourInner_.first.data(), contourInner_.count.data(), static_cast<GLsizei>(contourInner_.first.size()));
    }
    if (drawStaticPlan && showInfill_ && infill_.vao != 0 && !infill_.first.empty())
    {
        float rgb[3] = {0.25f, 0.75f, 1.0f};
        lineShader_->setFloat("uColor", rgb);
        glBindVertexArray(infill_.vao);
        glMultiDrawArrays(GL_LINE_STRIP, infill_.first.data(), infill_.count.data(), static_cast<GLsizei>(infill_.first.size()));
    }
    if (showOverhang_ && overhang_.vao != 0 && !overhang_.verts.empty())
    {
        float rgb[3] = {1.0f, 0.15f, 0.8f};
        lineShader_->setFloat("uColor", rgb);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
        glBindVertexArray(overhang_.vao);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(overhang_.verts.size() / 3));
        glDisable(GL_POLYGON_OFFSET_FILL);
    }
    glBindVertexArray(0);
    lineShader_->end();

    // 天空盒先于「模拟路径」绘制：路径关闭深度测试且不依赖是否画模型，放在最后可避免与深度缓冲/天空盒竞争导致可见度随模型开关变化。
    if (skyboxShader_ && skyboxVAO_ != 0 && skyboxCubemapTex_ != 0)
    {
        glDepthFunc(GL_LEQUAL);
        skyboxShader_->begin();
        const glm::mat4 skyView = glm::mat4(glm::mat3(view));
        skyboxShader_->setMat4x4("view", skyView);
        skyboxShader_->setMat4x4("projection", proj);
        glBindVertexArray(skyboxVAO_);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxCubemapTex_);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
        skyboxShader_->end();
        glDepthFunc(GL_LESS);
    }

    lineShader_->begin();
    lineShader_->setMat4x4("projection", proj);
    lineShader_->setMat4x4("view", view);
    lineShader_->setMat4x4("model", modelMat);
    glLineWidth(2.0f);

    if (simModeActive_ && !params_.enablePhysicalDeposition && simBatch_.vao != 0 && !simBatch_.first.empty())
    {
        if (simBatchDirty_)
        {
            updateLineBatch(simBatch_);
            simBatchDirty_ = false;
        }
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glLineWidth(3.0f);
        float rgb[3] = {1.0f, 1.0f, 0.35f};
        lineShader_->setFloat("uColor", rgb);
        glBindVertexArray(simBatch_.vao);
        glMultiDrawArrays(GL_LINE_STRIP, simBatch_.first.data(), simBatch_.count.data(), static_cast<GLsizei>(simBatch_.first.size()));
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }
    else if (simModeActive_ && !params_.enablePhysicalDeposition && simBatch_.vao == 0)
    {
        ensureLineBatch(simBatch_);
    }
    if (simModeActive_ && params_.enablePhysicalDeposition && simPhysBatch_.vao != 0)
    {
        if (simPhysBatchDirty_)
            rebuildPhysBatchFromBeams();
        if (!simPhysBatch_.verts.empty())
        {
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(-1.0f, -1.0f);
            float rgb[3] = {1.0f, 0.82f, 0.35f};
            lineShader_->setFloat("uColor", rgb);
            glBindVertexArray(simPhysBatch_.vao);
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(simPhysBatch_.verts.size() / 3));
            glDisable(GL_POLYGON_OFFSET_FILL);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);
        }
    }
    else if (simModeActive_ && params_.enablePhysicalDeposition && simPhysBatch_.vao == 0)
    {
        ensureLineBatch(simPhysBatch_);
    }
    if (simModeActive_ && simRunning_ && !simCurPts_.empty() &&
        simLayer_ < plannedOrder_.size() && simPlannedIdx_ < plannedOrder_[simLayer_].size())
    {
        LineBatch tmp;
        bool hasGeom = false;
        if (params_.enablePhysicalDeposition)
        {
            const float w = computeBeadWidth();
            const float h = std::max(1e-4f, params_.physicalLayerHeight);
            const float sink = params_.enableLayerSink
                ? glm::clamp(params_.layerSinkRatio, 0.0f, 0.5f) * h
                : 0.0f;
            int segs = params_.physCylinderSegments;
            if (segs < 4) segs = 4;
            if (segs > 64) segs = 64;
            for (size_t i = 0; i + 1 <= simSegIdx_ && i + 1 < simCurPts_.size(); ++i)
            {
                glm::vec3 a = simCurPts_[i];
                glm::vec3 b = simCurPts_[i + 1];
                a.y -= sink;
                b.y -= sink;
                appendPhysCylinderSegmentRaw(a, b, w, h, segs, tmp);
            }
            if (simSegIdx_ + 1 < simCurPts_.size())
            {
                glm::vec3 a = simCurPts_[simSegIdx_];
                glm::vec3 b = simCurPts_[simSegIdx_ + 1];
                a.y -= sink;
                b.y -= sink;
                appendPhysCylinderSegmentRaw(a, a + (b - a) * simSegU_, w, h, segs, tmp);
            }
            hasGeom = !tmp.verts.empty();
            if (hasGeom)
                updateLineBatch(tmp);
        }
        else
        {
            std::vector<glm::vec3> partial;
            partial.reserve(simSegIdx_ + 2);
            for (size_t i = 0; i <= simSegIdx_ && i < simCurPts_.size(); ++i)
                partial.push_back(simCurPts_[i]);
            if (simSegIdx_ + 1 < simCurPts_.size())
            {
                const glm::vec3 a = simCurPts_[simSegIdx_];
                const glm::vec3 b = simCurPts_[simSegIdx_ + 1];
                partial.push_back(a + (b - a) * simSegU_);
            }
            if (partial.size() >= 2)
            {
                appendLineStripRaw(partial, tmp);
                updateLineBatch(tmp);
                hasGeom = !tmp.first.empty();
            }
        }
        if (hasGeom)
        {
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            if (!params_.enablePhysicalDeposition)
                glLineWidth(4.0f);
            if (params_.enablePhysicalDeposition)
            {
                glEnable(GL_POLYGON_OFFSET_FILL);
                glPolygonOffset(-1.0f, -1.0f);
            }
            float rgb[3] = {1.0f, 0.35f, 0.15f};
            lineShader_->setFloat("uColor", rgb);
            glBindVertexArray(tmp.vao);
            if (params_.enablePhysicalDeposition)
                glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(tmp.verts.size() / 3));
            else
                glMultiDrawArrays(GL_LINE_STRIP, tmp.first.data(), tmp.count.data(), 1);
            if (params_.enablePhysicalDeposition)
                glDisable(GL_POLYGON_OFFSET_FILL);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);
        }
        releaseLineBatch(tmp);
    }
    glBindVertexArray(0);
    lineShader_->end();
}

void OpenGLWidget::mousePressEvent(QMouseEvent* e)
{
    if (e->button() == Qt::RightButton)
    {
        beginRmbFlyCamera();
        if (width() > 2 && height() > 2)
            lastMouse_ = QPoint(width() / 2, height() / 2);
        else
            lastMouse_ = e->pos();
        update();
        return;
    }
    lastMouse_ = e->pos();
}

void OpenGLWidget::mouseReleaseEvent(QMouseEvent* e)
{
    if (e->button() == Qt::RightButton && flyRmbActive_)
    {
        endRmbFlyCameraSyncOrbit();
        update();
    }
    QOpenGLWidget::mouseReleaseEvent(e);
}

void OpenGLWidget::mouseMoveEvent(QMouseEvent* e)
{
    if (flyRmbActive_ && (e->buttons() & Qt::RightButton))
    {
        if (width() > 2 && height() > 2)
        {
            const QPoint center(width() / 2, height() / 2);
            const QPoint d = e->pos() - center;
            constexpr float sens = 0.12f;
            flyYaw_ += static_cast<float>(d.x()) * sens;
            flyPitch_ -= static_cast<float>(d.y()) * sens;
            if (flyPitch_ > 89.0f) flyPitch_ = 89.0f;
            if (flyPitch_ < -89.0f) flyPitch_ = -89.0f;
            QCursor::setPos(mapToGlobal(center));
            lastMouse_ = center;
        }
        update();
        return;
    }

    const bool leftDown = (e->buttons() & Qt::LeftButton);
    const bool rightDown = (e->buttons() & Qt::RightButton);
    if (!leftDown && !rightDown)
    {
        lastMouse_ = e->pos();
        return;
    }
    const QPoint d = e->pos() - lastMouse_;
    lastMouse_ = e->pos();
    if (leftDown)
    {
        yaw_ += d.x() * 0.25f;
        pitch_ -= d.y() * 0.25f;
        if (pitch_ > 89.0f) pitch_ = 89.0f;
        if (pitch_ < -89.0f) pitch_ = -89.0f;
    }
    update();
}

void OpenGLWidget::wheelEvent(QWheelEvent* e)
{
    const float steps = static_cast<float>(e->angleDelta().y()) / 120.0f;
    if (flyRmbActive_)
    {
        const glm::vec3 f = cameraDirFromYawPitch(flyYaw_, flyPitch_);
        flyEye_ += f * (steps * 0.35f * params_.cameraFlySpeed);
        update();
        e->accept();
        return;
    }
    distance_ -= steps * 0.4f;
    if (distance_ < 0.5f) distance_ = 0.5f;
    update();
}

void OpenGLWidget::keyPressEvent(QKeyEvent* e)
{
    if (!flyRmbActive_)
    {
        QOpenGLWidget::keyPressEvent(e);
        return;
    }
    switch (e->key())
    {
    case Qt::Key_W:
        flyKeyW_ = true;
        e->accept();
        return;
    case Qt::Key_S:
        flyKeyS_ = true;
        e->accept();
        return;
    case Qt::Key_A:
        flyKeyA_ = true;
        e->accept();
        return;
    case Qt::Key_D:
        flyKeyD_ = true;
        e->accept();
        return;
    default:
                    break;
                }
    QOpenGLWidget::keyPressEvent(e);
}

void OpenGLWidget::keyReleaseEvent(QKeyEvent* e)
{
    if (!flyRmbActive_)
    {
        QOpenGLWidget::keyReleaseEvent(e);
        return;
    }
    if (e->isAutoRepeat())
    {
        QOpenGLWidget::keyReleaseEvent(e);
        return;
    }
    switch (e->key())
    {
    case Qt::Key_W:
        flyKeyW_ = false;
        e->accept();
        return;
    case Qt::Key_S:
        flyKeyS_ = false;
        e->accept();
        return;
    case Qt::Key_A:
        flyKeyA_ = false;
        e->accept();
        return;
    case Qt::Key_D:
        flyKeyD_ = false;
        e->accept();
        return;
    default:
        break;
    }
    QOpenGLWidget::keyReleaseEvent(e);
}

void OpenGLWidget::focusOutEvent(QFocusEvent* e)
{
    if (flyRmbActive_)
        endRmbFlyCameraSyncOrbit();
    QOpenGLWidget::focusOutEvent(e);
}

void OpenGLWidget::ensureLineBatch(LineBatch& b)
{
    if (b.vao != 0)
        return;
    glGenVertexArrays(1, &b.vao);
    glGenBuffers(1, &b.vbo);
    glBindVertexArray(b.vao);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void OpenGLWidget::updateLineBatch(LineBatch& b)
{
    ensureLineBatch(b);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(b.verts.size() * sizeof(float)), b.verts.empty() ? nullptr : b.verts.data(), GL_DYNAMIC_DRAW);
}

void OpenGLWidget::releaseLineBatch(LineBatch& b)
{
    if (b.vbo != 0) glDeleteBuffers(1, &b.vbo);
    if (b.vao != 0) glDeleteVertexArrays(1, &b.vao);
    b = LineBatch{};
}

void OpenGLWidget::appendLineStrip(const std::vector<Point3D>& pts, LineBatch& b)
{
    if (pts.size() < 2)
        return;
    const GLint base = static_cast<GLint>(b.verts.size() / 3);
    for (const Point3D& p : pts)
    {
        b.verts.push_back(p.x());
        b.verts.push_back(p.y());
        b.verts.push_back(p.z());
    }
    b.first.push_back(base);
    b.count.push_back(static_cast<GLsizei>(pts.size()));
}

void OpenGLWidget::buildContourGeometry(const std::vector<Layer>& layers, LineBatch& outer, LineBatch& inner)
{
    outer = LineBatch{};
    inner = LineBatch{};
    for (const Layer& l : layers)
    {
        for (const Path& p : l.getOuterContourPaths()) appendLineStrip(p.getPoints(), outer);
        for (const Path& p : l.getInnerContourPaths()) appendLineStrip(p.getPoints(), inner);
    }
}

void OpenGLWidget::buildInfillGeometry(const std::vector<Layer>& layers, LineBatch& infill)
{
    infill = LineBatch{};
    for (const Layer& l : layers)
        for (const Path& p : l.getInfillPaths())
            appendLineStrip(p.getPoints(), infill);
}

void OpenGLWidget::buildOverhangGeometry(const Model& model, const OverhangResult& overhang, LineBatch& triBatch)
{
    triBatch = LineBatch{};
    triBatch.verts.reserve(overhang.triangles.size() * 9);
    for (const OverhangTriangle& t : overhang.triangles)
    {
        if (t.meshIndex >= model.meshes.size()) continue;
        const NewMesh& m = model.meshes[t.meshIndex];
        const size_t base = t.triIndex * 3;
        if (base + 2 >= m.indices.size()) continue;
        const unsigned i0 = m.indices[base], i1 = m.indices[base + 1], i2 = m.indices[base + 2];
        if (i0 >= m.vertices.size() || i1 >= m.vertices.size() || i2 >= m.vertices.size()) continue;
        const glm::vec3 p0 = m.vertices[i0].Position;
        const glm::vec3 p1 = m.vertices[i1].Position;
        const glm::vec3 p2 = m.vertices[i2].Position;
        triBatch.verts.insert(triBatch.verts.end(), {p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z});
    }
    triBatch.first = {0};
    triBatch.count = {static_cast<GLsizei>(triBatch.verts.size() / 3)};
}

void OpenGLWidget::uploadIndexedMesh(const Mesh& mesh)
{
    releaseIndexedMesh();
    if (mesh.getPoints().empty() || mesh.getIndices().empty())
        return;
    glGenVertexArrays(1, &supportVAO_);
    glGenBuffers(1, &supportVBO_);
    glGenBuffers(1, &supportEBO_);
    glBindVertexArray(supportVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, supportVBO_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.getPoints().size() * sizeof(Point3D)), mesh.getPoints().data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, supportEBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.getIndices().size() * sizeof(uint32_t)), mesh.getIndices().data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Point3D)), reinterpret_cast<void*>(offsetof(Point3D, Position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Point3D)), reinterpret_cast<void*>(offsetof(Point3D, Normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Point3D)), reinterpret_cast<void*>(offsetof(Point3D, TexCoords)));
    glBindVertexArray(0);
    supportIndexCount_ = static_cast<GLsizei>(mesh.getIndices().size());
}

void OpenGLWidget::releaseIndexedMesh()
{
    if (supportEBO_) glDeleteBuffers(1, &supportEBO_);
    if (supportVBO_) glDeleteBuffers(1, &supportVBO_);
    if (supportVAO_) glDeleteVertexArrays(1, &supportVAO_);
    supportEBO_ = supportVBO_ = supportVAO_ = 0;
    supportIndexCount_ = 0;
}

void OpenGLWidget::processPipeline()
{
    if (!glReady_)
    {
        pendingProcess_ = true;
            return;
        }
    emit processingStarted();
    const auto tAll0 = std::chrono::steady_clock::now();
    timingStats_ = ProcessTimingStats{};
    makeCurrent();
    simRunning_ = false;
    simBatch_ = LineBatch{};
    simBatchDirty_ = false;
    simPhysBatch_ = LineBatch{};
    simPhysBatchDirty_ = false;
    simPrintedBeams_.clear();
    simLastT_ = 0.0;
    simLayer_ = 0;
    simPlannedIdx_ = 0;
    simSegIdx_ = 0;
    simSegU_ = 0.0f;
    simCurPts_.clear();

    const auto tModel0 = std::chrono::steady_clock::now();
    model_.reset(new Model(modelSourcePath_, modelBakeTransform_));

    modelFaceCount_ = 0;
    for (const NewMesh& m : model_->meshes)
        modelFaceCount_ += (m.indices.size() / 3);

    // 每次重算后将相机锚点重置到模型包围盒中心，确保左键始终围绕模型旋转
    glm::vec3 bmin(std::numeric_limits<float>::max());
    glm::vec3 bmax(std::numeric_limits<float>::lowest());
    bool hasVertex = false;
    for (const NewMesh& m : model_->meshes)
    {
        for (const auto& vtx : m.vertices)
        {
            bmin = glm::min(bmin, vtx.Position);
            bmax = glm::max(bmax, vtx.Position);
            hasVertex = true;
        }
    }
    if (hasVertex)
    {
        cameraTarget_ = 0.5f * (bmin + bmax);
        const float radius = 0.5f * glm::length(bmax - bmin);
        const float fitDist = std::max(1.0f, radius * 2.2f);
        if (!std::isfinite(distance_) || distance_ < 0.5f || distance_ > fitDist * 3.0f)
            distance_ = fitDist;
    }

    const auto tModel1 = std::chrono::steady_clock::now();
    timingStats_.modelLoadMs =
        std::chrono::duration<double, std::milli>(tModel1 - tModel0).count();

    const auto tSlice0 = std::chrono::steady_clock::now();
    Slicer slicer(*model_);
    SlicePipelineTiming sliceDetail{};
    std::vector<Layer> modelLayers;
    if (params_.useAdaptiveLayering)
    {
        Slicer::AdaptiveSliceParams asp;
        asp.minLayerHeight = params_.adaptiveMinLayerHeight;
        asp.maxLayerHeight = params_.adaptiveMaxLayerHeight;
        asp.lowCurvatureDeg = params_.adaptiveLowCurvatureDeg;
        asp.highCurvatureDeg = params_.adaptiveHighCurvatureDeg;
        asp.smoothAlpha = params_.adaptiveSmoothAlpha;
        modelLayers = slicer.sliceToLayersAdaptive(
            asp,
            params_.contourLineWidth,
            1e-5f,
            params_.useOctreePruneSlicing,
            params_.useTopologyStitch,
            &sliceDetail);
    }
    else
    {
        modelLayers = slicer.sliceToLayersUniform(
            params_.layerCount,
            params_.contourLineWidth,
            1e-5f,
            params_.useOctreePruneSlicing,
            params_.useTopologyStitch,
            &sliceDetail);
    }
    lastSliceLayerCount_ = static_cast<int>(modelLayers.size());
    const auto tSlice1 = std::chrono::steady_clock::now();
    timingStats_.slicingMs =
        std::chrono::duration<double, std::milli>(tSlice1 - tSlice0).count();
    timingStats_.slicingIntersectMs = sliceDetail.intersectMs;
    timingStats_.slicingContourMs = sliceDetail.contourMs;

    const auto tOver0 = std::chrono::steady_clock::now();
    OverhangParams op;
    op.angleThresholdDeg = params_.overhangAngleDeg;
    op.verticalWallAngleSlackDeg = params_.overhangVerticalWallSlackDeg;
    op.buildDirection = glm::vec3(0, 1, 0);
    op.requireDownFacing = true;
    op.enableLayerOverlapSecondPass = params_.enableOverhangSecondPass;
    op.sliceLayersForOverlapSecondPass = &modelLayers;
    OverhangResult over = OverhangDetector::detect(*model_, op);
    const auto tOver1 = std::chrono::steady_clock::now();
    timingStats_.overhangMs =
        std::chrono::duration<double, std::milli>(tOver1 - tOver0).count();

    const auto tSup0 = std::chrono::steady_clock::now();
    SupportCylinderParams sp;
    sp.radius = params_.supportRadius;
    sp.xzGridSpacing = params_.supportSpacing;
    sp.tipInsetBelowCentroid = params_.supportTipInset;
    sp.maxCylinders = static_cast<size_t>(params_.supportMax);
    sp.sliceCircleSegments = 24;
    Mesh supportMesh = buildSupportCylindersMesh(over, *model_, sp);
    uploadIndexedMesh(supportMesh);

    std::vector<float> yList;
    yList.reserve(modelLayers.size());
    for (const Layer& l : modelLayers) yList.push_back(l.getZHeight());
    std::vector<SupportCylinder> supportCyls = buildSupportCylinders(over, *model_, sp);
    std::vector<Layer> supportLayers = sliceSupportCylindersAtY(supportCyls, yList, params_.contourLineWidth, sp.sliceCircleSegments);
    const auto tSup1 = std::chrono::steady_clock::now();
    timingStats_.supportMs =
        std::chrono::duration<double, std::milli>(tSup1 - tSup0).count();

    const auto tPlan0 = std::chrono::steady_clock::now();
    std::vector<Layer> merged;
    merged.reserve(modelLayers.size());
    for (size_t i = 0; i < modelLayers.size(); ++i)
    {
        Layer l(modelLayers[i].getLayerId(), modelLayers[i].getZHeight());
        for (const Path& p : modelLayers[i].getOuterContourPaths()) l.addPath(p);
        for (const Path& p : modelLayers[i].getInnerContourPaths()) l.addPath(p);
        if (i < supportLayers.size())
        {
            for (const Path& p : supportLayers[i].getOuterContourPaths()) l.addPath(p);
            for (const Path& p : supportLayers[i].getInnerContourPaths()) l.addPath(p);
        }
        merged.push_back(std::move(l));
    }

    ParallelInfillParams ip;
    ip.spacing = params_.infillSpacing;
    ip.angleDeg = params_.infillAngleDeg;
    ip.lineWidth = params_.contourLineWidth;
    const InfillPattern infillPattern = (params_.infillPattern == 1)
        ? InfillPattern::ZigZag
        : InfillPattern::ParallelLines;
    plannedLayers_ = addInfillToLayers(merged, ip, infillPattern);
    plannedOrder_ = planAllLayersPathOrder(plannedLayers_);
    const auto tPlan1 = std::chrono::steady_clock::now();
    timingStats_.planningMs =
        std::chrono::duration<double, std::milli>(tPlan1 - tPlan0).count();

    const auto tUp0 = std::chrono::steady_clock::now();
    buildContourGeometry(plannedLayers_, contourOuter_, contourInner_);
    buildInfillGeometry(plannedLayers_, infill_);
    buildOverhangGeometry(*model_, over, overhang_);
    updateLineBatch(contourOuter_);
    updateLineBatch(contourInner_);
    updateLineBatch(infill_);
    updateLineBatch(overhang_);
    const auto tUp1 = std::chrono::steady_clock::now();
    timingStats_.uploadMs =
        std::chrono::duration<double, std::milli>(tUp1 - tUp0).count();
    doneCurrent();
    const auto tAll1 = std::chrono::steady_clock::now();
    timingStats_.totalMs =
        std::chrono::duration<double, std::milli>(tAll1 - tAll0).count();
    emit processingFinished();
    update();
}

std::vector<glm::vec3> OpenGLWidget::orderedPathPoints(const PlannedPath& pp) const
{
    std::vector<glm::vec3> out;
    if (!pp.path) return out;
    const auto& pts = pp.path->getPoints();
    out.reserve(pts.size());
    if (!pp.reverse)
        for (const auto& p : pts) out.push_back(p.Position);
    else
        for (size_t i = pts.size(); i-- > 0;) out.push_back(pts[i].Position);
    return out;
}

void OpenGLWidget::advanceSimulation(float dt)
{
    if (!simRunning_ || plannedOrder_.empty())
        return;
    float remain = dt * params_.playbackSpeed;
    while (remain > 1e-6f)
    {
        if (simLayer_ >= plannedOrder_.size())
        {
            simRunning_ = false;
            emit simulationStateChanged(false);
            return;
        }

        if (simPlannedIdx_ >= plannedOrder_[simLayer_].size())
        {
            simLayer_ += 1;
            simPlannedIdx_ = 0;
            simSegIdx_ = 0;
            simSegU_ = 0.0f;
            simCurPts_.clear();
            continue;
        }

        const PlannedPath& pp = plannedOrder_[simLayer_][simPlannedIdx_];
        if (pp.path == nullptr)
        {
            simPlannedIdx_ += 1;
            simSegIdx_ = 0;
            simSegU_ = 0.0f;
            simCurPts_.clear();
            continue;
        }

        if (simCurPts_.empty())
            simCurPts_ = orderedPathPoints(pp);
        if (simCurPts_.size() < 2)
        {
            simPlannedIdx_ += 1;
            simSegIdx_ = 0;
            simSegU_ = 0.0f;
            simCurPts_.clear();
            continue;
        }

        if (simSegIdx_ + 1 >= simCurPts_.size())
        {
            if (params_.enablePhysicalDeposition)
            {
                for (size_t i = 0; i + 1 < simCurPts_.size(); ++i)
                    appendBeamElement(simCurPts_[i], simCurPts_[i + 1]);
                simPhysBatchDirty_ = true;
            }
            else
            {
                appendLineStripRaw(simCurPts_, simBatch_);
                simBatchDirty_ = true;
            }
            simPlannedIdx_ += 1;
            simSegIdx_ = 0;
            simSegU_ = 0.0f;
            simCurPts_.clear();
            continue;
        }

        const glm::vec3 a = simCurPts_[simSegIdx_];
        const glm::vec3 b = simCurPts_[simSegIdx_ + 1];
        const float segLen = glm::length(b - a);
        if (segLen <= 1e-6f) { simSegIdx_ += 1; simSegU_ = 0.0f; continue; }
        const float left = (1.0f - simSegU_) * segLen;
        if (remain < left)
        {
            simSegU_ += remain / segLen;
            remain = 0.0f;
        }
        else
        {
            remain -= left;
            simSegIdx_ += 1;
            simSegU_ = 0.0f;
        }
    }
}

void OpenGLWidget::startSimulation()
{
    simModeActive_ = true;
    simLastT_ = 0.0;
    simLayer_ = 0;
    simPlannedIdx_ = 0;
    simSegIdx_ = 0;
    simSegU_ = 0.0f;
    simCurPts_.clear();
    simPrintedBeams_.clear();

    simRunning_ = true;
    simBatch_.verts.clear();
    simBatch_.first.clear();
    simBatch_.count.clear();
    simPhysBatch_.verts.clear();
    simPhysBatch_.first.clear();
    simPhysBatch_.count.clear();
    if (glReady_)
    {
            makeCurrent();
        releaseLineBatch(simBatch_);
        ensureLineBatch(simBatch_);
        releaseLineBatch(simPhysBatch_);
        ensureLineBatch(simPhysBatch_);
        doneCurrent();
    }
    simBatchDirty_ = false;
    simPhysBatchDirty_ = false;
    emit simulationStateChanged(true);
}

void OpenGLWidget::showNonPhysicalSimulationFinalResult()
{
    simModeActive_ = true;
    simRunning_ = false;
    simLastT_ = 0.0;
    simLayer_ = 0;
    simPlannedIdx_ = 0;
    simSegIdx_ = 0;
    simSegU_ = 0.0f;
    simCurPts_.clear();
    simPrintedBeams_.clear();
    simBatch_.verts.clear();
    simBatch_.first.clear();
    simBatch_.count.clear();
    simPhysBatch_.verts.clear();
    simPhysBatch_.first.clear();
    simPhysBatch_.count.clear();

    if (glReady_)
    {
            makeCurrent();
        releaseLineBatch(simBatch_);
        ensureLineBatch(simBatch_);
        releaseLineBatch(simPhysBatch_);
        ensureLineBatch(simPhysBatch_);
        doneCurrent();
    }

    for (const auto& layerPlans : plannedOrder_)
    {
        for (const PlannedPath& pp : layerPlans)
        {
            if (pp.path == nullptr)
                continue;
            const std::vector<glm::vec3> pts = orderedPathPoints(pp);
            if (pts.size() < 2)
                continue;
            appendLineStripRaw(pts, simBatch_);
        }
    }

    simBatchDirty_ = true;
    simPhysBatchDirty_ = false;
    emit simulationStateChanged(false);
    update();
}

void OpenGLWidget::showPhysicalSimulationFinalResult()
{
    simModeActive_ = true;
    simRunning_ = false;
    simLastT_ = 0.0;
    simLayer_ = 0;
    simPlannedIdx_ = 0;
    simSegIdx_ = 0;
    simSegU_ = 0.0f;
    simCurPts_.clear();
    simPrintedBeams_.clear();
    simBatch_.verts.clear();
    simBatch_.first.clear();
    simBatch_.count.clear();
    simPhysBatch_.verts.clear();
    simPhysBatch_.first.clear();
    simPhysBatch_.count.clear();

    if (glReady_)
    {
        makeCurrent();
        releaseLineBatch(simBatch_);
        ensureLineBatch(simBatch_);
        releaseLineBatch(simPhysBatch_);
        ensureLineBatch(simPhysBatch_);
        doneCurrent();
    }

    for (const auto& layerPlans : plannedOrder_)
    {
        for (const PlannedPath& pp : layerPlans)
        {
            if (pp.path == nullptr)
                continue;
            const std::vector<glm::vec3> pts = orderedPathPoints(pp);
            if (pts.size() < 2)
                continue;
            for (size_t i = 0; i + 1 < pts.size(); ++i)
                appendBeamElement(pts[i], pts[i + 1]);
        }
    }

    simBatchDirty_ = false;
    simPhysBatchDirty_ = true;
    emit simulationStateChanged(false);
    update();
}

void OpenGLWidget::pauseSimulation()
{
    if (!simModeActive_)
        return;
    simRunning_ = !simRunning_;
    emit simulationStateChanged(simRunning_);
}

void OpenGLWidget::stopSimulation()
{
    simModeActive_ = false;
    simRunning_ = false;
    simLastT_ = 0.0;
    simLayer_ = 0;
    simPlannedIdx_ = 0;
    simSegIdx_ = 0;
    simSegU_ = 0.0f;
    simCurPts_.clear();
    simPrintedBeams_.clear();
    if (glReady_)
    {
        makeCurrent();
        releaseLineBatch(simBatch_);
        releaseLineBatch(simPhysBatch_);
        doneCurrent();
    }
    else
    {
        simBatch_ = LineBatch{};
        simPhysBatch_ = LineBatch{};
    }
    simBatchDirty_ = false;
    simPhysBatchDirty_ = false;
    emit simulationStateChanged(false);
    update();
}

