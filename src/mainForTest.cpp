#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "glframework/shader.h"
#include "dataStructure/Model.h"
#include "application/callbackFunc.h"
#include "dataStructure/Layer.h"
#include "Slicer.h"
#include "OverhangDetector.h"
#include "SupportCylinderGenerator.h"
#include "PathPlanner.h"
#include "PathOrderPlanner.h"

#include "dataStructure/Mesh.h"
#include "dataStructure/Point3D.h"

#include <cstddef>
#include <iostream>
#include <utility>
#include <vector>

namespace {

/// @brief 将点序列转换为线段序列
/// @param pts 点序列
/// @param verts 顶点
/// @param first 起始索引
/// @param count 线段数量
/// @return 
/// @details 将点序列转换为线段序列，如果点序列小于2，则返回。如果点序列大于等于2，则将点序列转换为顶点，并添加到verts中。将起始索引和线段数量添加到first和count中。这么做是为了方便后续的绘制。
void appendLineStrip(
    const std::vector<Point3D>& pts,
    std::vector<float>& verts,
    std::vector<GLint>& first,
    std::vector<GLsizei>& count)
{
    //如果点序列小于2，则返回
    if (pts.size() < 2)
        return;
    //计算起始索引，verts.size() / 3 表示顶点数量，因为每个顶点有3个分量（x,y,z）
    const GLint base = static_cast<GLint>(verts.size() / 3);
    //将点序列转换为顶点
    for (const Point3D& pt : pts)
    {
        verts.push_back(pt.Position.x);
        verts.push_back(pt.Position.y);
        verts.push_back(pt.Position.z);
    }
    //将起始索引和线段数量添加到first和count中
    first.push_back(base);
    count.push_back(static_cast<GLsizei>(pts.size()));
}

/// @brief 构建轮廓几何体
/// @param layers 层
/// @param outerVerts 外轮廓顶点
/// @param outerFirst 外轮廓起始索引
/// @param outerCount 外轮廓顶点数量
/// @param innerVerts 内轮廓顶点
/// @param innerFirst 内轮廓起始索引
/// @param innerCount 内轮廓顶点数量
/// @return 
/// @details 这么做是为了方便后续的绘制。
void buildContourGeometry(
    const std::vector<Layer>& layers,
    std::vector<float>& outerVerts,
    std::vector<GLint>& outerFirst,
    std::vector<GLsizei>& outerCount,
    std::vector<float>& innerVerts,
    std::vector<GLint>& innerFirst,
    std::vector<GLsizei>& innerCount)
{
    outerVerts.clear();
    outerFirst.clear();
    outerCount.clear();
    innerVerts.clear();
    innerFirst.clear();
    innerCount.clear();
    //遍历所有层
    for (const Layer& lyr : layers)
    {
        //遍历所有外轮廓路径
        for (const Path& p : lyr.getOuterContourPaths())
            //将外轮廓路径转换为线段序列
            appendLineStrip(p.getPoints(), outerVerts, outerFirst, outerCount);
        for (const Path& p : lyr.getInnerContourPaths())
            //将内轮廓路径转换为线段序列
            appendLineStrip(p.getPoints(), innerVerts, innerFirst, innerCount);
    }
}
/// @brief 构建填充几何体
/// @param layers 层集合
/// @param infillVerts 填充顶点
/// @param infillFirst 填充起始索引
/// @param infillCount 填充顶点数量
/// @return 
/// @details 构建填充几何体，如果层集合为空，则返回。如果层集合不为空，则将填充路径转换为线段序列，并添加到infillVerts中。将起始索引和线段数量添加到infillFirst和infillCount中。这么做是为了方便后续的绘制。
void buildInfillGeometry(
    const std::vector<Layer>& layers,
    std::vector<float>& infillVerts,
    std::vector<GLint>& infillFirst,
    std::vector<GLsizei>& infillCount)
{
    infillVerts.clear();
    infillFirst.clear();
    infillCount.clear();
    for (const Layer& lyr : layers)
    {
        for (const Path& p : lyr.getInfillPaths())
            appendLineStrip(p.getPoints(), infillVerts, infillFirst, infillCount);
    }
}
/// @brief 动态线段批次
/// @details 动态线段批次，用于存储动态线段的顶点、起始索引和线段数量。这么做是为了方便后续的绘制。
struct DynamicLineBatch
{
    GLuint vao = 0;
    GLuint vbo = 0;
    std::vector<float> verts;
    std::vector<GLint> first;
    std::vector<GLsizei> count;
};

/// @brief 确保动态线段批次
/// @param b 动态线段批次
/// @return 
/// @details 确保动态线段批次，如果动态线段批次不为空，则返回。如果动态线段批次为空，则生成动态线段批次。这么做是为了方便后续的绘制。
void ensureDynamicLineBatch(DynamicLineBatch& b)
{
    if (b.vao != 0)
        return;
    glGenVertexArrays(1, &b.vao);
    glGenBuffers(1, &b.vbo);
    glBindVertexArray(b.vao);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);//参数说明：GL_ARRAY_BUFFER表示缓冲区类型，0表示数据大小，nullptr表示数据，GL_DYNAMIC_DRAW表示数据使用方式
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);//参数说明：0表示顶点属性索引，3表示每个顶点有3个分量，GL_FLOAT表示顶点数据类型，GL_FALSE表示是否归一化，3 * sizeof(float)表示步长，nullptr表示偏移量
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

/// @brief 更新动态线段批次
/// @param b 动态线段批次
/// @return 
/// @details 更新动态线段批次，如果动态线段批次不为空，则返回。如果动态线段批次为空，则生成动态线段批次。这么做是为了方便后续的绘制。
void updateDynamicLineBatch(DynamicLineBatch& b)
{
    if (b.vao == 0)
        ensureDynamicLineBatch(b);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(b.verts.size() * sizeof(float)),
        b.verts.empty() ? nullptr : b.verts.data(),
        GL_DYNAMIC_DRAW);
}

static void appendLineStripRaw(
    const std::vector<glm::vec3>& pts,
    std::vector<float>& verts,
    std::vector<GLint>& first,
    std::vector<GLsizei>& count)
{
    if (pts.size() < 2)
        return;
    const GLint base = static_cast<GLint>(verts.size() / 3);
    for (const auto& p : pts)
    {
        verts.push_back(p.x);
        verts.push_back(p.y);
        verts.push_back(p.z);
    }
    first.push_back(base);
    count.push_back(static_cast<GLsizei>(pts.size()));
}

static std::vector<glm::vec3> pathPointsOrdered(const Path& p, bool reverse)
{
    const auto& pts = p.getPoints();
    std::vector<glm::vec3> out;
    out.reserve(pts.size());
    if (!reverse)
    {
        for (const Point3D& q : pts)
            out.push_back(q.Position);
    }
    else
    {
        for (size_t i = pts.size(); i-- > 0;)
            out.push_back(pts[i].Position);
    }
    return out;
}
/// @brief 播放状态
/// @param enabled 是否启用
/// @param prevBPressed 是否按下B键
/// @param lastT 上次时间
/// @param speed 速度
/// @param layerIdx 层索引
/// @param plannedIdx 规划索引
/// @param segIdx 线段索引
/// @param segU 线段U
/// @param curPts 当前点
/// @param finishedStrips 完成线段
/// @return 
/// @details 播放状态，用于存储播放状态的变量。这么做是为了方便后续的播放。
struct PlaybackState
{
    bool enabled = false;
    bool prevBPressed = false;
    double lastT = 0.0;
    float speed =200.0f; // 世界单位/秒

    size_t layerIdx = 0;
    size_t plannedIdx = 0;
    size_t segIdx = 0;
    float segU = 0.0f;

    std::vector<glm::vec3> curPts;
    std::vector<glm::vec3> finishedStrips; // 仅用于临时拼接，实际 strip 存在 batch 里
};


std::vector<float> extractLayerHeights(const std::vector<Layer>& layers)
{
    std::vector<float> ys;
    ys.reserve(layers.size());
    for (const Layer& l : layers)
        ys.push_back(l.getZHeight());
    return ys;
}

std::vector<Layer> mergeModelAndSupportLayers(
    const std::vector<Layer>& modelLayers,
    const std::vector<Layer>& supportLayers)
{
    std::vector<Layer> merged;
    const size_t n = modelLayers.size();
    merged.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        Layer lyr(static_cast<int>(i), modelLayers[i].getZHeight());
        for (const Path& p : modelLayers[i].getOuterContourPaths())
            lyr.addPath(p);
        for (const Path& p : modelLayers[i].getInnerContourPaths())
            lyr.addPath(p);

        if (i < supportLayers.size())
        {
            for (const Path& p : supportLayers[i].getOuterContourPaths())
                lyr.addPath(p);
            for (const Path& p : supportLayers[i].getInnerContourPaths())
                lyr.addPath(p);
        }
        merged.push_back(std::move(lyr));
    }
    return merged;
}

/// @brief 构建悬垂三角片几何体（直接按 GL_TRIANGLES 绘制）
/// @param model 原始模型
/// @param overhangResult 悬垂识别结果
/// @param triVerts 输出顶点（xyz 连续）
void buildOverhangTriangleGeometry(
    const Model& model,
    const OverhangResult& overhangResult,
    std::vector<float>& triVerts)
{
    triVerts.clear();
    triVerts.reserve(overhangResult.triangles.size() * 9);
    for (const OverhangTriangle& triInfo : overhangResult.triangles)
    {
        if (triInfo.meshIndex >= model.meshes.size())
            continue;
        const NewMesh& mesh = model.meshes[triInfo.meshIndex];
        const size_t base = triInfo.triIndex * 3;
        if (base + 2 >= mesh.indices.size())
            continue;
        const unsigned int i0 = mesh.indices[base + 0];
        const unsigned int i1 = mesh.indices[base + 1];
        const unsigned int i2 = mesh.indices[base + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size())
            continue;

        const glm::vec3& p0 = mesh.vertices[i0].Position;
        const glm::vec3& p1 = mesh.vertices[i1].Position;
        const glm::vec3& p2 = mesh.vertices[i2].Position;
        triVerts.push_back(p0.x);
        triVerts.push_back(p0.y);
        triVerts.push_back(p0.z);
        triVerts.push_back(p1.x);
        triVerts.push_back(p1.y);
        triVerts.push_back(p1.z);
        triVerts.push_back(p2.x);
        triVerts.push_back(p2.y);
        triVerts.push_back(p2.z);
    }
}

/// @brief 上传线段批次
/// @param vao VAO
/// @param vbo VBO
/// @param verts 顶点
/// @return 
void uploadLineBatch(
    GLuint& vao,
    GLuint& vbo,
    const std::vector<float>& verts)
{
    //如果顶点为空，则返回
    if (verts.empty())
        return;
    //生成VAO和VBO
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    //绑定VAO和VBO
    glBindVertexArray(vao);
    //绑定VBO
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    //上传顶点数据
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
        verts.data(),
        GL_STATIC_DRAW);
    //设置顶点属性
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    //启用顶点属性
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

/// @brief 释放线段批次
/// @param vao VAO
/// @param vbo VBO
/// @return 
/// @brief 将 Mesh（Point3D 交错布局 + 索引）上传到 GPU，供 model_loading 着色器绘制
void uploadIndexedMeshPointLayout(
    const Mesh& mesh,
    GLuint& vao,
    GLuint& vbo,
    GLuint& ebo,
    GLsizei& outIndexCount)
{
    outIndexCount = 0;
    if (mesh.getPoints().empty() || mesh.getIndices().empty())
        return;

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(mesh.getPoints().size() * sizeof(Point3D)),
        mesh.getPoints().data(),
        GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(mesh.getIndices().size() * sizeof(uint32_t)),
        mesh.getIndices().data(),
        GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        static_cast<GLsizei>(sizeof(Point3D)),
        reinterpret_cast<void*>(offsetof(Point3D, Position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1,
        3,
        GL_FLOAT,
        GL_FALSE,
        static_cast<GLsizei>(sizeof(Point3D)),
        reinterpret_cast<void*>(offsetof(Point3D, Normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(
        2,
        2,
        GL_FLOAT,
        GL_FALSE,
        static_cast<GLsizei>(sizeof(Point3D)),
        reinterpret_cast<void*>(offsetof(Point3D, TexCoords)));

    glBindVertexArray(0);
    outIndexCount = static_cast<GLsizei>(mesh.getIndices().size());
}

void releaseIndexedMesh(GLuint& vao, GLuint& vbo, GLuint& ebo)
{
    if (ebo != 0)
    {
        glDeleteBuffers(1, &ebo);
        ebo = 0;
    }
    if (vbo != 0)
    {
        glDeleteBuffers(1, &vbo);
        vbo = 0;
    }
    if (vao != 0)
    {
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }
}

void releaseLineBatch(GLuint& vao, GLuint& vbo)
{
    //如果VBO不为0，则释放VBO
    if (vbo != 0)
    {
        glDeleteBuffers(1, &vbo);
        vbo = 0;
    }
    if (vao != 0)
    {
        //释放VAO
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }
}

} // namespace

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "3D Slicer", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // tell stb_image.h to flip loaded texture's on the y-axis (before loading model).
    stbi_set_flip_vertically_on_load(true);

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // 线框模式，显示三角网格线

    // build and compile shaders（可执行文件在 build/Debug 时，../assets 指向构建目录下的 assets）
    Shader ourShader("../assets/shaders/model_loading.vs", "../assets/shaders/model_loading.fs");
    Shader lineShader("../assets/shaders/color_line.vs", "../assets/shaders/color_line.fs");

    // load models
    glm::mat4 bakeModel = glm::mat4(1.0f);
    bakeModel = glm::translate(bakeModel, glm::vec3(0.0f, 0.0f, 0.0f));
    bakeModel = glm::scale(bakeModel, glm::vec3(1.0f, 1.0f, 1.0f));
    //Model ourModel("../assets/models/backpack/backpack.obj", bakeModel);
    Model ourModel("../assets/models/cylinder/Cylinder.stl", bakeModel);


    // 均匀分层切片 + 上传轮廓线到 GPU（LINE_STRIP）
    constexpr int kUniformLayerCount = 200;
    constexpr float kContourLineWidth = 0.1f;
    std::vector<Layer> modelSliceLayers;
    std::vector<Layer> supportSliceLayers;
    std::vector<Layer> mergedSliceLayers;
    std::vector<float> outerLineVerts;
    std::vector<GLint> outerLineFirst;
    std::vector<GLsizei> outerLineCount;
    std::vector<float> innerLineVerts;
    std::vector<GLint> innerLineFirst;
    std::vector<GLsizei> innerLineCount;
    std::vector<float> overhangTriVerts;
    std::vector<float> infillLineVerts;
    std::vector<GLint> infillLineFirst;
    std::vector<GLsizei> infillLineCount;
    GLuint contourOuterVAO = 0;
    GLuint contourOuterVBO = 0;
    GLuint contourInnerVAO = 0;
    GLuint contourInnerVBO = 0;
    GLuint infillVAO = 0;
    GLuint infillVBO = 0;
    GLuint overhangVAO = 0;
    GLuint overhangVBO = 0;

    {
        Slicer slicer(ourModel);
        modelSliceLayers = slicer.sliceToLayersUniform(kUniformLayerCount, kContourLineWidth);
        std::cout << "[Slicer] model slice layers: " << modelSliceLayers.size() << "\n";
        buildContourGeometry(
            modelSliceLayers,
            outerLineVerts,
            outerLineFirst,
            outerLineCount,
            innerLineVerts,
            innerLineFirst,
            innerLineCount);
        uploadLineBatch(contourOuterVAO, contourOuterVBO, outerLineVerts);
        uploadLineBatch(contourInnerVAO, contourInnerVBO, innerLineVerts);
        std::cout << "[Slicer] contour strips: outer=" << outerLineFirst.size()
                  << " inner=" << innerLineFirst.size() << "\n";
    }

    OverhangParams overhangParams;
    overhangParams.angleThresholdDeg = 45.0f;
    overhangParams.buildDirection = glm::vec3(0.0f, 1.0f, 0.0f);
    overhangParams.requireDownFacing = true;
    //开启层间二次验证,调试可用
    overhangParams.enableLayerOverlapSecondPass = false;
    overhangParams.sliceLayersForOverlapSecondPass = &modelSliceLayers;
    overhangParams.overlapRasterResolution = 128;
    overhangParams.overlapSupportKeepThreshold = 0.8f;
    overhangParams.overlapLocalWindowHalfCells = 2;

    OverhangResult overhangResult = OverhangDetector::detect(ourModel, overhangParams);
    std::cout << "[Overhang] total triangles: " << overhangResult.totalTriangles
              << " skipped: " << overhangResult.skippedDegenerateTriangles
              << " geom candidates: " << overhangResult.candidateCountAfterGeometry
              << " after 2nd pass: " << overhangResult.triangles.size()
              << " (removed by layer pass: " << overhangResult.removedByLayerSecondPass << ")\n";
    if (!overhangResult.triangles.empty())
    {
        const OverhangTriangle& sample = overhangResult.triangles.front();
        std::cout << "[Overhang] first centroid=("
                  << sample.centroid.x << ", "
                  << sample.centroid.y << ", "
                  << sample.centroid.z << ") angle="
                  << sample.angleToBuildDirDeg << " deg\n";
    }

    buildOverhangTriangleGeometry(ourModel, overhangResult, overhangTriVerts);
    uploadLineBatch(overhangVAO, overhangVBO, overhangTriVerts);
    std::cout << "[Overhang] visualized triangles: " << (overhangTriVerts.size() / 9) << "\n";

    // 生成支撑圆柱几何体
    SupportCylinderParams supportParams;
    supportParams.radius = 0.005f; // 柱半径（米/模型单位）；改小则柱子更细
    supportParams.xzGridSpacing = 0.20f;//XZ平面栅格边长：同一格内多个悬垂候选只保留「最高」的一个落柱点，控制密度
    supportParams.tipInsetBelowCentroid = 0.03f;//柱顶在三角重心下方留出间隙，避免与表面网格共面闪烁
    supportParams.maxCylinders = 350;//最多生成的圆柱数量（按重心高度从高到低截取）
    supportParams.radialSegments = 14;//圆柱侧面圆周分段数
    supportParams.sliceCircleSegments = 24;//参数化切片时圆截面离散段数

    const std::vector<SupportCylinder> supportCylinders = buildSupportCylinders(overhangResult, ourModel, supportParams);
    const std::vector<float> modelSliceY = extractLayerHeights(modelSliceLayers);
    supportSliceLayers = sliceSupportCylindersAtY(
        supportCylinders,
        modelSliceY,
        kContourLineWidth,
        supportParams.sliceCircleSegments);
    mergedSliceLayers = mergeModelAndSupportLayers(modelSliceLayers, supportSliceLayers);

    ParallelInfillParams infillParams;
    infillParams.spacing = 0.06f;
    infillParams.angleDeg = 45.0f;
    infillParams.lineWidth = kContourLineWidth;
    infillParams.minSegmentLength = 1e-4f;
    std::vector<Layer> plannedLayers = addParallelInfillToLayers(mergedSliceLayers, infillParams);
    std::cout << "[Infill] planned layers: " << plannedLayers.size() << " angle=" << infillParams.angleDeg
              << " spacing=" << infillParams.spacing << "\n";

    // 路径规划（层内顺序 + 允许反向 + 最近邻）：输出为“每层 PlannedPath 序列”
    const std::vector<std::vector<PlannedPath>> plannedOrder = planAllLayersPathOrder(plannedLayers);
    std::cout << "[Plan] plannedOrder layers: " << plannedOrder.size() << "\n";

    // 更新轮廓为“联合切片”结果（主体 + 支撑参数切片）
    releaseLineBatch(contourOuterVAO, contourOuterVBO);
    releaseLineBatch(contourInnerVAO, contourInnerVBO);
    buildContourGeometry(
        plannedLayers,
        outerLineVerts,
        outerLineFirst,
        outerLineCount,
        innerLineVerts,
        innerLineFirst,
        innerLineCount);
    buildInfillGeometry(plannedLayers, infillLineVerts, infillLineFirst, infillLineCount);
    uploadLineBatch(contourOuterVAO, contourOuterVBO, outerLineVerts);
    uploadLineBatch(contourInnerVAO, contourInnerVBO, innerLineVerts);
    uploadLineBatch(infillVAO, infillVBO, infillLineVerts);
    std::cout << "[SliceMerge] support layers: " << supportSliceLayers.size()
              << " merged contour strips outer=" << outerLineFirst.size()
              << " inner=" << innerLineFirst.size()
              << " infill=" << infillLineFirst.size() << "\n";

    Mesh supportMesh = buildSupportCylindersMesh(overhangResult, ourModel, supportParams);
    GLuint supportVAO = 0;
    GLuint supportVBO = 0;
    GLuint supportEBO = 0;
    GLsizei supportIndexCount = 0;
    uploadIndexedMeshPointLayout(supportMesh, supportVAO, supportVBO, supportEBO, supportIndexCount);
    std::cout << "[Support] cylinders mesh indices: " << supportIndexCount << "\n";

    // 调试开关：按 M 切换是否渲染“模型/支撑/悬垂面片”，便于只看轮廓与填充
    bool drawSolidScene = true;
    // 调试开关：按 N 切换是否渲染“支撑圆柱实体”
    bool drawSupportSolid = true;
    bool prevMPressed = false;
    bool prevNPressed = false;

    // 播放模式：按 B 从底层开始按规划顺序逐段打印
    PlaybackState playback;
    playback.speed = 1.2f;
    DynamicLineBatch playbackBatch;
    ensureDynamicLineBatch(playbackBatch);

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        const bool mPressed = (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS);
        if (mPressed && !prevMPressed)
        {
            drawSolidScene = !drawSolidScene;
            std::cout << "[Debug] M toggle drawSolidScene=" << (drawSolidScene ? "ON" : "OFF") << "\n";
        }
        prevMPressed = mPressed;

        const bool nPressed = (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS);
        if (nPressed && !prevNPressed)
        {
            drawSupportSolid = !drawSupportSolid;
            std::cout << "[Debug] N toggle drawSupportSolid=" << (drawSupportSolid ? "ON" : "OFF") << "\n";
        }
        prevNPressed = nPressed;

        const bool bPressed = (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS);
        if (bPressed && !playback.prevBPressed)
        {
            playback.enabled = !playback.enabled;
            playback.lastT = glfwGetTime();
            playback.layerIdx = 0;
            playback.plannedIdx = 0;
            playback.segIdx = 0;
            playback.segU = 0.0f;
            playback.curPts.clear();
            playbackBatch.verts.clear();
            playbackBatch.first.clear();
            playbackBatch.count.clear();
            updateDynamicLineBatch(playbackBatch);
            std::cout << "[Debug] B toggle playback=" << (playback.enabled ? "ON" : "OFF") << "\n";
        }
        playback.prevBPressed = bPressed;

        // render
        // ------
        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection = glm::perspective(
            glm::radians(fov),
            static_cast<float>(SCR_WIDTH) / static_cast<float>(SCR_HEIGHT),
            0.1f,
            100.0f);
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
            
        if (drawSolidScene)
        {
            ourShader.begin();
            ourShader.setMat4x4("projection", projection);
            ourShader.setMat4x4("view", view);

            // 由于导入时已经把 bakeModel 烘焙到顶点坐标，这里传 identity，避免重复变换。
            ourShader.setMat4x4("model", glm::mat4(1.0f));
            float modelRgb[3] = {0.2f, 0.5f, 0.8f};
            float supportRgb[3] = {0.92f, 0.48f, 0.10f};
            ourShader.setFloat("uBaseColor", modelRgb);
            ourModel.Draw(ourShader);

            if (drawSupportSolid && supportVAO != 0 && supportIndexCount > 0)
            {
                ourShader.setFloat("uBaseColor", supportRgb);
                glBindVertexArray(supportVAO);
                glDrawElements(GL_TRIANGLES, supportIndexCount, GL_UNSIGNED_INT, nullptr);
                glBindVertexArray(0);
            }

            ourShader.end();
        }

        // 叠加绘制切片轮廓（与模型同一 model 矩阵：烘焙后为单位阵）
        glLineWidth(2.0f);
        lineShader.begin();
        lineShader.setMat4x4("projection", projection);
        lineShader.setMat4x4("view", view);
        lineShader.setMat4x4("model", glm::mat4(1.0f));

        const bool drawStaticPaths = !playback.enabled;
        if (drawStaticPaths && contourOuterVAO != 0 && !outerLineFirst.empty())
        {
            float rgb[3] = {0.25f, 1.0f, 0.45f};
            lineShader.setFloat("uColor", rgb);
            glBindVertexArray(contourOuterVAO);
            glMultiDrawArrays(
                GL_LINE_STRIP,
                outerLineFirst.data(),
                outerLineCount.data(),
                static_cast<GLsizei>(outerLineFirst.size()));
            glBindVertexArray(0);
        }
        if (drawStaticPaths && contourInnerVAO != 0 && !innerLineFirst.empty())
        {
            float rgb[3] = {1.0f, 0.55f, 0.15f};
            lineShader.setFloat("uColor", rgb);
            glBindVertexArray(contourInnerVAO);
            glMultiDrawArrays(
                GL_LINE_STRIP,
                innerLineFirst.data(),
                innerLineCount.data(),
                static_cast<GLsizei>(innerLineFirst.size()));
            glBindVertexArray(0);
        }
        if (drawStaticPaths && infillVAO != 0 && !infillLineFirst.empty())
        {
            float rgb[3] = {0.25f, 0.75f, 1.0f};
            lineShader.setFloat("uColor", rgb);
            glBindVertexArray(infillVAO);
            glMultiDrawArrays(
                GL_LINE_STRIP,
                infillLineFirst.data(),
                infillLineCount.data(),
                static_cast<GLsizei>(infillLineFirst.size()));
            glBindVertexArray(0);
        }

        // 播放模式：动态绘制已打印路径
        if (playback.enabled)
        {
            // 推进“打印头”位置
            double nowT = glfwGetTime();
            float remain = static_cast<float>(nowT - playback.lastT) * playback.speed;
            playback.lastT = nowT;

            while (remain > 1e-6f)
            {
                if (playback.layerIdx >= plannedLayers.size())
                    break;

                const float layerY = plannedLayers[playback.layerIdx].getZHeight();
                (void)layerY;

                if (playback.plannedIdx >= plannedOrder[playback.layerIdx].size())
                {
                    playback.layerIdx += 1;
                    playback.plannedIdx = 0;
                    playback.segIdx = 0;
                    playback.segU = 0.0f;
                    playback.curPts.clear();
                    continue;
                }

                const PlannedPath& pp = plannedOrder[playback.layerIdx][playback.plannedIdx];
                if (pp.path == nullptr)
                {
                    playback.plannedIdx += 1;
                    playback.segIdx = 0;
                    playback.segU = 0.0f;
                    playback.curPts.clear();
                    continue;
                }

                if (playback.curPts.empty())
                    playback.curPts = pathPointsOrdered(*pp.path, pp.reverse);

                if (playback.curPts.size() < 2)
                {
                    playback.plannedIdx += 1;
                    playback.segIdx = 0;
                    playback.segU = 0.0f;
                    playback.curPts.clear();
                    continue;
                }

                if (playback.segIdx + 1 >= playback.curPts.size())
                {
                    // 完成该 path：把整条 strip 加入 batch
                    appendLineStripRaw(playback.curPts, playbackBatch.verts, playbackBatch.first, playbackBatch.count);
                    updateDynamicLineBatch(playbackBatch);
                    playback.plannedIdx += 1;
                    playback.segIdx = 0;
                    playback.segU = 0.0f;
                    playback.curPts.clear();
                    continue;
                }

                const glm::vec3 a = playback.curPts[playback.segIdx];
                const glm::vec3 b = playback.curPts[playback.segIdx + 1];
                const float segLen = glm::length(b - a);
                if (segLen <= 1e-6f)
                {
                    playback.segIdx += 1;
                    playback.segU = 0.0f;
                    continue;
                }

                const float adv = remain;
                const float left = (1.0f - playback.segU) * segLen;
                if (adv < left)
                {
                    playback.segU += adv / segLen;
                    remain = 0.0f;
                }
                else
                {
                    remain -= left;
                    playback.segIdx += 1;
                    playback.segU = 0.0f;
                }
            }

            // 绘制：已完成的 strips + 当前正在打印的 strip（部分）
            float rgb[3] = {1.0f, 1.0f, 0.35f};
            lineShader.setFloat("uColor", rgb);
            glBindVertexArray(playbackBatch.vao);
            if (!playbackBatch.first.empty())
            {
                glMultiDrawArrays(
                    GL_LINE_STRIP,
                    playbackBatch.first.data(),
                    playbackBatch.count.data(),
                    static_cast<GLsizei>(playbackBatch.first.size()));
            }
            glBindVertexArray(0);

            if (!playback.curPts.empty() && playback.layerIdx < plannedOrder.size() && playback.plannedIdx < plannedOrder[playback.layerIdx].size())
            {
                // 当前 strip：取 [0..segIdx] + 当前插值点
                std::vector<glm::vec3> partial;
                partial.reserve(playback.segIdx + 2);
                for (size_t i = 0; i <= playback.segIdx && i < playback.curPts.size(); ++i)
                    partial.push_back(playback.curPts[i]);
                if (playback.segIdx + 1 < playback.curPts.size())
                {
                    const glm::vec3 a = playback.curPts[playback.segIdx];
                    const glm::vec3 b = playback.curPts[playback.segIdx + 1];
                    partial.push_back(a + (b - a) * playback.segU);
                }
                if (partial.size() >= 2)
                {
                    DynamicLineBatch tmp;
                    tmp.verts.reserve(partial.size() * 3);
                    tmp.first.reserve(1);
                    tmp.count.reserve(1);
                    appendLineStripRaw(partial, tmp.verts, tmp.first, tmp.count);
                    ensureDynamicLineBatch(tmp);
                    updateDynamicLineBatch(tmp);
                    glBindVertexArray(tmp.vao);
                    glMultiDrawArrays(GL_LINE_STRIP, tmp.first.data(), tmp.count.data(), 1);
                    glBindVertexArray(0);
                    releaseLineBatch(tmp.vao, tmp.vbo);
                }
            }
        }
        lineShader.end();

        // 叠加绘制悬垂三角片（实体高亮）
        if (drawSolidScene && overhangVAO != 0 && !overhangTriVerts.empty())
        {
            lineShader.begin();
            lineShader.setMat4x4("projection", projection);
            lineShader.setMat4x4("view", view);
            lineShader.setMat4x4("model", glm::mat4(1.0f));
            float rgb[3] = {1.0f, 0.15f, 0.8f};
            lineShader.setFloat("uColor", rgb);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(-1.0f, -1.0f);
            glBindVertexArray(overhangVAO);
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(overhangTriVerts.size() / 3));
            glBindVertexArray(0);
            glDisable(GL_POLYGON_OFFSET_FILL);
            lineShader.end();
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    releaseLineBatch(contourOuterVAO, contourOuterVBO);
    releaseLineBatch(contourInnerVAO, contourInnerVBO);
    releaseLineBatch(infillVAO, infillVBO);
    releaseLineBatch(overhangVAO, overhangVBO);
    releaseIndexedMesh(supportVAO, supportVBO, supportEBO);

    glfwTerminate();
    return 0;
}

