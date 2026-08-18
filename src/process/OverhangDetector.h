#pragma once

// 悬垂区域识别：① 几何法向初筛 ② 可选：离散切片层间支撑覆盖二次验证（与论文思路一致）

#include "../dataStructure/Layer.h"
#include "../dataStructure/Model.h"
#include "OverhangLayerSupport.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

/// @brief 识别参数：阈值、堆积方向、以及可选的“层间重叠”二次验证输入
struct OverhangParams
{
    /// 法向与堆积方向夹角超过该值（度）才进入悬垂候选
    float angleThresholdDeg = 45.0f;

    /// 堆积方向（内部归一化）；与切片轴 +Y 一致时表示自下而上堆积
    glm::vec3 buildDirection = glm::vec3(0.0f, 1.0f, 0.0f);

    /// 仅保留下朝面（n·buildDir < 0），减轻竖直壁误报
    bool requireDownFacing = true;

    /// 法向与堆积方向接近垂直（竖壁）时一律不判悬垂：剖分/数值噪声常使 n·up 略负，侧面竖棱易被误判。
    /// 若 |n·buildDir| ≤ sin(slackDeg)，视为竖壁并跳过；设为 0 关闭该规则。
    float verticalWallAngleSlackDeg = 15.0f;

    /// 面积过小的三角形视为退化，跳过
    float minTriangleArea = 1e-9f;

    // ----- 二次验证（层间 XZ 投影 / 栅格支撑占比）-----

    /// 为 true 且下面指针非空时：在几何候选基础上用相邻切片实体掩码做局部支撑判定
    bool enableLayerOverlapSecondPass = false;

    /// 与 `Slicer::sliceToLayersUniform` 得到的层序列一致（通常同一模型、同一层数）
    const std::vector<Layer>* sliceLayersForOverlapSecondPass = nullptr;

    /// XZ 平面正方形栅格边长（越大越细，越慢）
    int overlapRasterResolution = 128;

    /// 局部窗口内，“本层实体格被紧邻下层（膨胀后）覆盖”的比例若 **≥** 该阈值，则认为该处有充分竖向支撑，**剔除**该三角（减少法向误检）
    /// 论文对应：重叠度足够高 → 不判为需支撑的悬垂
    float overlapSupportKeepThreshold = 0.8f;

    /// 局部正方形邻域半宽（格），例如 2 表示 5×5 窗口
    int overlapLocalWindowHalfCells = 2;
};

struct OverhangTriangle
{
    size_t meshIndex = 0;
    size_t triIndex = 0;

    glm::vec3 centroid = glm::vec3(0.0f);
    glm::vec3 normal = glm::vec3(0.0f);
    float angleToBuildDirDeg = 0.0f;
    float area = 0.0f;
};

struct OverhangResult
{
    size_t totalTriangles = 0;
    size_t skippedDegenerateTriangles = 0;

    /// 几何初筛得到的候选数（二次验证开启时填写）
    size_t candidateCountAfterGeometry = 0;

    /// 二次验证剔除数量（未开启时为 0）
    size_t removedByLayerSecondPass = 0;

    std::vector<OverhangTriangle> triangles;
};

class OverhangDetector
{
public:
    /// @brief 几何初筛 + 可选层间二次验证
    static OverhangResult detect(const Model& model, const OverhangParams& params = OverhangParams{})
    {
        constexpr float kPi = 3.14159265358979323846f;
        OverhangResult result;

        glm::vec3 buildDir = params.buildDirection;
        if (glm::length(buildDir) < 1e-8f)
            buildDir = glm::vec3(0.0f, 1.0f, 0.0f);
        buildDir = glm::normalize(buildDir);

        std::vector<OverhangTriangle> geomCandidates;
        geomCandidates.reserve(1024);

        for (size_t meshIdx = 0; meshIdx < model.meshes.size(); ++meshIdx)
        {
            const NewMesh& mesh = model.meshes[meshIdx];
            const std::vector<Point3D>& vertices = mesh.vertices;
            const std::vector<unsigned int>& indices = mesh.indices;
            const size_t triCount = indices.size() / 3;

            for (size_t triIdx = 0; triIdx < triCount; ++triIdx)
            {
                result.totalTriangles += 1;

                const unsigned int i0 = indices[triIdx * 3 + 0];
                const unsigned int i1 = indices[triIdx * 3 + 1];
                const unsigned int i2 = indices[triIdx * 3 + 2];
                if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size())
                {
                    result.skippedDegenerateTriangles += 1;
                    continue;
                }

                const glm::vec3& p0 = vertices[i0].Position;
                const glm::vec3& p1 = vertices[i1].Position;
                const glm::vec3& p2 = vertices[i2].Position;

                const glm::vec3 rawNormal = glm::cross(p1 - p0, p2 - p0);
                const float rawLen = glm::length(rawNormal);
                const float area = 0.5f * rawLen;
                if (area <= params.minTriangleArea || rawLen <= 1e-10f)
                {
                    result.skippedDegenerateTriangles += 1;
                    continue;
                }

                const glm::vec3 normal = rawNormal / rawLen;
                const float dotBuild = glm::dot(normal, buildDir);

                if (params.verticalWallAngleSlackDeg > 1e-5f)
                {
                    const float sinSlack =
                        std::sin(params.verticalWallAngleSlackDeg * (kPi / 180.0f));
                    if (std::fabs(dotBuild) <= sinSlack)
                        continue;
                }

                if (params.requireDownFacing && dotBuild >= 0.0f)
                    continue;

                const float clampedDot = std::max(-1.0f, std::min(1.0f, dotBuild));
                const float angle = std::acos(clampedDot) * (180.0f / kPi);
                if (angle <= params.angleThresholdDeg)
                    continue;

                OverhangTriangle tri;
                tri.meshIndex = meshIdx;
                tri.triIndex = triIdx;
                tri.centroid = (p0 + p1 + p2) / 3.0f;
                tri.normal = normal;
                tri.area = area;
                tri.angleToBuildDirDeg = angle;
                geomCandidates.push_back(tri);
            }
        }

        result.candidateCountAfterGeometry = geomCandidates.size();

        if (!params.enableLayerOverlapSecondPass || params.sliceLayersForOverlapSecondPass == nullptr ||
            params.sliceLayersForOverlapSecondPass->empty() || params.overlapRasterResolution < 8)
        {
            result.triangles = std::move(geomCandidates);
            return result;
        }

        // ---------- 二次验证 ----------
        float yMin = 0.0f;
        float yMax = 0.0f;
        float xmin = 0.0f;
        float xmax = 0.0f;
        float zmin = 0.0f;
        float zmax = 0.0f;
        computeModelBoundsXYZ(model, yMin, yMax, xmin, xmax, zmin, zmax);

        const float xPad = std::max(1e-4f, (xmax - xmin) * 1e-4f);
        const float zPad = std::max(1e-4f, (zmax - zmin) * 1e-4f);
        xmin -= xPad;
        xmax += xPad;
        zmin -= zPad;
        zmax += zPad;

        const int nx = params.overlapRasterResolution;
        const int nz = nx;
        overhang_layer_detail::XZGridSpec grid;
        grid.xmin = xmin;
        grid.xmax = xmax;
        grid.zmin = zmin;
        grid.zmax = zmax;
        grid.nx = nx;
        grid.nz = nz;

        const std::vector<Layer>& layers = *params.sliceLayersForOverlapSecondPass;
        const int layerCount = static_cast<int>(layers.size());

        std::vector<std::vector<uint8_t>> solidMasks(static_cast<size_t>(layerCount));
        std::vector<std::vector<uint8_t>> dilatedBelow(static_cast<size_t>(layerCount));

        for (int L = 0; L < layerCount; ++L)
        {
            overhang_layer_detail::rasterizeLayerSolidMask(layers[static_cast<size_t>(L)], grid, solidMasks[static_cast<size_t>(L)]);
            if (L > 0)
            {
                overhang_layer_detail::dilateChebyshev1(
                    nx,
                    nz,
                    solidMasks[static_cast<size_t>(L - 1)],
                    dilatedBelow[static_cast<size_t>(L)]);
            }
        }

        result.triangles.clear();
        result.triangles.reserve(geomCandidates.size());

        for (OverhangTriangle& tri : geomCandidates)
        {
            const int Lidx = layerIndexFromCentroidY(tri.centroid.y, yMin, yMax, layerCount);
            if (Lidx <= 0)
            {
                result.triangles.push_back(tri);
                continue;
            }

            int ix = 0;
            int iz = 0;
            centroidToGridIndices(tri.centroid.x, tri.centroid.z, grid, ix, iz);

            const float ratio = overhang_layer_detail::localSupportCoverageRatio(
                solidMasks[static_cast<size_t>(Lidx)],
                dilatedBelow[static_cast<size_t>(Lidx)],
                nx,
                nz,
                ix,
                iz,
                params.overlapLocalWindowHalfCells);

            if (ratio < 0.0f)
            {
                result.triangles.push_back(tri);
                continue;
            }

            if (ratio >= params.overlapSupportKeepThreshold)
            {
                result.removedByLayerSecondPass += 1;
                continue;
            }

            result.triangles.push_back(tri);
        }

        return result;
    }

private:
    static void computeModelBoundsXYZ(
        const Model& model,
        float& yMin,
        float& yMax,
        float& xMin,
        float& xMax,
        float& zMin,
        float& zMax)
    {
        bool first = true;
        for (const NewMesh& mesh : model.meshes)
        {
            for (const Point3D& v : mesh.vertices)
            {
                const float x = v.x();
                const float y = v.y();
                const float z = v.z();
                if (first)
                {
                    xMin = xMax = x;
                    yMin = yMax = y;
                    zMin = zMax = z;
                    first = false;
                }
                else
                {
                    xMin = std::min(xMin, x);
                    xMax = std::max(xMax, x);
                    yMin = std::min(yMin, y);
                    yMax = std::max(yMax, y);
                    zMin = std::min(zMin, z);
                    zMax = std::max(zMax, z);
                }
            }
        }
        if (first)
        {
            yMin = yMax = xMin = xMax = zMin = zMax = 0.0f;
        }
    }

    /// @brief 与 `Slicer::sliceToLayersUniform` 中层带划分一致：第 i 层对应 y ∈ [yMin+i*dy, yMin+(i+1)*dy)
    static int layerIndexFromCentroidY(float y, float yMin, float yMax, int layerCount)
    {
        if (layerCount <= 0)
            return 0;
        if (layerCount == 1)
            return 0;

        const float ySpan = yMax - yMin;
        if (ySpan <= 1e-6f)
            return 0;

        const float dy = ySpan / static_cast<float>(layerCount);
        float t = (y - yMin) / dy;
        int L = static_cast<int>(std::floor(t));
        if (L < 0)
            L = 0;
        if (L >= layerCount)
            L = layerCount - 1;
        return L;
    }

    static void centroidToGridIndices(float x, float z, const overhang_layer_detail::XZGridSpec& g, int& ix, int& iz)
    {
        const float xspan = g.xmax - g.xmin;
        const float zspan = g.zmax - g.zmin;
        if (xspan <= 1e-12f || zspan <= 1e-12f || g.nx <= 0 || g.nz <= 0)
        {
            ix = iz = 0;
            return;
        }
        int tix = static_cast<int>(std::floor((x - g.xmin) / xspan * static_cast<float>(g.nx)));
        int tiz = static_cast<int>(std::floor((z - g.zmin) / zspan * static_cast<float>(g.nz)));
        ix = std::max(0, std::min(tix, g.nx - 1));
        iz = std::max(0, std::min(tiz, g.nz - 1));
    }
};
