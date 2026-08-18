#pragma once

// 圆柱支撑（竖直，沿 +Y）：对候选三角形除重心外还在 XZ 上栅格化三角形投影（大三角不再只占一格），
// 每格保留最高点；超过 maxCylinders 时对栅格按 (ix,iz) 排序后等间距取样。
// 后续可接 CSG：此处仅输出独立 Mesh，不与主体做布尔。

#include "../dataStructure/Mesh.h"
#include "../dataStructure/Model.h"
#include "../dataStructure/Point3D.h"
#include "OverhangDetector.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

struct SupportCylinderParams
{
    /// 圆柱半径（截面圆半径）；内部生成时使用直径 = 2 * radius
    float radius = 0.025f;

    /// XZ 平面栅格边长：同一格内多个悬垂候选只保留「最高」的一个落柱点，控制密度
    float xzGridSpacing = 0.12f;

    /// 柱顶在三角重心下方留出间隙，避免与表面网格共面闪烁
    float tipInsetBelowCentroid = 0.02f;

    /// 最多生成的圆柱数量（按重心高度从高到低截取）
    size_t maxCylinders = 400;

    /// 圆柱侧面圆周分段数
    int radialSegments = 12;

    /// 参数化切片时，单个圆截面的离散段数（越大越圆）
    int sliceCircleSegments = 24;
};

struct SupportCylinder
{
    float cx = 0.0f;
    float cz = 0.0f;
    float yBottom = 0.0f;
    float yTop = 0.0f;
    float radius = 0.0f;
};

namespace support_cylinder_detail
{

inline float modelYMin(const Model& model)
{
    float yMin = std::numeric_limits<float>::infinity();
    for (const NewMesh& mesh : model.meshes)
    {
        for (const Point3D& v : mesh.vertices)
            yMin = std::min(yMin, v.y());
    }
    if (!std::isfinite(yMin))
        return 0.0f;
    return yMin;
}

struct IZKey
{
    int ix = 0;
    int iz = 0;
    bool operator==(const IZKey& o) const { return ix == o.ix && iz == o.iz; }
};

struct IZKeyHash
{
    size_t operator()(const IZKey& k) const
    {
        return static_cast<size_t>(k.ix * 73856093) ^ static_cast<size_t>(k.iz * 19349663 + 1);
    }
};

inline void upsertCellBest(
    std::unordered_map<IZKey, glm::vec3, IZKeyHash>& cellBest,
    float spacing,
    const glm::vec3& p)
{
    const int ix = static_cast<int>(std::floor(p.x / spacing));
    const int iz = static_cast<int>(std::floor(p.z / spacing));
    const IZKey key{ix, iz};
    const auto it = cellBest.find(key);
    if (it == cellBest.end() || p.y > it->second.y)
        cellBest[key] = p;
}

/// @brief 点 (x,z) 是否在三角形 XZ 投影内（含边界容差）
inline bool pointInTriangleXZ(float x, float z, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    const glm::vec2 p(x, z);
    const glm::vec2 v0(a.x, a.z);
    const glm::vec2 v1(b.x, b.z);
    const glm::vec2 v2(c.x, c.z);
    const glm::vec2 ab = v1 - v0;
    const glm::vec2 ac = v2 - v0;
    const glm::vec2 ap = p - v0;
    const float d00 = glm::dot(ab, ab);
    const float d01 = glm::dot(ab, ac);
    const float d11 = glm::dot(ac, ac);
    const float d20 = glm::dot(ap, ab);
    const float d21 = glm::dot(ap, ac);
    const float denom = d00 * d11 - d01 * d01;
    if (std::fabs(denom) < 1e-14f)
        return false;
    const float v = (d11 * d20 - d01 * d21) / denom;
    const float w = (d00 * d21 - d01 * d20) / denom;
    const float u = 1.0f - v - w;
    constexpr float e = -1e-4f;
    return u >= e && v >= e && w >= e;
}

/// @brief 三角形平面在 (x,z) 处对应的 y（三角形几乎竖直时退化）
inline float yOnTriangleAtXZ(float x, float z, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    const glm::vec2 p(x, z);
    const glm::vec2 v0(a.x, a.z);
    const glm::vec2 v1(b.x, b.z);
    const glm::vec2 v2(c.x, c.z);
    const glm::vec2 ab = v1 - v0;
    const glm::vec2 ac = v2 - v0;
    const glm::vec2 ap = p - v0;
    const float d00 = glm::dot(ab, ab);
    const float d01 = glm::dot(ab, ac);
    const float d11 = glm::dot(ac, ac);
    const float d20 = glm::dot(ap, ab);
    const float d21 = glm::dot(ap, ac);
    const float denom = d00 * d11 - d01 * d01;
    if (std::fabs(denom) < 1e-14f)
        return std::max(std::max(a.y, b.y), c.y);
    const float v = (d11 * d20 - d01 * d21) / denom;
    const float w = (d00 * d21 - d01 * d20) / denom;
    const float u = 1.0f - v - w;
    return u * a.y + v * b.y + w * c.y;
}

inline bool tryGetOverhangTriangleVerts(
    const Model& model,
    const OverhangTriangle& tri,
    glm::vec3& a,
    glm::vec3& b,
    glm::vec3& c)
{
    if (tri.meshIndex >= model.meshes.size())
        return false;
    const NewMesh& mesh = model.meshes[tri.meshIndex];
    const size_t base = tri.triIndex * 3;
    if (base + 2 >= mesh.indices.size())
        return false;
    const unsigned i0 = mesh.indices[base];
    const unsigned i1 = mesh.indices[base + 1];
    const unsigned i2 = mesh.indices[base + 2];
    if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size())
        return false;
    a = mesh.vertices[i0].Position;
    b = mesh.vertices[i1].Position;
    c = mesh.vertices[i2].Position;
    return true;
}

/// @brief 将三角形在 XZ 上的投影覆盖到的栅格逐一写入 cellBest（大包围盒时限幅步长避免单三角爆循环）
inline void rasterTriangleXZCoverage(
    std::unordered_map<IZKey, glm::vec3, IZKeyHash>& cellBest,
    float spacing,
    const glm::vec3& a,
    const glm::vec3& b,
    const glm::vec3& c)
{
    const float xmin = std::min(std::min(a.x, b.x), c.x);
    const float xmax = std::max(std::max(a.x, b.x), c.x);
    const float zmin = std::min(std::min(a.z, b.z), c.z);
    const float zmax = std::max(std::max(a.z, b.z), c.z);

    int ix0 = static_cast<int>(std::floor(xmin / spacing));
    int ix1 = static_cast<int>(std::floor(xmax / spacing));
    int iz0 = static_cast<int>(std::floor(zmin / spacing));
    int iz1 = static_cast<int>(std::floor(zmax / spacing));

    const long long nx = static_cast<long long>(ix1) - ix0 + 1;
    const long long nz = static_cast<long long>(iz1) - iz0 + 1;
    const long long total = nx * nz;
    int stride = 1;
    /// 单三角最多栅格采样数；过大三角用 stride 稀疏仍保留覆盖面
    constexpr long long kMaxRasterCells = 2048;
    if (total > kMaxRasterCells)
        stride = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(total) / static_cast<double>(kMaxRasterCells))));
    if (stride < 1)
        stride = 1;

    glm::vec3 n = glm::cross(b - a, c - a);
    const float nl = glm::length(n);
    const bool verticalish = nl < 1e-12f || (std::fabs(n.y / nl) < 0.06f);

    for (int ix = ix0; ix <= ix1; ix += stride)
    {
        for (int iz = iz0; iz <= iz1; iz += stride)
        {
            const float x = (static_cast<float>(ix) + 0.5f) * spacing;
            const float z = (static_cast<float>(iz) + 0.5f) * spacing;
            if (!pointInTriangleXZ(x, z, a, b, c))
                continue;
            float y = verticalish ? std::max(std::max(a.y, b.y), c.y) : yOnTriangleAtXZ(x, z, a, b, c);
            upsertCellBest(cellBest, spacing, glm::vec3(x, y, z));
        }
    }
}

/// @brief 在 XZ 上追加竖直圆柱（侧面法向朝外，顶/底法向 ±Y）
inline void appendVerticalCylinderY(
    Mesh& mesh,
    float cx,
    float cz,
    float yBottom,
    float yTop,
    float diameter,
    int segments)
{
    const float r = diameter * 0.5f;
    float y0 = yBottom;
    float y1 = yTop;
    if (y0 > y1)
        std::swap(y0, y1);
    if (y1 - y0 <= 1e-6f || r <= 1e-6f)
        return;
    if (segments < 3)
        segments = 3;

    constexpr float kTwoPi = 6.28318530717958647692f;
    for (int i = 0; i < segments; ++i)
    {
        const float a1 = kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
        const float a2 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(segments);
        const float c1 = std::cos(a1);
        const float s1 = std::sin(a1);
        const float c2 = std::cos(a2);
        const float s2 = std::sin(a2);

        Point3D b1(cx + r * c1, y0, cz + r * s1);
        Point3D b2(cx + r * c2, y0, cz + r * s2);
        Point3D t1(cx + r * c1, y1, cz + r * s1);
        Point3D t2(cx + r * c2, y1, cz + r * s2);
        const glm::vec3 n1(c1, 0.0f, s1);
        const glm::vec3 n2(c2, 0.0f, s2);
        b1.Normal = n1;
        b2.Normal = n2;
        t1.Normal = n1;
        t2.Normal = n2;

        mesh.addTriangle(b1, b2, t1);
        mesh.addTriangle(b2, t2, t1);
    }

    Point3D bc(cx, y0, cz);
    bc.Normal = glm::vec3(0.0f, -1.0f, 0.0f);
    Point3D tc(cx, y1, cz);
    tc.Normal = glm::vec3(0.0f, 1.0f, 0.0f);

    constexpr float kTwoPiCap = 6.28318530717958647692f;
    for (int i = 0; i < segments; ++i)
    {
        const float a1 = kTwoPiCap * static_cast<float>(i) / static_cast<float>(segments);
        const float a2 = kTwoPiCap * static_cast<float>(i + 1) / static_cast<float>(segments);
        Point3D b1(cx + r * std::cos(a1), y0, cz + r * std::sin(a1));
        Point3D b2(cx + r * std::cos(a2), y0, cz + r * std::sin(a2));
        b1.Normal = glm::vec3(0.0f, -1.0f, 0.0f);
        b2.Normal = glm::vec3(0.0f, -1.0f, 0.0f);
        mesh.addTriangle(bc, b2, b1);

        Point3D t1(cx + r * std::cos(a1), y1, cz + r * std::sin(a1));
        Point3D t2(cx + r * std::cos(a2), y1, cz + r * std::sin(a2));
        t1.Normal = glm::vec3(0.0f, 1.0f, 0.0f);
        t2.Normal = glm::vec3(0.0f, 1.0f, 0.0f);
        mesh.addTriangle(tc, t1, t2);
    }
}

} // namespace support_cylinder_detail

/// @brief 由悬垂结果生成支撑圆柱参数列表（用于参数化切片）
inline std::vector<SupportCylinder> buildSupportCylinders(
    const OverhangResult& overhang,
    const Model& model,
    const SupportCylinderParams& params = SupportCylinderParams{})
{
    std::vector<SupportCylinder> cylinders;
    if (overhang.triangles.empty() || params.xzGridSpacing <= 1e-6f || params.radius <= 1e-6f)
        return cylinders;

    const float spacing = params.xzGridSpacing;
    const float yPlatform = support_cylinder_detail::modelYMin(model);

    std::unordered_map<support_cylinder_detail::IZKey, glm::vec3, support_cylinder_detail::IZKeyHash> cellBest;
    for (const OverhangTriangle& tri : overhang.triangles)
    {
        support_cylinder_detail::upsertCellBest(cellBest, spacing, tri.centroid);
        glm::vec3 va;
        glm::vec3 vb;
        glm::vec3 vc;
        if (support_cylinder_detail::tryGetOverhangTriangleVerts(model, tri, va, vb, vc))
        {
            support_cylinder_detail::upsertCellBest(cellBest, spacing, va);
            support_cylinder_detail::upsertCellBest(cellBest, spacing, vb);
            support_cylinder_detail::upsertCellBest(cellBest, spacing, vc);
            support_cylinder_detail::rasterTriangleXZCoverage(cellBest, spacing, va, vb, vc);
        }
    }

    std::vector<std::pair<support_cylinder_detail::IZKey, glm::vec3>> items;
    items.reserve(cellBest.size());
    for (const auto& kv : cellBest)
        items.push_back({kv.first, kv.second});
    std::sort(items.begin(), items.end(), [](const auto& a, const auto& b) {
        if (a.first.ix != b.first.ix)
            return a.first.ix < b.first.ix;
        return a.first.iz < b.first.iz;
    });

    std::vector<glm::vec3> roots;
    if (items.size() <= params.maxCylinders)
    {
        roots.reserve(items.size());
        for (const auto& it : items)
            roots.push_back(it.second);
    }
    else
    {
        const size_t n = items.size();
        const size_t m = params.maxCylinders;
        roots.reserve(m);
        for (size_t k = 0; k < m; ++k)
        {
            const size_t idx = (m <= 1) ? 0 : (k * (n - 1)) / (m - 1);
            roots.push_back(items[idx].second);
        }
    }

    cylinders.reserve(roots.size());
    for (const glm::vec3& c : roots)
    {
        const float yTop = c.y - params.tipInsetBelowCentroid;
        if (yTop <= yPlatform + 1e-4f)
            continue;
        SupportCylinder one;
        one.cx = c.x;
        one.cz = c.z;
        one.yBottom = yPlatform;
        one.yTop = yTop;
        one.radius = params.radius;
        cylinders.push_back(one);
    }
    return cylinders;
}

/// @brief 对支撑圆柱做参数化切片：给定层高列表，每层生成圆形外轮廓 Path
inline std::vector<Layer> sliceSupportCylindersAtY(
    const std::vector<SupportCylinder>& cylinders,
    const std::vector<float>& sliceYList,
    float lineWidth,
    int circleSegments = 24)
{
    std::vector<Layer> layers;
    layers.reserve(sliceYList.size());
    const int seg = std::max(8, circleSegments);
    constexpr float kTwoPi = 6.28318530717958647692f;

    for (size_t li = 0; li < sliceYList.size(); ++li)
    {
        const float y = sliceYList[li];
        Layer layer(static_cast<int>(li), y);

        for (const SupportCylinder& c : cylinders)
        {
            if (y < c.yBottom || y > c.yTop)
                continue;

            Path p(PathType::OuterContour, lineWidth);
            for (int i = 0; i < seg; ++i)
            {
                const float a = kTwoPi * static_cast<float>(i) / static_cast<float>(seg);
                p.addPoint(Point3D(c.cx + c.radius * std::cos(a), y, c.cz + c.radius * std::sin(a)));
            }
            // 闭合
            p.addPoint(Point3D(c.cx + c.radius, y, c.cz));
            layer.addPath(p);
        }
        layers.push_back(std::move(layer));
    }

    return layers;
}

/// @brief 由悬垂检测结果生成圆柱支撑网格（世界坐标，与 Model 一致）
inline Mesh buildSupportCylindersMesh(
    const OverhangResult& overhang,
    const Model& model,
    const SupportCylinderParams& params = SupportCylinderParams{})
{
    Mesh mesh;
    const std::vector<SupportCylinder> cylinders = buildSupportCylinders(overhang, model, params);
    if (cylinders.empty())
        return mesh;

    const int seg = std::max(3, params.radialSegments);
    const float diameter = std::max(1e-6f, 2.0f * params.radius);
    for (const SupportCylinder& c : cylinders)
    {
        support_cylinder_detail::appendVerticalCylinderY(
            mesh, c.cx, c.cz, c.yBottom, c.yTop, diameter, seg);
    }

    return mesh;
}
