#pragma once

#include <cstdint>//uint32_t
#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>

#include "Point3D.h"

// 约定：二维切片平面坐标使用 glm::vec2，其中
// - vec2.x 对应 3D 的 X
// - vec2.y 对应 3D 的 Z
// 层高使用 3D 的 Y（对应你的 Path 生成逻辑：Point3D(x, height, z)）。
//意思是，切片方向在opengl的3D当中体现为y轴，为竖直方向

enum class SliceAxis : uint8_t
{
    X = 0,
    Y = 1,
    Z = 2
};

// ============================================================
/// @class Triangle3D
/// @brief  一个三角形，包含三个顶点索引,切片轴方向上的最小值和最大值。
/// ===========================================================
class Triangle3D
{
public:
    Triangle3D() : v0_(0), v1_(0), v2_(0), axisMin_(0.0f), axisMax_(0.0f) {}
    Triangle3D(uint32_t v0, uint32_t v1, uint32_t v2)
        : v0_(v0), v1_(v1), v2_(v2), axisMin_(0.0f), axisMax_(0.0f) {}

    uint32_t v0() const { return v0_; }
    uint32_t v1() const { return v1_; }
    uint32_t v2() const { return v2_; }

    float axisMin() const { return axisMin_; }
    float axisMax() const { return axisMax_; }

    void setAxisBounds(float mn, float mx)
    {
        axisMin_ = mn;
        axisMax_ = mx;
    }

    // 在指定轴（默认为 Y）上计算 min/max，用于“只遍历跨越切平面的三角形”剪枝。
    void computeAxisBounds(const std::vector<Point3D>& vertices, SliceAxis axis)
    {
        axisMin_ = axisValue(vertices[v0_], axis);
        axisMax_ = axisMin_;

        updateAxisBounds(vertices[v1_], axis);
        updateAxisBounds(vertices[v2_], axis);
    }

private:
    uint32_t v0_;//顶点索引
    uint32_t v1_;
    uint32_t v2_;

    // 记录切片轴方向上的最小/最大值（例如沿 Y 切片就记录 yMin/yMax）
    float axisMin_;
    float axisMax_;

    static float axisValue(const Point3D& p, SliceAxis axis)
    {
        switch (axis)
        {
            case SliceAxis::X: return p.x();
            case SliceAxis::Y: return p.y();
            case SliceAxis::Z: return p.z();
            default: return p.y();
        }
    }

    void updateAxisBounds(const Point3D& p, SliceAxis axis)
    {
        float v = axisValue(p, axis);
        if (v < axisMin_) axisMin_ = v;
        if (v > axisMax_) axisMax_ = v;
    }
};

/// ============================================================
/// @class EdgeKey
/// @brief  一个边，包含两个顶点索引。
/// @details
/// - 构造时：接收两个顶点索引。
/// - a()：获取第一个顶点索引。
/// - b()：获取第二个顶点索引。
/// - 保证a() <= b()，从而让同一条边具有一致的 key。
/// - Hasher()：哈希函数。
/// ===========================================================

class EdgeKey
{
public:
    EdgeKey() : a_(0), b_(0) {}

    // 自动保证 a_ <= b_，从而让同一条边具有一致的 key。
    EdgeKey(uint32_t i0, uint32_t i1)
    {
        if (i0 <= i1)
        {
            a_ = i0;
            b_ = i1;
        }
        else
        {
            a_ = i1;
            b_ = i0;
        }
    }

    uint32_t a() const { return a_; }
    uint32_t b() const { return b_; }

    bool operator==(const EdgeKey& other) const
    {
        return a_ == other.a_ && b_ == other.b_;
    }

    struct Hasher
    {
        size_t operator()(const EdgeKey& k) const
        {
            // 组合哈希（不追求完美，但够用）
            size_t h1 = static_cast<size_t>(k.a_ * 2654435761u);
            size_t h2 = static_cast<size_t>(k.b_ * 1597334677u);
            return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
        }
    };

private:
    uint32_t a_;
    uint32_t b_;
};
/// ============================================================
/// @class SliceSegment
/// @brief  一个切片段，包含两个2D点，一个三角形索引，一个边键。
/// ===========================================================
/// @details
/// - 构造时：接收两个2D点，一个三角形索引，一个边键。
/// - from2D()：获取第一个2D点。
/// - to2D()：获取第二个2D点。
/// - triId()：获取三角形索引。
/// - edgeKey()：获取边键。
/// ===========================================================
class SliceSegment
{
public:
    SliceSegment() : from2D_(0.0f), to2D_(0.0f), triId_(0), edgeKey_() {}

    SliceSegment(const glm::vec2& from2D, const glm::vec2& to2D, uint32_t triId, const EdgeKey& edgeKey)
        : from2D_(from2D), to2D_(to2D), triId_(triId), edgeKey_(edgeKey) {}

    const glm::vec2& from2D() const { return from2D_; }
    const glm::vec2& to2D() const { return to2D_; }

    uint32_t triId() const { return triId_; }
    const EdgeKey& edgeKey() const { return edgeKey_; }

private:
    glm::vec2 from2D_;
    glm::vec2 to2D_;
    uint32_t triId_;
    EdgeKey edgeKey_;
};
// ============================================================
/// @class SliceLoop
/// @brief  一个切片环，包含多个2D点。
/// ===========================================================
/// @details
/// - 构造时：接收多个2D点。
/// - points2D()：获取所有2D点。
/// - signedAreaXZ()：计算有符号面积。
/// ===========================================================
class SliceLoop
{
public:
    void setPoints(const std::vector<glm::vec2>& pts) { points2D_ = pts; }
    void addPoint(const glm::vec2& p) { points2D_.push_back(p); }

    const std::vector<glm::vec2>& points2D() const { return points2D_; }
    std::vector<glm::vec2>& points2D() { return points2D_; }

    // 用 XZ 平面计算有符号面积；用于判断环的方向（外环/内环等）。
    // 返回值正负只和顶点顺序相关，不代表“外/内”的绝对语义。
    // 的是shoelace公式计算有符号面积
    float signedAreaXZ() const
    {
        const size_t n = points2D_.size();
        if (n < 3) return 0.0f;

        double sum = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            const glm::vec2& p0 = points2D_[i];
            const glm::vec2& p1 = points2D_[(i + 1) % n];
            sum += static_cast<double>(p0.x) * static_cast<double>(p1.y)
                - static_cast<double>(p1.x) * static_cast<double>(p0.y);
        }
        return static_cast<float>(0.5 * sum);
    }

private:
    std::vector<glm::vec2> points2D_;
};
/// ============================================================
/// @class SliceResult
/// @brief  一个切片结果，包含一个层高和多个切片环。
/// ===========================================================
/// @details
/// - 构造时：接收一个层高和多个切片环。
/// - z()：获取层高。
/// - loops()：获取所有切片环。
/// ===========================================================
class SliceResult
{
public:
    // 注意：这里的“z”只是为了和现有 `Layer::z_height` 命名保持一致。
    // 实际切片轴是 Y：切片平面是 y = z_
    void setZ(float z) { z_ = z; }
    float z() const { return z_; }

    const std::vector<SliceLoop>& loops() const { return loops_; }
    std::vector<SliceLoop>& loops() { return loops_; }

    void clear()
    {
        loops_.clear();
        z_ = 0.0f;
    }

private:
    // 对工程：z_ 表示切片平面的“层高”（也就是 3D 的 Y）。
    float z_ = 0.0f;
    std::vector<SliceLoop> loops_;
};
// ============================================================
/// @class MeshSliceTopology
/// @brief  一个面-边拓扑，包含多个点，多个三角形，和多个边。
/// ===========================================================
/// @details
/// - 构造时：接收多个点，多个三角形。
/// - vertices()：获取所有点。
/// - triangles()：获取所有三角形。
/// - edgeToTriangles()：获取边到三角形的映射。
/// - buildEdgeToTriangles()：构建边到三角形的映射。
/// - 面-边拓扑：用于“快速找与相交面片相邻的面片”，减少拼接过程中的无效搜索。
/// - 边映射的三角形id是此类当中存储在vector当中的三角形的索引
/// ===========================================================
class MeshSliceTopology
{
public:
    void setVertices(const std::vector<Point3D>& vertices) { vertices_ = vertices; }
    const std::vector<Point3D>& vertices() const { return vertices_; }

    void setTriangles(const std::vector<Triangle3D>& triangles) { triangles_ = triangles; }
    const std::vector<Triangle3D>& triangles() const { return triangles_; }

    const std::unordered_map<EdgeKey, std::vector<uint32_t>, EdgeKey::Hasher>& edgeToTriangles() const
    {
        return edgeToTriangles_;
    }

    // 根据 triangles_ 构建 edge -> 相邻 triangles 的映射表。
    // triangles_ 中每个三角形的 v0/v1/v2 必须是有效索引。
    
    void buildEdgeToTriangles()
    {
        edgeToTriangles_.clear();

        for (uint32_t tId = 0; tId < static_cast<uint32_t>(triangles_.size()); ++tId)
        {
            const Triangle3D& t = triangles_[tId];

            EdgeKey e0(t.v0(), t.v1());
            EdgeKey e1(t.v1(), t.v2());
            EdgeKey e2(t.v2(), t.v0());

            edgeToTriangles_[e0].push_back(tId);
            edgeToTriangles_[e1].push_back(tId);
            edgeToTriangles_[e2].push_back(tId);
        }
    }

private:
    std::vector<Point3D> vertices_;
    std::vector<Triangle3D> triangles_;
    std::unordered_map<EdgeKey, std::vector<uint32_t>, EdgeKey::Hasher> edgeToTriangles_;
};

