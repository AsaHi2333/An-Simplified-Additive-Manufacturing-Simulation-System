#pragma once

#include "../dataStructure/Layer.h"
#include "../dataStructure/Path.h"
#include "../dataStructure/Point3D.h"

#include <algorithm>
#include <cmath>
#include <vector>

//@brief 平行填充参数
struct ParallelInfillParams
{
    float spacing = 0.12f;
    float angleDeg = 45.0f;
    float lineWidth = 0.08f;
    float minSegmentLength = 1e-4f;
};

enum class InfillPattern
{
    ParallelLines = 0,
    ZigZag = 1
};

//@brief 路径规划器详细实现
namespace path_planner_detail
{
struct Vec2
{
    float x = 0.0f;
    float z = 0.0f;
};

//@brief 交点
struct Intersection
{
    float s = 0.0f;
    Vec2 p;
};

//@brief 将点转换为XZ坐标
inline Vec2 toXZ(const Point3D& p) { return Vec2{p.x(), p.z()}; }
//@brief 点积
inline float dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.z * b.z; }
//@brief 加法
inline Vec2 add(const Vec2& a, const Vec2& b) { return Vec2{a.x + b.x, a.z + b.z}; }
//@brief 减法
inline Vec2 sub(const Vec2& a, const Vec2& b) { return Vec2{a.x - b.x, a.z - b.z}; }
inline Vec2 mul(const Vec2& a, float t) { return Vec2{a.x * t, a.z * t}; }
//@brief 长度
inline float len(const Vec2& a) { return std::sqrt(dot(a, a)); }
//@brief 是否相等

///@brief 是否相等
inline bool almostEqual(float a, float b, float eps = 1e-6f) { return std::fabs(a - b) <= eps; }
///@brief 点是否在多边形内,使用射线法
///@param x 点的X坐标
///@param z 点的Z坐标
///@param poly 多边形
///@return 是否在多边形内
///@details 射线法：从点向右画一条射线，如果射线与多边形相交奇数次，则点在多边形内，否则在外。
inline bool pointInPolygonXZ(float x, float z, const std::vector<Point3D>& poly)
{
    if (poly.size() < 3)
        return false;
    bool inside = false;
    const size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++)
    {
        const float xi = poly[i].x();
        const float zi = poly[i].z();
        const float xj = poly[j].x();
        const float zj = poly[j].z();
        const bool cond = ((zi > z) != (zj > z));
        if (!cond)
            continue;
        const float denom = (zj - zi);
        if (std::fabs(denom) < 1e-12f)
            continue;
        //@brief 计算交点X坐标
        const float xCross = (xj - xi) * (z - zi) / denom + xi;
        if (x < xCross)//
            inside = !inside;
    }
    return inside;
}


/// @brief 判断点是否在实体内
/// @param x 点的X坐标
/// @param z 点的Z坐标
/// @param outerPaths 外轮廓路径集合
/// @param innerPaths 内轮廓路径集合
/// @return 是否在实体内
/// @details 如果点在外轮廓内，则返回true，否则返回false。如果点在内轮廓内，则返回false，否则返回true。
inline bool isInsideSolid(
    float x,
    float z,
    const std::vector<Path>& outerPaths,//外轮廓路径集合
    const std::vector<Path>& innerPaths)//内轮廓路径集合
{
    bool inOuter = false;
    for (const Path& p : outerPaths)//遍历外轮廓路径集合    
    {
        if (pointInPolygonXZ(x, z, p.getPoints()))
        {
            inOuter = true;
            break;
        }
    }
    if (!inOuter)
        return false;

    for (const Path& p : innerPaths)
    {
        if (pointInPolygonXZ(x, z, p.getPoints()))
            return false;
    }
    return true;
}

/// @brief 判断线段是否完全落在「实体」内（用于 Z 型填充相邻扫描段的连接）
/// @details 仅用中点判断时，弦可能穿过轮廓外空隙却落在另一块外轮廓（如支撑/悬垂切片）内，
///          中点仍被判为体内，从而产生穿过外轮廓的错误连接线；平行填充无此连接故无该问题。
///          这里沿弦做多个采样点，要求全部在 isInsideSolid 内才允许连成一段折线。
inline bool chordFullyInsideSolid(
    float ax,
    float az,
    float bx,
    float bz,
    const std::vector<Path>& outerPaths,
    const std::vector<Path>& innerPaths,
    float spacingRef)
{
    const float dx = bx - ax;
    const float dz = bz - az;
    const float L = std::sqrt(dx * dx + dz * dz);
    int steps = 3;
    if (spacingRef > 1e-6f)
        steps = 2 + static_cast<int>(std::ceil(L / (spacingRef * 0.4f)));
    if (steps < 3)
        steps = 3;
    if (steps > 64)
        steps = 64;
    for (int i = 0; i <= steps; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const float x = ax + dx * t;
        const float z = az + dz * t;
        if (!isInsideSolid(x, z, outerPaths, innerPaths))
            return false;
    }
    return true;
}

/// @brief 体判断（略放宽）：交点常在轮廓边上，射线法边界不稳定，跨桥采样用此减少误判。
inline bool isInsideSolidLenient(
    float x,
    float z,
    const std::vector<Path>& outerPaths,
    const std::vector<Path>& innerPaths)
{
    if (isInsideSolid(x, z, outerPaths, innerPaths))
        return true;
    constexpr float e = 4e-4f;
    return isInsideSolid(x + e, z, outerPaths, innerPaths) || isInsideSolid(x - e, z, outerPaths, innerPaths)
        || isInsideSolid(x, z + e, outerPaths, innerPaths) || isInsideSolid(x, z - e, outerPaths, innerPaths);
}

/// @brief Z 型「相邻扫描行、同列」之间的跨桥是否可接受（比 chordFullyInsideSolid 更适合短跨桥）
/// @param deltaCAlongNrm 两行在填充法向 nrm 上的 c 差（≈spacing；中间跳过空行时会更大）
inline bool zigZagBridgeBetweenScanLines(
    float ax,
    float az,
    float bx,
    float bz,
    const std::vector<Path>& outerPaths,
    const std::vector<Path>& innerPaths,
    float spacingRef,
    float deltaCAlongNrm)
{
    const float dx = bx - ax;
    const float dz = bz - az;
    const float L = std::sqrt(dx * dx + dz * dz);
    if (L < 1e-7f)
        return true;

    const float su = std::max(spacingRef, 1e-6f);
    const float dc = std::max(deltaCAlongNrm, su * 0.2f);
    // 典型跨桥长度 ≈ sqrt((Δc)^2 + (沿填充方向错位)^2)，给一定余量；过长则可能是跨岛/穿洞
    const float maxLoose = dc * 2.35f + su * 3.2f;
    const float maxAllow = std::max(su * 12.0f, dc * 5.0f + su * 6.0f);
    if (L > maxAllow)
        return false;

    const int steps = (L <= maxLoose) ? 5 : std::max(6, std::min(28, 3 + static_cast<int>(std::ceil(L / (su * 0.9f)))));
    for (int i = 0; i <= steps; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const float x = ax + dx * t;
        const float z = az + dz * t;
        if (!isInsideSolidLenient(x, z, outerPaths, innerPaths))
            return false;
    }
    return true;
}


/// @brief 收集路径的交点
/// @param path 路径
/// @param dir 方向
/// @param nrm 法向
/// @param c 距离
/// @param out 交点集合
/// @details 收集路径的交点，如果交点在实体内，则添加到交点集合中。
inline void collectIntersectionsForPath(
    const Path& path,
    const Vec2& dir,
    const Vec2& nrm,
    float c,
    std::vector<Intersection>& out)
{
    const std::vector<Point3D>& pts3 = path.getPoints();
    if (pts3.size() < 2)
        return;
    
    const auto emitEdge = [&](const Vec2& p0, const Vec2& p1) {
        
        const float v0 = dot(nrm, p0) - c;
        const float v1 = dot(nrm, p1) - c;
        const float dv = v0 - v1;
        if (std::fabs(dv) < 1e-9f)
            return;
        if ((v0 > 0.0f && v1 > 0.0f) || (v0 < 0.0f && v1 < 0.0f))
            return;
        const float t = v0 / dv;
        if (t < -1e-6f || t > 1.0f + 1e-6f)
            return;
        const Vec2 p = add(p0, mul(sub(p1, p0), t));
        out.push_back(Intersection{dot(dir, p), p});
    };

    for (size_t i = 0; i + 1 < pts3.size(); ++i)
        emitEdge(toXZ(pts3[i]), toXZ(pts3[i + 1]));

    const Vec2 first = toXZ(pts3.front());
    const Vec2 last = toXZ(pts3.back());
    if (!almostEqual(first.x, last.x) || !almostEqual(first.z, last.z))
        emitEdge(last, first);
}
} // namespace path_planner_detail


/// @brief 添加平行填充到层
/// @param layer 层
/// @param params 平行填充参数
/// @return 添加平行填充到层后的层
/// @details 添加平行填充到层，如果层没有外轮廓，则返回空层。如果平行填充参数的间距小于1e-6f，则返回空层。
inline Layer addParallelInfillToLayer(const Layer& layer, const ParallelInfillParams& params)
{
    Layer out(layer.getLayerId(), layer.getZHeight());
    for (const Path& p : layer.getOuterContourPaths())
        out.addPath(p);
    for (const Path& p : layer.getInnerContourPaths())
        out.addPath(p);
    for (const Path& p : layer.getInfillPaths())
        out.addPath(p);

    const std::vector<Path>& outers = layer.getOuterContourPaths();
    if (outers.empty() || params.spacing <= 1e-6f)
        return out;

    constexpr float kPi = 3.14159265358979323846f;
    const float a = params.angleDeg * (kPi / 180.0f);
    const path_planner_detail::Vec2 dir{std::cos(a), std::sin(a)};
    const path_planner_detail::Vec2 nrm{-dir.z, dir.x};

    float cMin = 0.0f;
    float cMax = 0.0f;
    bool first = true;
    for (const Path& p : outers)
    {
        for (const Point3D& q : p.getPoints())
        {
            const path_planner_detail::Vec2 v{q.x(), q.z()};
            const float c = path_planner_detail::dot(nrm, v);
            if (first)
            {
                cMin = cMax = c;
                first = false;
            }
            else
            {
                cMin = std::min(cMin, c);
                cMax = std::max(cMax, c);
            }
        }
    }
    if (first)
        return out;

    cMin -= params.spacing;
    cMax += params.spacing;

    std::vector<path_planner_detail::Intersection> xs;
    xs.reserve(256);

    for (float c = cMin; c <= cMax; c += params.spacing)
    {
        xs.clear();
        for (const Path& p : layer.getOuterContourPaths())
            path_planner_detail::collectIntersectionsForPath(p, dir, nrm, c, xs);
        for (const Path& p : layer.getInnerContourPaths())
            path_planner_detail::collectIntersectionsForPath(p, dir, nrm, c, xs);
        if (xs.size() < 2)
            continue;

        std::sort(xs.begin(), xs.end(), [](const auto& a1, const auto& a2) { return a1.s < a2.s; });
        std::vector<path_planner_detail::Intersection> uniq;
        uniq.reserve(xs.size());
        for (const auto& it : xs)
        {
            if (!uniq.empty() && std::fabs(uniq.back().s - it.s) < 1e-5f)
                continue;
            uniq.push_back(it);
        }
        if (uniq.size() < 2)
            continue;

        for (size_t i = 0; i + 1 < uniq.size(); i += 2)
        {
            const auto& aPt = uniq[i].p;
            const auto& bPt = uniq[i + 1].p;
            const path_planner_detail::Vec2 m = path_planner_detail::mul(path_planner_detail::add(aPt, bPt), 0.5f);
            if (!path_planner_detail::isInsideSolid(m.x, m.z, layer.getOuterContourPaths(), layer.getInnerContourPaths()))
                continue;
            const path_planner_detail::Vec2 seg = path_planner_detail::sub(bPt, aPt);
            if (path_planner_detail::len(seg) < params.minSegmentLength)
                continue;

            Path infill(PathType::Infill, params.lineWidth);
            infill.addPoint(Point3D(aPt.x, layer.getZHeight(), aPt.z));
            infill.addPoint(Point3D(bPt.x, layer.getZHeight(), bPt.z));
            out.addPath(infill);
        }
    }

    return out;
}

inline Layer addZigZagInfillToLayer(const Layer& layer, const ParallelInfillParams& params)
{
    Layer out(layer.getLayerId(), layer.getZHeight());
    for (const Path& p : layer.getOuterContourPaths()) out.addPath(p);
    for (const Path& p : layer.getInnerContourPaths()) out.addPath(p);
    for (const Path& p : layer.getInfillPaths()) out.addPath(p);

    const std::vector<Path>& outers = layer.getOuterContourPaths();
    if (outers.empty() || params.spacing <= 1e-6f)
        return out;

    constexpr float kPi = 3.14159265358979323846f;
    const float a = params.angleDeg * (kPi / 180.0f);
    const path_planner_detail::Vec2 dir{std::cos(a), std::sin(a)};
    const path_planner_detail::Vec2 nrm{-dir.z, dir.x};

    float cMin = 0.0f, cMax = 0.0f;
    bool first = true;
    for (const Path& p : outers)
    {
        for (const Point3D& q : p.getPoints())
        {
            const path_planner_detail::Vec2 v{q.x(), q.z()};
            const float c = path_planner_detail::dot(nrm, v);
            if (first) { cMin = cMax = c; first = false; }
            else { cMin = std::min(cMin, c); cMax = std::max(cMax, c); }
        }
    }
    if (first)
        return out;
    cMin -= params.spacing;
    cMax += params.spacing;

    std::vector<path_planner_detail::Intersection> xs;
    xs.reserve(256);
    // 每一根扫描线一行；行内每条线段为 (lo,hi)，且 dot(dir,lo)<=dot(dir,hi)。Z 型只在「同一列 k、相邻扫描行」之间连桥，
    // 绝不在同一行上把多个不相邻区间连成折线（否则会穿空隙或误判，看起来像平行短线且疏密错乱）。
    std::vector<std::vector<std::pair<path_planner_detail::Vec2, path_planner_detail::Vec2>>> rows;
    rows.reserve(128);
    std::vector<float> rowC;
    rowC.reserve(128);

    const std::vector<Path>& outerPaths = layer.getOuterContourPaths();
    const std::vector<Path>& innerPaths = layer.getInnerContourPaths();

    for (float c = cMin; c <= cMax; c += params.spacing)
    {
        xs.clear();
        for (const Path& p : outerPaths)
            path_planner_detail::collectIntersectionsForPath(p, dir, nrm, c, xs);
        for (const Path& p : innerPaths)
            path_planner_detail::collectIntersectionsForPath(p, dir, nrm, c, xs);
        if (xs.size() < 2)
            continue;
        std::sort(xs.begin(), xs.end(), [](const auto& a1, const auto& a2) { return a1.s < a2.s; });

        std::vector<path_planner_detail::Intersection> uniq;
        uniq.reserve(xs.size());
        for (const auto& it : xs)
        {
            if (!uniq.empty() && std::fabs(uniq.back().s - it.s) < 1e-5f)
                continue;
            uniq.push_back(it);
        }
        if (uniq.size() < 2)
            continue;

        std::vector<std::pair<path_planner_detail::Vec2, path_planner_detail::Vec2>> row;
        row.reserve(uniq.size() / 2);
        for (size_t i = 0; i + 1 < uniq.size(); i += 2)
        {
            path_planner_detail::Vec2 lo = uniq[i].p;
            path_planner_detail::Vec2 hi = uniq[i + 1].p;
            const auto m = path_planner_detail::mul(path_planner_detail::add(lo, hi), 0.5f);
            if (!path_planner_detail::isInsideSolid(m.x, m.z, outerPaths, innerPaths))
                continue;
            const auto d = path_planner_detail::sub(hi, lo);
            if (path_planner_detail::len(d) < params.minSegmentLength)
                continue;
            if (path_planner_detail::dot(dir, lo) > path_planner_detail::dot(dir, hi))
                std::swap(lo, hi);
            row.push_back({lo, hi});
        }
        if (!row.empty())
        {
            std::sort(row.begin(), row.end(), [&](const auto& ab, const auto& cd) {
                return path_planner_detail::dot(dir, ab.first) < path_planner_detail::dot(dir, cd.first);
            });
            rows.push_back(std::move(row));
            rowC.push_back(c);
        }
    }

    if (rows.empty())
        return out;

    size_t maxK = 0;
    for (const auto& r : rows)
        maxK = std::max(maxK, r.size());

    const float z = layer.getZHeight();
    for (size_t k = 0; k < maxK; ++k)
    {
        Path zig(PathType::Infill, params.lineWidth);
        bool open = false;
        path_planner_detail::Vec2 lastExit{0.0f, 0.0f};

        for (size_t j = 0; j < rows.size(); ++j)
        {
            if (k >= rows[j].size())
            {
                if (open && zig.getPoints().size() >= 2)
                    out.addPath(zig);
                zig = Path(PathType::Infill, params.lineWidth);
                open = false;
                continue;
            }
            const auto& seg = rows[j][k];
            const path_planner_detail::Vec2& lo = seg.first;
            const path_planner_detail::Vec2& hi = seg.second;
            const path_planner_detail::Vec2 enter = (j % 2 == 0) ? lo : hi;
            const path_planner_detail::Vec2 exit = (j % 2 == 0) ? hi : lo;

            if (!open)
            {
                zig.addPoint(Point3D(enter.x, z, enter.z));
                zig.addPoint(Point3D(exit.x, z, exit.z));
                lastExit = exit;
                open = true;
            }
            else
            {
                const float dc = std::fabs(rowC[j] - rowC[j - 1]);
                if (path_planner_detail::zigZagBridgeBetweenScanLines(
                        lastExit.x,
                        lastExit.z,
                        enter.x,
                        enter.z,
                        outerPaths,
                        innerPaths,
                        params.spacing,
                        dc))
                {
                    zig.addPoint(Point3D(enter.x, z, enter.z));
                    zig.addPoint(Point3D(exit.x, z, exit.z));
                    lastExit = exit;
                }
                else
                {
                    if (zig.getPoints().size() >= 2)
                        out.addPath(zig);
                    zig = Path(PathType::Infill, params.lineWidth);
                    zig.addPoint(Point3D(enter.x, z, enter.z));
                    zig.addPoint(Point3D(exit.x, z, exit.z));
                    lastExit = exit;
                    open = true;
                }
            }
        }
        if (open && zig.getPoints().size() >= 2)
            out.addPath(zig);
    }

    return out;
}

/// @brief 添加平行填充到多层
/// @param layers 层集合
/// @param params 平行填充参数
/// @return 添加平行填充到多层后的层集合
/// @details 添加平行填充到多层，如果层集合为空，则返回空层集合。如果平行填充参数的间距小于1e-6f，则返回空层集合。
inline std::vector<Layer> addParallelInfillToLayers(const std::vector<Layer>& layers, const ParallelInfillParams& params)
{
    std::vector<Layer> out;
    out.reserve(layers.size());
    for (const Layer& l : layers)
        out.push_back(addParallelInfillToLayer(l, params));
    return out;
}

inline std::vector<Layer> addInfillToLayers(const std::vector<Layer>& layers, const ParallelInfillParams& params, InfillPattern pattern)
{
    std::vector<Layer> out;
    out.reserve(layers.size());
    for (const Layer& l : layers)
    {
        if (pattern == InfillPattern::ZigZag)
            out.push_back(addZigZagInfillToLayer(l, params));
        else
            out.push_back(addParallelInfillToLayer(l, params));
    }
    return out;
}

