#pragma once

#include <chrono>
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <functional>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

#include "../dataStructure/SliceData.h"
#include "../dataStructure/Layer.h"
#include "../dataStructure/Path.h"
#include "../dataStructure/Model.h"

// 切片器：沿 y = sliceY 切平面，将三角网格求交得到 XZ 封闭轮廓。
/// @brief 切片器：沿 y = sliceY 切平面，将三角网格求交得到 XZ 封闭轮廓。
/// @details
/// - sliceAtY：只切片：返回所有封闭轮廓环（由所有 mesh 全局拼接而来）
/// - sliceToLayerAtY：切片并转成你已有的 Layer/Path。
/// - sliceToLayersUniform：沿 Y 轴（竖直层高）均匀分层切片，得到多份 Layer（与 sliceToLayerAtY 同一套流程）。
/// - collectSegmentsAtY：阶段1：遍历模型所有 mesh，收集交线段
/// - buildSliceResultFromSegments：阶段2：从所有交线段全局拼接出封闭环（贪心版本），暂时没有引入拓扑结构进行加速
/// - convertToLayer：将 SliceResult 转换为 Layer
/// - classifyLoopHoles：统一 outer/inner 分类

/// @brief 切片管线分项耗时（毫秒），便于对比八叉树剪枝（主要影响求交）与拓扑拼环（主要影响轮廓提取）
struct SlicePipelineTiming
{
    double intersectMs = 0.0; ///< 三角网格与切平面求交得到 2D 线段
    double contourMs = 0.0;   ///< 线段拼环 + 内外洞分类 + 转 Layer/Path
};

class Slicer
{
public:
    struct AdaptiveSliceParams
    {
        float minLayerHeight = 0.02f;
        float maxLayerHeight = 0.10f;
        float lowCurvatureDeg = 10.0f;
        float highCurvatureDeg = 30.0f;
        float smoothAlpha = 0.35f;
    };

    explicit Slicer(const Model& model) : model_(model) {}

    // 只切片：返回所有封闭轮廓环（由所有 mesh 全局拼接而来）
    SliceResult sliceAtY(
        float sliceY,
        float eps = 1e-5f,
        bool useOctreePruning = false,
        bool useTopologyStitch = false,
        SlicePipelineTiming* outTiming = nullptr) const
    {
        using clock = std::chrono::steady_clock;
        const auto tInt0 = clock::now();
        std::vector<Segment2D> allSegments;
        if (useOctreePruning)
        {
            OctreeContext ctx = buildOctreeContext();
            allSegments = collectSegmentsAtYWithOctree(sliceY, eps, ctx);
        }
        else
        {
            allSegments = collectSegmentsAtY(sliceY, eps);
        }
        if (outTiming)
            outTiming->intersectMs += std::chrono::duration<double, std::milli>(clock::now() - tInt0).count();

        const auto tContour0 = clock::now();
        SliceResult out = useTopologyStitch
            ? buildSliceResultFromSegmentsTopology(sliceY, allSegments, eps)
            : buildSliceResultFromSegments(sliceY, allSegments, eps);
        if (outTiming)
            outTiming->contourMs += std::chrono::duration<double, std::milli>(clock::now() - tContour0).count();
        return out;
    }

    // 切片并转成你已有的 Layer/Path。
    // 流程：
    // 1) 所有 mesh 收集线段
    // 2) 全局拼接成 SliceLoop
    // 3) 统一 outer/inner 分类
    // 4) 一次性转为 Path 并写入 Layer
    Layer sliceToLayerAtY(
        float sliceY,
        float lineWidth,
        float eps = 1e-5f,
        bool useOctreePruning = false,
        bool useTopologyStitch = false,
        SlicePipelineTiming* outTiming = nullptr) const
    {
        SliceResult slice = sliceAtY(sliceY, eps, useOctreePruning, useTopologyStitch, outTiming);
        using clock = std::chrono::steady_clock;
        const auto tPost0 = clock::now();
        std::vector<char> holeFlags = classifyLoopHoles(slice, eps);
        Layer lyr = convertToLayer(slice, holeFlags, lineWidth);
        if (outTiming)
            outTiming->contourMs += std::chrono::duration<double, std::milli>(clock::now() - tPost0).count();
        return lyr;
    }

    /// 沿 Y 轴（竖直层高）均匀分层切片，得到多份 Layer（与 sliceToLayerAtY 同一套流程）。
    /// @param layerCount 层数（将模型在 Y 上的包围盒 [yMin,yMax] 均分为 layerCount 段，每段中心取一切平面）
    /// @param lineWidth 轮廓线宽（传给 Path）
    /// @param eps 端点量化容差
    /// @return 从下往上 layer_id = 0 .. layerCount-1；若 layerCount<=0 返回空
    std::vector<Layer> sliceToLayersUniform(
        int layerCount,
        float lineWidth,
        float eps = 1e-5f,
        bool useOctreePruning = false,
        bool useTopologyStitch = false,
        SlicePipelineTiming* outTiming = nullptr) const
    {
        using clock = std::chrono::steady_clock;
        std::vector<Layer> layers;
        if (layerCount <= 0)
            return layers;

        float yMin = 0.0f;
        float yMax = 0.0f;
        computeModelYBounds(model_, yMin, yMax);

        const float span = yMax - yMin;
        // 退化：所有顶点几乎同一高度，只生成一层，避免重复 N 次相同切片
        if (span <= 1e-6f)
        {
            Layer one = sliceToLayerAtY(yMin, lineWidth, eps, useOctreePruning, useTopologyStitch, outTiming);
            one.setLayerId(0);
            layers.push_back(std::move(one));
            return layers;
        }

        OctreeContext octreeCtx;
        if (useOctreePruning)
        {
            const auto tOct0 = clock::now();
            octreeCtx = buildOctreeContext();
            if (outTiming)
                outTiming->intersectMs += std::chrono::duration<double, std::milli>(clock::now() - tOct0).count();
        }

        const float dy = span / static_cast<float>(layerCount);
        layers.reserve(static_cast<size_t>(layerCount));
        for (int i = 0; i < layerCount; ++i)
        {
            const float sliceY = yMin + (static_cast<float>(i) + 0.5f) * dy;
            SliceResult slice;
            if (useOctreePruning)
            {
                const auto tInt0 = clock::now();
                std::vector<Segment2D> allSegments = collectSegmentsAtYWithOctree(sliceY, eps, octreeCtx);
                if (outTiming)
                    outTiming->intersectMs += std::chrono::duration<double, std::milli>(clock::now() - tInt0).count();
                const auto tContour0 = clock::now();
                slice = useTopologyStitch
                    ? buildSliceResultFromSegmentsTopology(sliceY, allSegments, eps)
                    : buildSliceResultFromSegments(sliceY, allSegments, eps);
                if (outTiming)
                    outTiming->contourMs += std::chrono::duration<double, std::milli>(clock::now() - tContour0).count();
            }
            else
            {
                slice = sliceAtY(sliceY, eps, false, useTopologyStitch, outTiming);
            }
            const auto tPost0 = clock::now();
            std::vector<char> holeFlags = classifyLoopHoles(slice, eps);
            Layer lyr = convertToLayer(slice, holeFlags, lineWidth);
            if (outTiming)
                outTiming->contourMs += std::chrono::duration<double, std::milli>(clock::now() - tPost0).count();
            lyr.setLayerId(i);
            layers.push_back(std::move(lyr));
        }
        return layers;
    }

    std::vector<Layer> sliceToLayersAdaptive(
        const AdaptiveSliceParams& params,
        float lineWidth,
        float eps = 1e-5f,
        bool useOctreePruning = false,
        bool useTopologyStitch = false,
        SlicePipelineTiming* outTiming = nullptr) const
    {
        using clock = std::chrono::steady_clock;
        std::vector<Layer> layers;
        const float minH = std::max(1e-5f, std::min(params.minLayerHeight, params.maxLayerHeight));
        const float maxH = std::max(minH, std::max(params.minLayerHeight, params.maxLayerHeight));

        float yMin = 0.0f;
        float yMax = 0.0f;
        computeModelYBounds(model_, yMin, yMax);
        const float span = yMax - yMin;
        if (span <= 1e-6f)
        {
            Layer one = sliceToLayerAtY(yMin, lineWidth, eps, useOctreePruning, useTopologyStitch, outTiming);
            one.setLayerId(0);
            layers.push_back(std::move(one));
            return layers;
        }

        const std::vector<CurvatureTriRef> curvRefs = buildCurvatureTriRefs();
        const CurvatureField curvField = buildCurvatureField(yMin, yMax, minH, curvRefs);
        OctreeContext octreeCtx;
        if (useOctreePruning)
        {
            const auto tOct0 = clock::now();
            octreeCtx = buildOctreeContext();
            if (outTiming)
                outTiming->intersectMs += std::chrono::duration<double, std::milli>(clock::now() - tOct0).count();
        }

        float curY = yMin;
        float prevDy = maxH;
        int layerId = 0;
        while (curY < yMax - 1e-6f)
        {
            const float localCurv = estimateCurvatureAtYFromField(curY, curvField, maxH);
            float targetDy = maxH;
            if (localCurv >= params.highCurvatureDeg)
                targetDy = minH;
            else if (localCurv > params.lowCurvatureDeg)
            {
                const float t = (localCurv - params.lowCurvatureDeg) /
                    std::max(1e-6f, params.highCurvatureDeg - params.lowCurvatureDeg);
                targetDy = maxH + (minH - maxH) * t;
            }

            float dy = prevDy + (targetDy - prevDy) * glm::clamp(params.smoothAlpha, 0.0f, 1.0f);
            dy = glm::clamp(dy, minH, maxH);
            if (curY + dy > yMax)
                dy = yMax - curY;
            if (dy <= 1e-6f)
                break;

            const float sliceY = curY + 0.5f * dy;
            SliceResult slice;
            if (useOctreePruning)
            {
                const auto tInt0 = clock::now();
                std::vector<Segment2D> allSegments = collectSegmentsAtYWithOctree(sliceY, eps, octreeCtx);
                if (outTiming)
                    outTiming->intersectMs += std::chrono::duration<double, std::milli>(clock::now() - tInt0).count();
                const auto tContour0 = clock::now();
                slice = useTopologyStitch
                    ? buildSliceResultFromSegmentsTopology(sliceY, allSegments, eps)
                    : buildSliceResultFromSegments(sliceY, allSegments, eps);
                if (outTiming)
                    outTiming->contourMs += std::chrono::duration<double, std::milli>(clock::now() - tContour0).count();
            }
            else
            {
                slice = sliceAtY(sliceY, eps, false, useTopologyStitch, outTiming);
            }

            const auto tPost0 = clock::now();
            std::vector<char> holeFlags = classifyLoopHoles(slice, eps);
            Layer lyr = convertToLayer(slice, holeFlags, lineWidth);
            if (outTiming)
                outTiming->contourMs += std::chrono::duration<double, std::milli>(clock::now() - tPost0).count();
            lyr.setLayerId(layerId++);
            layers.push_back(std::move(lyr));
            curY += dy;
            prevDy = dy;
        }
        return layers;
    }

private:
    const Model& model_;

    struct CurvatureField
    {
        float yMin = 0.0f;
        float yMax = 0.0f;
        float step = 0.01f;
        std::vector<float> maxCurvDeg; // 每个 bin 的最大曲率
    };

    // 2D 线段（XZ 平面）
    struct Segment2D
    {
        glm::vec2 a;
        glm::vec2 b;
    };

    //三角形轻量记录，服务于八叉树
    struct TriRef//三角形引用
    {
        size_t meshIndex = 0;
        uint32_t i0 = 0;//三角形第一个顶点索引  
        uint32_t i1 = 0;//三角形第二个顶点索引
        uint32_t i2 = 0;//三角形第三个顶点索引
        glm::vec3 aabbMin = glm::vec3(0.0f);//三角形包围盒最小点
        glm::vec3 aabbMax = glm::vec3(0.0f);//三角形包围盒最大点
        float yMin = 0.0f;//三角形包围盒最小点y坐标
        float yMax = 0.0f;//三角形包围盒最大点y坐标
    };

    struct CurvatureTriRef
    {
        float yMin = 0.0f;
        float yMax = 0.0f;
        float curvatureDeg = 0.0f;
    };

    //八叉树节点
    struct OctreeNode
    {
        glm::vec3 bmin = glm::vec3(0.0f);//包围盒最小点
        glm::vec3 bmax = glm::vec3(0.0f);//包围盒最大点
        std::vector<size_t> triIndices;//三角形索引集合(索引指的是在八叉树上下文当中的三角形引用集合数组当中的索引)
        std::array<int, 8> children = { -1, -1, -1, -1, -1, -1, -1, -1 };//8个子节点索引(索引指的是在八叉树上下文当中的节点集合数组当中的索引,如果子节点索引为-1，则表示该子节点不存在)
    };
    //八叉树上下文
    struct OctreeContext
    {
        std::vector<TriRef> tris;//三角形引用集合,记录这一棵八叉树当中所有三角形引用
        std::vector<OctreeNode> nodes;//八叉树节点集合,记录这一棵八叉树当中所有节点
        bool valid = false;//八叉树是否有效
    };

    // 量化端点的 key（用于匹配线段端点）
    struct KeyXZ
    {
        int64_t xi = 0;
        int64_t zi = 0;
        bool operator==(const KeyXZ& o) const { return xi == o.xi && zi == o.zi; }
    };

    struct KeyXZHasher
    {
        size_t operator()(const KeyXZ& k) const
        {
            size_t h1 = std::hash<int64_t>{}(k.xi);
            size_t h2 = std::hash<int64_t>{}(k.zi);
            return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
        }
    };

    struct EdgeKey
    {
        size_t meshIndex = 0;
        uint32_t a = 0;
        uint32_t b = 0;
        bool operator==(const EdgeKey& o) const
        {
            return meshIndex == o.meshIndex && a == o.a && b == o.b;
        }
    };

    struct EdgeKeyHasher
    {
        size_t operator()(const EdgeKey& k) const
        {
            size_t h = std::hash<size_t>{}(k.meshIndex);
            h ^= std::hash<uint32_t>{}(k.a) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<uint32_t>{}(k.b) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };

    /// @brief 量化：将浮点数 v 转换为整数，精度为 eps。
    /// @param v 浮点数
    /// @param eps 精度
    /// @return 量化后的整数
    static int64_t quant(float v, float eps)
    {
        return static_cast<int64_t>(std::llround(static_cast<double>(v) / static_cast<double>(eps)));
    }

    static KeyXZ keyFromPointXZ(const glm::vec2& p, float eps)
    {
        // vec2.x = X，vec2.y = Z
        return KeyXZ{ quant(p.x, eps), quant(p.y, eps) };
    }

    /// 将 XZ 点吸附到 stitchEps 量化网格，使相邻三角形贡献的共点线段端点整数一致，避免拼环时「键对不上」断链。
    static glm::vec2 snapPointXZToQuantGrid(const glm::vec2& p, float stitchEps)
    {
        if (stitchEps <= 1e-15f)
            return p;
        const double dq = static_cast<double>(stitchEps);
        const double x = std::llround(static_cast<double>(p.x) / dq) * dq;
        const double z = std::llround(static_cast<double>(p.y) / dq) * dq;
        return glm::vec2(static_cast<float>(x), static_cast<float>(z));
    }

    /// 拼环前对线段子端点做网格对齐（几何 eps 仍用于求交；此处仅用 stitchEps 焊端点）
    static std::vector<Segment2D> weldSliceSegmentsForStitch(
        const std::vector<Segment2D>& segments,
        float stitchEps)
    {
        const float minLenSq = (stitchEps * 0.5f) * (stitchEps * 0.5f);
        std::vector<Segment2D> out;
        out.reserve(segments.size());
        for (const Segment2D& s : segments)
        {
            Segment2D t;
            t.a = snapPointXZToQuantGrid(s.a, stitchEps);
            t.b = snapPointXZToQuantGrid(s.b, stitchEps);
            const glm::vec2 d = t.b - t.a;
            if (glm::dot(d, d) >= minLenSq)
                out.push_back(t);
        }
        return out;
    }

    // 射线法：判断点 p 是否在多边形 poly 内（XZ 平面）
    // 边界点当作 inside=true。
    /// @brief 射线法：判断点 p 是否在多边形 poly 内（XZ 平面）
    /// @param p 点
    /// @param poly 多边形
    /// @return 是否在多边形内
    static bool pointInPolygonXZ(const glm::vec2& p, const std::vector<glm::vec2>& poly)
    {
        const size_t n = poly.size();
        if (n < 3) return false;

        bool inside = false;
        for (size_t i = 0, j = n - 1; i < n; j = i++)//这样写是为了让第一个循环时,i=0,j=n-1,多边形的首尾可以自然连接
        {
            const glm::vec2& pi = poly[i];
            const glm::vec2& pj = poly[j];

            // 边界：叉积接近 0 且投影落在线段范围内
            glm::vec2 v0 = p - pj;
            glm::vec2 v1 = pi - pj;
            float cross = v0.x * v1.y - v0.y * v1.x;
            if (std::fabs(cross) <= 1e-7f)
            {
                float dot = v0.x * v1.x + v0.y * v1.y;//公式求投影按理说还应该除以v1长度，但Len2计算的是v1的模长的平方，所以可以省略除法
                float len2 = v1.x * v1.x + v1.y * v1.y;
                if (len2 > 0.0f && dot >= -1e-7f && dot <= len2 + 1e-7f)
                    return true;
            }

            bool intersect = ((pi.y > p.y) != (pj.y > p.y)) &&
                (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x);
            if (intersect) inside = !inside;
        }
        return inside;
    }
    /// @brief 将点p添加到out中，如果out中已经存在p，则不添加   
    /// @param out 存储点集的向量
    /// @param p 要添加的点
    /// @param eps 精度
    /// @note 这里使用哈希表来判断是否存在，而不是使用遍历，因为哈希表的查找时间复杂度是O(1)，而遍历的时间复杂度是O(n)
    /// @note 这里使用点量化后的数据来判断是否存在，而不是使用点本身，因为点量化后的数据是整数，而点本身是浮点数，浮点数比较不精确
    static void pushUniquePoint2D(std::vector<glm::vec3>& out, const glm::vec3& p, float eps)
    {
        glm::vec2 pXZ(p.x, p.z);
        //利用点量化后的数据取哈希值
        KeyXZ k = keyFromPointXZ(pXZ, eps);
        //利用哈希值判断是否存在
        for (const auto& q : out)
        {
            glm::vec2 qXZ(q.x, q.z);
            if (keyFromPointXZ(qXZ, eps) == k)
                return;
        }
        out.push_back(p);
    }

    static float distSqXZGeom(const glm::vec2& u, const glm::vec2& v)
    {
        const float dx = u.x - v.x;
        const float dz = u.y - v.y;
        return dx * dx + dz * dz;
    }

    /// 当前顶点处多条未用线段共用同一量化键时，仅用「键相等」会错误跨接到另一支轮廓（典型：空心圆柱内外圆）。
    /// 先用附着点到当前几何点的距离收紧候选，再用与前一段夹角最顺（点积最大）的边延续同一环。
    static void pickNextSegmentAtVertex(
        const std::vector<Segment2D>& segments,
        const std::vector<size_t>& incident,
        const std::vector<char>& used,
        const glm::vec2& curPt,
        const glm::vec2& prevPt,
        float eps,
        size_t& outSeg,
        glm::vec2& outNext)
    {
        outSeg = SIZE_MAX;
        const KeyXZ curK = keyFromPointXZ(curPt, eps);
        const float tolSq = std::max(1e-14f, (eps * 16.0f) * (eps * 16.0f));

        struct Cand
        {
            size_t si;
            glm::vec2 next;
            float attachDist;
        };
        std::vector<Cand> raw;
        raw.reserve(incident.size() * 2u);

        for (size_t cand : incident)
        {
            if (used[cand])
                continue;
            const Segment2D& seg = segments[cand];
            if (keyFromPointXZ(seg.a, eps) == curK)
                raw.push_back(Cand{ cand, seg.b, distSqXZGeom(seg.a, curPt) });
            if (keyFromPointXZ(seg.b, eps) == curK)
                raw.push_back(Cand{ cand, seg.a, distSqXZGeom(seg.b, curPt) });
        }

        if (raw.empty())
            return;

        std::vector<Cand> tight;
        tight.reserve(raw.size());
        for (const Cand& c : raw)
        {
            if (c.attachDist <= tolSq)
                tight.push_back(c);
        }

        std::vector<Cand> nearBest;
        const std::vector<Cand>* pool = &tight;
        if (tight.empty())
        {
            float bestD = raw[0].attachDist;
            for (const Cand& c : raw)
                bestD = std::min(bestD, c.attachDist);
            for (const Cand& c : raw)
            {
                if (c.attachDist <= bestD + 1e-10f)
                    nearBest.push_back(c);
            }
            pool = nearBest.empty() ? &raw : &nearBest;
        }

        if (pool->size() == 1)
        {
            outSeg = (*pool)[0].si;
            outNext = (*pool)[0].next;
            return;
        }

        glm::vec2 din = curPt - prevPt;
        const float dinLen = glm::length(din);
        if (dinLen <= 1e-10f)
        {
            size_t bi = 0;
            float bd = (*pool)[0].attachDist;
            for (size_t i = 1; i < pool->size(); ++i)
            {
                if ((*pool)[i].attachDist < bd)
                {
                    bd = (*pool)[i].attachDist;
                    bi = i;
                }
            }
            outSeg = (*pool)[bi].si;
            outNext = (*pool)[bi].next;
            return;
        }
        din /= dinLen;

        float bestDot = -2.0f;
        for (const Cand& c : *pool)
        {
            glm::vec2 dout = c.next - curPt;
            const float dl = glm::length(dout);
            if (dl <= 1e-10f)
                continue;
            dout /= dl;
            const float dotv = glm::dot(din, dout);
            if (dotv > bestDot)
            {
                bestDot = dotv;
                outSeg = c.si;
                outNext = c.next;
            }
        }
        if (outSeg == SIZE_MAX)
        {
            outSeg = (*pool)[0].si;
            outNext = (*pool)[0].next;
        }
    }

    /// 按「簇 id」选下一条边：与 pickNextSegmentAtVertex 相同的几何收紧 + 顺接，但不依赖量化键（用于并查集焊点后的图）。
    static void pickNextSegmentAtCluster(
        const std::vector<Segment2D>& segments,
        const std::vector<std::pair<int, int>>& segClusterOfEndpoint,
        const std::vector<size_t>& incident,
        const std::vector<char>& used,
        int curCluster,
        const glm::vec2& curPt,
        const glm::vec2& prevPt,
        float attachTolSq,
        size_t& outSeg,
        glm::vec2& outNext)
    {
        outSeg = SIZE_MAX;
        struct Cand
        {
            size_t si;
            glm::vec2 next;
            float attachDist;
        };
        std::vector<Cand> raw;
        raw.reserve(incident.size() * 2u);

        for (size_t cand : incident)
        {
            if (used[cand])
                continue;
            const int ca = segClusterOfEndpoint[cand].first;
            const int cb = segClusterOfEndpoint[cand].second;
            const Segment2D& seg = segments[cand];
            if (ca == curCluster)
                raw.push_back(Cand{ cand, seg.b, distSqXZGeom(seg.a, curPt) });
            if (cb == curCluster)
                raw.push_back(Cand{ cand, seg.a, distSqXZGeom(seg.b, curPt) });
        }

        if (raw.empty())
            return;

        std::vector<Cand> tight;
        tight.reserve(raw.size());
        for (const Cand& c : raw)
        {
            if (c.attachDist <= attachTolSq)
                tight.push_back(c);
        }

        std::vector<Cand> nearBest;
        const std::vector<Cand>* pool = &tight;
        if (tight.empty())
        {
            float bestD = raw[0].attachDist;
            for (const Cand& c : raw)
                bestD = std::min(bestD, c.attachDist);
            for (const Cand& c : raw)
            {
                if (c.attachDist <= bestD + 1e-10f)
                    nearBest.push_back(c);
            }
            pool = nearBest.empty() ? &raw : &nearBest;
        }

        if (pool->size() == 1)
        {
            outSeg = (*pool)[0].si;
            outNext = (*pool)[0].next;
            return;
        }

        glm::vec2 din = curPt - prevPt;
        const float dinLen = glm::length(din);
        if (dinLen <= 1e-10f)
        {
            size_t bi = 0;
            float bd = (*pool)[0].attachDist;
            for (size_t i = 1; i < pool->size(); ++i)
            {
                if ((*pool)[i].attachDist < bd)
                {
                    bd = (*pool)[i].attachDist;
                    bi = i;
                }
            }
            outSeg = (*pool)[bi].si;
            outNext = (*pool)[bi].next;
            return;
        }
        din /= dinLen;

        float bestDot = -2.0f;
        for (const Cand& c : *pool)
        {
            glm::vec2 dout = c.next - curPt;
            const float dl = glm::length(dout);
            if (dl <= 1e-10f)
                continue;
            dout /= dl;
            const float dotv = glm::dot(din, dout);
            if (dotv > bestDot)
            {
                bestDot = dotv;
                outSeg = c.si;
                outNext = c.next;
            }
        }
        if (outSeg == SIZE_MAX)
        {
            outSeg = (*pool)[0].si;
            outNext = (*pool)[0].next;
        }
    }

    struct SliceClusterGraph
    {
        std::vector<Segment2D> segments;
        /// 与 segments 同序：线段两端点所属簇 id（已剔除 ca==cb 的退化段）
        std::vector<std::pair<int, int>> segClusters;
        std::vector<std::vector<size_t>> incidentByCluster;
        float mergeTol = 0.0f;
    };

    /// 用空间哈希 + 并查集在 XZ 上合并距离 ≤ mergeTol 的端点，再建拼环用的多重图（比单一量化网格更能接上外轮廓断点）。
    static bool buildSliceClusterGraph(const std::vector<Segment2D>& raw, float eps, SliceClusterGraph& g)
    {
        const float minChordSq = (eps * 0.5f) * (eps * 0.5f);
        std::vector<Segment2D> segs;
        segs.reserve(raw.size());
        for (const Segment2D& s : raw)
        {
            const glm::vec2 d = s.b - s.a;
            if (glm::dot(d, d) >= minChordSq)
                segs.push_back(s);
        }
        if (segs.empty())
            return false;

        const size_t n = segs.size();
        const size_t nEp = 2 * n;
        std::vector<glm::vec2> epPos(nEp);
        for (size_t i = 0; i < n; ++i)
        {
            epPos[2 * i] = segs[i].a;
            epPos[2 * i + 1] = segs[i].b;
        }

        const float mergeTol = std::max(eps * 16.0f, 1e-7f);
        const float mergeTolSq = mergeTol * mergeTol;
        g.mergeTol = mergeTol;

        std::vector<int> parent(nEp);
        for (size_t i = 0; i < nEp; ++i)
            parent[i] = static_cast<int>(i);

        auto ufFind = [&](int x) {
            int r = x;
            while (parent[r] != r)
                r = parent[r];
            while (parent[x] != x)
            {
                const int nx = parent[x];
                parent[x] = r;
                x = nx;
            }
            return r;
        };
        auto ufUnite = [&](int a, int b) {
            a = ufFind(a);
            b = ufFind(b);
            if (a != b)
                parent[a] = b;
        };

        std::map<std::pair<int, int>, std::vector<int>> cellPts;
        for (size_t i = 0; i < nEp; ++i)
        {
            const glm::vec2& p = epPos[i];
            const int ix = static_cast<int>(std::floor(p.x / mergeTol));
            const int iz = static_cast<int>(std::floor(p.y / mergeTol));

            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    const auto it = cellPts.find({ ix + dx, iz + dz });
                    if (it == cellPts.end())
                        continue;
                    for (int j : it->second)
                    {
                        if (distSqXZGeom(p, epPos[static_cast<size_t>(j)]) <= mergeTolSq)
                            ufUnite(static_cast<int>(i), j);
                    }
                }
            }
            cellPts[{ ix, iz }].push_back(static_cast<int>(i));
        }

        std::unordered_map<int, int> rootDense;
        std::vector<int> epClus(nEp);
        for (size_t i = 0; i < nEp; ++i)
        {
            const int r = ufFind(static_cast<int>(i));
            auto it = rootDense.find(r);
            if (it == rootDense.end())
            {
                const int id = static_cast<int>(rootDense.size());
                rootDense[r] = id;
            }
            epClus[i] = rootDense[r];
        }

        const int K = static_cast<int>(rootDense.size());
        g.segments.clear();
        g.segClusters.clear();
        g.incidentByCluster.assign(static_cast<size_t>(K), {});
        g.segments.reserve(n);
        g.segClusters.reserve(n);

        for (size_t si = 0; si < n; ++si)
        {
            const int ca = epClus[2 * si];
            const int cb = epClus[2 * si + 1];
            if (ca == cb)
                continue;
            const size_t idx = g.segments.size();
            g.segments.push_back(segs[si]);
            g.segClusters.push_back({ ca, cb });
            g.incidentByCluster[static_cast<size_t>(ca)].push_back(idx);
            g.incidentByCluster[static_cast<size_t>(cb)].push_back(idx);
        }

        return !g.segments.empty();
    }

    // 单个三角形与平面 y=sliceY 的交线段（XZ 两点）
    /// @brief 单个三角形与平面 y=sliceY 的交线段（XZ 两点）
    /// @param p0 三角形第一个点
    /// @param p1 三角形第二个点
    /// @param p2 三角形第三个点
    /// @param sliceY 切片平面y值
    /// @param eps 精度
    /// @param outSeg 输出线段,有效的线段当中一定有且仅有两个交点
    /// @return 是否存在交线段
    /// @note 这里使用插值计算与平面的交点，而不是使用几何方法，因为几何方法计算交点比较复杂，而且精度不高
    static bool intersectTriangleWithYPlaneXZ(
        const glm::vec3& p0,
        const glm::vec3& p1,
        const glm::vec3& p2,
        float sliceY,
        float eps,
        Segment2D& outSeg)
    {
        const float d0 = p0.y - sliceY;
        const float d1 = p1.y - sliceY;
        const float d2 = p2.y - sliceY;

        // 同侧无交
        if ((d0 > 0 && d1 > 0 && d2 > 0) || (d0 < 0 && d1 < 0 && d2 < 0))
            return false;

        // 整三角形落在切平面上：交为面片而非线段，当前管线只收集线段，跳过
        if (std::fabs(d0) <= eps && std::fabs(d1) <= eps && std::fabs(d2) <= eps)
            return false;

        std::vector<glm::vec3> inter;//存储交点
        inter.reserve(6);

        // 求交点时用几何距离去重，避免两个真实交点因共用同一量化格被 merge 成一点导致丢段
        const float mergeSq = (eps * 0.2f) * (eps * 0.2f);
        auto pushUniqueInter = [&](const glm::vec3& p)
        {
            for (const auto& q : inter)
            {
                const float dx = q.x - p.x;
                const float dz = q.z - p.z;
                if (dx * dx + dz * dz <= mergeSq)
                    return;
            }
            inter.push_back(p);
        };

        const glm::vec3 pts[3] = { p0, p1, p2 };
        const float ds[3] = { d0, d1, d2 };

        auto tryEdge = [&](int i, int j)
        {
            const float di = ds[i];
            const float dj = ds[j];
            const glm::vec3& a = pts[i];
            const glm::vec3& b = pts[j];

            // 整条边落在切平面上：该边即为截痕的一部分，需保留两端（旧实现直接 return 会丢 cap/共面缝上的线段）
            if (std::fabs(di) <= eps && std::fabs(dj) <= eps)
            {
                pushUniqueInter(a);
                pushUniqueInter(b);
                return;
            }
            // 仅一个点落在切平面上，直接添加该点
            if (std::fabs(di) <= eps)
            {
                pushUniqueInter(a);
                return;
            }
            if (std::fabs(dj) <= eps)
            {
                pushUniqueInter(b);
                return;
            }
            // 两个点分别在切平面的两侧，用插值计算与平面的交点并添加
            if ((di > 0 && dj < 0) || (di < 0 && dj > 0))
            {
                float t = di / (di - dj); // y = sliceY
                glm::vec3 p = a + t * (b - a);
                pushUniqueInter(p);
            }
        };

        tryEdge(0, 1);
        tryEdge(1, 2);
        tryEdge(2, 0);

        if (inter.size() < 2)
            return false;

        size_t ia = 0;
        size_t ib = 1;
        float bestDs = -1.0f;
        for (size_t i = 0; i < inter.size(); ++i)
        {
            for (size_t j = i + 1; j < inter.size(); ++j)
            {
                const float dx = inter[i].x - inter[j].x;
                const float dz = inter[i].z - inter[j].z;
                const float s = dx * dx + dz * dz;
                if (s > bestDs)
                {
                    bestDs = s;
                    ia = i;
                    ib = j;
                }
            }
        }

        if (bestDs <= 0.0f)
            return false;

        outSeg.a = glm::vec2(inter[ia].x, inter[ia].z);
        outSeg.b = glm::vec2(inter[ib].x, inter[ib].z);
        return true;
    }

    // 对单个 mesh 计算交线段集合（只收集，不做拼接）
    /// @brief 对单个 mesh 计算交线段集合（只收集，不做拼接）
    /// @param mesh 网格
    /// @param sliceY 切片平面y值
    /// @param eps 精度
    /// @param outSegments 输出线段集合
    /// @note 这里使用线段长度来剔除过短的线段，而不是使用几何方法，因为几何方法计算线段长度比较复杂，而且精度不高
    static void collectSegmentsFromMesh(
        const NewMesh& mesh,
        float sliceY,
        float eps,
        std::vector<Segment2D>& outSegments)
    {
        const auto& verts = mesh.vertices;
        const auto& idx = mesh.indices;
        if (verts.empty() || idx.size() < 3) return;

        //遍历网格当中所有的三角形，当前没有使用加速结构，所以需要遍历所有三角形
        for (size_t i = 0; i + 2 < idx.size(); i += 3)
        {
            uint32_t i0 = idx[i];
            uint32_t i1 = idx[i + 1];
            uint32_t i2 = idx[i + 2];
            if (i0 >= verts.size() || i1 >= verts.size() || i2 >= verts.size())
                continue;

            Segment2D seg;
            const glm::vec3 p0 = verts[i0].Position;
            const glm::vec3 p1 = verts[i1].Position;
            const glm::vec3 p2 = verts[i2].Position;

            if (intersectTriangleWithYPlaneXZ(p0, p1, p2, sliceY, eps, seg))
            {
                glm::vec2 d = seg.a - seg.b;
                //判断线段长度是否大于0.5*eps,剔除过短的线段
                if (glm::dot(d, d) > (eps * 0.5f) * (eps * 0.5f))
                    outSegments.push_back(seg);
            }
        }
    }

    // 模型在切片轴 Y 上的包围盒（所有 mesh 顶点）
    static void computeModelYBounds(const Model& model, float& yMin, float& yMax)
    {
        yMin = std::numeric_limits<float>::max();
        yMax = std::numeric_limits<float>::lowest();
        bool any = false;
        for (const auto& mesh : model.meshes)
        {
            for (const auto& v : mesh.vertices)
            {
                const float y = v.Position.y;
                if (y < yMin) yMin = y;
                if (y > yMax) yMax = y;
                any = true;
            }
        }
        if (!any)
        {
            yMin = 0.0f;
            yMax = 0.0f;
        }
    }

    // 阶段1：遍历模型所有 mesh，收集交线段
    std::vector<Segment2D> collectSegmentsAtY(float sliceY, float eps) const
    {
        std::vector<Segment2D> allSegments;
        allSegments.reserve(4096);

        for (const auto& mesh : model_.meshes)
        {
            collectSegmentsFromMesh(mesh, sliceY, eps, allSegments);
        }
        return allSegments;
    }

    static glm::vec3 triangleNormal(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2)
    {
        const glm::vec3 n = glm::cross(p1 - p0, p2 - p0);
        const float len = glm::length(n);
        if (len <= 1e-8f)
            return glm::vec3(0.0f, 1.0f, 0.0f);
        return n / len;
    }

    std::vector<CurvatureTriRef> buildCurvatureTriRefs() const
    {
        struct TriLocal
        {
            glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
            float yMin = 0.0f;
            float yMax = 0.0f;
            float curvatureDeg = 0.0f;
        };

        std::vector<TriLocal> tris;
        std::unordered_map<EdgeKey, std::vector<size_t>, EdgeKeyHasher> edgeToTris;

        auto makeEdgeKey = [](size_t meshIndex, uint32_t i0, uint32_t i1) -> EdgeKey
        {
            if (i0 > i1)
                std::swap(i0, i1);
            return EdgeKey{ meshIndex, i0, i1 };
        };

        for (size_t mi = 0; mi < model_.meshes.size(); ++mi)
        {
            const NewMesh& mesh = model_.meshes[mi];
            const auto& verts = mesh.vertices;
            const auto& idx = mesh.indices;
            if (verts.empty() || idx.size() < 3)
                continue;
            for (size_t i = 0; i + 2 < idx.size(); i += 3)
            {
                const uint32_t i0 = idx[i];
                const uint32_t i1 = idx[i + 1];
                const uint32_t i2 = idx[i + 2];
                if (i0 >= verts.size() || i1 >= verts.size() || i2 >= verts.size())
                    continue;
                const glm::vec3 p0 = verts[i0].Position;
                const glm::vec3 p1 = verts[i1].Position;
                const glm::vec3 p2 = verts[i2].Position;

                TriLocal t;
                t.normal = triangleNormal(p0, p1, p2);
                t.yMin = std::min(p0.y, std::min(p1.y, p2.y));
                t.yMax = std::max(p0.y, std::max(p1.y, p2.y));
                const size_t triIdx = tris.size();
                tris.push_back(t);

                edgeToTris[makeEdgeKey(mi, i0, i1)].push_back(triIdx);
                edgeToTris[makeEdgeKey(mi, i1, i2)].push_back(triIdx);
                edgeToTris[makeEdgeKey(mi, i2, i0)].push_back(triIdx);
            }
        }

        for (const auto& kv : edgeToTris)
        {
            const std::vector<size_t>& adj = kv.second;
            if (adj.size() < 2)
                continue;
            for (size_t i = 0; i < adj.size(); ++i)
            {
                for (size_t j = i + 1; j < adj.size(); ++j)
                {
                    const float d = glm::clamp(glm::dot(tris[adj[i]].normal, tris[adj[j]].normal), -1.0f, 1.0f);
                    const float ang = glm::degrees(std::acos(d));
                    tris[adj[i]].curvatureDeg = std::max(tris[adj[i]].curvatureDeg, ang);
                    tris[adj[j]].curvatureDeg = std::max(tris[adj[j]].curvatureDeg, ang);
                }
            }
        }

        std::vector<CurvatureTriRef> out;
        out.reserve(tris.size());
        for (const TriLocal& t : tris)
            out.push_back(CurvatureTriRef{ t.yMin, t.yMax, t.curvatureDeg });
        return out;
    }

    static float estimateCurvatureAtY(float y, const std::vector<CurvatureTriRef>& tris, float band)
    {
        float c = 0.0f;
        const float halfBand = std::max(1e-5f, band);
        for (const CurvatureTriRef& t : tris)
        {
            if (t.yMax < y - halfBand || t.yMin > y + halfBand)
                continue;
            c = std::max(c, t.curvatureDeg);
        }
        return c;
    }

    static CurvatureField buildCurvatureField(
        float yMin,
        float yMax,
        float desiredStep,
        const std::vector<CurvatureTriRef>& tris)
    {
        CurvatureField f;
        f.yMin = yMin;
        f.yMax = yMax;
        const float span = std::max(0.0f, yMax - yMin);
        float step = std::max(1e-5f, desiredStep);
        // 避免 bins 过多导致预处理/内存膨胀
        const int maxBins = 4096;
        const int binsByStep = static_cast<int>(std::ceil(span / step)) + 1;
        if (binsByStep > maxBins && span > 1e-6f)
            step = span / static_cast<float>(maxBins - 1);
        f.step = step;
        const int nBins = std::max(1, static_cast<int>(std::ceil(span / step)) + 1);
        f.maxCurvDeg.assign(static_cast<size_t>(nBins), 0.0f);

        auto toBin = [&](float y) -> int
        {
            const float t = (y - f.yMin) / f.step;
            return glm::clamp(static_cast<int>(std::floor(t)), 0, nBins - 1);
        };

        for (const CurvatureTriRef& t : tris)
        {
            int b0 = toBin(t.yMin);
            int b1 = toBin(t.yMax);
            if (b0 > b1) std::swap(b0, b1);
            for (int b = b0; b <= b1; ++b)
                f.maxCurvDeg[static_cast<size_t>(b)] = std::max(f.maxCurvDeg[static_cast<size_t>(b)], t.curvatureDeg);
        }
        return f;
    }

    static float estimateCurvatureAtYFromField(float y, const CurvatureField& f, float band)
    {
        if (f.maxCurvDeg.empty())
            return 0.0f;
        const float span = std::max(0.0f, f.yMax - f.yMin);
        if (span <= 1e-8f)
            return f.maxCurvDeg.front();
        const int nBins = static_cast<int>(f.maxCurvDeg.size());
        auto toBin = [&](float yy) -> int
        {
            const float t = (yy - f.yMin) / f.step;
            return glm::clamp(static_cast<int>(std::floor(t)), 0, nBins - 1);
        };
        const float halfBand = std::max(1e-5f, band);
        const int b0 = toBin(y - halfBand);
        const int b1 = toBin(y + halfBand);
        float c = 0.0f;
        for (int b = b0; b <= b1; ++b)
            c = std::max(c, f.maxCurvDeg[static_cast<size_t>(b)]);
        return c;
    }

    /// @brief 构建八叉树上下文
    /// @return 八叉树上下文
    /// @note 这里使用八叉树来加速三角形与平面的求交，而不是使用遍历，因为八叉树的查找时间复杂度是O(logn)，而遍历的时间复杂度是O(n)
    /// @details 八叉树的构建过程如下：
    /// 1. 遍历所有三角形，计算每个三角形的包围盒
    /// 2. 将所有三角形按照包围盒进行排序，排序方式为包围盒最小点x坐标，y坐标，z坐标
    /// 3. 将所有三角形按照包围盒进行分割，分割成8个子包围盒
    OctreeContext buildOctreeContext() const
    {
        OctreeContext ctx;
        glm::vec3 globalMin(std::numeric_limits<float>::max());
        glm::vec3 globalMax(std::numeric_limits<float>::lowest());

        for (size_t mi = 0; mi < model_.meshes.size(); ++mi)
        {
            const NewMesh& mesh = model_.meshes[mi];
            const auto& verts = mesh.vertices;
            const auto& idx = mesh.indices;
            if (verts.empty() || idx.size() < 3)
                continue;
            for (size_t i = 0; i + 2 < idx.size(); i += 3)
            {
                const uint32_t i0 = idx[i];
                const uint32_t i1 = idx[i + 1];
                const uint32_t i2 = idx[i + 2];
                if (i0 >= verts.size() || i1 >= verts.size() || i2 >= verts.size())
                    continue;
                const glm::vec3 p0 = verts[i0].Position;
                const glm::vec3 p1 = verts[i1].Position;
                const glm::vec3 p2 = verts[i2].Position;
                //记录三角形信息
                TriRef t;//三角形引用
                t.meshIndex = mi;//三角形所属网格索引
                t.i0 = i0;//三角形第一个顶点索引
                t.i1 = i1;//三角形第二个顶点索引
                t.i2 = i2;//三角形第三个顶点索引
                t.aabbMin = glm::min(p0, glm::min(p1, p2));//三角形包围盒最小点
                t.aabbMax = glm::max(p0, glm::max(p1, p2));//三角形包围盒最大点
                t.yMin = t.aabbMin.y;//三角形包围盒最小点y坐标
                t.yMax = t.aabbMax.y;//三角形包围盒最大点y坐标
                ctx.tris.push_back(t);//将三角形引用添加到八叉树上下文当中
                globalMin = glm::min(globalMin, t.aabbMin);//更新全局最小点
                globalMax = glm::max(globalMax, t.aabbMax);//更新全局最大点
            }
        }
        //如果八叉树上下文当中没有三角形，则返回空
        if (ctx.tris.empty())
            return ctx;

        OctreeNode root;//根节点
        root.bmin = globalMin;//根节点包围盒最小点
        root.bmax = globalMax;//根节点包围盒最大点
        root.triIndices.resize(ctx.tris.size());//根节点三角形索引集合
        for (size_t i = 0; i < ctx.tris.size(); ++i)
            root.triIndices[i] = i;//将三角形索引添加到根节点三角形索引集合当中
        ctx.nodes.push_back(std::move(root));//将根节点添加到八叉树节点集合当中

        constexpr int kMaxDepth = 10;//最大深度
        constexpr size_t kLeafTriLimit = 64;//叶子节点三角形数量限制，如果当前节点三角形数量小于叶子节点三角形数量限制，则不进行分割
        const glm::vec3 minExtent = (globalMax - globalMin) / 256.0f;//最小扩展，如果当前节点包围盒大小小于最小扩展，则不进行分割

        //分割节点函数
        /// @brief 分割节点函数
        /// @param nodeIdx 当前节点索引
        /// @param depth 当前节点深度
        /// @note 这里使用递归的方式来分割节点，分割过程如下：
        /// 1. 计算当前节点包围盒中心点
        /// 2. 计算当前节点包围盒大小
        /// 3. 如果当前节点深度大于最大深度，或者当前节点三角形数量小于叶子节点三角形数量限制，或者当前节点包围盒大小小于最小扩展，则返回
        /// 4. 分割当前节点为8个子节点
        /// @details 一开始为根节点填充数据，后面的递归从根节点开始，在第一次分割的时候会重新分配根节点包含的三角形引用，因此根节点当中不会重复包含子节点当中的三角形引用
        std::function<void(int, int)> splitNode = [&](int nodeIdx, int depth)
        {
            const glm::vec3 nodeBmin = ctx.nodes[nodeIdx].bmin;//当前节点包围盒最小点
            const glm::vec3 nodeBmax = ctx.nodes[nodeIdx].bmax;//当前节点包围盒最大点
            const std::vector<size_t> nodeTris = ctx.nodes[nodeIdx].triIndices;//当前节点三角形索引集合
            const glm::vec3 ext = nodeBmax - nodeBmin;//当前节点包围盒大小
            if (depth >= kMaxDepth || nodeTris.size() <= kLeafTriLimit ||
                ext.x <= minExtent.x || ext.y <= minExtent.y || ext.z <= minExtent.z)//如果当前节点深度大于最大深度，或者当前节点三角形数量小于叶子节点三角形数量限制，或者当前节点包围盒大小小于最小扩展，则返回
                return;

            const glm::vec3 c = (nodeBmin + nodeBmax) * 0.5f;//当前节点中心点
            std::array<glm::vec3, 8> cbmin;//8个子节点包围盒最小点
            std::array<glm::vec3, 8> cbmax;//8个子节点包围盒最大点
            //计算8个子节点包围盒最小点和最大点，把当前节点的包围盒切成8个子包围盒
            for (int ci = 0; ci < 8; ++ci)
            {
                //八个子节点的索引是0-7，二进制对应三位,xyz分别对应从低到高的三位，该位位0则表示该子包围盒的这一个维度处于低位
                //低位：前，下，左
                //分别与1(001)，2(010)，4(100)进行与运算，如果结果为0，则表示该子包围盒的这一个维度处于低位，如果结果为1，则表示该子包围盒的这一个维度处于高位
                cbmin[ci] = glm::vec3(
                    (ci & 1) ? c.x : nodeBmin.x,
                    (ci & 2) ? c.y : nodeBmin.y,
                    (ci & 4) ? c.z : nodeBmin.z);
                cbmax[ci] = glm::vec3(
                    (ci & 1) ? nodeBmax.x : c.x,
                    (ci & 2) ? nodeBmax.y : c.y,
                    (ci & 4) ? nodeBmax.z : c.z);
            }

            std::array<std::vector<size_t>, 8> childTris;//8个子节点三角形索引集合
            std::vector<size_t> remain;//剩余三角形索引集合
            remain.reserve(nodeTris.size());

            auto fullyInside = [](const TriRef& t, const glm::vec3& bmin, const glm::vec3& bmax)//判断三角形是否完全在包围盒内
            {
                return t.aabbMin.x >= bmin.x && t.aabbMax.x <= bmax.x &&//三角形包围盒最小点x坐标大于等于包围盒最小点x坐标，且三角形包围盒最大点x坐标小于等于包围盒最大点x坐标
                    t.aabbMin.y >= bmin.y && t.aabbMax.y <= bmax.y &&//三角形包围盒最小点y坐标大于等于包围盒最小点y坐标，且三角形包围盒最大点y坐标小于等于包围盒最大点y坐标
                    t.aabbMin.z >= bmin.z && t.aabbMax.z <= bmax.z;//三角形包围盒最小点z坐标大于等于包围盒最小点z坐标，且三角形包围盒最大点z坐标小于等于包围盒最大点z坐标
            };

            for (size_t ti : nodeTris)//遍历当前节点三角形索引集合  
            {
                const TriRef& t = ctx.tris[ti];
                int hitChild = -1;
                //遍历8个子节点，判断当前三角形是否在哪个子节点内
                for (int ci = 0; ci < 8; ++ci)
                {
                    if (fullyInside(t, cbmin[ci], cbmax[ci]))
                    {
                        hitChild = ci;
                        break;
                    }
                }
                if (hitChild >= 0)//如果当前三角形在某个子节点内，则将当前三角形索引添加到该子节点三角形索引集合当中
                    childTris[hitChild].push_back(ti);
                else//如果当前三角形不在任何一个子节点内，则将当前三角形索引添加到剩余三角形索引集合当中
                    remain.push_back(ti);
            }

            bool hasChild = false;//是否有子节点
            //遍历8个子节点，创建子节点
            for (int ci = 0; ci < 8; ++ci)
            {
                if (childTris[ci].empty())//如果当前子节点三角形索引集合为空，则跳过
                    continue;
                hasChild = true;
                OctreeNode ch;//子节点  
                ch.bmin = cbmin[ci];//子节点包围盒最小点
                ch.bmax = cbmax[ci];//子节点包围盒最大点
                ch.triIndices = std::move(childTris[ci]);//子节点三角形索引集合
                const int newIdx = static_cast<int>(ctx.nodes.size());//新节点索引
                ctx.nodes.push_back(std::move(ch));//将子节点添加到八叉树节点集合当中
                ctx.nodes[nodeIdx].children[ci] = newIdx;//将子节点索引添加到当前节点子节点索引集合当中
            }

            if (!hasChild)//如果没有子节点，则返回
                return;

            ctx.nodes[nodeIdx].triIndices = std::move(remain);//将剩余三角形索引集合添加到当前节点三角形索引集合当中
            for (int ci = 0; ci < 8; ++ci)
            {
                if (ctx.nodes[nodeIdx].children[ci] >= 0)//如果当前节点子节点索引大于等于0，则递归分割当前节点
                    splitNode(ctx.nodes[nodeIdx].children[ci], depth + 1);
            }
        };

        splitNode(0, 0);//分割根节点
        ctx.valid = true;
        return ctx;
    }



    /// @brief 使用八叉树收集交线段
    /// @param sliceY 切片y坐标
    /// @param eps 精度
    /// @param ctx 八叉树上下文
    /// @return 交线段集合
    /// @note 这里使用八叉树来加速三角形与平面的求交，而不是使用遍历，因为八叉树的查找时间复杂度是O(logn)，而遍历的时间复杂度是O(n)
    /// @details 八叉树的查找过程如下：
    /// 1. 从根节点开始，判断当前节点是否在切片y坐标范围内
    /// 2. 如果当前节点在切片y坐标范围内，则将当前节点三角形索引集合添加到候选集合当中
    std::vector<Segment2D> collectSegmentsAtYWithOctree(float sliceY, float eps, const OctreeContext& ctx) const
    {
        if (!ctx.valid || ctx.nodes.empty())//如果八叉树上下文无效或者八叉树节点集合为空，则使用遍历收集交线段
            return collectSegmentsAtY(sliceY, eps);

        std::vector<Segment2D> allSegments;
        allSegments.reserve(4096);

        std::vector<size_t> candidates;//候选三角形索引集合
        candidates.reserve(4096);

        std::function<void(int)> visit = [&](int nodeIdx)
        {
            const OctreeNode& node = ctx.nodes[nodeIdx];//当前节点
            if (sliceY < node.bmin.y - eps || sliceY > node.bmax.y + eps)
                return;
            for (size_t ti : node.triIndices)//将当前节点三角形索引集合添加到候选集合当中
                candidates.push_back(ti);
            //遍历8个子节点，递归访问子节点
            for (int ci = 0; ci < 8; ++ci)
            {
                if (node.children[ci] >= 0)//如果当前节点子节点索引大于等于0，则递归访问子节点
                    visit(node.children[ci]);
            }
        };
        visit(0);//访问根节点

        for (size_t ti : candidates)
        {
            const TriRef& t = ctx.tris[ti];//当前三角形引用
            if (sliceY < t.yMin - eps || sliceY > t.yMax + eps)//如果当前三角形y坐标不在切片y坐标范围内，则跳过
                continue;
            if (t.meshIndex >= model_.meshes.size())//如果当前三角形所属网格索引大于等于模型网格数量，则跳过
                continue;
            const NewMesh& mesh = model_.meshes[t.meshIndex];//当前网格
            if (t.i0 >= mesh.vertices.size() || t.i1 >= mesh.vertices.size() || t.i2 >= mesh.vertices.size())//如果当前三角形第一个顶点索引大于等于网格顶点数量，或者当前三角形第二个顶点索引大于等于网格顶点数量，或者当前三角形第三个顶点索引大于等于网格顶点数量，则跳过
                continue;

            Segment2D seg;//交线段
            const glm::vec3 p0 = mesh.vertices[t.i0].Position;
            const glm::vec3 p1 = mesh.vertices[t.i1].Position;
            const glm::vec3 p2 = mesh.vertices[t.i2].Position;
            if (intersectTriangleWithYPlaneXZ(p0, p1, p2, sliceY, eps, seg))
            {
                const glm::vec2 d = seg.a - seg.b;
                if (glm::dot(d, d) > (eps * 0.5f) * (eps * 0.5f))
                    allSegments.push_back(seg);
            }
        }
        return allSegments;
    }

    // 阶段2：从所有交线段全局拼接出封闭环（贪心版本），暂时没有引入拓扑结构进行加速
    static SliceResult buildSliceResultFromSegments(
        float sliceY,
        const std::vector<Segment2D>& segments,
        float eps)
    {
        SliceResult result;
        result.setZ(sliceY);
        result.loops().clear();

        if (segments.empty())
            return result;

        SliceClusterGraph cg;
        if (!buildSliceClusterGraph(segments, eps, cg))
            return result;

        const std::vector<Segment2D>& segs = cg.segments;
        const auto& segCl = cg.segClusters;
        const auto& byCl = cg.incidentByCluster;
        const float attachTolSq =
            std::max(1e-14f, (cg.mergeTol * 16.0f) * (cg.mergeTol * 16.0f));
        const float loopCloseTol =
            std::max(cg.mergeTol * 10.0f, eps * 48.0f);

        std::vector<char> used(segs.size(), 0);
        std::vector<char> deadStart(segs.size(), 0);

        for (size_t startSeg = 0; startSeg < segs.size(); ++startSeg)
        {
            if (used[startSeg] || deadStart[startSeg])
                continue;

            std::vector<char> snapUsed = used;
            bool placed = false;
            std::vector<glm::vec2> savedPts;

            for (int flip = 0; flip < 2 && !placed; ++flip)
            {
                std::vector<char> trial = snapUsed;
                std::vector<glm::vec2> loopPts;
                loopPts.reserve(128);

                const int ca = segCl[startSeg].first;
                const int cb = segCl[startSeg].second;
                glm::vec2 startPt = flip ? segs[startSeg].b : segs[startSeg].a;
                glm::vec2 currentPt = flip ? segs[startSeg].a : segs[startSeg].b;
                const int startCluster = flip ? cb : ca;
                int curCluster = flip ? ca : cb;

                trial[startSeg] = 1;
                loopPts.push_back(startPt);
                loopPts.push_back(currentPt);

                bool closed = false;
                size_t guard = 0;
                const size_t maxSteps = segs.size() + 5u;

                while (guard++ < maxSteps)
                {
                    if (curCluster < 0 ||
                        curCluster >= static_cast<int>(byCl.size()))
                        break;
                    const std::vector<size_t>& inc = byCl[static_cast<size_t>(curCluster)];
                    if (inc.empty())
                        break;

                    const glm::vec2 prevPt = loopPts[loopPts.size() - 2];
                    const glm::vec2 curPt = loopPts.back();

                    size_t nextSegIdx = SIZE_MAX;
                    glm::vec2 nextPt(0.0f);
                    pickNextSegmentAtCluster(
                        segs,
                        segCl,
                        inc,
                        trial,
                        curCluster,
                        curPt,
                        prevPt,
                        attachTolSq,
                        nextSegIdx,
                        nextPt);

                    if (nextSegIdx == SIZE_MAX)
                        break;

                    trial[nextSegIdx] = 1;
                    const int nca = segCl[nextSegIdx].first;
                    const int ncb = segCl[nextSegIdx].second;
                    curCluster = (nca == curCluster) ? ncb : nca;
                    currentPt = nextPt;
                    loopPts.push_back(currentPt);

                    if (curCluster == startCluster &&
                        glm::distance(currentPt, startPt) <= loopCloseTol)
                    {
                        closed = true;
                        break;
                    }
                }

                if (closed && loopPts.size() >= 4)
                {
                    used = std::move(trial);
                    savedPts = std::move(loopPts);
                    placed = true;
                }
            }

            if (placed)
            {
                savedPts.pop_back();
                SliceLoop loop;
                loop.setPoints(savedPts);
                result.loops().push_back(loop);
            }
            else
                deadStart[startSeg] = 1;
        }

        return result;
    }

    // 阶段2（拓扑版）：端点建图后按“边访问一次”提取闭环
    static SliceResult buildSliceResultFromSegmentsTopology(
        float sliceY,
        const std::vector<Segment2D>& segments,
        float eps)
    {
        SliceResult result;
        result.setZ(sliceY);
        result.loops().clear();
        if (segments.empty())
            return result;

        SliceClusterGraph cg;
        if (!buildSliceClusterGraph(segments, eps, cg))
            return result;

        const std::vector<Segment2D>& segs = cg.segments;
        const auto& segCl = cg.segClusters;

        struct EdgeTopo
        {
            int ka;
            int kb;
            glm::vec2 a;
            glm::vec2 b;
        };

        std::vector<EdgeTopo> edges;
        edges.reserve(segs.size());
        std::unordered_map<int, std::vector<size_t>> adj;
        adj.reserve(segs.size() * 2u);

        for (size_t i = 0; i < segs.size(); ++i)
        {
            const int ka = segCl[i].first;
            const int kb = segCl[i].second;
            if (ka == kb)
                continue;
            const size_t ei = edges.size();
            edges.push_back(EdgeTopo{ ka, kb, segs[i].a, segs[i].b });
            adj[ka].push_back(ei);
            adj[kb].push_back(ei);
        }

        std::vector<char> used(edges.size(), 0);
        std::vector<char> deadStart(edges.size(), 0);
        const float loopCloseTol =
            std::max(cg.mergeTol * 10.0f, eps * 48.0f);
        const float tolSqT =
            std::max(1e-14f, (cg.mergeTol * 16.0f) * (cg.mergeTol * 16.0f));

        for (size_t startE = 0; startE < edges.size(); ++startE)
        {
            if (used[startE] || deadStart[startE])
                continue;

            std::vector<char> snapUsed = used;
            bool placed = false;
            std::vector<glm::vec2> savedPts;

            for (int orient = 0; orient < 2 && !placed; ++orient)
            {
                std::vector<char> trial = snapUsed;
                trial[startE] = 1;
                const EdgeTopo& e0 = edges[startE];

                int startKey;
                int curKey;
                std::vector<glm::vec2> loopPts;
                loopPts.reserve(128);
                if (orient == 0)
                {
                    startKey = e0.ka;
                    curKey = e0.kb;
                    loopPts.push_back(e0.a);
                    loopPts.push_back(e0.b);
                }
                else
                {
                    startKey = e0.kb;
                    curKey = e0.ka;
                    loopPts.push_back(e0.b);
                    loopPts.push_back(e0.a);
                }

                bool closed = false;
                size_t guard = 0;
                const size_t maxSteps = edges.size() + 5u;

                while (guard++ < maxSteps)
                {
                    if (curKey == startKey)
                    {
                        if (glm::distance(loopPts.back(), loopPts.front()) <= loopCloseTol)
                        {
                            closed = true;
                            break;
                        }
                    }
                    auto it = adj.find(curKey);
                    if (it == adj.end())
                        break;

                    const glm::vec2 prevPos = loopPts[loopPts.size() - 2];
                    const glm::vec2 curPos = loopPts.back();

                    struct TCand
                    {
                        size_t ei;
                        glm::vec2 next;
                        int nk;
                        float attachDist;
                    };
                    std::vector<TCand> traw;
                    traw.reserve(it->second.size() * 2u);
                    for (size_t cand : it->second)
                    {
                        if (trial[cand])
                            continue;
                        const EdgeTopo& ec = edges[cand];
                        if (ec.ka == curKey)
                            traw.push_back(TCand{ cand, ec.b, ec.kb, distSqXZGeom(ec.a, curPos) });
                        if (ec.kb == curKey)
                            traw.push_back(TCand{ cand, ec.a, ec.ka, distSqXZGeom(ec.b, curPos) });
                    }

                    size_t nextE = SIZE_MAX;
                    int nextKey = -1;
                    glm::vec2 nextPt(0.0f);

                    if (!traw.empty())
                    {
                        std::vector<TCand> ttight;
                        for (const TCand& c : traw)
                        {
                            if (c.attachDist <= tolSqT)
                                ttight.push_back(c);
                        }
                        std::vector<TCand> tnear;
                        const std::vector<TCand>* tpool = &ttight;
                        if (ttight.empty())
                        {
                            float bestD = traw[0].attachDist;
                            for (const TCand& c : traw)
                                bestD = std::min(bestD, c.attachDist);
                            for (const TCand& c : traw)
                            {
                                if (c.attachDist <= bestD + 1e-10f)
                                    tnear.push_back(c);
                            }
                            tpool = tnear.empty() ? &traw : &tnear;
                        }

                        if (tpool->size() == 1)
                        {
                            nextE = (*tpool)[0].ei;
                            nextKey = (*tpool)[0].nk;
                            nextPt = (*tpool)[0].next;
                        }
                        else
                        {
                            glm::vec2 din = curPos - prevPos;
                            const float dinLen = glm::length(din);
                            if (dinLen <= 1e-10f)
                            {
                                size_t bi = 0;
                                float bd = (*tpool)[0].attachDist;
                                for (size_t i = 1; i < tpool->size(); ++i)
                                {
                                    if ((*tpool)[i].attachDist < bd)
                                    {
                                        bd = (*tpool)[i].attachDist;
                                        bi = i;
                                    }
                                }
                                nextE = (*tpool)[bi].ei;
                                nextKey = (*tpool)[bi].nk;
                                nextPt = (*tpool)[bi].next;
                            }
                            else
                            {
                                din /= dinLen;
                                float bestDot = -2.0f;
                                for (const TCand& c : *tpool)
                                {
                                    glm::vec2 dout = c.next - curPos;
                                    const float dl = glm::length(dout);
                                    if (dl <= 1e-10f)
                                        continue;
                                    dout /= dl;
                                    const float dotv = glm::dot(din, dout);
                                    if (dotv > bestDot)
                                    {
                                        bestDot = dotv;
                                        nextE = c.ei;
                                        nextKey = c.nk;
                                        nextPt = c.next;
                                    }
                                }
                                if (nextE == SIZE_MAX)
                                {
                                    nextE = (*tpool)[0].ei;
                                    nextKey = (*tpool)[0].nk;
                                    nextPt = (*tpool)[0].next;
                                }
                            }
                        }
                    }

                    if (nextE == SIZE_MAX)
                        break;
                    trial[nextE] = 1;
                    curKey = nextKey;
                    loopPts.push_back(nextPt);
                }

                if (closed && loopPts.size() >= 4)
                {
                    used = std::move(trial);
                    savedPts = std::move(loopPts);
                    placed = true;
                }
            }

            if (placed)
            {
                savedPts.pop_back();
                SliceLoop loop;
                loop.setPoints(savedPts);
                result.loops().push_back(loop);
            }
            else
                deadStart[startE] = 1;
        }
        return result;
    }

    /// 用于“被其它环包含”判定的代表点：取环上 x 坐标最大的顶点（XZ 平面 vec2.x 即 X）。
    /// 同心空心圆柱切片为两圆时，顶点重心仍接近公共圆心，会被误判为落在内圆内；
    /// 而外环「最大 x」顶点落在大圆上、不在内圆盘内，内环最大 x 顶点仍在内圆上且必在外圆内，
    /// 奇偶深度与真实内外一致。非轴对齐模型也可用该不对称点降低共心误判。
    static glm::vec2 representativePointForContainment(const std::vector<glm::vec2>& poly)
    {
        if (poly.empty())
            return glm::vec2(0.0f);
        glm::vec2 pick = poly[0];
        float mx = poly[0].x;
        for (const auto& p : poly)
        {
            if (p.x > mx)
            {
                mx = p.x;
                pick = p;
            }
        }
        return pick;
    }

    static float closedPolylineLengthXZ(const std::vector<glm::vec2>& poly)
    {
        const size_t n = poly.size();
        if (n < 2)
            return 0.0f;
        float sum = 0.0f;
        for (size_t i = 0; i < n; ++i)
        {
            const glm::vec2& a = poly[i];
            const glm::vec2& b = poly[(i + 1) % n];
            sum += glm::length(b - a);
        }
        return sum;
    }

    // 阶段3：统一分类 outer / inner
    // 返回 holeFlags：1 表示内孔，0 表示外轮廓
    static std::vector<char> classifyLoopHoles(
        const SliceResult& slice,
        float eps)
    {
        //获取所有环
        const auto& loops = slice.loops();
        const size_t n = loops.size();
        std::vector<char> isHole(n, 0);
        //如果环的数量为0，则返回空数组
        if (n == 0) return isHole;

        (void)eps;
        // 遍历所有环，使用“包含层级奇偶（parity）”统一 outer/inner：
        // depth = 代表点被其它环（j!=i）的多边形包含的次数
        // depth 为奇数 => hole（内孔）
        // depth 为偶数 => outer（外轮廓 / 岛）
        //
        // 注意：不能用顶点均值（重心）作唯一点——空心圆柱等同心切片上，外环与内环重心
        // 几乎重合且都落在内圆孔区域内，射线法会认为外环重心“在内环多边形内”，
        // 从而把外环误判为孔洞。改用环上分散采样的代表点，使外环点落在外部材料带上、
        // 不在内圆盘内，奇偶计数才与几何一致。
        std::vector<glm::vec2> repPts;
        repPts.resize(n);
        for (size_t i = 0; i < n; ++i)
        {
            const auto& ptsI = loops[i].points2D();
            repPts[i] = representativePointForContainment(ptsI);
        }

        for (size_t i = 0; i < n; ++i)
        {
            const auto& ptsI = loops[i].points2D();
            if (ptsI.empty()) continue;

            int depth = 0;
            for (size_t j = 0; j < n; ++j)
            {
                if (i == j) continue;
                const auto& ptsJ = loops[j].points2D();
                if (ptsJ.empty()) continue;

                if (pointInPolygonXZ(repPts[i], ptsJ))
                    depth += 1;
            }
            isHole[i] = (depth % 2) ? 1 : 0;
        }

        // 退化修正：若全部被标成孔（例如代表点仍偶发对称失败），则周长最大者必为外轮廓（典型双圆环）
        size_t holeCount = 0;
        for (size_t i = 0; i < n; ++i)
        {
            if (isHole[i])
                ++holeCount;
        }
        if (holeCount == n && n >= 2)
        {
            size_t best = 0;
            float bestLen = -1.0f;
            for (size_t i = 0; i < n; ++i)
            {
                const float len = closedPolylineLengthXZ(loops[i].points2D());
                if (len > bestLen)
                {
                    bestLen = len;
                    best = i;
                }
            }
            isHole[best] = 0;
        }

        // 两环且奇偶给出相同标签时：仅当存在一方代表点落在另一方多边形内（嵌套）时，
        // 用周长大者为外轮廓；两环不相连时保持同为外轮廓，避免误伤双岛结构。
        if (n == 2 && isHole[0] == isHole[1])
        {
            const bool r0in1 = pointInPolygonXZ(repPts[0], loops[1].points2D());
            const bool r1in0 = pointInPolygonXZ(repPts[1], loops[0].points2D());
            if (r0in1 || r1in0)
            {
                const float L0 = closedPolylineLengthXZ(loops[0].points2D());
                const float L1 = closedPolylineLengthXZ(loops[1].points2D());
                if (L0 >= L1)
                {
                    isHole[0] = 0;
                    isHole[1] = 1;
                }
                else
                {
                    isHole[0] = 1;
                    isHole[1] = 0;
                }
            }
        }

        //返回内孔标记数组
        return isHole;
    }

    // 阶段4：一次性把 SliceLoop 转为 Path 并写入 Layer
    static Layer convertToLayer(
        const SliceResult& slice,
        const std::vector<char>& isHole,
        float lineWidth)
    {
        Layer layer;
        layer.setZHeight(slice.z());

        const auto& loops = slice.loops();
        const size_t n = loops.size();
        for (size_t i = 0; i < n; ++i)
        {
            const auto& loop = loops[i];
            const auto& pts = loop.points2D();
            if (pts.size() < 3) continue;

            PathType t = isHole[i] ? PathType::InnerContour : PathType::OuterContour;
            Path path(t, lineWidth);

            for (const auto& p : pts)
            {
                // XZ -> (x, y=sliceY, z)
                path.addPoint(Point3D(p.x, slice.z(), p.y));
            }

            // 补齐首点，让最后一条边也存在
            const glm::vec2& first = pts.front();
            path.addPoint(Point3D(first.x, slice.z(), first.y));

            layer.addPath(path);
        }

        return layer;
    }
};