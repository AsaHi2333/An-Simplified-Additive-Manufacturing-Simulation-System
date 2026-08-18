#pragma once

// 层间轮廓二次验证：将每层外/内轮廓投影到 XZ 栅格，用“下层对当前层邻域的覆盖比例”剔除法向误检。
// 论文表述：在离散切片域上估计竖向支撑充分性，与纯几何法向判据级联。

#include "../dataStructure/Layer.h"
#include "../dataStructure/Path.h"
#include "../dataStructure/Point3D.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace overhang_layer_detail
{

/// @brief XZ 平面上的均匀栅格（与切片 Path 中 Point3D 的 x、z 一致）
struct XZGridSpec
{
    float xmin = 0.0f;
    float xmax = 1.0f;
    float zmin = 0.0f;
    float zmax = 1.0f;
    int nx = 1;
    int nz = 1;
};

inline int gridIndex(int ix, int iz, int nx)
{
    return iz * nx + ix;
}

/// @brief 射线法判断 (x,z) 是否在多边形内（Path 点序为 XZ 投影）
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
        const float denom = (zj - zi);
        const float inv = (std::fabs(denom) > 1e-12f) ? (1.0f / denom) : 0.0f;
        if (((zi > z) != (zj > z)) && (x < (xj - xi) * (z - zi) * inv + xi))
            inside = !inside;
    }
    return inside;
}

/// @brief 将单层外轮廓并减去内轮廓（孔洞）栅格化为 0/1 实体掩码
inline void rasterizeLayerSolidMask(const Layer& layer, const XZGridSpec& g, std::vector<uint8_t>& outMask)
{
    const int nx = g.nx;
    const int nz = g.nz;
    outMask.assign(static_cast<size_t>(nx) * static_cast<size_t>(nz), 0);
    if (nx <= 0 || nz <= 0)
        return;

    const float xspan = g.xmax - g.xmin;
    const float zspan = g.zmax - g.zmin;
    if (xspan <= 1e-12f || zspan <= 1e-12f)
        return;

    for (int iz = 0; iz < nz; ++iz)
    {
        const float wz = g.zmin + (static_cast<float>(iz) + 0.5f) * zspan / static_cast<float>(nz);
        for (int ix = 0; ix < nx; ++ix)
        {
            const float wx = g.xmin + (static_cast<float>(ix) + 0.5f) * xspan / static_cast<float>(nx);

            bool inAnyOuter = false;
            for (const Path& p : layer.getOuterContourPaths())
            {
                if (pointInPolygonXZ(wx, wz, p.getPoints()))
                {
                    inAnyOuter = true;
                    break;
                }
            }
            if (!inAnyOuter)
                continue;

            bool inAnyInner = false;
            for (const Path& p : layer.getInnerContourPaths())
            {
                if (pointInPolygonXZ(wx, wz, p.getPoints()))
                {
                    inAnyInner = true;
                    break;
                }
            }
            outMask[static_cast<size_t>(gridIndex(ix, iz, nx))] = inAnyInner ? 0 : 1;
        }
    }
}

/// @brief Chebyshev 距离 1 的膨胀（填小缝、抗离散误差）
inline void dilateChebyshev1(int nx, int nz, const std::vector<uint8_t>& src, std::vector<uint8_t>& dst)
{
    dst.resize(src.size());
    for (int iz = 0; iz < nz; ++iz)
    {
        for (int ix = 0; ix < nx; ++ix)
        {
            uint8_t v = 0;
            for (int dz = -1; dz <= 1 && !v; ++dz)
            {
                const int jz = iz + dz;
                if (jz < 0 || jz >= nz)
                    continue;
                for (int dx = -1; dx <= 1; ++dx)
                {
                    const int jx = ix + dx;
                    if (jx < 0 || jx >= nx)
                        continue;
                    if (src[static_cast<size_t>(gridIndex(jx, jz, nx))])
                    {
                        v = 1;
                        break;
                    }
                }
            }
            dst[static_cast<size_t>(gridIndex(ix, iz, nx))] = v;
        }
    }
}

/// @brief 以 (ix,iz) 为中心的正方形窗口内：本层实体格中，被 dilatedBelow 覆盖的比例
inline float localSupportCoverageRatio(
    const std::vector<uint8_t>& maskL,
    const std::vector<uint8_t>& dilatedBelow,
    int nx,
    int nz,
    int ix,
    int iz,
    int halfWindow)
{
    int denom = 0;
    int numer = 0;
    for (int dz = -halfWindow; dz <= halfWindow; ++dz)
    {
        const int jz = iz + dz;
        if (jz < 0 || jz >= nz)
            continue;
        for (int dx = -halfWindow; dx <= halfWindow; ++dx)
        {
            const int jx = ix + dx;
            if (jx < 0 || jx >= nx)
                continue;
            const size_t idx = static_cast<size_t>(gridIndex(jx, jz, nx));
            if (maskL[idx])
            {
                ++denom;
                if (dilatedBelow[idx])
                    ++numer;
            }
        }
    }
    if (denom <= 0)
        return -1.0f;
    return static_cast<float>(numer) / static_cast<float>(denom);
}

} // namespace overhang_layer_detail
