#pragma once

#include "../dataStructure/Layer.h"
#include "../dataStructure/Path.h"
#include "../dataStructure/Point3D.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

///@brief 规划路径
struct PlannedPath
{
    const Path* path = nullptr;
    bool reverse = false;
    PathType type = PathType::Infill;
};

///@brief 路径规划器详细实现
namespace path_order_detail
{
/// @brief 获取路径起点
/// @param p 路径
/// @param rev 是否反向
/// @return 路径起点
/// @details 获取路径起点，如果路径为空，则返回0,0,0。如果反向，则返回路径末点，否则返回路径起点。
inline glm::vec3 startPoint(const Path& p, bool rev)
{
    const auto& pts = p.getPoints();
    if (pts.empty())
        return glm::vec3(0.0f);
    return rev ? pts.back().Position : pts.front().Position;
}
/// @brief 获取路径终点
/// @param p 路径
/// @param rev 是否反向
/// @return 路径终点
/// @details 获取路径终点，如果路径为空，则返回0,0,0。如果反向，则返回路径起点，否则返回路径末点。
inline glm::vec3 endPoint(const Path& p, bool rev)
{
    const auto& pts = p.getPoints();
    if (pts.empty())
        return glm::vec3(0.0f);
    return rev ? pts.front().Position : pts.back().Position;
}
/// @brief 计算两点之间的距离平方
/// @param a 点1
/// @param b 点2
/// @return 距离平方
/// @details 计算两点之间的距离平方，如果两点为0,0,0，则返回0。
inline float dist2(const glm::vec3& a, const glm::vec3& b)
{
    const glm::vec3 d = a - b;
    return glm::dot(d, d);
}

/// @brief 贪婪最近邻 + 允许反向
/// @param paths 路径集合
/// @param startPos 起点
/// @return 规划路径
/// @details 贪婪最近邻 + 允许反向，如果路径集合为空，则返回空路径集合
/// @details 贪婪最近邻 + 允许反向，从起点开始，找到距离起点最近的未使用路径，如果路径的起点距离起点最近，则使用该路径，否则使用反向路径。说白了就是对于当前点，无论从起点还是终点出发，距离最近的路径就是最佳路径。
inline std::vector<PlannedPath> greedyNearestWithReverse(
    const std::vector<Path>& paths,
    const glm::vec3& startPos)
{
    //规划路径
    std::vector<PlannedPath> plan;
    plan.reserve(paths.size());
    if (paths.empty())
        return plan;

    //使用过的路径
    std::vector<char> used(paths.size(), 0);
    glm::vec3 cur = startPos;

    for (size_t k = 0; k < paths.size(); ++k)
    {
        float best = std::numeric_limits<float>::infinity();//初始化最佳距离为无穷大
        size_t bestIdx = 0;//初始化最佳索引为0
        bool bestRev = false;//初始化最佳反向为false
        bool found = false;//初始化是否找到最佳路径为false
        for (size_t i = 0; i < paths.size(); ++i)
        {
            if (used[i])//如果路径已使用，则跳过
                continue;
            const Path& p = paths[i];//获取路径
            const auto& pts = p.getPoints();//获取路径点
            if (pts.size() < 2)//如果路径点小于2，则跳过
                continue;

            const float dF = dist2(cur, startPoint(p, false));
            if (dF < best)//如果距离小于最佳距离，则更新最佳距离、最佳索引、最佳反向、找到最佳路径
            {
                best = dF;
                bestIdx = i;
                bestRev = false;
                found = true;
            }
            //反向
            const float dR = dist2(cur, startPoint(p, true));
            if (dR < best)//如果距离小于最佳距离，则更新最佳距离、最佳索引、最佳反向、找到最佳路径
            {
                best = dR;
                bestIdx = i;
                bestRev = true;
                found = true;
            }
        }

        if (!found)
            break;

        used[bestIdx] = 1;
        PlannedPath one;
        one.path = &paths[bestIdx];
        one.reverse = bestRev;
        one.type = paths[bestIdx].getType();
        plan.push_back(one);

        cur = endPoint(*one.path, one.reverse);//更新当前点
    }

    return plan;
}
} // namespace path_order_detail

/// @brief 规划层路径
/// @param layer 层
/// @return 规划路径
/// @details 规划层路径，如果层为空，则返回空路径集合。如果层的外轮廓路径为空，则返回空路径集合。如果层的外轮廓路径不为空，则规划外轮廓路径。如果层的外轮廓路径不为空，则规划内轮廓路径。如果层的外轮廓路径不为空，则规划填充路径。
inline std::vector<PlannedPath> planLayerPathOrder(const Layer& layer)
{
    // 规则：外轮廓 -> 内轮廓 -> 填充；每一类内部用最近邻 + 允许反向
    std::vector<PlannedPath> out;
    glm::vec3 cursor(0.0f, layer.getZHeight(), 0.0f);//路径规划的起始点

    auto addGroup = [&](const std::vector<Path>& group) {//添加组
        std::vector<PlannedPath> g = path_order_detail::greedyNearestWithReverse(group, cursor);
        if (!g.empty())//如果组不为空，则更新当前点
            cursor = path_order_detail::endPoint(*g.back().path, g.back().reverse);
        out.insert(out.end(), g.begin(), g.end());
    };

    addGroup(layer.getOuterContourPaths());//规划外轮廓路径
    addGroup(layer.getInnerContourPaths());//规划内轮廓路径
    addGroup(layer.getInfillPaths());//规划填充路径
    return out;
}

/// @brief 规划所有层路径
/// @param layers 层集合
/// @return 规划所有层路径
/// @details 规划所有层路径，如果层集合为空，则返回空路径集合。如果层集合不为空，则规划所有层路径。
/// @return 规划所有层路径
inline std::vector<std::vector<PlannedPath>> planAllLayersPathOrder(const std::vector<Layer>& layers)
{
    std::vector<std::vector<PlannedPath>> all;//所有层路径
    all.reserve(layers.size());
    for (const Layer& l : layers)
        all.push_back(planLayerPathOrder(l));//规划所有层路径
    return all;
}

