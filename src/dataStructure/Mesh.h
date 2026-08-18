#pragma once
#include <vector>
#include <unordered_map>
#include <iostream>
#include <QDebug>
#include "Point3D.h"
using namespace std;

// ============================================================
/// @class Mesh
/// @brief  一个网格，包含所有点和索引。这个类是之前项目初期，测试逐层堆积时用的。模型导入现在已经使用NewMesh类。
/// @details
/// - 构造时：接收 points / indices，并调用 addTriangle() 添加三角形。
/// - addMesh()：将另一个网格添加到当前网格，自动处理顶点去重。
/// - clear()：清空网格。
/// - getPoints()：获取所有点。
/// - getIndices()：获取所有索引。
/// ============================================================
class Mesh
{
private:
    vector<Point3D> points;
    vector<uint32_t> indices;
    unordered_map<Point3D, uint32_t> pointIndexMap;//用于顶点去重

public:
    //用三个点加三角形网格
    void addTriangle(const Point3D& p1, const Point3D& p2, const Point3D& p3)
    {
        auto addVertex = [&](const Point3D& p){
            auto it = pointIndexMap.find(p);
            if(it != pointIndexMap.end())
            {
                return it->second;
            }
           uint32_t newIndex = points.size();
           points.push_back(p);
           pointIndexMap[p] = newIndex;
           return newIndex;
        };

        uint32_t index1 = addVertex(p1);
        uint32_t index2 = addVertex(p2);
        uint32_t index3 = addVertex(p3);
        indices.push_back(index1);
        indices.push_back(index2);
        indices.push_back(index3);
    }

    //将另一个网格添加到当前网格
    void addMesh(const Mesh& mesh)
    {
        if(mesh.getPoints().empty() || mesh.getIndices().empty())
        {
            return;
        }
        
        try {
            // 提前检查并预留空间，避免频繁重新分配
            size_t newPointCount = points.size() + mesh.getPoints().size();
            size_t newIndexCount = indices.size() + mesh.getIndices().size();
            
            // 如果新数据会导致容量过大，采取保护措施
            if(newPointCount > 1000000 || newIndexCount > 3000000) {
                throw std::runtime_error("网格合并后数据过大");
            }
            
            // 预留空间
            if(points.capacity() < newPointCount) {
                points.reserve(newPointCount);
            }
            if(indices.capacity() < newIndexCount) {
                indices.reserve(newIndexCount);
            }
            
            // 使用批量处理减少map查找次数
            std::vector<uint32_t> indexMapping(mesh.getPoints().size(), UINT32_MAX);
            
            // 逐三角形合并，保证索引和顶点唯一性
            const std::vector<Point3D>& pts = mesh.getPoints();
            const std::vector<uint32_t>& idx = mesh.getIndices();
            
            // 首先确保索引数量是3的倍数（三角形）
            if(idx.size() % 3 != 0) {
                throw std::runtime_error("无效的索引数量，不是三角形网格");
            }
            
            for (size_t i = 0; i + 2 < idx.size(); i += 3) {
                // 安全检查，避免索引越界
                if(idx[i] >= pts.size() || idx[i+1] >= pts.size() || idx[i+2] >= pts.size()) {
                    continue; // 跳过无效三角形
                }
                
                // 获取三个顶点
                const Point3D& p1 = pts[idx[i]];
                const Point3D& p2 = pts[idx[i+1]];
                const Point3D& p3 = pts[idx[i+2]];
                
                // 添加三角形（会自动处理顶点去重）
                addTriangle(p1, p2, p3);
            }
        } catch(const std::exception& e) {
            // 打印错误但不抛出，让程序继续运行
            qDebug() << "Mesh::addMesh 错误: " << e.what();
        } catch(...) {
            qDebug() << "Mesh::addMesh 未知错误";
        }
    }

    //清空网格
    void clear()
    {
        points.clear();
        indices.clear();
        pointIndexMap.clear();
    }

    // 获取所有点
    const vector<Point3D>& getPoints() const {
        return points;
    }

    // 获取所有索引
    const vector<uint32_t>& getIndices() const {
        return indices;
    }
};