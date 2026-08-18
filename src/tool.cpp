#include "tool.h"
#include <iostream>
#include <QDebug>
#include <QElapsedTimer> // Added for QElapsedTimer

//@brief 为一个线段创建圆柱体
//@param start 起点
//@param end 终点
//@param lineWidth 线宽
//@param mesh 网格
Mesh createCylinderFromLineSegment(const Point3D& start, const Point3D& end, float lineWidth)
{
    Mesh mesh;
    createCylinderFromLineSegment(start, end, lineWidth, mesh);
    return mesh;
}

void createCylinderFromLineSegment(const Point3D& start, const Point3D& end, float lineWidth, Mesh& mesh)
{
    Point3D direction = end - start;//获取方向
    float length = direction.length();
    direction.normalize();

    int segments = 12;
    Point3D yAxis(0, 1, 0);
    for(int i = 0; i < segments; i++)
    {
        float angle1 = 2 * M_PI * i / segments;
        float angle2 = 2 * M_PI * (i + 1) / segments;
        
        //计算圆柱体截面上的两个点
        Point3D offset1 = yAxis * cos(angle1) * lineWidth/2 
            + direction.cross(yAxis) * sin(angle1) * lineWidth/2;
        
        Point3D offset2 = yAxis * cos(angle2) * lineWidth/2 
            + direction.cross(yAxis) * sin(angle2) * lineWidth/2;
        //添加侧面四边形（两个三角形）
        mesh.addTriangle(start + offset1, start + offset2, end + offset1);
        mesh.addTriangle(start + offset2, end + offset2, end + offset1);
        //添加上下底面
        mesh.addTriangle(start, start + offset1, start + offset2);
        mesh.addTriangle(end, end + offset2, end + offset1);
    }
}


//@brief 根据一条路径生成网格
//@param path 路径
//@param mesh 一条路径的网格
//@param vertices 顶点数据
//@param indices 索引数组
//@param lineWidth 线宽
void generateMeshFromPath(const Path& path, Mesh& mesh)
{
    if(path.getPoints().size()<2)
    {
        return;
    }
    uint32_t offset=0;
    for(size_t i=0;i<=path.getPoints().size()-2;i++)
    {
        //每两个点之间生成线段的网格
        Mesh tempMesh;
        const Point3D& start = path.getPoints()[i];
        const Point3D& end = path.getPoints()[i+1];
        createCylinderFromLineSegment(start,end,path.getLineWidth(),tempMesh);
        //生成偏移量
        uint32_t offetIncrement=tempMesh.getPoints().size();
        //将线段的网格添加到总的网格当中
        mesh.addMesh(tempMesh);

        offset+=offetIncrement;
    }
}

//@brief 根据一条路径生成网格
//@param path 路径
//@param vertices 输出顶点
//@param indices 输出索引
//@return 路径的网格
Mesh generateMeshFromPath(const Path& path) 
{
    Mesh mesh;
   generateMeshFromPath(path, mesh);
    return mesh;
}




//@brief 根据一层生成网格
//@param layer 层
//@param mesh 层网格
//@param vertices 输出顶点
//@param indices 输出索引
void generateMeshFromLayer(const Layer& layer, Mesh& mesh)
{
    Mesh outerContourMesh;
    Mesh innerContourMesh;
    Mesh infillMesh;
    
    //生成外轮廓网格
    for(const Path& path : layer.getOuterContourPaths())
    {
        outerContourMesh.addMesh(generateMeshFromPath(path));
    }
    //生成内轮廓网格
    for(const Path& path : layer.getInnerContourPaths())
    {
        innerContourMesh.addMesh(generateMeshFromPath(path));
    }
    //生成填充网格      
    for(const Path& path : layer.getInfillPaths())
    {
        infillMesh.addMesh(generateMeshFromPath(path));
    }

    //将三个网格添加到总网格当中
    mesh.addMesh(outerContourMesh);
    mesh.addMesh(innerContourMesh);
    mesh.addMesh(infillMesh);
}

//@brief 根据一层生成网格
//@param layer 层
//@param vertices 输出顶点
//@param indices 输出索引
//@return 层的网格
Mesh generateMeshFromLayer(const Layer& layer)
{
    Mesh mesh;
    generateMeshFromLayer(layer, mesh);
    return mesh;
}


//@brief 根据多层生成网格
//@param layers 多层
//@param mesh 多层的网格
//@param vertices 输出顶点
//@param indices 输出索引
void generateMeshFromLayerVertices(const std::vector<Layer>& layers, Mesh& mesh, std::vector<float>& vertices, std::vector<unsigned int>& indices){
    try {
        // 清空输出数组
        vertices.clear();
        indices.clear();
        mesh.clear();
        
        if (layers.empty()) {
            qDebug() << "警告: 没有层数据提供";
            return;
        }
        
        // 限制处理的层数，更灵活地处理
        size_t layerCount = layers.size();
        size_t maxLayers = layerCount;
        
        // 预先估计内存需求
        size_t estimatedPoints = 0;
        size_t estimatedIndices = 0;
        
        // 预扫描前几层来估计大小
        size_t scanLayers = std::min(layerCount, size_t(5));
        for (size_t i = 0; i < scanLayers; i++) {
            const Layer& layer = layers[i];
            
            // 估算外轮廓点数
            for (const Path& path : layer.getOuterContourPaths()) {
                estimatedPoints += path.getPoints().size() * 12 * 4; // 每个点生成约48个网格点
                estimatedIndices += path.getPoints().size() * 12 * 6; // 每个点生成约72个索引
            }
            
            // 估算内轮廓点数
            for (const Path& path : layer.getInnerContourPaths()) {
                estimatedPoints += path.getPoints().size() * 12 * 4;
                estimatedIndices += path.getPoints().size() * 12 * 6;
            }
            
            // 估算填充点数
            for (const Path& path : layer.getInfillPaths()) {
                estimatedPoints += path.getPoints().size() * 12 * 4;
                estimatedIndices += path.getPoints().size() * 12 * 6;
            }
        }
        
        // 根据估计调整最大层数
        if (scanLayers > 0) {
            // 估计总点数 - 更保守的估计
            size_t totalEstimatedPoints = estimatedPoints * (layerCount / scanLayers) * 1.2; // 添加20%余量
            size_t totalEstimatedIndices = estimatedIndices * (layerCount / scanLayers) * 1.2;
            
            // 根据可用内存动态调整
            const size_t targetMaxPoints = 600000; // 目标最大点数
            if (totalEstimatedPoints > targetMaxPoints) {
                // 计算最大处理层数以保持在内存限制内
                double layerRatio = static_cast<double>(targetMaxPoints) / totalEstimatedPoints;
                maxLayers = std::max(size_t(1), static_cast<size_t>(layerCount * layerRatio));
                qDebug() << "警告: 估计网格大小过大 (" << totalEstimatedPoints << " 点)，减少到 " 
                         << maxLayers << "/" << layerCount << " 层";
            }
        }
        
        // 设置超时检测
        QElapsedTimer meshTimer;
        meshTimer.start();
        const int maxMeshGenTimeMs = 10000; // 最大网格生成时间10秒
        
        // 逐层处理
        qDebug() << "开始处理网格层，总层数:" << layerCount << "，将处理:" << maxLayers << "层";
        
        for(size_t i = 0; i < maxLayers; i++){
            if (i % 5 == 0) {
                qDebug() << "处理第 " << i << " 层...";
            }
            
            // 检查是否超时
            if (meshTimer.elapsed() > maxMeshGenTimeMs) {
                qDebug() << "警告: 网格生成时间过长，已处理" << i << "层，停止处理";
                break;
            }
            
            const Layer& layer = layers[i];
            
            // 安全检查：跳过空层
            if (layer.isEmpty()) {
                continue;
            }
            
            // 创建临时网格，避免直接修改主网格
            Mesh layerMesh;
            try {
                layerMesh = generateMeshFromLayer(layer);
            } catch (const std::exception& e) {
                qDebug() << "错误: 生成第 " << i << " 层网格时异常: " << e.what();
                continue;
            } catch (...) {
                qDebug() << "错误: 生成第 " << i << " 层网格时未知异常";
                continue;
            }
            
            // 检查网格大小，防止过大
            if(layerMesh.getPoints().size() > 100000 || layerMesh.getIndices().size() > 200000) {
                qDebug() << "警告: 层 " << i << " 网格过大，跳过此层";
                continue;
            }
            
            // 添加到主网格前检查大小
            if(mesh.getPoints().size() + layerMesh.getPoints().size() > 800000 || 
               mesh.getIndices().size() + layerMesh.getIndices().size() > 1600000) {
                qDebug() << "警告: 总网格即将超限，停止在第 " << i << " 层";
                break;
            }
            
            // 添加到主网格
            try {
                mesh.addMesh(layerMesh);
            } catch (const std::exception& e) {
                qDebug() << "错误: 添加第 " << i << " 层网格时异常: " << e.what();
                break;
            } catch (...) {
                qDebug() << "错误: 添加第 " << i << " 层网格时未知异常";
                break;
            }
            
            // 定期检查总网格大小
            if(mesh.getPoints().size() > 800000 || mesh.getIndices().size() > 1600000) {
                qDebug() << "警告: 网格总大小超过限制，停止添加更多层";
                break;
            }
            
            // 每20层显示一次进度
            if (i % 20 == 19) {
                qDebug() << "已处理 " << i+1 << " 层，网格点数: " << mesh.getPoints().size() 
                         << "，索引数: " << mesh.getIndices().size() 
                         << "，耗时: " << meshTimer.elapsed() << "毫秒";
            }
        }
        
        qDebug() << "网格层处理完成，耗时:" << meshTimer.elapsed() << "毫秒";
        
        // 最终网格规模检查
        if (mesh.getPoints().size() == 0 || mesh.getIndices().size() == 0) {
            qDebug() << "警告: 生成的网格为空";
            return;
        }
        
        // 将顶点添加到vertices当中
        try {
            vertices.reserve(mesh.getPoints().size() * 3); // 预分配内存
            for(const auto& pt : mesh.getPoints()){
                vertices.push_back(pt.Position.x);
                vertices.push_back(pt.Position.y);
                vertices.push_back(pt.Position.z);
            }
        } catch (const std::exception& e) {
            qDebug() << "错误: 顶点数组分配失败: " << e.what();
            vertices.clear();
            return;
        }
        
        // 将索引添加到indices当中
        try {
            indices.reserve(mesh.getIndices().size()); // 预分配内存
            for(const auto& idx : mesh.getIndices()){
                indices.push_back(idx);
            }
        } catch (const std::exception& e) {
            qDebug() << "错误: 索引数组分配失败: " << e.what();
            vertices.clear();
            indices.clear();
            return;
        }
        
        qDebug() << "网格生成完成: " << vertices.size()/3 << " 顶点, " << indices.size()/3 << " 三角形";
        
    } catch (const std::exception& e) {
        qDebug() << "严重错误: generateMeshFromLayerVertices 异常: " << e.what();
        vertices.clear();
        indices.clear();
    } catch (...) {
        qDebug() << "严重错误: generateMeshFromLayerVertices 未知异常";
        vertices.clear();
        indices.clear();
    }
}

//@brief 根据多层生成网格
//@param layers 多层
//@param vertices 输出顶点
//@param indices 输出索引
//@return 多层的网格
Mesh generateMeshFromLayerVertices(const std::vector<Layer>& layers,std::vector<float>& vertices,std::vector<unsigned int>& indices){
    Mesh mesh;
    generateMeshFromLayerVertices(layers, mesh, vertices, indices);
    return mesh;
}








