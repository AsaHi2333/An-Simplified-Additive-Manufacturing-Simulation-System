#pragma once
#include "dataStructure/Mesh.h"
#include "dataStructure/Path.h"
#include "dataStructure/Layer.h"
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//@brief 为一个线段创建圆柱体
//@param start 起点
//@param end 终点
//@param lineWidth 线宽
//@param mesh 网格
void createCylinderFromLineSegment(const Point3D& start, const Point3D& end, float lineWidth, Mesh& mesh);


Mesh createCylinderFromLineSegment(const Point3D& start, const Point3D& end, float lineWidth);


//@brief 根据一条路径生成网格
//@param path 路径
//@param mesh 网格
//@param vertices 输出顶点
//@param indices 输出索引
//@param lineWidth 线宽
void generateMeshFromPath(const Path& path, Mesh& mesh);

//@brief 根据一条路径生成网格
//@param path 路径
//@param vertices 输出顶点
//@param indices 输出索引
//@return 路径的网格
Mesh generateMeshFromPath(const Path& path);

//@brief 根据一层生成网格
//@param layer 层
//@param mesh 网格
//@param vertices 输出顶点
//@param indices 输出索引
void generateMeshFromLayer(const Layer& layer, Mesh& mesh);


Mesh generateMeshFromLayer(const Layer& layer);

void generateMeshFromLayerVertices(const std::vector<Layer>& layers,Mesh& mesh,std::vector<float>& vertices,std::vector<unsigned int>& indices);

Mesh generateMeshFromLayerVertices(const std::vector<Layer>& layers,std::vector<float>& vertices,std::vector<unsigned int>& indices);





