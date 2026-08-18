#pragma once
#include "Point3D.h"
#include <vector>

enum class PathType {
    OuterContour,   // 外轮廓
    InnerContour,   // 内轮廓（孔洞）
    Infill          // 填充
};


class Path {

private:
    PathType type;
    std::vector<Point3D> points;
    float lineWidth;

public:
   
    Path(){}
    Path(PathType t,float lineWidth) : type(t),lineWidth(lineWidth) {}
    Path(PathType t,float lineWidth,const std::vector<Point3D>& points) : type(t),lineWidth(lineWidth),points(points) {}
    
    void addPoint(const Point3D& p) { points.push_back(p); }
    // 其他方法：长度计算、平滑、G代码导出等

    PathType getType() const { return type; }
    const std::vector<Point3D>& getPoints() const { return points; }
    float getLineWidth() const { return lineWidth; }
    float setLineWidth(float _lineWidth)
    {
        lineWidth=_lineWidth;
    } 




};