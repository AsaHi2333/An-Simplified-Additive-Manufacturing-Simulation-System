#include "temp.h"
#include <cmath>
#define M_PI 3.14159265358979323846

// 生成圆形轮廓
//@param centerX 圆心X坐标
//@param centerZ 圆心Z坐标
//@param height 圆心Y坐标
//@param radius 圆的半径
//@param segments 圆的段数
//@param lineWidth 线的宽度
//@return 圆形轮廓路径
Path generateCirclePath(float centerX, float centerZ, float height, float radius, int segments, float lineWidth) {
    Path path(PathType::OuterContour, lineWidth);
    
    // 限制段数，确保合理范围
    if (segments > 120) segments = 120;
    if (segments < 8) segments = 8;
    
    // 为轮廓计算实际半径 - 考虑线宽的一半
    // 半径需要内缩线宽的一半，这样渲染时线的外边缘正好在指定半径处
    float actualRadius = radius - lineWidth * 0.5f;
    
    // 防止半径过小导致负值
    if (actualRadius < 0.001f) actualRadius = 0.001f;
    
    // 生成闭合的圆形轮廓
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = centerX + actualRadius * cos(angle);
        float z = centerZ + actualRadius * sin(angle);
        path.addPoint(Point3D(x, height, z));
    }
    
    return path;
}


// 生成圆形轮廓的内部填充路径
//@param centerX 圆心X坐标
//@param centerZ 圆心Z坐标
//@param height 圆心Y坐标
//@param radius 圆的半径
//@param step 填充步长
//@param lineWidth 线的宽度
Path generateZigzagFillInCircle(float centerX, float centerZ, float height, float radius, float step, float lineWidth) {
    Path path(PathType::Infill, lineWidth);
    bool leftToRight = true;
    
    // 确保步长为正数
    if (step <= 0.0f) step = 0.01f;
    
    // 计算内部填充的有效半径 - 需要比轮廓线再内缩一些
    // 考虑线宽影响，轮廓线内缩了lineWidth*0.5，填充也需要内缩
    float effectiveRadius = radius - lineWidth * 0.6f; // 比轮廓多内缩10%线宽，避免重叠
    
    // 防止半径过小导致负值
    if (effectiveRadius < 0.001f) effectiveRadius = 0.001f;
    
    // 计算填充范围
    float minZ = centerZ - effectiveRadius;
    float maxZ = centerZ + effectiveRadius;
    
    // 动态计算合适的线数量，避免过多或过少
    float zRange = maxZ - minZ;
    if (zRange <= 0) return path; // 如果范围无效，返回空路径
    
    int estimatedLineCount = static_cast<int>(zRange / step) + 1;
    
    // 如果估计的线条太多，限制数量
    int maxLines = 500; // 允许更多填充线
    if (estimatedLineCount > maxLines) {
        step = zRange / maxLines;
    }
    
    // 确保第一条线从minZ开始
    for (float z = minZ; z <= maxZ; z += step) {
        // 计算当前z位置与圆心的距离
        float dz = z - centerZ;
        float dzSquared = dz * dz;
        
        // 如果超出有效半径范围，跳过
        if (dzSquared >= effectiveRadius * effectiveRadius) continue;
        
        // 计算水平方向上的交点 - 使用更精确的计算
        float dx = std::sqrt(effectiveRadius * effectiveRadius - dzSquared);
        
        // 确保dx是有效值
        if (std::isnan(dx) || dx <= 0) continue;
        
        float x1 = centerX - dx;
        float x2 = centerX + dx;
        
        // 添加点 - 确保交替方向以优化打印路径
        if (leftToRight) {
            path.addPoint(Point3D(x1, height, z));
            path.addPoint(Point3D(x2, height, z));
        } else {
            path.addPoint(Point3D(x2, height, z));
            path.addPoint(Point3D(x1, height, z));
        }
        
        // 切换方向
        leftToRight = !leftToRight;
    }
    
    return path;
}
