#pragma once
#include "Path.h"
#include <vector>

class Layer {
private:
    int layer_id;                   // 层编号（从0开始）
    float z_height;                 // 当前层Z高度
    std::vector<Path> outerContourPaths;        // 外轮廓路径
    std::vector<Path> innerContourPaths;        // 内轮廓路径
    std::vector<Path> infillPaths;        // 填充路径

public:
    // 构造函数
    Layer() : layer_id(0), z_height(0.0f) {}
    Layer(int id, float z) : layer_id(id), z_height(z) {}

    // 基础方法
    void addPath(const Path& path) {
        if (path.getType() == PathType::OuterContour) {
            outerContourPaths.push_back(path);
        } else if (path.getType() == PathType::InnerContour) {
            innerContourPaths.push_back(path);
        } else if (path.getType() == PathType::Infill) {
            infillPaths.push_back(path);
        }
    }


    // getter
    int getLayerId() const { return layer_id; }
    float getZHeight() const { return z_height; }
    const std::vector<Path>& getOuterContourPaths() const { return outerContourPaths; }
    const std::vector<Path>& getInnerContourPaths() const { return innerContourPaths; }
    const std::vector<Path>& getInfillPaths() const { return infillPaths; }


    // setter
    void setLayerId(int id) { layer_id = id; }
    void setZHeight(float z) { z_height = z; }


    // 实用功能
    bool isEmpty() const {
        return outerContourPaths.empty() && innerContourPaths.empty() && infillPaths.empty();
    }

   
};