#pragma once
#include <cmath>
#include <functional>
#include <glm/glm.hpp>

#define MAX_BONE_INFLUENCE 4
// ============================================================
/// @class Point3D
/// @brief  一个3D点，包含位置，法线，纹理坐标，切线，副切线，骨骼ID和权重。
/// ===========================================================
/// @details
/// - 构造时：接收位置，法线，纹理坐标，切线，副切线。
/// - Position()：获取位置。
/// - Normal()：获取法线。
/// - TexCoords()：获取纹理坐标。
class Point3D {
public:
    // 几何位置（核心：直接使用 glm::vec3）
    glm::vec3 Position;

    // 为模型渲染复用，兼容原 Vertex 语义
    glm::vec3 Normal;
    glm::vec2 TexCoords;
    glm::vec3 Tangent;
    glm::vec3 Bitangent;
    int m_BoneIDs[MAX_BONE_INFLUENCE]{0, 0, 0, 0};
    float m_Weights[MAX_BONE_INFLUENCE]{0.f, 0.f, 0.f, 0.f};

    Point3D(float x, float y, float z)
        : Position(x, y, z), Normal(0.0f), TexCoords(0.0f), Tangent(0.0f), Bitangent(0.0f) {}
    Point3D() : Position(0.0f), Normal(0.0f), TexCoords(0.0f), Tangent(0.0f), Bitangent(0.0f) {}
    Point3D(const Point3D& other) = default;
    Point3D(Point3D&& other) = default;
    Point3D& operator=(const Point3D& other) = default;
    Point3D& operator=(Point3D&& other) = default;

    Point3D operator-(const Point3D& other) const {
        return Point3D(Position.x - other.Position.x, Position.y - other.Position.y, Position.z - other.Position.z);
    }
    Point3D operator+(const Point3D& other) const {
        return Point3D(Position.x + other.Position.x, Position.y + other.Position.y, Position.z + other.Position.z);
    }
    Point3D operator*(float scalar) const {
        return Point3D(Position.x * scalar, Position.y * scalar, Position.z * scalar);
    }
    Point3D operator/(float scalar) const {
        return Point3D(Position.x / scalar, Position.y / scalar, Position.z / scalar);
    }

    bool operator==(const Point3D& other) const {
        const float epsilon = 1e-6f;
        return std::abs(Position.x - other.Position.x) < epsilon &&
               std::abs(Position.y - other.Position.y) < epsilon &&
               std::abs(Position.z - other.Position.z) < epsilon;
    }

    float length() const {
        return std::sqrt(Position.x * Position.x + Position.y * Position.y + Position.z * Position.z);
    }
    void normalize() {
        float len = length();
        if (len > 0.0f) {
            Position.x /= len;
            Position.y /= len;
            Position.z /= len;
        }
    }
    Point3D cross(const Point3D& other) const {
        return Point3D(
            Position.y * other.Position.z - Position.z * other.Position.y,
            Position.z * other.Position.x - Position.x * other.Position.z,
            Position.x * other.Position.y - Position.y * other.Position.x
        );
    }
    float dot(const Point3D& other) const {
        return Position.x * other.Position.x + Position.y * other.Position.y + Position.z * other.Position.z;
    }

    // 便于老代码迁移：读取 xyz
    float x() const { return Position.x; }
    float y() const { return Position.y; }
    float z() const { return Position.z; }
};

//告诉标准库：当key是point3d时，使用这个哈希函数
namespace std {
template <>
struct hash<Point3D> {
    size_t operator()(const Point3D& p) const {
        int x_int = static_cast<int>(p.Position.x * 1000000);
        int y_int = static_cast<int>(p.Position.y * 1000000);
        int z_int = static_cast<int>(p.Position.z * 1000000);

        size_t h1 = std::hash<int>{}(x_int);
        size_t h2 = std::hash<int>{}(y_int);
        size_t h3 = std::hash<int>{}(z_int);
        return ((h1 ^ (h2 << 1)) >> 1) ^ (h3 << 1);
    }
};
}