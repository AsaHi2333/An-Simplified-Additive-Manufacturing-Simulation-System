#pragma once
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

// ========== 相机参数 ===========

// 相机参数
// 摄像机位置
extern glm::vec3 cameraPos;
// 摄像机朝向
extern glm::vec3 cameraFront;
// 摄像机上方向
extern glm::vec3 cameraUp;
// 视场角
extern float fov;

// ========== 其余全局变量 ===========
extern float deltaTime; // 每帧时间间隔
extern float lastFrame; // 上一帧时间
extern float yaw;     // 摄像机偏航角
extern float pitch;     // 摄像机俯仰角
extern float lastX, lastY; // 上一帧鼠标位置
extern bool firstMouse;         // 是否为第一次捕获鼠标

extern const unsigned int SCR_WIDTH;  // 窗口宽度
extern const unsigned int SCR_HEIGHT; // 窗口高度

// 函数声明
void processInput(GLFWwindow* window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);

