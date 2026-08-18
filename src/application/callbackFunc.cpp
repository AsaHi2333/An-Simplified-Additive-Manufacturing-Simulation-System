//回调函数
#include "callbackFunc.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// ========== 相机参数 ===========

// 相机参数
// 摄像机位置
glm::vec3 cameraPos(2.0f, 2.0f, 4.0f);  // 调整相机位置以便看到所有点
// 摄像机朝向
glm::vec3 cameraFront(0.0f, -0.3f, -1.0f);
// 摄像机上方向
glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);
// 视场角
float fov = 45.0f;

// ========== 其余全局变量 ===========
float deltaTime = 0.0f; // 每帧时间间隔
float lastFrame = 0.0f; // 上一帧时间
float yaw = -90.0f;     // 摄像机偏航角
float pitch = 0.0f;     // 摄像机俯仰角
float lastX = 400, lastY = 300; // 上一帧鼠标位置
bool firstMouse = true;         // 是否为第一次捕获鼠标

const unsigned int SCR_WIDTH = 800;  // 窗口宽度
const unsigned int SCR_HEIGHT = 600; // 窗口高度

void processInput(GLFWwindow* window) {
    float speed = 2.5f * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += speed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= speed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * speed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * speed;
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }
    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;
    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;
    yaw += xoffset;
    pitch += yoffset;
    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;
    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    fov -= (float)yoffset;
    if (fov < 1.0f) fov = 1.0f;
    if (fov > 45.0f) fov = 45.0f;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}
