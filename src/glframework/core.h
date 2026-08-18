#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>



//一定要将#include<glad\glad.h>放到#include <GLFW\glfw3.h>前面，
//否则回出现错误：
//#error 指令 : OpenGL header already included, remove this include, glad already provides it OpenGLMFCTest