#pragma once
#include"core.h"
#include<string>

class Shader {
public:

    GLuint mProgram{ 0 };//shader

	Shader(const char* vertexPath, const char* fragmentPath);
	~Shader();

	void begin();//开始使用当前shader

	void end();//结束使用当前shader

	void setFloat(const std::string& name, float value);

	void setFloat(const std::string& name, float* values);

	void setInt(const std::string& name, int value);

	void setMat4x4(const std::string& name, glm::mat4 values);

private:
	void checkShaderErrors(GLuint target,std::string type);
};