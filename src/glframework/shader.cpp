#include"shader.h"
#include"../wrapper/errorCheck.h"

#include<string>
#include<fstream>
#include<sstream>
#include<iostream>
using namespace std;


Shader::Shader(const char* vertexPath, const char* fragmentPath) {
	//存储shader代码
	string vertexCode;
	string fragmentCode;

	//用于读取vs fs文件
	ifstream vertexShaderFile;
	ifstream fragmentShaderFile;
	
	//异常抛出
	vertexShaderFile.exceptions(ifstream::failbit | ifstream::badbit);
	fragmentShaderFile.exceptions(ifstream::failbit | ifstream::badbit);

	//打开文件
	try {
		cout << "Trying to open vertex shader: " << vertexPath << endl;
		vertexShaderFile.open(vertexPath);
		cout << "Trying to open fragment shader: " << fragmentPath << endl;
		fragmentShaderFile.open(fragmentPath);

		//将文件内容读入字符串流
		stringstream vertexShaderStream, fragmentShaderStream;
		vertexShaderStream << vertexShaderFile.rdbuf();
		fragmentShaderStream << fragmentShaderFile.rdbuf();

		vertexShaderFile.close();
		fragmentShaderFile.close();
		//将字符串从字符串流当中读出来
		vertexCode = vertexShaderStream.str();
		fragmentCode = fragmentShaderStream.str();
		
		cout << "Vertex shader loaded successfully, size: " << vertexCode.length() << endl;
		cout << "Fragment shader loaded successfully, size: " << fragmentCode.length() << endl;
	}
	catch (ifstream::failure& e)
	{
		cout << "ERROR:SHADER FILE ERROR: " << e.what() << endl;
		cout << "Vertex path: " << vertexPath << endl;
		cout << "Fragment path: " << fragmentPath << endl;
		throw; // 重新抛出异常，让程序终止
	}

	//创建shader程序
	GLuint vertex, fragment;
	vertex = glCreateShader(GL_VERTEX_SHADER);
	fragment = glCreateShader(GL_FRAGMENT_SHADER);

	//为shader程序输入shader代码
	const char* vertexShaderSource = vertexCode.c_str();
	const char* fragmentShaderSource = fragmentCode.c_str();

	// cout <<"vs:"<<endl<< vertexCode << endl;
	// cout << "fs:" << endl << fragmentCode << endl;

	glShaderSource(vertex, 1, &vertexShaderSource, NULL);
	glShaderSource(fragment, 1, &fragmentShaderSource, NULL);

	

	//shader编译
	glCompileShader(vertex);//编译

	checkShaderErrors(vertex, "COMPILE");


	glCompileShader(fragment);

	checkShaderErrors(fragment, "COMPILE");

	//shader链接

	mProgram = glCreateProgram();//创建盒子
	glAttachShader(mProgram, vertex);//附着shader
	glAttachShader(mProgram, fragment);

	glLinkProgram(mProgram);//链接程序与shader

	//检查
	checkShaderErrors(mProgram, "LINK");

	//清理
	glDeleteShader(vertex);
	glDeleteShader(fragment);



}
Shader::~Shader() {
}

void Shader::begin() {
	GL_CALL(glUseProgram(mProgram));
}//开始使用当前shader

void Shader::end() {
	GL_CALL(glUseProgram(0));
}//结束使用当前shader

void Shader::setFloat(const string& name, float value) {
	//通过名称拿到uniform变量的位置location
	GLuint location = GL_CALL(glGetUniformLocation(mProgram, name.c_str()));

	//通过location更新uniform变量的值
	glUniform1f(location, value);


}

void Shader::setFloat(const std::string& name, float* values) {

	GLuint location = GL_CALL(glGetUniformLocation(mProgram, name.c_str()));

	glUniform3fv(location, 1, values);


}

void Shader::setInt(const std::string& name, int value) {

	GLuint location = GL_CALL(glGetUniformLocation(mProgram, name.c_str()));

	glUniform1i(location,value);

}

void  Shader::setMat4x4(const std::string& name, glm::mat4 values)
{
	GLuint location = glGetUniformLocation(mProgram, name.c_str());
	glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(values));

}

void  Shader::checkShaderErrors(GLuint target, std::string type) {

	int success = 0;
	char infoLog[1024];

	if(type == "COMPILE"){
		glGetShaderiv(target, GL_COMPILE_STATUS, &success);//检查
		if (!success) {
			glGetShaderInfoLog(target, 1024, NULL, infoLog);
			cout << "Error:SHADER COMPILE ERROR" << endl << infoLog << endl;
			return;
		}
		return;
	}
	if(type == "LINK") {

		glGetProgramiv(target, GL_LINK_STATUS, &success);//检查
		if (!success) {
			glGetProgramInfoLog(target, 1024, NULL, infoLog);
			cout << "Error:SHADER LINK ERROR --PROGRAM" << endl << infoLog << endl;
			return;
		}
		return;
	}
	else {
		cout << "Error: Check shader errors Type is wrong" << endl;
		return;
	}
}