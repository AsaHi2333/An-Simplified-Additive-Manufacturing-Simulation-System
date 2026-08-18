#include "errorCheck.h"
#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<iostream>
#include<string>
#include<assert.h>
using namespace std;

void errorCheck() {

	GLenum errorCode = glGetError();
	string errorName = " ";
	switch (errorCode)
	{
	case GL_NO_ERROR: return ;
	case GL_INVALID_ENUM: errorName = "INVALID_ENUM"; break;
	case GL_INVALID_VALUE: errorName = "INVALID_VALUE"; break;
	case GL_INVALID_OPERATION: errorName = "INVALID_OPERATION"; break;
	case GL_OUT_OF_MEMORY: errorName = "GL_OUT_OF_MEMORY"; break;
	default:
		errorName = "UNKNOWN ERROR";
	}
	cout << "error:  " << errorName << endl;
	assert(false);
}
