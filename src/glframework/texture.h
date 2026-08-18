#pragma once

#include<string>
#include"core.h"


class Texture {
public:
	GLuint mTexture{ 0 };
	Texture(const std::string& path,unsigned int unit);
	~Texture();
	GLuint getMUnit() {
		return mUnit;
	}
	//在初始化之后，切换纹理绑定
	void bind();


private:
	int mWidth{ 0 };
	int mHeight{ 0 };
	GLuint mUnit{ 0 };


};