#include"texture.h"

#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION //使用stb_image前必须定义宏
#endif // !STB_IMAGE_IMPLEMENTATION
#include"../application/stb_image.h"


Texture::Texture(const std::string& path, unsigned int unit) {


	mUnit = unit;
	//stbImage 读取图片
	int channels;
	//--反转y轴
	stbi_set_flip_vertically_on_load(true);
	//--读取数据
	unsigned char* data = stbi_load(path.c_str(), &mWidth, &mHeight, &channels, STBI_rgb_alpha);
	//cout << width << endl << height << endl << channels << endl;

	//生成纹理并激活单元绑定
	glGenTextures(1, &mTexture);
	glActiveTexture(GL_TEXTURE0+mUnit);//激活0号纹理单元
	glBindTexture(GL_TEXTURE_2D, mTexture);

	//传输纹理数据,开辟显存
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, mWidth, mHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

	//--释放数据
	stbi_image_free(data);

	//设置纹理的过滤方式
	//所需<纹理大小
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//所需>纹理大小
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	//设置纹理的包裹方式,s-u,t-v
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);





}
Texture::~Texture() {



}
void Texture::bind() {

	glActiveTexture(GL_TEXTURE0 + mUnit);
	glBindTexture(GL_TEXTURE_2D, mTexture);

}