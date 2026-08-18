#pragma once

#include<glad/glad.h>

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<cstddef>

#include"../glframework/shader.h"
#include"Point3D.h"

#include<string>
#include<vector>

using namespace std;

// 材质贴图描述（勿命名为全局 ::Texture，会与 glframework 的 class Texture 链接触发 LNK2005）
struct MeshTextureInfo {
    unsigned int id;
    string type;
    string path;
};
// ============================================================
/// @class NewMesh
/// @brief  将 Assimp mesh 数据封装为 OpenGL 可渲染对象。
/// @details
// - 构造时：接收 vertices / indices / textures，并调用 setupMesh() 上传到 GPU，初始化 VAO。
// - Draw()：绑定多个纹理贴图到 GL_TEXTURE0+i，并通过 shader 的 sampler uniform 指定采样来源，
//          然后调用 glDrawElements 绘制当前 mesh。
///@note
// - Shader 顶点属性布局 location 与 Point3D 结构体的字段偏移需要保持一致。
// ============================================================
class NewMesh{
public:
    //模型数据:顶点、索引、纹理
    vector<Point3D> vertices;
    vector<unsigned int>indices;
    vector<MeshTextureInfo> textures;//mesh关联的所有贴图
    unsigned int VAO;

    NewMesh(vector<Point3D>_vertices,vector<unsigned int>_indices,vector<MeshTextureInfo>_textures):
    vertices(_vertices),indices(_indices),textures(_textures)
    {
        setupMesh();
    }
    void Draw(Shader& shader)
    {
        unsigned int diffuseNr = 1;//漫反射贴图
        unsigned int specularNr = 1;//镜面反射(高光、镜面)贴图
        unsigned int normalNr = 1;//法线贴图
        unsigned int heightNr = 1;//高度贴图
        for(unsigned int i = 0; i < textures.size(); i++)
        {
            //
            //每张纹理都会占用一个纹理单元(GL_TEXTURE0+i)，然后告诉shader：
            //texture_diffuse1 这个 sampler 去读纹理单元 i 上的纹理
            glActiveTexture(GL_TEXTURE0 + i);// 把当前 OpenGL 状态的“激活纹理单元”切到 i 号单元。
            string number;
            string name=textures[i].type;
            if(name=="texture_diffuse")
            {
                number=std::to_string(diffuseNr++);
            }
            else if(name=="texture_specular")
            {
                number=std::to_string(specularNr++);
            }
            else if
            (name=="texture_normal")
            {
                number=std::to_string(normalNr++);
            }
            else if
            (name=="texture_height")
            {
                number=std::to_string(heightNr++);
            }
            //在shader当中找到uniform变量(sampler2D)对应的位置，并将其投射为i
            glUniform1i(glGetUniformLocation(shader.mProgram,(name+number).c_str()),i); 
            //把贴图绑定到纹理单元i上
            glBindTexture(GL_TEXTURE_2D,textures[i].id);
        }
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES,static_cast<unsigned int>(indices.size()),GL_UNSIGNED_INT,0);
        glActiveTexture(GL_TEXTURE0);
    }
private:
    unsigned int VBO,EBO;
    
    void setupMesh()
    {
        glGenVertexArrays(1,&VAO);
        glGenBuffers(1,&VBO);
        glGenBuffers(1,&EBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER,VBO);

        glBufferData(GL_ARRAY_BUFFER,vertices.size()*sizeof(Point3D),&vertices[0],GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,indices.size()*sizeof(unsigned int),&indices[0],GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Point3D),(void*)offsetof(Point3D, Position));

        glEnableVertexAttribArray(1);	
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Point3D), (void*)offsetof(Point3D, Normal));

        glEnableVertexAttribArray(2);	
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Point3D), (void*)offsetof(Point3D, TexCoords));

        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Point3D), (void*)offsetof(Point3D, Tangent));

        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Point3D), (void*)offsetof(Point3D, Bitangent));
		// ids
		glEnableVertexAttribArray(5);
		glVertexAttribIPointer(5, 4, GL_INT, sizeof(Point3D), (void*)offsetof(Point3D, m_BoneIDs));

		// weights
		glEnableVertexAttribArray(6);
		glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(Point3D), (void*)offsetof(Point3D, m_Weights));

    glBindVertexArray(0);

    }
};