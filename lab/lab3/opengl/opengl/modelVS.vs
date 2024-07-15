#version 330 core
// ************ 修改 ************
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out vec3 FragPos;   // 片段的世界空间位置
out vec3 Normal;    // 片段的法线
out vec2 TexCoords; // 纹理坐标（将纹理坐标传递给片段着色器）

uniform mat4 model;       // 模型矩阵
uniform mat4 view;        // 视图矩阵
uniform mat4 projection;  // 投影矩阵

void main()
{
    FragPos = vec3(model * vec4(aPos, 1.0)); // 将顶点位置转换到世界空间
    Normal = mat3(transpose(inverse(model))) * aNormal; // 转换法线到世界空间
    TexCoords = aTexCoords;  // 传递纹理坐标
    gl_Position = projection * view * vec4(FragPos, 1.0);  // 计算顶点的裁剪空间位置
}
// ************ 修改 ************
