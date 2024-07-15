#version 330 core
// ************ 修改 ************
out vec4 FragColor;

in vec3 FragPos;  // 从顶点着色器传来的片段位置
in vec3 Normal;   // 从顶点着色器传来的法线
in vec2 TexCoords; // 从顶点着色器传来的纹理坐标

uniform sampler2D texture_diffuse1; // 纹理
uniform vec3 viewPos;               // 观察者位置
uniform vec3 lightPos;              // 光源位置
uniform vec3 lightColor;            // 光源颜色
uniform vec3 lightPos2;             // 第二个光源位置
uniform vec3 lightColor2;           // 第二个光源颜色

void main()
{
    // Phong Lighting Model
    // 环境(Ambient)、漫反射(Diffuse)和镜面(Specular)光照

    // 环境光
    float ambientStrength = 0.4f;
    vec3 ambient = ambientStrength * lightColor;

    // 漫反射光照
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // 镜面反射光照
    float specularStrength = 1.0f;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 64);
    vec3 specular = specularStrength * spec * lightColor;

    // 漫反射光照 - 第二个光源
    vec3 lightDir2 = normalize(lightPos2 - FragPos);
    float diff2 = max(dot(norm, lightDir2), 0.0);
    vec3 diffuse2 = diff2 * lightColor2;

    // 镜面反射光照 - 第二个光源
    vec3 reflectDir2 = reflect(-lightDir2, norm);  
    float spec2 = pow(max(dot(viewDir, reflectDir2), 0.0), 32);
    vec3 specular2 = specularStrength * spec2 * lightColor2;

    // 纹理颜色
    vec3 textureColor = texture(texture_diffuse1, TexCoords).rgb;

    // 结合两个光源的光照效果和纹理颜色
    vec3 result = (ambient + diffuse + specular + diffuse2 + specular2) * textureColor;
    FragColor = vec4(result, 1.0);
}
// ************ 修改 ************