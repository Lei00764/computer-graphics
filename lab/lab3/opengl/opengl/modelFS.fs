#version 330 core
// ************ �޸� ************
out vec4 FragColor;

in vec3 FragPos;  // �Ӷ�����ɫ��������Ƭ��λ��
in vec3 Normal;   // �Ӷ�����ɫ�������ķ���
in vec2 TexCoords; // �Ӷ�����ɫ����������������

uniform sampler2D texture_diffuse1; // ����
uniform vec3 viewPos;               // �۲���λ��
uniform vec3 lightPos;              // ��Դλ��
uniform vec3 lightColor;            // ��Դ��ɫ
uniform vec3 lightPos2;             // �ڶ�����Դλ��
uniform vec3 lightColor2;           // �ڶ�����Դ��ɫ

void main()
{
    // Phong Lighting Model
    // ����(Ambient)��������(Diffuse)�;���(Specular)����

    // ������
    float ambientStrength = 0.4f;
    vec3 ambient = ambientStrength * lightColor;

    // ���������
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // ���淴�����
    float specularStrength = 1.0f;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 64);
    vec3 specular = specularStrength * spec * lightColor;

    // ��������� - �ڶ�����Դ
    vec3 lightDir2 = normalize(lightPos2 - FragPos);
    float diff2 = max(dot(norm, lightDir2), 0.0);
    vec3 diffuse2 = diff2 * lightColor2;

    // ���淴����� - �ڶ�����Դ
    vec3 reflectDir2 = reflect(-lightDir2, norm);  
    float spec2 = pow(max(dot(viewDir, reflectDir2), 0.0), 32);
    vec3 specular2 = specularStrength * spec2 * lightColor2;

    // ������ɫ
    vec3 textureColor = texture(texture_diffuse1, TexCoords).rgb;

    // ���������Դ�Ĺ���Ч����������ɫ
    vec3 result = (ambient + diffuse + specular + diffuse2 + specular2) * textureColor;
    FragColor = vec4(result, 1.0);
}
// ************ �޸� ************