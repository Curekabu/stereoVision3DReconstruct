#pragma once
#include <iostream>
#include <vector>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Camera.h"
#include "PointsArray.h"
#include <mutex>

#define SIZE_WIDTH 1280
#define SIZE_HEIGHT 720
#define PI 3.1415926535898f
#define SENSITY 0.001
#define MOVE_SENSITY 0.005

struct VBO
{
    unsigned int vbo;
    unsigned int quantity;
    unsigned int vao;
    VBO():vbo(0),quantity(0),vao(0){}
};

class View3D
{
public:
    View3D() :
        vertexShaderSource(
            "#version 330 core                                   \n"
            "layout(location = 0) in vec3 aPos;                  \n"
            "layout(location = 1) in vec3 inColor;               \n"
            "out vec3 Color;                                     \n"
            "uniform mat4 viewMat;                               \n"
            "uniform mat4 projMat;                               \n"
            "uniform mat4 modelMat;                              \n"
            "void main(){                                        \n"
            "gl_Position = projMat * viewMat * vec4(aPos, 1.0f); \n"
            "Color = inColor;}                                   \n"),
        fragmentShaderSource(                                               
            "#version 330 core                                   \n"
            "out vec4 FragColor;                                 \n"
            "in vec3 Color;                                      \n"
            "void main(){                                        \n"
            "FragColor = vec4(Color, 1.0f);}                     \n") 
    {
        init();
    }

    void processEvents();


    //���㴦����
    void vertexProcess();

    void addPoints(std::vector<Point3D> P);

	//��Ⱦ����
    void draw();

    enum buttonState
    {
        Pressed,
        released
    };

private:
    //���ڻص�����
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);

    //�������ص�����
    static void mouse_pos_callback(GLFWwindow* window, double xPos, double yPos);

    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);

    //���벢������ɫ������
    void createShaderProgram();

    //glew �� glfw �ĳ�ʼ�� �������ûص�����
    int init();

    //��VAO�������ǻ��Ƶ�������ֻ��Ҫ��һ�� 
    void initVAO();

    //���봦����
    void processinput(GLFWwindow* window);

    const char* vertexShaderSource;
    const char* fragmentShaderSource;
    unsigned int VAO;
    GLFWwindow* window;
    unsigned int shaderProgram;
    unsigned int viewMatLoc;
    unsigned int projMatLoc;
    unsigned int modelMatLoc;
    glm::mat4 projMat;
    
    std::vector<VBO> vbo;
    
    
    
    
};

