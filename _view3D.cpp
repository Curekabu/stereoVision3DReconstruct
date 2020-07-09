#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Camera.h"

#define SIZE_WIDTH 1000
#define SIZE_HEIGHT 1000
#define PI 3.1415926535898f
#define SENSITY 0.001
#define KEYBOARD_SENSITY 0.0001

void framebuffer_size_callback(GLFWwindow* window, int width, int height);  //窗口回调函数
void mouse_callback(GLFWwindow* window, double xPos, double yPos);  //鼠标输入回调函数
void processinput(GLFWwindow* window);  //输入处理

const char* vertexShaderSource =
    "#version 330 core                            \n"
    "layout(location = 0) in vec3 aPos;           \n"
    "uniform mat4 viewMat;                        \n"  
    "uniform mat4 projMat;                        \n"
    "uniform mat4 modelMat;                       \n"
    "void main(){                                 \n"
    "gl_Position = projMat *    viewMat * vec4(aPos, 1.0f);} \n";

const char* fragmentShaderSource =
    "#version 330 core                            \n"
    "out vec4 FragColor;                          \n"
    "void main(){                                 \n"
    "FragColor = vec4(0.1f, 0.1f, 0.5f, 0.5f);}   \n";

float modelRotationX = 0;
float modelRotationY = 0;
float Scale = 1;

float cursorLastX = 0;
float cursorLastY = 0;
bool firstMouse = true;

//初始化Camera
extern Camera camera;

int main2(float* points, int  number) {
    //number：点的个数

    //float* points1 = new float[3000];
    ////float points[3000];
    //long i = 0;
    //for (size_t x = 0; x < 10; x++)
    //{
    //    for (size_t y = 0; y < 10; y++)
    //    {
    //        for (size_t z = 0; z < 10; z++)
    //        {
    //            points1[i * 3 + 0] = x * 0.1 - 1.1;
    //            points1[i * 3 + 1] = y * 0.1 - 1.1;
    //            points1[i * 3 + 2] = z * 0.1 - 1.1;
    //            i++;
    //        }
    //    }
    //}

    //init glfw
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(SIZE_WIDTH, SIZE_HEIGHT, "view3D", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "failed";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    //Init glew
    glewExperimental = true;
    if (glewInit() != GLEW_OK)
    {
        std::cout << "glew init failed!";
        glfwTerminate();
        return -1;
    }
    glViewport(0, 0, SIZE_WIDTH, SIZE_HEIGHT);

    //set callback  拖拽窗口大小时，窗口的绘图区跟随改变

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(window, mouse_callback);

    glPointSize(1.0f);
    //glEnable(GL_DEPTH_TEST);

    //创建并编码shader
    unsigned int fragmentShader;
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    unsigned int vertexShader;
    vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    //创建program
    unsigned int shaderProgram;
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
  
    //生成 (VAO)
    unsigned int VAO[8];
    glGenVertexArrays(8, VAO);


    //生成 (VBO)
    unsigned int VBO[8];
    glGenBuffers(8, VBO);
 

    glBindVertexArray(VAO[0]);  //绑定VAO  
    glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
    glBufferData(GL_ARRAY_BUFFER, number * 3 * sizeof(float), points, GL_STATIC_DRAW); //复制顶点数据到缓冲
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); //设置顶点属性
    glEnableVertexAttribArray(0);

    //glBindVertexArray(VAO[1]);  //绑定VAO  
    //glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
    //glBufferData(GL_ARRAY_BUFFER, 3000 * sizeof(float), points2, GL_STATIC_DRAW); //复制顶点数据到缓冲
    //delete[] points2;
    //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); //设置顶点属性
    //glEnableVertexAttribArray(0);

    unsigned int viewMatLoc = glGetUniformLocation(shaderProgram, "viewMat");
    unsigned int projMatLoc = glGetUniformLocation(shaderProgram, "projMat");


    glm::mat4 projMat = glm::perspective(glm::radians(45.0f), (float)SIZE_WIDTH / (float)SIZE_HEIGHT, 0.1f, 100.0f);

    //rander loop
    while (!glfwWindowShouldClose(window))
    {
        processinput(window);

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        //glClear(GL_COLOR_BUFFER_ BIT | GL_DEPTH_BUFFER_BIT);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);

        /*for (size_t i = 0; i < 8; i++)
        {
            glBindVertexArray(VAO[i]);
            
            glm::mat4 viewMat = camera.GetViewMatix();
            
            glUniformMatrix4fv(viewMatLoc, 1, GL_FALSE, glm::value_ptr(viewMat));
            glUniformMatrix4fv(projMatLoc, 1, GL_FALSE, glm::value_ptr(projMat));

            glDrawArrays(GL_POINTS, 0, 1000);
        }*/

        glBindVertexArray(VAO[0]);

        glm::mat4 viewMat = camera.GetViewMatix();

        glUniformMatrix4fv(viewMatLoc, 1, GL_FALSE, glm::value_ptr(viewMat));
        glUniformMatrix4fv(projMatLoc, 1, GL_FALSE, glm::value_ptr(projMat));

 /*       glDrawArrays(GL_POINTS, 0, number);*/

        glfwPollEvents();
        glfwSwapBuffers(window);
        
    }

    glDeleteVertexArrays(8, VAO);
    //glDeleteBuffers(8, VBO);

    glfwTerminate();
    return 0;
}


void processinput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {  //Y轴
        /*modelRotationY = modelRotationY - PI / 3600;
        if (modelRotationY > 2 * PI) {
            modelRotationY = modelRotationY - 2 * PI;
        }
        if (modelRotationY < -2 * PI) {
            modelRotationY = modelRotationY + 2 * PI;
        }*/

    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        /*modelRotationY = modelRotationY + PI / 3600;
        if (modelRotationY > 2 * PI) {
            modelRotationY = modelRotationY - 2 * PI;
        }
        if (modelRotationY < -2 * PI) {
            modelRotationY = modelRotationY + 2 * PI;
        }*/

    }
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) { //X轴
        /*modelRotationX = modelRotationX - PI / 3600;
        if (modelRotationX > 2 * PI) {
            modelRotationX = modelRotationX - 2 * PI;
        }
        if (modelRotationX < -2 * PI) {
            modelRotationX = modelRotationX + 2 * PI;
        }*/

    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        /*modelRotationX = modelRotationX + PI / 3600;
        if (modelRotationX > 2 * PI) {
            modelRotationX = modelRotationX - 2 * PI;
        }
        if (modelRotationX < -2 * PI) {
            modelRotationX = modelRotationX + 2 * PI;
        }*/

    }
    
    bool W = false;
    bool A = false;
    bool S = false;
    bool D = false;
    
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        S = true;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        A = true;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        D = true;
    }
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        W = true;
    }

    if (W == true)
        if (S == true)
            camera.moveZ = 0;
        else
            camera.moveZ = KEYBOARD_SENSITY;
    else
        if (S == false)
            camera.moveZ = 0;
        else
            camera.moveZ = -KEYBOARD_SENSITY;

    if (A == true)
        if (D == true)
            camera.moveX = 0;
        else
            camera.moveX = -KEYBOARD_SENSITY;
    else
        if (D == false)
            camera.moveX = 0;
        else
            camera.moveX = KEYBOARD_SENSITY;


    bool Space = false;
    bool Ctrl = false;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        Space = true;
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        Ctrl = true;
    }

    if (Space == true)
        if (Ctrl == true)
            camera.moveY = 0;
        else
            camera.moveY = KEYBOARD_SENSITY;
    else
        if (Ctrl == false)
            camera.moveY = 0;
        else
            camera.moveY = -KEYBOARD_SENSITY;

    camera.updateCameraPos();
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height){
    glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xPos, double yPos) {
    //std::cout << "xPos:" << xPos << "   yPos:" << yPos << std::endl;
    if (firstMouse == true) {
        cursorLastX = xPos;
        cursorLastY = yPos;
        firstMouse = false;
    }
    float deltaX = cursorLastX - xPos;
    float deltaY = cursorLastY - yPos;
    camera.ProcessMouseMovement(deltaX * SENSITY, deltaY * SENSITY);
    cursorLastX = xPos;
    cursorLastY = yPos;
}