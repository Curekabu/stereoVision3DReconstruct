#include "view3D.h"
extern std::vector<PointsArray> points;
extern std::mutex pointsMutex;
int mouseButtonLState = View3D::released;
int mouseButtonRState = View3D::released;
bool firstMousePress = false;
extern bool STOP;
extern bool PAUSE;

//初始化Camera (由于受回调函数必须为静态函数的限制， camera 初始化为全局变量。）
Camera camera = Camera(glm::vec3(0, 0, 3), glm::radians(0.0f), glm::radians(180.0f), glm::vec3(0, 1.0f, 0));


void View3D::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void View3D::mouse_pos_callback(GLFWwindow* window, double xPos, double yPos)
{
    static float cursorLastX = 0;
    static float cursorLastY = 0;
    if (mouseButtonLState == Pressed)
    {
        //std::cout << "xPos:" << xPos << "   yPos:" << yPos << std::endl;
        //刚按下鼠标时初始化Last为当前鼠标坐标
        if (firstMousePress == true) {
            cursorLastX = xPos;
            cursorLastY = yPos;
            firstMousePress = false;
        }
        float deltaX = cursorLastX - xPos;
        float deltaY = cursorLastY - yPos;
        camera.ProcessMouseMovement(-deltaX * SENSITY, -deltaY * SENSITY);
        cursorLastX = xPos;
        cursorLastY = yPos;
    }

    if (mouseButtonRState == Pressed)
    {
        if (firstMousePress == true) {
            cursorLastX = xPos;
            cursorLastY = yPos;
            firstMousePress = false;
        }
        float deltaX = cursorLastX - xPos;
        float deltaY = cursorLastY - yPos;
        camera.moveX = deltaX * MOVE_SENSITY;
        camera.moveY = -deltaY * MOVE_SENSITY;
        camera.updateCameraPos();
        cursorLastX = xPos;
        cursorLastY = yPos;
    }

    
}

void View3D::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.moveZ = yoffset * 10 * MOVE_SENSITY;
    camera.updateCameraPos();
    camera.moveZ = 0;
}

void View3D::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            mouseButtonLState = Pressed;
            firstMousePress = true;
        }
        if (action == GLFW_RELEASE)
        {
            mouseButtonLState = released;
            firstMousePress = false;
        }    
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        if (action == GLFW_PRESS)
        {
            mouseButtonRState = Pressed;       
            firstMousePress = true;
        }
        if (action == GLFW_RELEASE)
        {
            mouseButtonRState = released;
            firstMousePress = false;

        }

    }
}

void View3D::processinput(GLFWwindow* window)
{
    static bool SPACE_PERSSED = false;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
        STOP = true;
    }
        
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
    {
        SPACE_PERSSED = true;
    }

    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
    {
        if (SPACE_PERSSED)
        {
            PAUSE = PAUSE ? false : true;
            if (PAUSE)
                std::cout << "暂停重建" << std::endl;
            else
                std::cout << "开始重建" << std::endl;
            SPACE_PERSSED = false;
        }
        
    }

    /*bool W = false;
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

    camera.updateCameraPos();*/
}

int View3D::init()
{
    //init glfw
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(SIZE_WIDTH, SIZE_HEIGHT, "view3D", NULL, NULL);
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
    glfwSetCursorPosCallback(window, mouse_pos_callback);

    glfwSetMouseButtonCallback(window, mouse_button_callback);

    glfwSetScrollCallback(window, scroll_callback);

    glPointSize(2.5f);
    glEnable(GL_DEPTH_TEST);
    createShaderProgram();
}

void View3D::createShaderProgram()
{
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
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glUseProgram(shaderProgram);

    viewMatLoc = glGetUniformLocation(shaderProgram, "viewMat");
    projMatLoc = glGetUniformLocation(shaderProgram, "projMat");
    //modelMatLoc = glGetUniformLocation(shaderProgram, "modelMat");

    projMat = glm::perspective(glm::radians(45.0f), (float)SIZE_WIDTH / (float)SIZE_HEIGHT, 0.1f, 100.0f);

}

void View3D::initVAO()
{
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    //设置顶点位置属性
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    //设置顶点颜色属性
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(0);

}

void View3D::processEvents()
{

    while (!glfwWindowShouldClose(window))
    {
        processinput(window);
        glfwPollEvents();
    }
    glfwTerminate();
}

void View3D::vertexProcess()
{
    pointsMutex.lock();
    if (points.empty()) {
        pointsMutex.unlock();
        return;
    }
    else {
        PointsArray _points;
        VBO _VBO;     //临时变量存储新申请的VBO
        unsigned int _VAO;
        glGenBuffers(1, &_VBO.vbo);//申请VBO
        _points = points.back();
        points.pop_back();
        pointsMutex.unlock();
        _VBO.quantity = _points.quantity;
        glGenVertexArrays(1, &_VAO);
        glBindVertexArray(_VAO);
        _VBO.vao = _VAO;
        vbo.push_back(_VBO);
        glBindBuffer(GL_ARRAY_BUFFER, _VBO.vbo);//绑定刚pushback的VBO
        glBufferData(GL_ARRAY_BUFFER, _points.quantity * 6 * sizeof(float), _points.data.data(), GL_STATIC_DRAW); //复制一组顶点数据到VBO
        
        //设置顶点位置属性
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        //设置顶点颜色属性
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1); 
        glBindVertexArray(0);
        
    }
    
}

void View3D::addPoints(std::vector<Point3D> P)
{

}




void View3D::draw()
{
    while (!glfwWindowShouldClose(window))
    {
        vertexProcess();
        processinput(window);
        glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //glClear(GL_COLOR_BUFFER_BIT);

        glm::mat4 viewMat = camera.GetViewMatix();

        glUniformMatrix4fv(viewMatLoc, 1, GL_FALSE, glm::value_ptr(viewMat));
        glUniformMatrix4fv(projMatLoc, 1, GL_FALSE, glm::value_ptr(projMat));

        for (std::vector<VBO>::iterator i = vbo.begin(); i != vbo.end(); ++i)
        {
            glBindVertexArray(i->vao);
            glDrawArrays(GL_POINTS, 0, i->quantity);
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    return;
}
