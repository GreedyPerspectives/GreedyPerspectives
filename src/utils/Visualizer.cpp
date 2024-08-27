/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "Visualizer.h"

// Callback to quickly close visualizer window
static void keyCallback(GLFWwindow * window, int key, int scancode, int action, int mods)
{
    if(key == GLFW_KEY_Q && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE); // Close the window
}

// Callback to display any opengl errors
static void GLAPIENTRY messageCallback(GLenum source, GLenum type, GLuint id,
                                       GLenum severity, GLsizei length,
                                       const GLchar * message, const void * userParam)
{
    spdlog::error("[OpenGL Error]({}) {}", type, message);
}

Visualizer::Visualizer(World & world)
  : _world(world)
  , _bev(BEVRenderer(world, WINDOW_WIDTH / 2, WINDOW_HEIGHT))
  , _coverage(CoverageRenderer(world, false))
{
    // Initialize GLFW, GLEW and create window
    if(!glfwInit())
    {
        spdlog::critical("GLFW init failed");
        exit(-1);
    }
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "", NULL, NULL);
    if(!window)
    {
        glfwTerminate();
        spdlog::critical("GLFW window creation failed");
        exit(-1);
    }
    glfwSetKeyCallback(window, keyCallback);
    glfwMakeContextCurrent(window);
    glewInit();

    // Initialize Imgui
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();
    ImGui::StyleColorsDark();

    opengl_helpers::logOpenGLVersion();

    // Set opengl error callback
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(messageCallback, 0);
}

Visualizer::~Visualizer()
{
    // ImGui cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    // Close window
    glfwDestroyWindow(window);
    glfwTerminate();
}

void Visualizer::initializeRenderers()
{
    _coverage.glInit();
    _bev.glInit();
    _bev.saveImage(0);
}

void Visualizer::cleanupRenderers()
{
    _bev.glCleanup();
    _coverage.glCleanup();
}
void Visualizer::mouseCallback(GLFWwindow* window, int button, int action, int mods) {
    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) 
    {
       double xpos, ypos;
       //getting cursor position
       glfwGetCursorPos(window, &xpos, &ypos);
       std::cout << (int)(xpos/w2P) << "," << (int)((WINDOW_HEIGHT-ypos)/w2P) << std::endl;
    }
}
float Visualizer::w2P = 0;
void Visualizer::displayLoop()
{
    Visualizer::w2P = _bev._worldToPixels;
    // Set up window frame
    float aspectRatio = (float)_coverage.windowHeight / (float)_coverage.windowWidth;
    int viewWidth = WINDOW_WIDTH / 4;
    int viewHeight = aspectRatio * viewWidth;

    spdlog::debug("Camera View Dimensions: {}x{}", viewWidth, viewHeight);
    int time = 0;
    auto t_start = std::chrono::high_resolution_clock::now();
    bool start = false;
    // glfwSetMouseButtonCallback(window, mouseCallback);
    while(!glfwWindowShouldClose(window))
    {
        // Timing increment on start/stop
        if(start)
        {
            auto t_now = std::chrono::high_resolution_clock::now();
            float currentTime =
                std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start)
                    .count();
            if(currentTime >= _world.timeInfo().timeStepLength)
            {
                t_start = t_now;
                if((time + 1) == _world.timeInfo().stepCount)
                    start = false;
                else
                    time = (time + 1) % _world.timeInfo().stepCount;
            }
        }

        // Clear and start new gl frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render BEV image
        glViewport(0, 0, WINDOW_WIDTH / 2, WINDOW_HEIGHT);
        _bev.glDraw(time);

        // Render each agent's view on the right side of the screen
        for(int i = 0; i < _world.agents().size(); i++)
        {
            glViewport(WINDOW_WIDTH / 2 + (i>1 ? i % 2 : 1 - i % 2) * viewWidth,
                       (i / 2) * viewHeight,
                       viewWidth,
                       viewHeight);
            _coverage.glDrawAgentView(i, time, _world.agent(i).getPosition(time));
        }

        // ImGui panel
        if(ImGui::Button("Start"))
        {

            time = 0;
            start = true;
        }
        if(ImGui::Button("Stop"))
            start = false;
        if(ImGui::Button("Save"))
            _bev.saveImage(time);
        ImGui::SliderInt("Time: ", &time, 0, _world.timeInfo().stepCount - 1);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Display to screen
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}
