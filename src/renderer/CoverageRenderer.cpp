/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/
#include "CoverageRenderer.h"

ActorVertexArray::ActorVertexArray(const Actor & actor, GLint posAttrib,
                                   GLint colorAttrib)
  : _actor(actor)
{
    // Load the actor's geometry
    vertices = actor._geometry->getVertexData();

    // Create and vertex array and set up buffers
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    // Load vertexes into buffer
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(float) * vertices.size(),
                 &vertices.front(),
                 GL_DYNAMIC_DRAW);

    // Setup vertex array format (x,y,z,r,g,b)
    glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);
    glEnableVertexAttribArray(posAttrib);

    glVertexAttribPointer(colorAttrib,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(colorAttrib);
}

// CoverageRenderer
CoverageRenderer::CoverageRenderer(World & world, bool picking)
  : _world(world)
  , windowWidth(_world.agents().begin()->_camera._imageWidth / COVERAGE_IMAGE_RESCALE)
  , windowHeight(_world.agents().begin()->_camera._imageHeight / COVERAGE_IMAGE_RESCALE)
{
    if(!picking)
    {
        _world.mutableMap().setColorMode(geometry::polygon::ColorMode::GRAYSCALE);
        for(auto & actor : _world.mutableActors())
            actor.setColorMode(geometry::polygon::ColorMode::BLUEID);
    }
    spdlog::debug("Creating Coverage Renderer");
};
CoverageRenderer::~CoverageRenderer()
{}

void CoverageRenderer::createHeadlessWindow()
{
    if(!glfwInit())
    {
        spdlog::critical("GLFW init failed");
        exit(-1);
    }
    glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
    window = glfwCreateWindow(windowWidth, windowHeight, "", NULL, NULL);
    if(!window)
    {
        glfwTerminate();
        spdlog::critical("GLFW window creation failed");
        exit(-1);
    }
    glfwMakeContextCurrent(window);
    glewInit();

    opengl_helpers::logOpenGLVersion();
}

CoverageImage CoverageRenderer::agentCoverageImage(int id, int time)
{
    const Agent & agent = _world.agents()[id];
    Pose3D agentPose = agent.getPosition(time);
    return agentCoverageImage(id, time, agentPose);
}

CoverageImage CoverageRenderer::agentCoverageImage(int id, State agentState)
{
    return agentCoverageImage(id,
                              agentState[3],
                              space::toPose3D(agentState,
                                              _world.disConfig.gridSize,
                                              _world.disConfig.heading,
                                              _world.agent(id).droneHeight()));
}
CoverageImage CoverageRenderer::agentCoverageImage(int id, int time, Pose3D agentPose)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawAgentView(id, time, agentPose);
    glFlush();
    glFinish();

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    spdlog::trace("Start Read Pixels");
    std::vector<unsigned char> pixels(windowWidth * windowHeight * 3);
    glReadPixels(
        0, 0, windowWidth, windowHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    spdlog::trace("Finish Read Pixels");
    spdlog::trace("Start Count");
    std::map<std::array<unsigned char, 3>, int> pixelsCount;
    for(int i = 0; i < windowHeight; i++)
    {
        for(int j = 0; j < windowWidth; j++)
        {
            int pixelIdx = 3 * (windowWidth * i + j);
            if((pixels[pixelIdx] + pixels[pixelIdx + 1] + pixels[pixelIdx + 2]) == 0)
                continue;
            pixelsCount[{pixels[pixelIdx], pixels[pixelIdx + 1], pixels[pixelIdx + 2]}]++;
        }
    }
    spdlog::trace("End Count");
    CoverageImage img;
    img.width = windowWidth * COVERAGE_IMAGE_RESCALE;
    img.height = windowHeight * COVERAGE_IMAGE_RESCALE;
    img.rescale = COVERAGE_IMAGE_RESCALE;
    for(auto pair : pixelsCount)
    {
        const Face & face = Face::fromByte(pair.first);
        if(face.isInvalid())
            continue;
        img.facePPA[face.id] = pair.second * COVERAGE_IMAGE_RESCALE
                               * COVERAGE_IMAGE_RESCALE / face._surfaceArea;
    }
    return img;
}

void CoverageRenderer::glInit()
{
    glEnable(GL_DEPTH_TEST);
    // load shaders
    std::string mapShaderPath = "map.shader";
    auto mapShaderData = opengl_helpers::parseShader(mapShaderPath);
    mapShaderProgram = opengl_helpers::createShader(mapShaderData);

    std::string shaderPath = "basic.shader";
    auto shaderData = opengl_helpers::parseShader(shaderPath);
    shaderProgram = opengl_helpers::createShader(shaderData);
    glUseProgram(shaderProgram);
    // Load uniforms and attributes
    uniView = glGetUniformLocation(shaderProgram, "view");
    uniProj = glGetUniformLocation(shaderProgram, "proj");
    GLint posAttrib = glGetAttribLocation(shaderProgram, "position");
    GLint colorAttrib = glGetAttribLocation(shaderProgram, "color");
    uniModel = glGetUniformLocation(shaderProgram, "model");
    // Set up actor buffers
    for(auto & actor : _world.actors())
    {
        actorVaos.push_back(ActorVertexArray(actor, posAttrib, colorAttrib));
    }

    // Set up map
    glUseProgram(mapShaderProgram);
    GLint uniGridWidthMap = glGetUniformLocation(mapShaderProgram, "gridWidth");
    GLint uniGridLengthMap = glGetUniformLocation(mapShaderProgram, "gridLength");
    glUniform1f(uniGridWidthMap, _world.map().getGridSize());
    glUniform1f(uniGridLengthMap, _world.map().getGridSize());
    uniModelMap = glGetUniformLocation(mapShaderProgram, "model");
    glm::mat4 eye = glm::mat4(1.0f);
    glUniformMatrix4fv(uniModelMap, 1, GL_FALSE, glm::value_ptr(eye));
    uniViewMap = glGetUniformLocation(mapShaderProgram, "view");
    uniProjMap = glGetUniformLocation(mapShaderProgram, "proj");

    // shader inputs
    GLint posAttribMap = glGetAttribLocation(mapShaderProgram, "position");
    GLint colorAttribMap = glGetAttribLocation(mapShaderProgram, "color");
    std::vector<float> verticesMap = _world.map().getMapDataVector();

    glGenVertexArrays(1, &vaoMap);
    glBindVertexArray(vaoMap);
    GLuint vboMap;
    glGenBuffers(1, &vboMap);
    glBindBuffer(GL_ARRAY_BUFFER, vboMap);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(float) * verticesMap.size(),
                 &verticesMap.front(),
                 GL_DYNAMIC_DRAW);

    glVertexAttribPointer(posAttribMap, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);
    glEnableVertexAttribArray(posAttribMap);

    glVertexAttribPointer(colorAttribMap,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(colorAttribMap);

    // unbind all buffers
    glUseProgram(0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void CoverageRenderer::glDrawAgentView(int id, int time, Pose3D agentPose)
{
    const Agent & agent = _world.agents()[id];
    Camera cam = agent._camera;
    Pose3D cameraPoseInWorld = agentPose * cam._agentToCameraTransform;
    Pose3D viewMat = cameraPoseInWorld.inverse();
    glm::mat4 view = opengl_helpers::E2GLM(viewMat.matrix());
    glm::mat4 proj = cam.glmProjectionMatrix();
    glUseProgram(shaderProgram);
    glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));
    for(auto & actorVao : actorVaos)
    {
        glBindVertexArray(actorVao.vao);
        glm::mat4 model =
            opengl_helpers::E2GLM(actorVao._actor.getPosition(time).matrix());
        glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(model));
        glDrawArrays(GL_TRIANGLES, 0, actorVao.vertices.size());
    }
    glUseProgram(mapShaderProgram);
    glUniformMatrix4fv(uniViewMap, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(uniProjMap, 1, GL_FALSE, glm::value_ptr(proj));
    glBindVertexArray(vaoMap);
    glDrawArrays(GL_POINTS, 0, _world.map().getElevationMap().size());
};

void CoverageRenderer::glCleanup()
{}

void CoverageRenderer::destroyHeadlessWindow()
{
    // Cleanup windows and glfw if exists
    glfwDestroyWindow(window);
    glfwTerminate();
}