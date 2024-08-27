/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/
#include "BEVRenderer.h"

BEVRenderer::BEVRenderer(World & world, int windowWidth, int windowHeight)
  : _world(world)
{
    spdlog::debug("Creating BEV Renderer ({}x{})", windowWidth, windowHeight);
    // Rescale BEV to window width
    _worldToPixels = (float)windowWidth / world.map().getMapSize().col;
    spdlog::debug("BEV Map Pixel Rescale: {}", _worldToPixels);

    // Create Blend2D image and context
    img = BLImage(windowWidth, windowHeight, BL_FORMAT_PRGB32);
    ctx = BLContext(img);

    clearImage();
}

void BEVRenderer::drawLine(Vector2d start, Vector2d end, BLRgba32 color,
                           double strokeWidth)
{
    // convert from world coordinates to map coordinates
    Vector2d worldStart = start * _worldToPixels;
    Vector2d worldEnd = end * _worldToPixels;

    // draw line on screen
    BLLine line(worldStart.x(), worldStart.y(), worldEnd.x(), worldEnd.y());
    ctx.setStrokeWidth(strokeWidth);
    ctx.setStrokeStyle(color); // red color
    ctx.strokeLine(line);
}

void BEVRenderer::drawCameraView(const Camera & camera, Pose3D agentPose)
{
    Matrix3d p;
    // clang-format off
    p << 0, 1,-1, 
         0, 1, 1,
         1, 1, 1;
    // clang-format on
    p = agentPose * p;
    p *= _worldToPixels;
    BLPath path;
    path.moveTo(p(0, 0), p(1, 0));
    path.lineTo(p(0, 1), p(1, 1));
    path.lineTo(p(0, 2), p(1, 2));
    path.lineTo(p(0, 0), p(1, 0));
    ctx.setStrokeStyle(BLRgba32(50, 50, 50, 128));
    ctx.setFillStyle(BLRgba32(50, 50, 50, 40));

    ctx.strokePath(path);
    ctx.fillPath(path);
    // corners of the image to be ray cast to the ground plane
    // std::vector<Vector2i> imageCorners = {{0, 0},
    //                                       {0, camera._imageHeight},
    //                                       {camera._imageWidth, camera._imageHeight},
    //                                       {camera._imageWidth, 0}};

    // // compute the points in the ground plane which intersect with the rays of the
    // // corners
    // std::vector<Point2D> imageCornerRayWorldIntersectionPoints;
    // for(auto corner : imageCorners)
    // {
    //     auto ray = camera.pixelToWorldRay3D(corner, agentPose);
    //     // std::cout << ray << "\n";
    //     auto maybe_point = rayPlaneIntersect(ray, xyPlane());
    //     if(!maybe_point.has_value())
    //     {
    //         spdlog::warn("camera ray does not intersect with plane");
    //         return;
    //     }
    //     auto point = maybe_point.value().head<2>();
    //     drawLine(ray.point.head<2>(), point, BLRgba32(0, 0, 0, 128), 1);
    //     imageCornerRayWorldIntersectionPoints.push_back(point * _worldToPixels);
    // }
    // // draw rays and visual region
    // BLPath path;
    // auto points = imageCornerRayWorldIntersectionPoints;
    // path.moveTo(points[0].x(), points[0].y());
    // for(size_t i = 1; i < points.size(); ++i)
    //     path.lineTo(points[i].x(), points[i].y());
    // path.close();

    // ctx.setStrokeStyle(BLRgba32(50, 50, 50, 128));
    // ctx.setFillStyle(BLRgba32(50, 50, 50, 40));

    // ctx.strokePath(path);
    // ctx.fillPath(path);
}

void BEVRenderer::drawAgent(const Agent & agent, int time)
{
    // Draw agent trajectory
    for(int i = 1; i <= time; i++)
    {
        int g = ((float)i / (float)time) * 255;
        int b = (1 - (float)i / (float)time) * 255;
        drawLine(agent._trajectory[i - 1].translation().head<2>(),
                 agent._trajectory[i].translation().head<2>(),
                 BLRgba32(0, g, b, 128));
    }

    Pose3D agentPose = agent.getPosition(time);
    // drone X frame points
    Matrix3Xd droneFrame(3, 4);
    droneFrame << -0.35, 0.35, -0.35, 0.35, -0.35, 0.35, 0.35, -0.35, 1, 1, 1, 1;
    droneFrame = pose2d::fromPose3DXY(agentPose) * droneFrame;
    drawLine(droneFrame.col(0).head<2>(), droneFrame.col(1).head<2>());
    drawLine(droneFrame.col(2).head<2>(), droneFrame.col(3).head<2>());

    // draw propeller circles
    double dronePropRadius = 0.15;
    for(int i = 0; i < droneFrame.cols(); ++i)
    {
        Eigen::VectorXd column = droneFrame.col(i).head<2>();
        BLCircle circle(column.x() * _worldToPixels,
                        column.y() * _worldToPixels,
                        dronePropRadius * _worldToPixels);
        ctx.setStrokeStyle(BLRgba32(0, 0, 0, 128));
        ctx.setStrokeWidth(2);
        ctx.strokeCircle(circle);
    }
    drawCameraView(agent._camera, agentPose);
}

void BEVRenderer::drawActor(const Actor & actor, int time)
{
    auto rgb = geometry::polygon::hsv2rgb(
        geometry::polygon::hsv(actor._id == 0? 0 : 250, 0.83, 0.7));
    // std::cout << rgb.r << " " << rgb.g << " " <<rgb.b << " " << "\n";
    for(int i = 0; i < actor._trajectory.size() - 1; i++)
        drawLine(actor._trajectory[i].translation().head<2>(),
                 actor._trajectory[i + 1].translation().head<2>(),
                 BLRgba32(rgb.r * 255, rgb.g * 255, rgb.b * 255, 128));
    //  BLRgba32((256 / _world.actors().size()) * actor._id, 0, 0, 128));
    // draw axis in the actor center
    auto actorPose = actor.getPosition(time);
    Matrix3Xd actorFrame(3, 4);
    // actorFrame.setOnes();
    // actorFrame.block<2, 4>(0, 0) << actor._geometry->get2DPolygon().vertices;
    actorFrame << 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1;
    actorFrame = pose2d::fromPose3DXY(actorPose) * actorFrame;
    drawLine(actorFrame.col(0).head<2>(),
             actorFrame.col(1).head<2>(),
             BLRgba32(255, 0, 0, 255));
    drawLine(actorFrame.col(2).head<2>(),
             actorFrame.col(3).head<2>(),
             BLRgba32(0, 255, 0, 255));
    // draw a small circle in the center of the actor
    BLCircle circle(
        actorPose(0, 3) * _worldToPixels, actorPose(1, 3) * _worldToPixels, 5);
    ctx.setFillStyle(BLRgba32(0, 0, 255, 128));
    ctx.fillCircle(circle);

    // draw actor 2d geometry
    Pose2D pose = pose2d::fromPose3DXY(actorPose);
    auto shape = actor._geometry->get2DPolygon();
    auto edges = shape.edges(pose);
    for(auto edge : edges)
    {
        drawLine(edge.start, edge.end);
    }
}

void BEVRenderer::drawMap()
{
    float height = _world.map()._maxHeight;
    auto map = _world.map().getElevationMap();
    float gridSize = _world.map().getGridSize() * _worldToPixels;
    // clang-format off
    Matrix<double, 2, 4> grid;
    grid << 0, gridSize, gridSize, 0,
            0, 0       , gridSize, gridSize;
    // clang-format on
    for(int i = 0; i < map.rows(); i++)
    {
        for(int j = 0; j < map.cols(); j++)
        {
            Matrix<double, 2, 4> gridT = grid;
            gridT.colwise() += Vector2d{i * gridSize, j * gridSize};
            BLPath path;
            path.moveTo(gridT(0, 0), gridT(1, 0));
            for(size_t i = 1; i < gridT.cols(); ++i)
                path.lineTo(gridT(0, i), gridT(1, i));
            path.close();
            int c = (1 - map(i, j) / height) * 255;
            ctx.setStrokeStyle(BLRgba32(50, 50, 50, 20));
            ctx.setStrokeWidth(0.5);
            ctx.strokePath(path);
            ctx.setFillStyle(BLRgba32(c, c, c, 50));
            ctx.fillPath(path);
        }
    }
}

void BEVRenderer::clearImage()
{
    // Set background to white
    ctx.setCompOp(BL_COMP_OP_SRC_COPY);
    ctx.setFillStyle(BLRgba32(0xFFFFFFFF));
    ctx.fillAll();

    // Set composition to source over
    ctx.setCompOp(BL_COMP_OP_SRC_OVER);
}

void BEVRenderer::glInit()
{
    // Load texture shader
    std::string texShaderPath = "texture.shader";
    auto texShaderData = opengl_helpers::parseShader(texShaderPath);
    _shaderProgram = opengl_helpers::createShader(texShaderData);
    glUseProgram(_shaderProgram);

    GLint posAttrib = glGetAttribLocation(_shaderProgram, "position");
    GLint texAttrib = glGetAttribLocation(_shaderProgram, "texcoord");

    // Rectangle texture vertices
    // clang-format off
    std::vector<float> vertices = {
    //   Pos    Texcoords
         -1, -1, 0.0f, 0.0f, // Top-left
          1, -1, 1.0f, 0.0f, // Top-right
          1,  1, 1.0f, 1.0f, // Bottom-right
         -1,  1, 0.0f, 1.0f  // Bottom-left
    };
    // clang-format on

    // Create vertex array and buffers
    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);
    glGenBuffers(1, &_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);

    // Load display vertexes into buffer
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(float) * vertices.size(),
                 &vertices.front(),
                 GL_STATIC_DRAW);
    // Set up vertex array layout
    glVertexAttribPointer(posAttrib, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glEnableVertexAttribArray(posAttrib);

    glVertexAttribPointer(
        texAttrib, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(texAttrib);

    // Set up element buffer to draw texture
    glGenBuffers(1, &_ebo);
    GLuint elements[] = {0, 1, 2, 2, 3, 0};
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(elements), elements, GL_STATIC_DRAW);

    // Create and init texture
    glGenTextures(1, &_tex);
    glBindTexture(GL_TEXTURE_2D, _tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    // unbind all objects
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glUseProgram(0);
}

void BEVRenderer::draw(int time, bool cache)
{
    // Only draw if new time
    if(cache && time != _previousDrawTime)
    {
        _previousDrawTime = time;
        return;
    }

    clearImage();

    // Draw world elements
    drawMap();

    for(const auto & agent : _world.agents())
        drawAgent(agent, time);

    for(const auto & actor : _world.actors())
        drawActor(actor, time);
}

void BEVRenderer::glDraw(int time)
{
    // Draw world to current image
    draw(time);

    // Load image data to bytes
    imgData = new BLImageData();
    img.getData(imgData);

    // Setup and bind texture
    glUseProgram(_shaderProgram);
    glBindVertexArray(_vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glBindTexture(GL_TEXTURE_2D, _tex);

    // Load the Blend2D image data into the OpenGL texture
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGBA,
                 img.width(),
                 img.height(),
                 0,
                 GL_BGRA,
                 GL_UNSIGNED_BYTE,
                 imgData->pixelData);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // Unbind objects
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}
void BEVRenderer::saveImage(int time, std::string name)
{
    draw(time, false);
    // TODO name image using world time
    img.writeToFile(name.c_str());
}

void BEVRenderer::glCleanup()
{
    // delete all opengl elements and image data
    glDeleteProgram(_shaderProgram);
    glDeleteVertexArrays(1, &_vao);
    glDeleteTextures(1, &_tex);
    glDeleteBuffers(1, &_ebo);
    delete imgData;
}