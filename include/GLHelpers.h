#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_GLHELPERS_H
#define VISUALGROUPCOVERAGEPLANNER_GLHELPERS_H

#include "Logging.h"
#include <Eigen/Dense>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <random>
#include <sstream>

// All helper functions must be inline or static
namespace opengl_helpers
{

    // GLM to Eigen Matrix Stanislav Pidhorskyi
    template<typename T, int m, int n>
    inline glm::mat<m, n, float, glm::precision::highp> E2GLM(
        const Eigen::Matrix<T, m, n> & em)
    {
        glm::mat<m, n, float, glm::precision::highp> mat;
        for(int i = 0; i < m; ++i)
        {
            for(int j = 0; j < n; ++j)
            {
                mat[j][i] = em(i, j);
            }
        }
        return mat;
    }

    template<typename T, int m>
    inline glm::vec<m, float, glm::precision::highp> E2GLM(
        const Eigen::Matrix<T, m, 1> & em)
    {
        glm::vec<m, float, glm::precision::highp> v;
        for(int i = 0; i < m; ++i)
        {
            v[i] = em(i);
        }
        return v;
    }

    // Shader Helpers
    struct ShaderProgramSource
    {
        std::string vertexSource;
        std::string fragmentSource;
        std::string geometrySource;
    };

    static ShaderProgramSource parseShader(std::string & filepath)
    {
        std::ifstream stream(filepath);
        enum class ShaderType
        {
            NONE = -1,
            VERTEX = 0,
            FRAGMENT = 1,
            GEOMETRY = 2
        };
        std::string line;
        std::stringstream ss[3];
        ShaderType type = ShaderType::NONE;
        while(getline(stream, line))
        {
            if(line.find("#shader") != std::string::npos)
            {
                if(line.find("vertex") != std::string::npos)
                {
                    type = ShaderType::VERTEX;
                }
                else if(line.find("fragment") != std::string::npos)
                {
                    type = ShaderType::FRAGMENT;
                }
                else if(line.find("geometry") != std::string::npos)
                {
                    type = ShaderType::GEOMETRY;
                }
            }
            else
            {
                ss[(int)type] << line << '\n';
            }
        }
        return {ss[0].str(), ss[1].str(), ss[2].str()};
    }

    static unsigned int compileShader(unsigned int type, const std::string & source)
    {
        unsigned int id = glCreateShader(type);
        const char * src = source.c_str();
        glShaderSource(id, 1, &src, nullptr);
        glCompileShader(id);

        int result;
        glGetShaderiv(id, GL_COMPILE_STATUS, &result);
        if(result == GL_FALSE)
        {
            int len;
            glGetShaderiv(id, GL_INFO_LOG_LENGTH, &len);
            char * msg = (char *)alloca(len * sizeof(char));
            glGetShaderInfoLog(id, len, &len, msg);
            std::cout << "Failed to compile\n" << msg << "\n";
            glDeleteShader(id);
        }
        return id;
    }

    static unsigned int createShader(const ShaderProgramSource & shaderSources)
    {
        unsigned int program = glCreateProgram();
        unsigned int vs = compileShader(GL_VERTEX_SHADER, shaderSources.vertexSource);
        unsigned int fs = compileShader(GL_FRAGMENT_SHADER, shaderSources.fragmentSource);
        unsigned int gs = 0;
        glAttachShader(program, vs);
        glAttachShader(program, fs);
        if(!shaderSources.geometrySource.empty())
        {
            gs = compileShader(GL_GEOMETRY_SHADER, shaderSources.geometrySource);
            glAttachShader(program, gs);
        }
        glLinkProgram(program);
        glValidateProgram(program);
        glDeleteShader(vs);
        glDeleteShader(fs);
        glDeleteShader(gs);
        return program;
    }

    static void logOpenGLVersion()
    {
        const GLubyte * version = glGetString(GL_VERSION);

        if(version)
            spdlog::info("OpenGL Version: {}", reinterpret_cast<const char *>(version));
        else
            spdlog::error("Failed to get OpenGL version");
    }

} // namespace opengl_helpers
#endif // VISUALGROUPCOVERAGEPLANNER_GLHELPERS_H
