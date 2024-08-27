#shader vertex
#version 330 core

in vec3 position;
in vec3 color;

out vec3 Color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

void main()
{
    Color = color;
    gl_Position = proj * view * model * vec4(position, 1.0);
};

#shader fragment
#version 330 core

in vec3 Color;
out vec4 color;

void main()
{
    color = vec4(Color,1.0);
};