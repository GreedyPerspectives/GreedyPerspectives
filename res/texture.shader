#shader vertex
#version 330 core

in vec2 position;
in vec2 texcoord;

out vec2 Texcoord;

void main()
{
    Texcoord = texcoord;
    gl_Position = vec4(position, 0.0, 1.0);
};

#shader fragment
#version 330 core

in vec2 Texcoord;

out vec4 outColor;

uniform sampler2D tex;

void main()
{
    outColor = texture(tex, Texcoord);
}