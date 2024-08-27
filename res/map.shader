#shader vertex
#version 330 core

in vec3 position;
in vec3 color;

out vec3 vColor;
void main()
{
    vColor = color;
    gl_Position = vec4(position, 1.0);
};

#shader fragment
#version 330 core

in vec3 fColor;

out vec4 color;

void main()
{
    color = vec4(fColor, 1.0);
};

#shader geometry
#version 330 core

layout(points) in;
layout(triangle_strip, max_vertices = 14) out;

in vec3 vColor[];
out vec3 fColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
uniform float gridWidth;
uniform float gridLength;

void main()
{
    fColor = vColor[0];
    // point is the top back corner
    float height = gl_in[0].gl_Position[2];
    gl_Position = proj * view * model * gl_in[0].gl_Position;                                                 // 0
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4( gridWidth, 0.0        , 0.0, 0.0));     // 1
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4(0.0       ,  gridLength, 0.0, 0.0));     // 3
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4( gridWidth,  gridLength, 0.0, 0.0));     // 2
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4( gridWidth,  gridLength, -height, 0.0)); // 6
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4( gridWidth, 0.0        , 0.0, 0.0));     // 1
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4( gridWidth, 0.0        , -height, 0.0)); // 5
    EmitVertex();
    gl_Position = proj * view * model * gl_in[0].gl_Position;                                                 // 0
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4( 0.0,        0.0,        -height, 0.0)); // 4
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4(0.0       ,  gridLength, 0.0, 0.0));     // 3
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4(0.0       ,  gridLength, -height, 0.0)); // 7
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4( gridWidth,  gridLength, -height, 0.0)); // 6
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4( 0.0,        0.0,        -height, 0.0)); // 4
    EmitVertex();
    gl_Position = proj * view * model * (gl_in[0].gl_Position + vec4( gridWidth, 0.0        , -height, 0.0)); // 5
    EmitVertex();
    EndPrimitive();
}