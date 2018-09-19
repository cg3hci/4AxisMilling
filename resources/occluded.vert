#version 430 core

layout (location = 1) in vec3 position;
layout (location = 0) in int faceIdx;

uniform mat4 mvp;
//layout (std140, binding = 0) uniform MVP
//{
//    mat4 mvp;
//};

flat out int faceIdxVar;
out vec4 vPosition;

void main(void)
{
    gl_Position = mvp * vec4(position,1.0);
    vPosition = gl_Position;
    faceIdxVar = (faceIdx);
}
