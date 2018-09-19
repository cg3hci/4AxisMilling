#version 430 core


layout (location = 1) in vec3 position;
layout (location = 0) in int faceIdx;

uniform mat4 mvp;
//layout (std140, binding = 0) uniform MVP
//{
//    mat4 mvp;
//};

flat out int faceIdxVar;

void main(void)
{
    gl_Position = mvp * vec4(position,1.0);
    faceIdxVar = faceIdx + 1;
}
