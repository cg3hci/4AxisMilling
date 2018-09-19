#version 430 core
#extension GL_ARB_shader_storage_buffer_object : require
flat in int faceIdxVar;
in vec4 vPosition;

layout (std430, binding = 0) coherent buffer occlusion
{
    int occlIdx[];
};

uniform sampler2D depthmap;

out vec4 color;

void main(void)
{
    vec2 vPos = (vPosition.xy / vPosition.w) * vec2(0.5) + vec2(0.5);

    ivec2 sam = ivec2(gl_FragCoord.xy - 0.5);

    if(gl_FragCoord.z > texelFetch(depthmap, sam, 0).r)
    {
        occlIdx[faceIdxVar] = 1;
        color = vec4(1.0);
    }
    else
       discard;
}
