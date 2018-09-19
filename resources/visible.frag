#version 430 core


layout(pixel_center_integer) in vec4 gl_FragCoord;

flat in int faceIdxVar;

out vec4 color;



//if the fragment is behind something in the lipschitz depth map => not visible
void main(void)
{
    color = vec4(faceIdxVar);
}
