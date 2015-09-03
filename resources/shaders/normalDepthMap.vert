#version 130

out vec3 pos;
out vec3 normal;
uniform float farPlane;
uniform float limitHorizontalAngle;
uniform float limitVerticalAngle;
uniform bool drawNormal;
uniform bool drawDepth;

void main() {
    vec4 temp_pos = gl_ModelViewMatrix * gl_Vertex;
    pos = temp_pos.xyz;

    normal = gl_NormalMatrix * gl_Normal;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
