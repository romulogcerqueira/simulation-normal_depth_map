#version 130

out vec3 pos;
out vec3 normal;

void main() {
    pos = (gl_ModelViewMatrix * gl_Vertex).xyz;

    normal = gl_NormalMatrix * gl_Normal;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
