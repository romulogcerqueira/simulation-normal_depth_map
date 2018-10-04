#version 130

uniform mat4 osg_ViewMatrixInverse;

attribute vec3 tangent;
attribute vec3 binormal;
attribute vec3 normal;

out vec3 worldPos;
out vec3 worldNormal;
out vec3 cameraPos;
out mat3 TBN;

void main() {

    // World space * View matrix = Camera (eye) space
    // Camera (eye) space * Projection matrix = Screen space
    // We simply translate vertex from Model space to Screen space
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    // Pass the texture coordinate further to the fragment shader
    gl_TexCoord[0] = gl_MultiTexCoord0;

    // position in world space
    mat4 modelWorld = osg_ViewMatrixInverse * gl_ModelViewMatrix;
    worldPos = (modelWorld * gl_Vertex).xyz;

    // normal in world space
    worldNormal = mat3(modelWorld) * gl_Normal;

    // camera position in world space
    cameraPos = osg_ViewMatrixInverse[3].xyz / osg_ViewMatrixInverse[3].w;

    // Normal maps are built in tangent space, interpolating the vertex normal and a RGB texture.
    // TBN is the conversion matrix between Tangent Space -> World Space.
    vec3 N = worldNormal;
    vec3 T = mat3(modelWorld) * gl_MultiTexCoord0.xyz;
    vec3 B = cross(N, T);
    TBN = transpose(mat3(T, B, N));
}