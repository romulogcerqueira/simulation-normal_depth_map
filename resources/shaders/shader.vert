#version 130

uniform mat4 osg_ViewMatrixInverse;

out vec3 worldPos;
out vec3 worldNormal;
out vec3 cameraPos;
out vec3 viewPos;
out vec3 viewNormal;
out mat3 TBN;

void main() {

    // World space * View matrix = Camera (eye) space
    // Camera (eye) space * Projection matrix = Screen space
    // We simply translate vertex from Model space to Screen space
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    // Pass the texture coordinate further to the fragment shader
    gl_TexCoord[0] = gl_MultiTexCoord0;

    // position and normal in world space
    mat4 modelWorld = osg_ViewMatrixInverse * gl_ModelViewMatrix;
    worldPos = (modelWorld * gl_Vertex).xyz;
    worldNormal = mat3(modelWorld) * gl_Normal;

    // position and normal in view space
    viewPos = (gl_ModelViewMatrix * gl_Vertex).xyz;
    viewNormal = gl_NormalMatrix * gl_Normal;

    // camera position in world space
    cameraPos = osg_ViewMatrixInverse[3].xyz / osg_ViewMatrixInverse[3].w;

    // Normal maps are built in tangent space, interpolating the vertex normal and a RGB texture.
    // TBN is the conversion matrix between Tangent Space -> World Space.
    vec3 N = normalize(viewNormal);
    vec3 T = cross(N, vec3(-1, 0, 0));
    vec3 B = cross(N, T) + cross(T, N);
    TBN = mat3(T, B, N);
}