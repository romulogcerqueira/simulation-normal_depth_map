#version 130

uniform mat4 osg_ViewMatrixInverse;

out vec3 worldPos;
out vec3 worldNormal;
out vec3 cameraPos;
out mat3 TBN;

void main() {
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    // position/normal in world space
    mat4 modelWorld = osg_ViewMatrixInverse * gl_ModelViewMatrix;
    worldPos = vec3(modelWorld * gl_Vertex);
    worldNormal = mat3(modelWorld) * gl_Normal;

    // camera position in world space
    cameraPos = osg_ViewMatrixInverse[3].xyz / osg_ViewMatrixInverse[3].w;

    // Normal maps are built in tangent space, interpolating the vertex normal and a RGB texture.
    // TBN is the conversion matrix between Tangent Space -> World Space.
    vec3 n = normalize(worldNormal);
    vec3 t = cross(n, vec3(1,0,0));
    vec3 b = cross(t, n);
    TBN = mat3(t, b, n);

    // Texture for normal mapping (irregularities surfaces)
    gl_TexCoord[0] = gl_TextureMatrix[0] * gl_MultiTexCoord0;
}