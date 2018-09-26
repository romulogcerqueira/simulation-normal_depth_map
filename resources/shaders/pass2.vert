#version 130

void main() {
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    // Texture for normal mapping (irregularities surfaces)
    gl_TexCoord[0] = gl_TextureMatrix[0] * gl_MultiTexCoord0;
}
