#version 130

uniform sampler2D reflectionsTex;   // first and second reflections from first pass
uniform vec2 reflectionsTexSize;    // reflections texture size

void main() {
    vec4 reflections = texture2D(reflectionsTex, gl_FragCoord.xy / reflectionsTexSize);

    gl_FragData[0] = reflections;

}