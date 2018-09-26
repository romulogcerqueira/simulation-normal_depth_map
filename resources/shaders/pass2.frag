#version 130

uniform sampler2D firstReflectionTex;   // first reflections by rasterization
uniform vec2 rttTexSize;                // texture size of RTTs

void main() {
    // primary reflections by rasterization
    vec4 primaryRefl = texture2D(firstReflectionTex, gl_FragCoord.xy / rttTexSize);

    gl_FragData[0] = primaryRefl;

}