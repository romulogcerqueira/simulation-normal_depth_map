#version 130

in vec3 pos;
in vec3 normal;
uniform float farPlane;
uniform bool drawNormal;
uniform bool drawDepth;

out vec4 out_data;

void main() {
    out_data = vec4(0, 0, 0, 0);
    vec3 normPosition = normalize(-pos);
    vec3 normNormal = normalize(normal);

    float linearDepth = sqrt(pos.z * pos.z + pos.x * pos.x + pos.y * pos.y);
    linearDepth = linearDepth / farPlane;

    if (!(linearDepth > 1)) {
        if (drawNormal)
            out_data.zw = vec2(max(dot(normPosition, normNormal), 0), 1.0);

        if (drawDepth)
            out_data.yw = vec2(linearDepth, 1.0);
    }

    gl_FragDepth = linearDepth;
}
