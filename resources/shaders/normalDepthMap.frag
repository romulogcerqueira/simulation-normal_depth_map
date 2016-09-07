#version 130

in vec3 pos;
in vec3 normal;
uniform float farPlane;
uniform float limitHorizontalAngle;
uniform float limitVerticalAngle;
uniform bool drawNormal;
uniform bool drawDepth;

out vec4 out_data;

void main() {
    vec4 tempInfo = vec4(0, 0, 0, 0);
    vec3 normPosition = normalize(-pos);

    float linearDepth = sqrt(pos.z * pos.z + pos.x * pos.x + pos.y * pos.y);
    linearDepth = linearDepth / farPlane;

    if (!(linearDepth > 1)) {
        if (drawNormal)
            tempInfo.zw = vec2(max(dot(normPosition, normalize(normal)), 0), 1.0);

        if (drawDepth)
            tempInfo.yw = vec2(linearDepth, 1.0);
    }

    out_data = tempInfo;
    gl_FragDepth = linearDepth;
}
