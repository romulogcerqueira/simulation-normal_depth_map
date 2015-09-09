#version 130

in vec3 pos;
in vec3 normal;
uniform float farPlane;
uniform float limitHorizontalAngle;
uniform float limitVerticalAngle;
uniform bool drawNormal;
uniform bool drawDepth;

void main() {
    vec4 tempInfo = vec4(0, 0, 0, 0);

    vec3 normPosition = normalize(-pos);
    float angleX = abs(normPosition.x / normPosition.z) / limitHorizontalAngle;
    float angleY = abs(normPosition.y / normPosition.z) / limitVerticalAngle;

    float linearDepth = sqrt(pos.z * pos.z + pos.x * pos.x + pos.y * pos.y);
    linearDepth = linearDepth / farPlane;

    if (!(angleX > 1 || angleY > 1 || linearDepth > 1)) {
        if (drawNormal)
            tempInfo.zw = vec2(max(dot(normPosition, normalize(normal)), 0), 1.0);

        if (drawDepth)
            tempInfo.xyw = vec3(angleX, linearDepth, 1.0);
    }

    gl_FragData[0] = tempInfo;
}

