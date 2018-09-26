#version 130

in vec3 worldPos;
in vec3 worldNormal;
in vec3 cameraPos;
in mat3 TBN;

uniform float farPlane;
uniform bool drawNormal;
uniform bool drawDistance;
uniform float reflectance;
uniform float attenuationCoeff;

uniform sampler2D normalTex;
uniform bool useNormalTex;

uniform sampler2D trianglesTex;         // all triangles/meshes collected from the simulated scene
uniform vec2 trianglesTexSize;          // texture size of triangles

// ray definition
struct Ray {
    vec3 origin, direction;
};

// triangle definition
struct Triangle {
    vec3 v0;        // vertex A
    vec3 v1;        // vertex B
    vec3 v2;        // vertex C
    vec3 center;    // centroid
    vec3 normal;    // normal
};

float getTexData(int i, int j) {
    return texelFetch(trianglesTex, ivec2(i,j), 0).r;
}

Triangle getTriangleData(int idx) {
    Triangle triangle;
    triangle.v0     = vec3(getTexData(idx,0), getTexData(idx,1), getTexData(idx,2));
    triangle.v1     = vec3(getTexData(idx,3), getTexData(idx,4), getTexData(idx,5));
    triangle.v2     = vec3(getTexData(idx,6), getTexData(idx,7), getTexData(idx,8));
    triangle.center = vec3(getTexData(idx,9), getTexData(idx,10), getTexData(idx,11));
    triangle.normal = vec3(getTexData(idx,12), getTexData(idx,13), getTexData(idx,14));
    return triangle;
}

// Möller–Trumbore ray-triangle intersection algorithm
// source: http://bit.ly/2MxnPMG
bool rayIntersectsTriangle(Ray ray, Triangle triangle)
{
    float EPSILON = 0.0000001;

    vec3 edge1, edge2, h, s, q;
    float a, f, u, v;
    edge1 = triangle.v1 - triangle.v0;
    edge2 = triangle.v2 - triangle.v0;

    h = cross(ray.direction, edge2);
    a = dot(edge1, h);
    if (a > -EPSILON && a < EPSILON)
        return false;

    f = 1 / a;
    s = ray.origin - triangle.v0;
    u = f * dot(s, h);
    if (u < 0.0 || u > 1.0)
        return false;

    q = cross(s, edge1);
    v = f * dot(ray.direction, q);
    if (v < 0.0 || (u + v) > 1.0)
        return false;

    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * dot(edge2, q);
    if (t <= EPSILON)   // this means that there is a line intersection but not a ray intersection.
        return false;

    return true;        // ray intersection
}

// ============================================================================================================================

// primary reflections: rasterization
vec4 primaryReflections() {
    vec3 worldIncident = cameraPos - worldPos;
    vec3 nWorldPos = normalize(worldIncident);
    vec3 nWorldNormal = normalize(worldNormal);

    // normal for textured scenes (by normal mapping)
    if (useNormalTex) {
        vec3 normalRGB = texture2D(normalTex, gl_TexCoord[0].xy).rgb;
        vec3 normalMap = (normalRGB * 2.0 - 1.0) * TBN;
        nWorldNormal = normalize(normalMap);
    }

    // material's reflectivity property
    if (reflectance > 0)
        nWorldNormal = min(nWorldNormal * reflectance, 1.0);

    // distance calculation
    float viewDistance = length(worldIncident);

    // attenuation effect of sound in the water
    nWorldNormal = nWorldNormal * exp(-2 * attenuationCoeff * viewDistance);

    // normalize distance using range value (farPlane)
    float nViewDistance = viewDistance / farPlane;

    // presents the normal and depth data as matrix
    vec4 output = vec4(0, 0, 0, 1);
    if (nViewDistance <= 1) {
        if (drawDistance)   output.y = nViewDistance;
        if (drawNormal)     output.z = abs(dot(nWorldPos, nWorldNormal));
    }

    return output;
}

// ============================================================================================================================

// secondary reflections: ray-triangle intersection
vec4 secondaryReflections(vec4 firstR) {

    // calculate the reflection direction for an incident vector
    // TODO: use the nWorldNormal after first reflection process: normal mapping, reflectivity and attenuation.
    vec3 worldIncident = cameraPos - worldPos;
    vec3 nWorldNormal = normalize(worldNormal);
    vec3 reflectedDir = reflect(-worldIncident, nWorldNormal);

    // set current ray
    Ray ray = Ray(worldPos, reflectedDir);

    // perform ray-triangle intersection only for pixels with valid normal values
    vec4 output = vec4(0,0,0,1);
    if (firstR.z > 0) {

        bool intersected = false;

        // test ray-triangle intersection
        Triangle tri;
        for (int idx = 0; intersected == false, idx < trianglesTexSize.x; idx++) {
            tri = getTriangleData(idx);
            intersected = rayIntersectsTriangle(ray, tri);
        }

        // if intersected, calculates the distance and normal values
        if (intersected) {

            // distance calculation
            float reverbDistance = length(ray.origin - tri.center);
            float nReverbDistance = reverbDistance / farPlane;

            // normal calculation
            vec3 nTrianglePos = normalize(cameraPos - tri.center);
            vec3 nTriangleNormal = normalize(tri.normal);

            // presents the normal and distance data as matrix
            if (nReverbDistance <= 1) {
                if (drawDistance)   output.y = nReverbDistance;
                if (drawNormal)     output.z = abs(dot(nTrianglePos, nTriangleNormal));
            }
        }
   }

    return output;
}

// ============================================================================================================================

void main() {
    // output: primary reflections by rasterization
    vec4 firstR = primaryReflections();

    // output: secondary reflections by ray-tracing
    vec4 secndR = secondaryReflections(firstR);

    // gl_FragData[0] = firstR;
    gl_FragData[0] = secndR;
}