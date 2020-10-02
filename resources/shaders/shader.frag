#version 130

in vec3 worldPos;                   // position in world space
in vec3 worldNormal;                // normal in world space
in vec3 cameraPos;                  // viewer's position in world space
in vec3 viewPos;                    // position in view space
in vec3 viewNormal;                 // normal in view space
in mat3 TBN;                        // TBN matrix

uniform bool drawNormal;            // enable normal drawing in final shader image
uniform bool drawDistance;          // enable distance drawing in final shader image
uniform bool drawReverb;            // enable reverberation effect in final shader image
uniform float farPlane;             // maximum range
uniform float reflectance;          // reflectance value
uniform float attenuationCoeff;     // attenuation coefficient

uniform sampler2D normalTex;        // texture for normal mapping
uniform bool useNormalTex;          // enable normal mapping process

uniform sampler2D trianglesTex;     // all triangles and bounding boxes collected from the simulated scene
uniform vec4 trianglesTexSize;      // texture size of triangles

// ray definition
struct Ray {
    vec3 origin;
    vec3 direction;
    vec3 invDirection;
    int sign[3];
};

Ray makeRay(vec3 origin, vec3 direction) {
    vec3 invDirection = vec3(1.0) / direction;
    return Ray(
        origin,
        direction,
        invDirection,
        int[3] (int(invDirection.x < 0.0),
                int(invDirection.y < 0.0),
                int(invDirection.z < 0.0)));
}

// triangle definition
struct Triangle {
    vec3 v0, v1, v2;    // vertices A, B and C
    vec3 center;        // centroid
    vec3 normal;        // normal
};

// box definition
struct Box {
    vec3 aabb[2];    // Bottom left and Top right vertices
};

// get data element from texture
float getTexData(sampler2D tex, int i, int j) {
    return texelFetch(tex, ivec2(i,j), 0).r;
}

// get triangle data from texture
Triangle getTriangleData(sampler2D tex, int idx) {
    Triangle triangle;
    triangle.v0     = vec3(getTexData(tex, idx, 0),     getTexData(tex, idx, 1),    getTexData(tex, idx, 2));
    triangle.v1     = vec3(getTexData(tex, idx, 3),     getTexData(tex, idx, 4),    getTexData(tex, idx, 5));
    triangle.v2     = vec3(getTexData(tex, idx, 6),     getTexData(tex, idx, 7),    getTexData(tex, idx, 8));
    triangle.center = vec3(getTexData(tex, idx, 9),     getTexData(tex, idx, 10),   getTexData(tex, idx, 11));
    triangle.normal = vec3(getTexData(tex, idx, 12),    getTexData(tex, idx, 13),   getTexData(tex, idx, 14));
    return triangle;
}

// get box data from texture
Box getBoxData(sampler2D tex, int idx) {
    Box box;
    box.aabb[0]     = vec3(getTexData(tex, idx, 0),     getTexData(tex, idx, 1),    getTexData(tex, idx, 2));
    box.aabb[1]     = vec3(getTexData(tex, idx, 3),     getTexData(tex, idx, 4),    getTexData(tex, idx, 5));
    return box;
}

// check if the point is within the bounding box
bool boxContainsPoint(Box box, vec3 p) {
    if ((box.aabb[0].x <= p.x) && (p.x <= box.aabb[1].x)
        && (box.aabb[0].y <= p.y) && (p.y <= box.aabb[1].y)
        && (box.aabb[0].z <= p.z) && (p.z <= box.aabb[1].z))
        return true;

    return false;
}

// Ray-AABB (axis-aligned bounding box) intersection algorithm
// source: https://bit.ly/2oCdyEP
bool rayIntersectsBox(Ray ray, Box box)
{
    float tmin  = (box.aabb[ray.sign[0]    ].x - ray.origin.x) * ray.invDirection.x;
    float tmax  = (box.aabb[1 - ray.sign[0]].x - ray.origin.x) * ray.invDirection.x;
    float tymin = (box.aabb[ray.sign[1]    ].y - ray.origin.y) * ray.invDirection.y;
    float tymax = (box.aabb[1 - ray.sign[1]].y - ray.origin.y) * ray.invDirection.y;
    float tzmin = (box.aabb[ray.sign[2]    ].z - ray.origin.z) * ray.invDirection.z;
    float tzmax = (box.aabb[1 - ray.sign[2]].z - ray.origin.z) * ray.invDirection.z;

    tmin = max(max(tmin, tymin), tzmin);
    tmax = min(min(tmax, tymax), tzmax);

    return (tmin < tmax);
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
    vec3 nViewPos = normalize(viewPos);
    vec3 nViewNormal = normalize(viewNormal);

    // normal for textured scenes (by normal mapping)
    if (useNormalTex) {
        // convert Tangent space to World space with TBN matrix
        vec3 normalRGB = texture2D(normalTex, gl_TexCoord[0].xy).rgb;
        vec3 normalMap = (normalRGB * 2.0 - 1.0) * TBN;
        nViewNormal = normalize(normalMap);
    }

    // material's reflectivity property
    if (reflectance > 0)
        nViewNormal = min(nViewNormal * reflectance, 1.0);

    // distance calculation
    float viewDistance = length(viewPos);

    // normalize distance using range value (farPlane)
    float nViewDistance = viewDistance / farPlane;

    // presents the normal and depth data as matrix
    vec4 result = vec4(0, 0, 0, 1);
    if (nViewDistance <= 1) {
        if (drawDistance)   result.y = nViewDistance;
        if (drawNormal)     result.z = abs(dot(nViewPos, nViewNormal));
    }

    return result;
}

// ============================================================================================================================

// secondary reflections: ray-box and ray-triangle intersection
vec4 secondaryReflections(vec4 firstR) {

    // calculate the reflection direction for an incident vector
    vec3 worldIncident = cameraPos - worldPos;
    vec3 nWorldNormal = normalize(worldNormal);
    vec3 reflectedDir = reflect(-worldIncident, nWorldNormal);

    // set current ray
    Ray ray = makeRay(worldPos, reflectedDir);

    // perform ray-triangle intersection only for pixels with valid normal values
    vec4 result = vec4(0,0,0,1);

    if (firstR.z > 0) {

        // test ray-box intersection
        bool triangleIntersected = false;
        for (int i = int(trianglesTexSize.y); triangleIntersected == false, i < int(trianglesTexSize.z); i++)
        {
            Box box = getBoxData(trianglesTex, i);
            bool boxIntersected = !boxContainsPoint(box, ray.origin) && rayIntersectsBox(ray, box);

            if (boxIntersected) {
                // test ray-triangle intersection
                int j = (i - int(trianglesTexSize.y) + int(trianglesTexSize.x));

                int idx0 = int(getTexData(trianglesTex, j + 0, 0));
                int idx1 = int(getTexData(trianglesTex, j + 1, 0));

                Triangle tri;
                for (int k = idx0; triangleIntersected == false, k < idx1; k++) {
                    tri = getTriangleData(trianglesTex, k);
                    triangleIntersected = rayIntersectsTriangle(ray, tri);
                }

                // if triangle is intersected, calculates the distance and normal values
                if (triangleIntersected) {
                    // distance calculation
                    float reverbDistance = length(ray.origin - tri.center);
                    float nReverbDistance = reverbDistance / farPlane;

                    // normal calculation
                    vec3 nTrianglePos = normalize(cameraPos - tri.center);
                    vec3 nTriangleNormal = normalize(tri.normal);

                    // presents the normal and distance data as matrix
                    if (nReverbDistance <= 1) {
                        if (drawDistance)   result.y = nReverbDistance;
                        if (drawNormal)     result.z = abs(dot(nTrianglePos, nTriangleNormal));
                    }
                }
            }
        }
    }

    return result;
}

// ============================================================================================================================

// merge primary and secondary reflections
vec4 unifiedReflections (vec4 firstR, vec4 secndR) {

    // distance calculation
    float nDistance = (firstR.y + secndR.y);

    // normal calculation
    float nNormal = (firstR.z + secndR.z);

    // results the merged data (distance + normal) for both reflections
    vec4 result = vec4(0, 0, 0, 1);
    if (nDistance <= 1) {
        result.y = nDistance;
        result.z = nNormal;
    }
    return result;
}

// ============================================================================================================================

void main() {
    // primary reflections by rasterization
    vec4 firstR = primaryReflections();
    vec4 result = firstR;

    if (drawReverb) {
        // secondary reflections by ray-tracing
        vec4 secndR = secondaryReflections(firstR);

        // unified reflections (primary + secondary)
        result = unifiedReflections(firstR, secndR);
    }

    // attenuation effect of sound in the water
    float value = result.z * exp(-2 * attenuationCoeff * result.y * farPlane);
    result.z = value;

    // presents the final sonar image
    gl_FragData[0] = result;
}
