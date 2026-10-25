#ifndef BODY_MODEL_H
#define BODY_MODEL_H

#include "raylib.h"
#include "raymath.h"
#include "pose.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_GIF
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#include "external/stb_image.h"
#pragma GCC diagnostic pop

#define BODY_MODEL_FILE "body.glb"
#define BODY_MAX_JOINTS 256
#define BODY_MAX_PRIMITIVES 8
#define BODY_NAME_VARIANTS 2
#define BODY_CAMERA_DISTANCE 2000.0f
#define BODY_FOOT_LIFT 0.25f
#define BODY_SPINE_SHARE 0.3333333f
#define BODY_SEGMENT_EPSILON 0.001f
#define BODY_NO_ANCHOR 0
#define BODY_GLB_MAGIC 0x46546C67u
#define BODY_GLB_JSON 0x4E4F534Au
#define BODY_GLB_BIN 0x004E4942u

typedef enum {
    BODY_HIPS, BODY_SPINE, BODY_SPINE1, BODY_SPINE2, BODY_NECK, BODY_HEAD,
    BODY_LCLAVICLE, BODY_LARM, BODY_LFOREARM, BODY_LHAND,
    BODY_RCLAVICLE, BODY_RARM, BODY_RFOREARM, BODY_RHAND,
    BODY_LUPLEG, BODY_LLEG, BODY_LFOOT, BODY_LTOE,
    BODY_RUPLEG, BODY_RLEG, BODY_RFOOT, BODY_RTOE,
    BODY_BONES
} BodyBone;

typedef struct {
    BodyBone bone, aim;
    PoseJoint to;
} BodyLimb;

typedef struct {
    int type, start, end, next;
} BodyToken;

typedef struct {
    const char *json;
    BodyToken *tokens;
    const unsigned char *bin;
    int binSize;
} BodyGlb;

typedef struct {
    const unsigned char *data;
    int count, width, stride, componentType, normalized;
} BodyAccessor;

typedef struct {
    BodyAccessor position, normal, uv, joints, weights, indices;
    int primitive;
    int jointBone[BODY_MAX_JOINTS];
} BodyPrimitive;

typedef struct {
    Mesh mesh;
    Material material;
    Shader shader;
    Texture2D texture;
    Vector3 *bindVertices;
    Vector3 *bindNormals;
    unsigned char *boneIds;
    float *boneWeights;
    Vector3 restPos[BODY_BONES];
    Quaternion restPelvis, restChest, restHead;
    Vector3 restHeadOffset;
    Vector3 restAxis[BODY_BONES];
    Vector3 restSpine;
    float restTorso, restLeg, restSpineLength;
} BodyModel;

enum { BODY_OBJECT, BODY_ARRAY, BODY_STRING, BODY_PRIMITIVE };

static const char *const BodyBoneName[BODY_BONES][BODY_NAME_VARIANTS] = {
    { "Hips", "pelvis" }, { "Spine", "spine_01" }, { "Spine1", "spine_02" }, { "Spine2", "spine_03" },
    { "Neck", "neck_01" }, { "Head", "Head" },
    { "LeftShoulder", "clavicle_l" }, { "LeftArm", "upperarm_l" }, { "LeftForeArm", "lowerarm_l" }, { "LeftHand", "hand_l" },
    { "RightShoulder", "clavicle_r" }, { "RightArm", "upperarm_r" }, { "RightForeArm", "lowerarm_r" }, { "RightHand", "hand_r" },
    { "LeftUpLeg", "thigh_l" }, { "LeftLeg", "calf_l" }, { "LeftFoot", "foot_l" }, { "LeftToeBase", "ball_l" },
    { "RightUpLeg", "thigh_r" }, { "RightLeg", "calf_r" }, { "RightFoot", "foot_r" }, { "RightToeBase", "ball_r" }
};

static const int BodyBoneParent[BODY_BONES] = {
    -1, BODY_HIPS, BODY_SPINE, BODY_SPINE1, BODY_SPINE2, BODY_NECK,
    BODY_SPINE2, BODY_LCLAVICLE, BODY_LARM, BODY_LFOREARM,
    BODY_SPINE2, BODY_RCLAVICLE, BODY_RARM, BODY_RFOREARM,
    BODY_HIPS, BODY_LUPLEG, BODY_LLEG, BODY_LFOOT,
    BODY_HIPS, BODY_RUPLEG, BODY_RLEG, BODY_RFOOT
};

static const BodyLimb BodyLimbs[] = {
    { BODY_LCLAVICLE, BODY_LARM, POSE_LSHOULDER },
    { BODY_LARM, BODY_LFOREARM, POSE_LELBOW },
    { BODY_LFOREARM, BODY_LHAND, POSE_LWRIST },
    { BODY_RCLAVICLE, BODY_RARM, POSE_RSHOULDER },
    { BODY_RARM, BODY_RFOREARM, POSE_RELBOW },
    { BODY_RFOREARM, BODY_RHAND, POSE_RWRIST },
    { BODY_LUPLEG, BODY_LLEG, POSE_LKNEE },
    { BODY_LLEG, BODY_LFOOT, POSE_LANKLE },
    { BODY_RUPLEG, BODY_RLEG, POSE_RKNEE },
    { BODY_RLEG, BODY_RFOOT, POSE_RANKLE }
};

static const int BodyAnchor[BODY_BONES] = {
    [BODY_LARM] = POSE_LSHOULDER, [BODY_LFOREARM] = POSE_LELBOW, [BODY_LHAND] = POSE_LWRIST,
    [BODY_RARM] = POSE_RSHOULDER, [BODY_RFOREARM] = POSE_RELBOW, [BODY_RHAND] = POSE_RWRIST,
    [BODY_LUPLEG] = POSE_LHIP, [BODY_LLEG] = POSE_LKNEE, [BODY_LFOOT] = POSE_LANKLE,
    [BODY_RUPLEG] = POSE_RHIP, [BODY_RLEG] = POSE_RKNEE, [BODY_RFOOT] = POSE_RANKLE
};

static const char *const BodyVertexShader =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "in vec2 vertexTexCoord;\n"
    "in vec3 vertexNormal;\n"
    "uniform mat4 mvp;\n"
    "out vec2 fragTexCoord;\n"
    "out vec3 fragNormal;\n"
    "void main()\n"
    "{\n"
    "    fragTexCoord = vertexTexCoord;\n"
    "    fragNormal = vertexNormal;\n"
    "    gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
    "}\n";

static const char *const BodyFragmentShader =
    "#version 330\n"
    "in vec2 fragTexCoord;\n"
    "in vec3 fragNormal;\n"
    "uniform sampler2D texture0;\n"
    "uniform vec4 colDiffuse;\n"
    "out vec4 finalColor;\n"
    "void main()\n"
    "{\n"
    "    vec3 n = normalize(fragNormal);\n"
    "    float key = max(dot(n, normalize(vec3(-0.45, 0.6, 0.65))), 0.0);\n"
    "    float fill = max(dot(n, normalize(vec3(0.6, 0.1, 0.5))), 0.0);\n"
    "    vec3 light = vec3(0.32) + 0.75 * key + 0.25 * fill;\n"
    "    vec4 albedo = texture(texture0, fragTexCoord) * colDiffuse;\n"
    "    finalColor = vec4(albedo.rgb * light, 1.0);\n"
    "}\n";

static int body_json_parse(BodyGlb *g, int length, int *pos, int *count)
{
    const char *s = g->json;

    while (*pos < length && (s[*pos] == ' ' || s[*pos] == '\t' || s[*pos] == '\r' || s[*pos] == '\n')) (*pos)++;
    if (*pos >= length) return -1;

    int index = (*count)++;
    char c = s[*pos];

    g->tokens[index].start = *pos;
    if (c == '{' || c == '[') {
        char close = c == '{' ? '}' : ']';
        g->tokens[index].type = c == '{' ? BODY_OBJECT : BODY_ARRAY;
        (*pos)++;
        for (;;) {
            while (*pos < length && (s[*pos] == ' ' || s[*pos] == '\t' || s[*pos] == '\r' || s[*pos] == '\n' || s[*pos] == ',')) (*pos)++;
            if (*pos >= length) return -1;
            if (s[*pos] == close) break;
            if (c == '{') {
                if (body_json_parse(g, length, pos, count) < 0) return -1;
                while (*pos < length && (s[*pos] == ' ' || s[*pos] == '\t' || s[*pos] == '\r' || s[*pos] == '\n')) (*pos)++;
                if (*pos >= length || s[*pos] != ':') return -1;
                (*pos)++;
            }
            if (body_json_parse(g, length, pos, count) < 0) return -1;
        }
        (*pos)++;
        g->tokens[index].end = *pos;
    } else if (c == '"') {
        g->tokens[index].type = BODY_STRING;
        g->tokens[index].start = ++(*pos);
        while (*pos < length && s[*pos] != '"') *pos += s[*pos] == '\\' ? 2 : 1;
        if (*pos >= length) return -1;
        g->tokens[index].end = *pos;
        (*pos)++;
    } else {
        g->tokens[index].type = BODY_PRIMITIVE;
        while (*pos < length && !strchr(",]} \t\r\n", s[*pos])) (*pos)++;
        g->tokens[index].end = *pos;
    }
    g->tokens[index].next = *count;
    return index;
}

static int body_json_get(const BodyGlb *g, int object, const char *key)
{
    if (object < 0 || g->tokens[object].type != BODY_OBJECT) return -1;
    size_t length = strlen(key);
    for (int i = object + 1; i < g->tokens[object].next; i = g->tokens[i + 1].next)
        if ((size_t)(g->tokens[i].end - g->tokens[i].start) == length && !memcmp(g->json + g->tokens[i].start, key, length))
            return i + 1;
    return -1;
}

static int body_json_at(const BodyGlb *g, int array, int index)
{
    if (array < 0 || index < 0 || g->tokens[array].type != BODY_ARRAY) return -1;
    int i = array + 1;
    for (; index > 0 && i < g->tokens[array].next; index--) i = g->tokens[i].next;
    return i < g->tokens[array].next ? i : -1;
}

static int body_json_length(const BodyGlb *g, int array)
{
    int n = 0;
    if (array < 0 || g->tokens[array].type != BODY_ARRAY) return 0;
    for (int i = array + 1; i < g->tokens[array].next; i = g->tokens[i].next) n++;
    return n;
}

static int body_json_int(const BodyGlb *g, int token, int fallback)
{
    return token >= 0 && g->tokens[token].type == BODY_PRIMITIVE ? (int)strtol(g->json + g->tokens[token].start, NULL, 10) : fallback;
}

static int body_json_is(const BodyGlb *g, int token, const char *text)
{
    size_t length = strlen(text);
    return token >= 0 && (size_t)(g->tokens[token].end - g->tokens[token].start) == length &&
           !memcmp(g->json + g->tokens[token].start, text, length);
}

static int body_glb_open(BodyGlb *g, const unsigned char *file, int size)
{
    uint32_t magic, jsonLength, jsonType, binLength, binType;

    if (size < 28) return 0;
    memcpy(&magic, file, 4);
    memcpy(&jsonLength, file + 12, 4);
    memcpy(&jsonType, file + 16, 4);
    if (magic != BODY_GLB_MAGIC || jsonType != BODY_GLB_JSON || (size_t)20 + jsonLength + 8 > (size_t)size) return 0;
    memcpy(&binLength, file + 20 + jsonLength, 4);
    memcpy(&binType, file + 24 + jsonLength, 4);
    if (binType != BODY_GLB_BIN || (size_t)28 + jsonLength + binLength > (size_t)size) return 0;

    g->json = (const char *)file + 20;
    g->bin = file + 28 + jsonLength;
    g->binSize = (int)binLength;
    g->tokens = malloc(((size_t)jsonLength + 1) * sizeof *g->tokens);
    if (!g->tokens) return 0;

    int pos = 0, count = 0;
    return body_json_parse(g, (int)jsonLength, &pos, &count) == 0;
}

static int body_glb_view(const BodyGlb *g, int view, int offset, const unsigned char **data, int *length, int *stride)
{
    int views = body_json_get(g, 0, "bufferViews");
    int token = body_json_at(g, views, view);
    int start = body_json_int(g, body_json_get(g, token, "byteOffset"), 0) + offset;
    int size = body_json_int(g, body_json_get(g, token, "byteLength"), -1);

    if (token < 0 || size < 0 || start < 0 || (size_t)start + (size_t)size > (size_t)g->binSize) return 0;
    *data = g->bin + start;
    *length = size - offset;
    if (stride) *stride = body_json_int(g, body_json_get(g, token, "byteStride"), 0);
    return 1;
}

static int body_component_size(int componentType)
{
    switch (componentType) {
    case 5120: case 5121: return 1;
    case 5122: case 5123: return 2;
    case 5125: case 5126: return 4;
    }
    return 0;
}

static int body_type_width(const BodyGlb *g, int token)
{
    static const char *const names[] = { "SCALAR", "VEC2", "VEC3", "VEC4", "MAT4" };
    static const int widths[] = { 1, 2, 3, 4, 16 };

    for (int i = 0; i < 5; i++)
        if (body_json_is(g, token, names[i])) return widths[i];
    return 0;
}

static int body_accessor(const BodyGlb *g, int index, int width, BodyAccessor *a)
{
    int token = body_json_at(g, body_json_get(g, 0, "accessors"), index);
    int length = 0, stride = 0;

    if (token < 0) return 0;
    a->componentType = body_json_int(g, body_json_get(g, token, "componentType"), 0);
    a->count = body_json_int(g, body_json_get(g, token, "count"), 0);
    a->width = body_type_width(g, body_json_get(g, token, "type"));
    a->normalized = body_json_is(g, body_json_get(g, token, "normalized"), "true");
    int size = body_component_size(a->componentType);
    if (!size || a->width != width || a->count <= 0) return 0;
    if (!body_glb_view(g, body_json_int(g, body_json_get(g, token, "bufferView"), -1),
                       body_json_int(g, body_json_get(g, token, "byteOffset"), 0), &a->data, &length, &stride)) return 0;
    a->stride = stride > 0 ? stride : width * size;
    return (size_t)(a->count - 1) * a->stride + (size_t)width * size <= (size_t)length;
}

static float body_accessor_value(const BodyAccessor *a, int element, int component)
{
    const unsigned char *p = a->data + (size_t)element * a->stride + (size_t)component * body_component_size(a->componentType);

    switch (a->componentType) {
    case 5126: { float v; memcpy(&v, p, sizeof v); return v; }
    case 5125: { uint32_t v; memcpy(&v, p, sizeof v); return (float)v; }
    case 5123: { uint16_t v; memcpy(&v, p, sizeof v); return a->normalized ? v / 65535.0f : (float)v; }
    case 5121: return a->normalized ? *p / 255.0f : (float)*p;
    }
    return 0.0f;
}

static Vector3 body_accessor_vec3(const BodyAccessor *a, int element)
{
    return (Vector3){ body_accessor_value(a, element, 0), body_accessor_value(a, element, 1), body_accessor_value(a, element, 2) };
}

static Vector3 body_unit(Vector3 v, Vector3 fallback)
{
    float length = Vector3Length(v);

    return length > 0.0001f ? Vector3Scale(v, 1.0f / length) : fallback;
}

static Vector3 body_mid(Vector3 a, Vector3 b)
{
    return Vector3Lerp(a, b, 0.5f);
}

static Quaternion body_frame(Vector3 side, Vector3 up)
{
    Vector3 y = body_unit(up, (Vector3){ 0.0f, 1.0f, 0.0f });
    Vector3 x = body_unit(Vector3Subtract(side, Vector3Scale(y, Vector3DotProduct(side, y))), (Vector3){ 1.0f, 0.0f, 0.0f });
    Vector3 z = Vector3CrossProduct(x, y);
    Matrix m = {
        .m0 = x.x, .m4 = y.x, .m8 = z.x,
        .m1 = x.y, .m5 = y.y, .m9 = z.y,
        .m2 = x.z, .m6 = y.z, .m10 = z.z,
        .m15 = 1.0f
    };

    return QuaternionNormalize(QuaternionFromMatrix(m));
}

static Quaternion body_swing(Vector3 from, Vector3 to)
{
    float d = Vector3DotProduct(from, to);

    if (d < -0.9999f) {
        Vector3 axis = body_unit(Vector3CrossProduct(from, (Vector3){ 1.0f, 0.0f, 0.0f }), (Vector3){ 0.0f, 1.0f, 0.0f });

        return (Quaternion){ axis.x, axis.y, axis.z, 0.0f };
    }

    Vector3 c = Vector3CrossProduct(from, to);

    return QuaternionNormalize((Quaternion){ c.x, c.y, c.z, 1.0f + d });
}

static float body_chain(const Vector3 *p, int a, int b, int c)
{
    return Vector3Distance(p[a], p[b]) + Vector3Distance(p[b], p[c]);
}

static int body_load_texture(BodyModel *b, const BodyGlb *g, int primitive)
{
    int material = body_json_at(g, body_json_get(g, 0, "materials"), body_json_int(g, body_json_get(g, primitive, "material"), -1));
    int base = body_json_get(g, body_json_get(g, material, "pbrMetallicRoughness"), "baseColorTexture");
    int texture = body_json_at(g, body_json_get(g, 0, "textures"), body_json_int(g, body_json_get(g, base, "index"), -1));
    int image = body_json_at(g, body_json_get(g, 0, "images"), body_json_int(g, body_json_get(g, texture, "source"), -1));
    const unsigned char *data;
    int length;

    if (base < 0) {
        unsigned char white[4] = { 255, 255, 255, 255 };

        b->texture = LoadTextureFromImage((Image){ white, 1, 1, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 });
        return b->texture.id != 0;
    }
    if (!body_glb_view(g, body_json_int(g, body_json_get(g, image, "bufferView"), -1), 0, &data, &length, NULL)) return 0;

    int width, height, channels;
    unsigned char *pixels = stbi_load_from_memory(data, length, &width, &height, &channels, 4);

    if (pixels == NULL) return 0;
    b->texture = LoadTextureFromImage((Image){ pixels, width, height, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 });
    free(pixels);
    if (b->texture.id == 0) return 0;
    GenTextureMipmaps(&b->texture);
    SetTextureFilter(b->texture, TEXTURE_FILTER_TRILINEAR);
    return 1;
}

static int body_bone_match(const BodyGlb *g, int name)
{
    for (int i = 0; i < BODY_BONES; i++)
        for (int v = 0; v < BODY_NAME_VARIANTS; v++)
            if (body_json_is(g, name, BodyBoneName[i][v])) return i;
    return -1;
}

static int body_node_parent(const BodyGlb *g, int node)
{
    int nodes = body_json_get(g, 0, "nodes");
    int count = body_json_length(g, nodes);

    for (int i = 0; i < count; i++) {
        int children = body_json_get(g, body_json_at(g, nodes, i), "children");
        int length = body_json_length(g, children);

        for (int c = 0; c < length; c++)
            if (body_json_int(g, body_json_at(g, children, c), -1) == node) return i;
    }
    return -1;
}

static int body_joint_bone(const BodyGlb *g, int node, int *direct)
{
    int nodes = body_json_get(g, 0, "nodes");

    *direct = 1;
    for (; node >= 0; node = body_node_parent(g, node)) {
        int bone = body_bone_match(g, body_json_get(g, body_json_at(g, nodes, node), "name"));

        if (bone >= 0) return bone;
        *direct = 0;
    }
    return BODY_HIPS;
}

static int body_load_skin(BodyModel *b, const BodyGlb *g, int index, int *jointBone)
{
    int skin = body_json_at(g, body_json_get(g, 0, "skins"), index);
    int joints = body_json_get(g, skin, "joints");
    int count = body_json_length(g, joints);
    int seen[BODY_BONES] = { 0 };
    BodyAccessor inverse;

    if (count <= 0 || count > BODY_MAX_JOINTS) return 0;
    if (!body_accessor(g, body_json_int(g, body_json_get(g, skin, "inverseBindMatrices"), -1), 16, &inverse) || inverse.count < count) return 0;

    for (int k = 0; k < count; k++) {
        int direct;

        jointBone[k] = body_joint_bone(g, body_json_int(g, body_json_at(g, joints, k), -1), &direct);
        if (!direct) continue;
        seen[jointBone[k]] = 1;
        float m[16];
        for (int e = 0; e < 16; e++) m[e] = body_accessor_value(&inverse, k, e);
        b->restPos[jointBone[k]] = (Vector3){
            -(m[0] * m[12] + m[1] * m[13] + m[2] * m[14]),
            -(m[4] * m[12] + m[5] * m[13] + m[6] * m[14]),
            -(m[8] * m[12] + m[9] * m[13] + m[10] * m[14])
        };
    }
    for (int i = 0; i < BODY_BONES; i++)
        if (!seen[i]) return 0;
    return 1;
}

static void body_measure_rest(BodyModel *b)
{
    const Vector3 *r = b->restPos;
    Vector3 hips = body_mid(r[BODY_LUPLEG], r[BODY_RUPLEG]);
    Vector3 shoulders = body_mid(r[BODY_LARM], r[BODY_RARM]);
    Vector3 up = Vector3Subtract(shoulders, hips);

    b->restPelvis = body_frame(Vector3Subtract(r[BODY_LUPLEG], r[BODY_RUPLEG]), up);
    b->restChest = body_frame(Vector3Subtract(r[BODY_LARM], r[BODY_RARM]), up);
    Vector3 headUp = Vector3Subtract(r[BODY_HEAD], r[BODY_NECK]);
    Vector3 forward = Vector3CrossProduct(Vector3Subtract(r[BODY_LUPLEG], r[BODY_RUPLEG]), up);
    b->restHead = body_frame(Vector3CrossProduct(headUp, forward), headUp);
    b->restTorso = Vector3Length(up);
    Vector3 spine = Vector3Subtract(r[BODY_NECK], r[BODY_HIPS]);
    b->restSpineLength = Vector3Length(spine);
    b->restSpine = body_unit(spine, (Vector3){ 0.0f, 1.0f, 0.0f });
    for (int i = 0; i < BODY_BONES; i++)
        b->restAxis[i] = b->restSpine;
    for (size_t n = 0; n < sizeof BodyLimbs / sizeof BodyLimbs[0]; n++)
        b->restAxis[BodyLimbs[n].bone] = body_unit(Vector3Subtract(r[BodyLimbs[n].aim], r[BodyLimbs[n].bone]), (Vector3){ 0.0f, -1.0f, 0.0f });
    Vector3 headCenter = { 0.0f, 0.0f, 0.0f };
    int headCount = 0;
    for (int i = 0; i < b->mesh.vertexCount; i++) {
        int best = 0; float bw = -1.0f;
        for (int j = 0; j < 4; j++)
            if (b->boneWeights[4 * i + j] > bw) { bw = b->boneWeights[4 * i + j]; best = b->boneIds[4 * i + j]; }
        if (best != BODY_HEAD) continue;
        headCenter = Vector3Add(headCenter, b->bindVertices[i]);
        headCount++;
    }
    b->restHeadOffset = Vector3Subtract(Vector3Scale(headCenter, 1.0f / (headCount ? headCount : 1)), r[BODY_HEAD]);
    b->restLeg = 0.5f * (body_chain(r, BODY_LUPLEG, BODY_LLEG, BODY_LFOOT) + body_chain(r, BODY_RUPLEG, BODY_RLEG, BODY_RFOOT));
}

static int body_primitive(const BodyGlb *g, int primitive, BodyPrimitive *p)
{
    int attributes = body_json_get(g, primitive, "attributes");

    p->primitive = primitive;
    if (!body_accessor(g, body_json_int(g, body_json_get(g, attributes, "POSITION"), -1), 3, &p->position) ||
        !body_accessor(g, body_json_int(g, body_json_get(g, attributes, "NORMAL"), -1), 3, &p->normal) ||
        !body_accessor(g, body_json_int(g, body_json_get(g, attributes, "TEXCOORD_0"), -1), 2, &p->uv) ||
        !body_accessor(g, body_json_int(g, body_json_get(g, attributes, "JOINTS_0"), -1), 4, &p->joints) ||
        !body_accessor(g, body_json_int(g, body_json_get(g, attributes, "WEIGHTS_0"), -1), 4, &p->weights) ||
        !body_accessor(g, body_json_int(g, body_json_get(g, primitive, "indices"), -1), 1, &p->indices)) return 0;

    int n = p->position.count;

    return p->normal.count == n && p->uv.count == n && p->joints.count == n && p->weights.count == n && p->indices.count % 3 == 0;
}

static int body_collect(BodyModel *b, const BodyGlb *g, BodyPrimitive *prims, int *primitives, int *vertices, int *indices)
{
    int nodes = body_json_get(g, 0, "nodes");
    int count = body_json_length(g, nodes);

    for (int i = 0; i < count; i++) {
        int node = body_json_at(g, nodes, i);
        int mesh = body_json_int(g, body_json_get(g, node, "mesh"), -1);
        int skin = body_json_int(g, body_json_get(g, node, "skin"), -1);

        if (mesh < 0) continue;
        if (skin < 0) return 0;
        int list = body_json_get(g, body_json_at(g, body_json_get(g, 0, "meshes"), mesh), "primitives");
        int length = body_json_length(g, list);

        for (int k = 0; k < length; k++) {
            BodyPrimitive *p = &prims[*primitives];

            if (*primitives == BODY_MAX_PRIMITIVES) return 0;
            if (!body_primitive(g, body_json_at(g, list, k), p) || !body_load_skin(b, g, skin, p->jointBone)) return 0;
            *vertices += p->position.count;
            *indices += p->indices.count;
            (*primitives)++;
        }
    }
    return *primitives > 0 && *vertices <= 65535;
}

static int body_build(BodyModel *b, const BodyGlb *g)
{
    BodyPrimitive prims[BODY_MAX_PRIMITIVES];
    int primitives = 0, n = 0, indexCount = 0;
    Mesh *mesh = &b->mesh;

    if (!body_collect(b, g, prims, &primitives, &n, &indexCount)) return 0;

    mesh->vertexCount = n;
    mesh->triangleCount = indexCount / 3;
    mesh->vertices = MemAlloc((unsigned int)n * 3 * sizeof(float));
    mesh->normals = MemAlloc((unsigned int)n * 3 * sizeof(float));
    mesh->texcoords = MemAlloc((unsigned int)n * 2 * sizeof(float));
    mesh->indices = MemAlloc((unsigned int)indexCount * sizeof(unsigned short));
    b->bindVertices = malloc((size_t)n * sizeof *b->bindVertices);
    b->bindNormals = malloc((size_t)n * sizeof *b->bindNormals);
    b->boneIds = malloc((size_t)n * 4);
    b->boneWeights = malloc((size_t)n * 4 * sizeof(float));
    if (!mesh->vertices || !mesh->normals || !mesh->texcoords || !mesh->indices ||
        !b->bindVertices || !b->bindNormals || !b->boneIds || !b->boneWeights) return 0;

    int vertexBase = 0, indexBase = 0;

    for (int q = 0; q < primitives; q++) {
        const BodyPrimitive *p = &prims[q];

        for (int v = 0; v < p->position.count; v++) {
            int i = vertexBase + v;

            b->bindVertices[i] = body_accessor_vec3(&p->position, v);
            b->bindNormals[i] = body_accessor_vec3(&p->normal, v);
            mesh->texcoords[2 * i] = body_accessor_value(&p->uv, v, 0);
            mesh->texcoords[2 * i + 1] = body_accessor_value(&p->uv, v, 1);
            for (int k = 0; k < 4; k++) {
                int joint = (int)body_accessor_value(&p->joints, v, k);

                if (joint < 0 || joint >= BODY_MAX_JOINTS) return 0;
                b->boneIds[4 * i + k] = (unsigned char)p->jointBone[joint];
                b->boneWeights[4 * i + k] = body_accessor_value(&p->weights, v, k);
            }
        }
        for (int e = 0; e < p->indices.count; e++) {
            int index = (int)body_accessor_value(&p->indices, e, 0);

            if (index >= p->position.count) return 0;
            mesh->indices[indexBase + e] = (unsigned short)(vertexBase + index);
        }
        vertexBase += p->position.count;
        indexBase += p->indices.count;
    }

    body_measure_rest(b);
    if (!body_load_texture(b, g, prims[0].primitive)) return 0;
    b->shader = LoadShaderFromMemory(BodyVertexShader, BodyFragmentShader);
    if (!IsShaderValid(b->shader)) return 0;
    b->material = LoadMaterialDefault();
    b->material.shader = b->shader;
    b->material.maps[MATERIAL_MAP_ALBEDO].texture = b->texture;
    memcpy(mesh->vertices, b->bindVertices, (size_t)n * sizeof *b->bindVertices);
    memcpy(mesh->normals, b->bindNormals, (size_t)n * sizeof *b->bindNormals);
    UploadMesh(mesh, true);
    return mesh->vaoId != 0;
}

static void BodyModelClose(BodyModel *b)
{
    if (b->mesh.vaoId != 0) UnloadMesh(b->mesh);
    else {
        MemFree(b->mesh.vertices);
        MemFree(b->mesh.normals);
        MemFree(b->mesh.texcoords);
        MemFree(b->mesh.indices);
    }
    if (b->material.maps) MemFree(b->material.maps);
    if (b->shader.id != 0) UnloadShader(b->shader);
    if (b->texture.id != 0) UnloadTexture(b->texture);
    free(b->bindVertices);
    free(b->bindNormals);
    free(b->boneIds);
    free(b->boneWeights);
    memset(b, 0, sizeof *b);
}

static int BodyModelOpen(BodyModel *b, const char *path)
{
    BodyGlb g = { 0 };
    int size = 0;
    unsigned char *file = LoadFileData(path, &size);

    memset(b, 0, sizeof *b);
    int ok = file && body_glb_open(&g, file, size) && body_build(b, &g);
    free(g.tokens);
    UnloadFileData(file);
    if (!ok) BodyModelClose(b);
    return ok;
}

static Vector3 body_side(Vector3 side, Vector3 up, Vector3 gaze)
{
    Vector3 forward = Vector3CrossProduct(side, up);

    return Vector3DotProduct(forward, gaze) < 0.0f ? Vector3Negate(side) : side;
}

static Vector3 body_stretch(Vector3 v, Vector3 axis, float along, float across)
{
    Vector3 parallel = Vector3Scale(axis, Vector3DotProduct(v, axis));

    return Vector3Add(Vector3Scale(parallel, along), Vector3Scale(Vector3Subtract(v, parallel), across));
}

static Vector3 body_along(Vector3 from, Vector3 to, Vector3 restFrom, Vector3 restTo, Vector3 restPoint)
{
    Vector3 axis = Vector3Subtract(restTo, restFrom);
    float t = Vector3DotProduct(Vector3Subtract(restPoint, restFrom), axis) / fmaxf(Vector3DotProduct(axis, axis), 0.0001f);

    return Vector3Add(from, Vector3Scale(Vector3Subtract(to, from), t));
}

static void body_pose(BodyModel *b, const Vector3 *k)
{
    const Vector3 *r = b->restPos;
    Quaternion rot[BODY_BONES];
    Vector3 pos[BODY_BONES];
    float along[BODY_BONES], across[BODY_BONES];
    Vector3 hipMid = body_mid(k[POSE_LHIP], k[POSE_RHIP]);
    Vector3 shoulderMid = body_mid(k[POSE_LSHOULDER], k[POSE_RSHOULDER]);
    Vector3 up = Vector3Subtract(k[POSE_NECK], hipMid);
    float torso = Vector3Distance(shoulderMid, hipMid);
    float leg = 0.5f * (body_chain(k, POSE_LHIP, POSE_LKNEE, POSE_LANKLE) + body_chain(k, POSE_RHIP, POSE_RKNEE, POSE_RANKLE));
    float scale = (torso + leg) / (b->restTorso + b->restLeg);

    if (scale < 0.0001f) return;

    Vector3 restHipMid = body_mid(r[BODY_LUPLEG], r[BODY_RUPLEG]);
    Vector3 earMid = body_mid(k[POSE_LEAR], k[POSE_REAR]);
    Vector3 headUp = Vector3Subtract(earMid, k[POSE_NECK]);
    Vector3 gaze = Vector3Subtract(k[POSE_NOSE], earMid);
    Vector3 hipSide = body_side(Vector3Subtract(k[POSE_LHIP], k[POSE_RHIP]), up, gaze);
    Vector3 shoulderSide = Vector3Subtract(k[POSE_LSHOULDER], k[POSE_RSHOULDER]);
    Quaternion pelvis = QuaternionMultiply(body_frame(hipSide, up), QuaternionInvert(b->restPelvis));
    Quaternion chest = QuaternionMultiply(body_frame(shoulderSide, up), QuaternionInvert(b->restChest));
    Vector3 neckLocal = Vector3RotateByQuaternion(Vector3Subtract(r[BODY_NECK], body_mid(r[BODY_LARM], r[BODY_RARM])), QuaternionInvert(b->restChest));
    Vector3 neck = Vector3Add(shoulderMid, Vector3Scale(Vector3RotateByQuaternion(neckLocal, QuaternionMultiply(chest, b->restChest)), scale));
    float spineStretch = fmaxf(Vector3Distance(neck, hipMid) / fmaxf(Vector3Distance(r[BODY_NECK], restHipMid), 0.0001f), BODY_SEGMENT_EPSILON);
    Quaternion head;
    Quaternion flat;
    float lowest = fminf(k[POSE_LANKLE].y, k[POSE_RANKLE].y);

    headUp = Vector3Subtract(earMid, neck);
    head = QuaternionMultiply(body_frame(Vector3CrossProduct(headUp, gaze), headUp), QuaternionInvert(b->restHead));
    hipSide.y = 0.0f;
    flat = body_frame(hipSide, (Vector3){ 0.0f, 1.0f, 0.0f });
    rot[BODY_HIPS] = pelvis;
    rot[BODY_SPINE] = QuaternionSlerp(pelvis, chest, BODY_SPINE_SHARE);
    rot[BODY_SPINE1] = QuaternionSlerp(pelvis, chest, 2.0f * BODY_SPINE_SHARE);
    rot[BODY_SPINE2] = chest;
    rot[BODY_HEAD] = head;
    rot[BODY_NECK] = QuaternionSlerp(chest, head, 0.5f);

    for (int i = 0; i < BODY_BONES; i++) {
        int parent = BodyBoneParent[i];
        const BodyLimb *limb = NULL;

        along[i] = across[i] = scale;
        if (i <= BODY_SPINE2) along[i] = spineStretch;

        if (parent < 0)
            pos[i] = Vector3Subtract(hipMid, Vector3RotateByQuaternion(
                body_stretch(Vector3Subtract(restHipMid, r[BODY_HIPS]), b->restAxis[i], along[i], across[i]), pelvis));
        else if (i == BODY_NECK)
            pos[i] = neck;
        else if (i == BODY_SPINE || i == BODY_SPINE1 || i == BODY_SPINE2)
            pos[i] = body_along(pos[BODY_HIPS], neck, r[BODY_HIPS], r[BODY_NECK], r[i]);
        else if (i == BODY_HEAD)
            pos[i] = Vector3Subtract(earMid, Vector3Scale(Vector3RotateByQuaternion(b->restHeadOffset, rot[BODY_HEAD]), scale));
        else if (BodyAnchor[i] != BODY_NO_ANCHOR)
            pos[i] = k[BodyAnchor[i]];
        else
            pos[i] = Vector3Add(pos[parent], Vector3RotateByQuaternion(
                body_stretch(Vector3Subtract(r[i], r[parent]), b->restAxis[parent], along[parent], across[parent]), rot[parent]));

        for (size_t n = 0; n < sizeof BodyLimbs / sizeof BodyLimbs[0]; n++)
            if (BodyLimbs[n].bone == (BodyBone)i) limb = &BodyLimbs[n];

        if (limb) {
            Quaternion inherited = rot[parent];
            Vector3 restDir = Vector3Subtract(r[limb->aim], r[i]);
            Vector3 rest = Vector3RotateByQuaternion(b->restAxis[i], inherited);
            Vector3 reach = Vector3Subtract(k[limb->to], pos[i]);

            rot[i] = QuaternionMultiply(body_swing(rest, body_unit(reach, rest)), inherited);
            along[i] = fmaxf(Vector3Length(reach) / fmaxf(Vector3Length(restDir), 0.0001f), BODY_SEGMENT_EPSILON);
        } else if (i == BODY_LHAND || i == BODY_RHAND) {
            rot[i] = rot[parent];
        } else if (i == BODY_LFOOT || i == BODY_RFOOT) {
            int ankle = i == BODY_LFOOT ? POSE_LANKLE : POSE_RANKLE;
            float lift = Clamp((k[ankle].y - lowest) / (BODY_FOOT_LIFT * scale * b->restLeg), 0.0f, 1.0f);

            rot[i] = QuaternionSlerp(flat, rot[parent], lift);
        } else if (i == BODY_LTOE || i == BODY_RTOE) {
            rot[i] = rot[parent];
        }
    }

    Vector3 *vertices = (Vector3 *)b->mesh.vertices;
    Vector3 *normals = (Vector3 *)b->mesh.normals;

    for (int i = 0; i < b->mesh.vertexCount; i++) {
        Vector3 v = { 0.0f, 0.0f, 0.0f }, n = { 0.0f, 0.0f, 0.0f };

        for (int j = 0; j < 4; j++) {
            float w = b->boneWeights[4 * i + j];
            int bone = b->boneIds[4 * i + j];

            if (w <= 0.0f) continue;
            Vector3 local = body_stretch(Vector3Subtract(b->bindVertices[i], r[bone]), b->restAxis[bone], along[bone], across[bone]);

            v = Vector3Add(v, Vector3Scale(Vector3Add(pos[bone], Vector3RotateByQuaternion(local, rot[bone])), w));
            n = Vector3Add(n, Vector3Scale(Vector3RotateByQuaternion(b->bindNormals[i], rot[bone]), w));
        }
        vertices[i] = v;
        normals[i] = body_unit(n, (Vector3){ 0.0f, 0.0f, 1.0f });
    }
    UpdateMeshBuffer(b->mesh, 0, b->mesh.vertices, b->mesh.vertexCount * (int)sizeof(Vector3), 0);
    UpdateMeshBuffer(b->mesh, 2, b->mesh.normals, b->mesh.vertexCount * (int)sizeof(Vector3), 0);
}

static Camera3D BodyCamera(void)
{
    Vector2 center = { (float)GetScreenWidth() * 0.5f, -(float)GetScreenHeight() * 0.5f };

    return (Camera3D){
        .position = { center.x, center.y, BODY_CAMERA_DISTANCE },
        .target = { center.x, center.y, 0.0f },
        .up = { 0.0f, 1.0f, 0.0f },
        .fovy = (float)GetScreenHeight(),
        .projection = CAMERA_ORTHOGRAPHIC
    };
}

#endif
