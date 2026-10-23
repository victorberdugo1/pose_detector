#ifndef POSE_MODEL_H
#define POSE_MODEL_H

#include "pose.h"
#include "litert/c/litert_common.h"
#include "litert/c/litert_compiled_model.h"
#include "litert/c/litert_environment.h"
#include "litert/c/litert_model.h"
#include "litert/c/litert_options.h"
#include "litert/c/litert_tensor_buffer.h"
#include "litert/c/litert_tensor_buffer_requirements.h"

#define POSE_MODEL_INPUT 256
#define POSE_MODEL_LANDMARK_VALUES 5
#define POSE_MODEL_WORLD_VALUES 3
#define POSE_MODEL_OUTPUTS 5
#define POSE_MODEL_OUT_LANDMARKS 0
#define POSE_MODEL_OUT_PRESENCE 1
#define POSE_MODEL_OUT_WORLD 4
#define POSE_MODEL_ROI_SCALE 1.25f
#define POSE_MODEL_PRESENCE 0.5f
#define POSE_MODEL_GRAY 128.0f
#define POSE_MODEL_TASK "pose_landmarker_heavy.task"
#define POSE_MODEL_ENTRY "pose_landmarks_detector.tflite"

typedef struct {
    LiteRtEnvironment env;
    LiteRtModel model;
    LiteRtCompiledModel compiled;
    LiteRtTensorBuffer input;
    LiteRtTensorBuffer outputs[POSE_MODEL_OUTPUTS];
    unsigned char *file;
} PoseModel;

static const int PoseModelLandmark[POSE_JOINTS] = {
    0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28, -1
};

static unsigned char *pose_read_file(const char *path, size_t *size)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *data = n > 0 ? malloc((size_t)n) : NULL;
    if (data && fread(data, 1, (size_t)n, f) != (size_t)n) {
        free(data);
        data = NULL;
    }
    fclose(f);
    *size = (size_t)n;
    return data;
}

static uint16_t pose_u16(const unsigned char *p)
{
    uint16_t v;
    memcpy(&v, p, sizeof v);
    return v;
}

static uint32_t pose_u32(const unsigned char *p)
{
    uint32_t v;
    memcpy(&v, p, sizeof v);
    return v;
}

static int pose_zip_entry(const unsigned char *zip, size_t size, const char *name, const unsigned char **data, size_t *dataSize)
{
    if (size < 22) return 0;
    size_t eocd = size - 22;
    while (eocd > 0 && memcmp(zip + eocd, "PK\5\6", 4)) eocd--;
    if (memcmp(zip + eocd, "PK\5\6", 4)) return 0;
    uint16_t entries = pose_u16(zip + eocd + 10);
    size_t cd = pose_u32(zip + eocd + 16);
    size_t nameLen = strlen(name);
    for (int e = 0; e < entries && cd + 46 <= size; e++) {
        uint16_t method = pose_u16(zip + cd + 10);
        uint32_t csz = pose_u32(zip + cd + 20);
        uint32_t usz = pose_u32(zip + cd + 24);
        uint16_t nl = pose_u16(zip + cd + 28);
        uint16_t el = pose_u16(zip + cd + 30);
        uint16_t cl = pose_u16(zip + cd + 32);
        uint32_t lho = pose_u32(zip + cd + 42);
        if (nl == nameLen && !memcmp(zip + cd + 46, name, nameLen)) {
            if (method != 0 || csz != usz) return 0;
            if ((size_t)lho + 30 > size) return 0;
            uint16_t lnl = pose_u16(zip + lho + 26);
            uint16_t lel = pose_u16(zip + lho + 28);
            size_t start = (size_t)lho + 30 + lnl + lel;
            if (start + usz > size) return 0;
            *data = zip + start;
            *dataSize = usz;
            return 1;
        }
        cd += 46 + nl + el + cl;
    }
    return 0;
}

static int pose_check(LiteRtStatus s)
{
    return s == kLiteRtStatusOk;
}

static void PoseModelClose(PoseModel *m)
{
    for (int k = 0; k < POSE_MODEL_OUTPUTS; k++)
        if (m->outputs[k]) LiteRtDestroyTensorBuffer(m->outputs[k]);
    if (m->input) LiteRtDestroyTensorBuffer(m->input);
    if (m->compiled) LiteRtDestroyCompiledModel(m->compiled);
    if (m->model) LiteRtDestroyModel(m->model);
    if (m->env) LiteRtDestroyEnvironment(m->env);
    free(m->file);
    memset(m, 0, sizeof *m);
}

static int PoseModelOpen(PoseModel *m, const char *task)
{
    memset(m, 0, sizeof *m);
    size_t zipSize = 0;
    unsigned char *zip = pose_read_file(task, &zipSize);
    if (!zip) return 0;
    const unsigned char *data = NULL;
    size_t dataSize = 0;
    m->file = zip;
    if (!pose_zip_entry(zip, zipSize, POSE_MODEL_ENTRY, &data, &dataSize)) goto fail;

    LiteRtOptions options = NULL;
    LiteRtSignature signature = NULL;
    LiteRtLayout inputLayout;
    if (!pose_check(LiteRtCreateEnvironment(0, NULL, &m->env))) goto fail;
    if (!pose_check(LiteRtCreateModelFromBuffer(m->env, data, dataSize, &m->model))) goto fail;
    if (!pose_check(LiteRtCreateOptions(&options))) goto fail;
    if (!pose_check(LiteRtSetOptionsHardwareAccelerators(options, kLiteRtHwAcceleratorCpu))) {
        LiteRtDestroyOptions(options);
        goto fail;
    }
    int compiled = pose_check(LiteRtCreateCompiledModel(m->env, m->model, options, &m->compiled));
    LiteRtDestroyOptions(options);
    if (!compiled) goto fail;

    if (!pose_check(LiteRtGetCompiledModelInputTensorLayout(m->compiled, 0, 0, &inputLayout))) goto fail;
    LiteRtRankedTensorType inputType = { kLiteRtElementTypeFloat32, inputLayout };
    size_t inputBytes = (size_t)POSE_MODEL_INPUT * POSE_MODEL_INPUT * 3 * sizeof(float);
    if (!pose_check(LiteRtCreateManagedTensorBuffer(m->env, kLiteRtTensorBufferTypeHostMemory, &inputType, inputBytes, &m->input))) goto fail;

    if (!pose_check(LiteRtGetModelSignature(m->model, 0, &signature))) goto fail;
    for (int k = 0; k < POSE_MODEL_OUTPUTS; k++) {
        LiteRtTensor tensor = NULL;
        LiteRtRankedTensorType type;
        LiteRtTensorBufferRequirements requirements = NULL;
        if (!pose_check(LiteRtGetSignatureOutputTensorByIndex(signature, k, &tensor))) goto fail;
        if (!pose_check(LiteRtGetRankedTensorType(tensor, &type))) goto fail;
        if (!pose_check(LiteRtGetCompiledModelOutputBufferRequirements(m->compiled, 0, k, &requirements))) goto fail;
        if (!pose_check(LiteRtCreateManagedTensorBufferFromRequirements(m->env, &type, requirements, &m->outputs[k]))) goto fail;
    }
    return 1;

fail:
    PoseModelClose(m);
    return 0;
}

static float pose_model_sample(const uint8_t *rgba, int w, int h, int x, int y, int c)
{
    if (x < 0 || y < 0 || x >= w || y >= h) return POSE_MODEL_GRAY;
    const uint8_t *p = rgba + ((size_t)y * w + x) * 4;
    return (p[c] * p[3] + POSE_MODEL_GRAY * (255 - p[3])) / 255.0f;
}

static void pose_model_fill_input(const uint8_t *rgba, int w, int h, float cx, float cy, float size, float *in)
{
    for (int v = 0; v < POSE_MODEL_INPUT; v++)
        for (int u = 0; u < POSE_MODEL_INPUT; u++) {
            float sx = cx - size * 0.5f + (u + 0.5f) * size / POSE_MODEL_INPUT - 0.5f;
            float sy = cy - size * 0.5f + (v + 0.5f) * size / POSE_MODEL_INPUT - 0.5f;
            int x0 = (int)floorf(sx), y0 = (int)floorf(sy);
            float fx = sx - x0, fy = sy - y0;
            for (int c = 0; c < 3; c++) {
                float top = pose_model_sample(rgba, w, h, x0, y0, c) * (1 - fx) + pose_model_sample(rgba, w, h, x0 + 1, y0, c) * fx;
                float bottom = pose_model_sample(rgba, w, h, x0, y0 + 1, c) * (1 - fx) + pose_model_sample(rgba, w, h, x0 + 1, y0 + 1, c) * fx;
                in[((size_t)v * POSE_MODEL_INPUT + u) * 3 + c] = (top * (1 - fy) + bottom * fy) / 255.0f;
            }
        }
}

static float pose_model_pixels_per_meter(const Pose *pose, const float *world)
{
    const PoseJoint top[2] = { POSE_LSHOULDER, POSE_RSHOULDER };
    const PoseJoint bottom[2] = { POSE_LHIP, POSE_RHIP };
    float px[2] = { 0.0f, 0.0f }, wx[2] = { 0.0f, 0.0f };
    for (int k = 0; k < 2; k++) {
        const float *a = world + (size_t)PoseModelLandmark[top[k]] * POSE_MODEL_WORLD_VALUES;
        const float *b = world + (size_t)PoseModelLandmark[bottom[k]] * POSE_MODEL_WORLD_VALUES;
        px[0] += (pose->p[bottom[k]].x - pose->p[top[k]].x) * 0.5f;
        px[1] += (pose->p[bottom[k]].y - pose->p[top[k]].y) * 0.5f;
        wx[0] += (b[0] - a[0]) * 0.5f;
        wx[1] += (b[1] - a[1]) * 0.5f;
    }
    float meters = hypotf(wx[0], wx[1]);
    return meters > 0.001f ? hypotf(px[0], px[1]) / meters : 0.0f;
}

static int PoseModelDetect(PoseModel *m, const uint8_t *rgba, int w, int h, Pose *out)
{
    int x0 = w, x1 = -1, y0 = h, y1 = -1;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (pose_on(rgba, w, x, y)) {
                if (x < x0) x0 = x;
                if (x > x1) x1 = x;
                if (y < y0) y0 = y;
                if (y > y1) y1 = y;
            }
    if (y1 < 0) return 0;

    float bw = (float)(x1 - x0 + 1), bh = (float)(y1 - y0 + 1);
    float cx = (x0 + x1 + 1) * 0.5f, cy = (y0 + y1 + 1) * 0.5f;
    float size = fmaxf(bw, bh) * POSE_MODEL_ROI_SCALE;

    float *in = NULL;
    if (!pose_check(LiteRtLockTensorBuffer(m->input, (void **)&in, kLiteRtTensorBufferLockModeWrite))) return 0;
    pose_model_fill_input(rgba, w, h, cx, cy, size, in);
    LiteRtUnlockTensorBuffer(m->input);

    LiteRtTensorBuffer inputs[1] = { m->input };
    if (!pose_check(LiteRtRunCompiledModel(m->compiled, 0, 1, inputs, POSE_MODEL_OUTPUTS, m->outputs))) return 0;

    float *presence = NULL, *landmarks = NULL, *world = NULL;
    if (!pose_check(LiteRtLockTensorBuffer(m->outputs[POSE_MODEL_OUT_PRESENCE], (void **)&presence, kLiteRtTensorBufferLockModeRead))) return 0;
    float present = presence[0];
    LiteRtUnlockTensorBuffer(m->outputs[POSE_MODEL_OUT_PRESENCE]);
    if (present < POSE_MODEL_PRESENCE) return 0;

    if (!pose_check(LiteRtLockTensorBuffer(m->outputs[POSE_MODEL_OUT_LANDMARKS], (void **)&landmarks, kLiteRtTensorBufferLockModeRead))) return 0;
    if (!pose_check(LiteRtLockTensorBuffer(m->outputs[POSE_MODEL_OUT_WORLD], (void **)&world, kLiteRtTensorBufferLockModeRead))) {
        LiteRtUnlockTensorBuffer(m->outputs[POSE_MODEL_OUT_LANDMARKS]);
        return 0;
    }
    Pose pose = { 0 };
    for (int j = 0; j < POSE_JOINTS; j++) {
        if (PoseModelLandmark[j] < 0) continue;
        const float *lm = landmarks + (size_t)PoseModelLandmark[j] * POSE_MODEL_LANDMARK_VALUES;
        pose.p[j].x = pose_limit(cx + (lm[0] / POSE_MODEL_INPUT - 0.5f) * size, 0.0f, (float)(w - 1));
        pose.p[j].y = pose_limit(cy + (lm[1] / POSE_MODEL_INPUT - 0.5f) * size, 0.0f, (float)(h - 1));
    }
    float pixelsPerMeter = pose_model_pixels_per_meter(&pose, world);
    for (int j = 0; j < POSE_JOINTS; j++)
        if (PoseModelLandmark[j] >= 0)
            pose.zModel[j] = world[(size_t)PoseModelLandmark[j] * POSE_MODEL_WORLD_VALUES + 2] * pixelsPerMeter;
    LiteRtUnlockTensorBuffer(m->outputs[POSE_MODEL_OUT_WORLD]);
    LiteRtUnlockTensorBuffer(m->outputs[POSE_MODEL_OUT_LANDMARKS]);
    if (pixelsPerMeter <= 0.0f) return 0;

    pose.p[POSE_NECK].x = (pose.p[POSE_LSHOULDER].x + pose.p[POSE_RSHOULDER].x) * 0.5f;
    pose.p[POSE_NECK].y = (pose.p[POSE_LSHOULDER].y + pose.p[POSE_RSHOULDER].y) * 0.5f;
    pose.zModel[POSE_NECK] = (pose.zModel[POSE_LSHOULDER] + pose.zModel[POSE_RSHOULDER]) * 0.5f;
    float pelvis = (pose.p[POSE_LHIP].x + pose.p[POSE_RHIP].x) * 0.5f;
    pose.yaw = pose_limit((pose.p[POSE_NOSE].x - pelvis) / (POSE_YAW_RANGE * bh), -1.0f, 1.0f);
    pose.height = bh;
    pose.hasZModel = 1;
    pose.valid = 1;
    *out = pose;
    return 1;
}

#endif
