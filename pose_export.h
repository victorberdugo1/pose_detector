#ifndef POSE_EXPORT_H
#define POSE_EXPORT_H

#include "pose.h"

#define POSE_EXPORT_LEVELS 3
#define POSE_EXPORT_PROCESS_NOISE 0.005f
#define POSE_EXPORT_MIN_BONE 0.0001f

typedef struct {
    int ground;
    int aspect;
    int smooth;
    int keys;
    int emptyFrames;
} PoseExportOptions;

typedef struct {
    int index;
    float p[POSE_JOINTS][3];
} PoseExportFrame;

typedef struct {
    PoseExportFrame *raw;
    PoseExportFrame *frames;
    unsigned char *keep;
    int n;
    int count;
} PoseExportResult;

static const float PoseExportMeasurementNoise[POSE_EXPORT_LEVELS] = { 0.01f, 0.05f, 0.2f };
static const float PoseExportTolerance[POSE_EXPORT_LEVELS] = { 0.005f, 0.015f, 0.04f };

static int pose_export_collect(const Pose *poses, int count, int w, int h, int aspect, PoseExportFrame *out)
{
    int n = 0;
    for (int i = 0; i < count; i++) {
        if (!poses[i].valid) continue;
        out[n].index = i;
        for (int j = 0; j < POSE_JOINTS; j++) {
            const PosePoint *p = &poses[i].p[j];
            out[n].p[j][0] = aspect ? 0.5f + (p->x - w * 0.5f) / h : p->x / w;
            out[n].p[j][1] = p->y / h;
            out[n].p[j][2] = p->z;
        }
        n++;
    }
    return n;
}

static void pose_export_filter(float *v, int n, int stride, float noise)
{
    float pos = v[0], vel = 0.0f;
    float a = 1.0f, b = 0.0f, d = 1.0f;
    for (int k = 1; k < n; k++) {
        float *z = v + k * stride;
        float pa = a + 2.0f * b + d + POSE_EXPORT_PROCESS_NOISE;
        float pb = b + d;
        float pd = d + POSE_EXPORT_PROCESS_NOISE;
        float gainPos = pa / (pa + noise);
        float gainVel = pb / (pa + noise);
        float innovation = *z - (pos + vel);
        pos += vel + gainPos * innovation;
        vel += gainVel * innovation;
        a = (1.0f - gainPos) * pa;
        b = (1.0f - gainPos) * pb;
        d = pd - gainVel * pb;
        *z = pos;
    }
}

static void pose_export_smooth(PoseExportFrame *f, int n, float noise)
{
    float *series = malloc((size_t)n * sizeof *series);
    if (!series) return;
    for (int j = 0; j < POSE_JOINTS; j++)
        for (int c = 0; c < 3; c++) {
            for (int k = 0; k < n; k++) series[k] = f[k].p[j][c];
            pose_export_filter(series, n, 1, noise);
            pose_export_filter(series + n - 1, n, -1, noise);
            for (int k = 0; k < n; k++) f[k].p[j][c] = series[k];
        }
    free(series);
}

static float pose_export_distance(const float *a, const float *b)
{
    return sqrtf((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

static void pose_export_lengths(PoseExportFrame *f, const PoseExportFrame *raw)
{
    float smooth[POSE_JOINTS + 1][3], source[POSE_JOINTS + 1][3], fitted[POSE_JOINTS + 1][3];
    int placed[POSE_JOINTS + 1] = { 0 };
    for (int j = 0; j < POSE_JOINTS; j++)
        for (int c = 0; c < 3; c++) {
            smooth[j][c] = f->p[j][c];
            source[j][c] = raw->p[j][c];
        }
    for (int c = 0; c < 3; c++) {
        smooth[POSE_PELVIS][c] = (smooth[POSE_LHIP][c] + smooth[POSE_RHIP][c]) * 0.5f;
        source[POSE_PELVIS][c] = (source[POSE_LHIP][c] + source[POSE_RHIP][c]) * 0.5f;
        fitted[POSE_PELVIS][c] = smooth[POSE_PELVIS][c];
    }
    placed[POSE_PELVIS] = 1;
    for (int pass = 0; pass < POSE_JOINTS; pass++)
        for (int i = 0; i < POSE_BONE_COUNT; i++) {
            int a = PoseBones[i][0], b = PoseBones[i][1];
            if (placed[a] == placed[b]) continue;
            int from = placed[a] ? a : b, to = placed[a] ? b : a;
            float length = pose_export_distance(source[a], source[b]);
            const float (*shape)[3] = pose_export_distance(smooth[from], smooth[to]) > POSE_EXPORT_MIN_BONE ? smooth : source;
            float reach = pose_export_distance(shape[from], shape[to]);
            for (int c = 0; c < 3; c++)
                fitted[to][c] = fitted[from][c] + (reach > 0.0f ? (shape[to][c] - shape[from][c]) / reach * length : 0.0f);
            placed[to] = 1;
        }
    for (int j = 0; j < POSE_JOINTS; j++)
        for (int c = 0; c < 3; c++) f->p[j][c] = fitted[j][c];
}

static void pose_export_restore(PoseExportFrame *f, const PoseExportFrame *raw, int n)
{
    for (int k = 0; k < n; k++) pose_export_lengths(&f[k], &raw[k]);
}

static void pose_export_ground(PoseExportFrame *f, int n)
{
    float lowest = -INFINITY;
    for (int k = 0; k < n; k++)
        lowest = fmaxf(lowest, fmaxf(f[k].p[POSE_LANKLE][1], f[k].p[POSE_RANKLE][1]));
    float shift[3] = {
        0.5f - (f[0].p[POSE_LHIP][0] + f[0].p[POSE_RHIP][0]) * 0.5f,
        1.0f - lowest,
        0.5f - (f[0].p[POSE_LHIP][2] + f[0].p[POSE_RHIP][2]) * 0.5f
    };
    for (int k = 0; k < n; k++)
        for (int j = 0; j < POSE_JOINTS; j++)
            for (int c = 0; c < 3; c++) f[k].p[j][c] += shift[c];
}

static float pose_export_ease(const PoseExportFrame *a, const PoseExportFrame *b, int index)
{
    float t = (float)(index - a->index) / (float)(b->index - a->index);
    return t * t * (3.0f - 2.0f * t);
}

static float pose_export_error(const PoseExportFrame *f, int a, int b, int k)
{
    float t = pose_export_ease(&f[a], &f[b], f[k].index);
    float worst = 0.0f;
    for (int j = 0; j < POSE_JOINTS; j++) {
        float sum = 0.0f;
        for (int c = 0; c < 3; c++) {
            float e = f[a].p[j][c] + (f[b].p[j][c] - f[a].p[j][c]) * t - f[k].p[j][c];
            sum += e * e;
        }
        worst = fmaxf(worst, sqrtf(sum));
    }
    return worst;
}

static void pose_export_select(const PoseExportFrame *f, int n, float tolerance, unsigned char *keep)
{
    keep[f[0].index] = keep[f[n - 1].index] = 1;
    for (;;) {
        float worst = tolerance;
        int pick = -1;
        int a = 0;
        for (int b = 1; b < n; b++) {
            if (!keep[f[b].index]) continue;
            for (int k = a + 1; k < b; k++) {
                float e = pose_export_error(f, a, b, k);
                if (e > worst) {
                    worst = e;
                    pick = k;
                }
            }
            a = b;
        }
        if (pick < 0) return;
        keep[f[pick].index] = 1;
    }
}

static void pose_export_write(FILE *file, const PoseExportResult *r, int gap)
{
    fputs("{\n", file);
    int first = 1;
    for (int k = 0; k < r->n; k++) {
        const PoseExportFrame *f = &r->frames[k];
        if (!r->keep[f->index]) continue;
        fprintf(file, "%s  \"frame_%04d\": {\n    \"person_0\": {\n", first ? "" : ",\n", f->index * (gap + 1));
        first = 0;
        for (int j = 0; j < POSE_JOINTS; j++)
            fprintf(file, "      \"%s\": {\n        \"x\": %.6f,\n        \"y\": %.6f,\n        \"z\": %.6f\n      }%s\n",
                    PoseNames[j], f->p[j][0], f->p[j][1], f->p[j][2], j + 1 < POSE_JOINTS ? "," : "");
        fputs("    }\n  }", file);
    }
    fputs("\n}\n", file);
}

static void PoseExportFree(PoseExportResult *r)
{
    free(r->raw);
    free(r->frames);
    free(r->keep);
    memset(r, 0, sizeof *r);
}

static int PoseExportBuild(PoseExportResult *r, const Pose *poses, int count, int w, int h, const PoseExportOptions *options)
{
    size_t capacity = (size_t)(count > 0 ? count : 1);
    PoseExportFree(r);
    r->raw = malloc(capacity * sizeof *r->raw);
    r->frames = malloc(capacity * sizeof *r->frames);
    r->keep = calloc(capacity, 1);
    if (!r->raw || !r->frames || !r->keep) {
        PoseExportFree(r);
        return 0;
    }
    r->count = count;
    r->n = pose_export_collect(poses, count, w, h, options->aspect, r->raw);
    pose_export_collect(poses, count, w, h, options->aspect, r->frames);
    if (r->n > 1 && options->smooth) {
        pose_export_smooth(r->frames, r->n, PoseExportMeasurementNoise[options->smooth - 1]);
        pose_export_restore(r->frames, r->raw, r->n);
    }
    if (r->n > 0 && options->ground)
        pose_export_ground(r->frames, r->n);
    if (r->n > 0 && options->keys)
        pose_export_select(r->frames, r->n, PoseExportTolerance[options->keys - 1], r->keep);
    else
        for (int k = 0; k < r->n; k++) r->keep[r->frames[k].index] = 1;
    return 1;
}

static int PoseExportFind(const PoseExportResult *r, int index)
{
    for (int k = 0; k < r->n; k++)
        if (r->frames[k].index == index) return k;
    return -1;
}

static int PoseExportSample(const PoseExportResult *r, int index, float out[POSE_JOINTS][3])
{
    const PoseExportFrame *a = NULL, *b = NULL;
    for (int k = 0; k < r->n; k++) {
        const PoseExportFrame *f = &r->frames[k];
        if (!r->keep[f->index]) continue;
        if (f->index <= index) a = f;
        if (f->index >= index && !b) b = f;
    }
    if (!a || !b) return 0;
    float t = a == b ? 0.0f : pose_export_ease(a, b, index);
    for (int j = 0; j < POSE_JOINTS; j++)
        for (int c = 0; c < 3; c++) out[j][c] = a->p[j][c] + (b->p[j][c] - a->p[j][c]) * t;
    return 1;
}

static int PoseExportJson(const char *path, const Pose *poses, int count, int w, int h, const PoseExportOptions *options)
{
    PoseExportResult result = { 0 };
    if (!PoseExportBuild(&result, poses, count, w, h, options)) return 0;
    FILE *file = fopen(path, "w");
    if (!file) {
        PoseExportFree(&result);
        return 0;
    }
    pose_export_write(file, &result, options->emptyFrames);
    PoseExportFree(&result);
    return fclose(file) == 0;
}

#endif
