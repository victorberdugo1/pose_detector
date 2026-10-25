#ifndef POSE_H
#define POSE_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define POSE_DEPTH_STEP 4
#define POSE_DEPTH_ITERATIONS 300
#define POSE_DEPTH_RELAXATION 1.9f
#define POSE_Z_RANGE 0.1f
#define POSE_HEAD_RADIUS 0.085f
#define POSE_YAW_RANGE 0.2f
#define POSE_HALF_PI 1.5707963f
#define POSE_EYE_AZIMUTH 0.45f
#define POSE_EAR_AZIMUTH 1.75f
#define POSE_UPPER_ARM 0.56f
#define POSE_ARM_RATIO 0.332f
#define POSE_LIMB_MARGIN 1.03f
#define POSE_LIMB_MIN 0.8f
#define POSE_BONE_KINDS 4
#define POSE_FOLD_COS 0.94f
#define POSE_ARM_TWIST 1.75f
#define POSE_LEG_TWIST 1.05f
#define POSE_FLIP_MIN 0.01f
#define POSE_HEAD_REACH 0.3f
#define POSE_NECK_RATIO 0.4f
#define POSE_BG_TOLERANCE 16

typedef enum {
    POSE_NOSE, POSE_LEYE, POSE_REYE, POSE_LEAR, POSE_REAR,
    POSE_LSHOULDER, POSE_RSHOULDER, POSE_LELBOW, POSE_RELBOW,
    POSE_LWRIST, POSE_RWRIST, POSE_LHIP, POSE_RHIP,
    POSE_LKNEE, POSE_RKNEE, POSE_LANKLE, POSE_RANKLE, POSE_NECK,
    POSE_JOINTS
} PoseJoint;

typedef struct {
    float x, y, z, dz;
} PosePoint;

typedef struct {
    PosePoint p[POSE_JOINTS];
    float yaw, height;
    float zModel[POSE_JOINTS];
    int hasZModel;
    int solve[2];
    int valid;
} Pose;

typedef struct {
    int head;
} PoseFlips;

typedef struct {
    float x, y, z;
} PoseVec;

typedef struct {
    PoseVec forward, down, side;
} PoseBody;

typedef struct {
    float arm;
    float torso;
    float bone[POSE_BONE_KINDS];
    PoseFlips flips;
} PoseLimbs;

static const PoseJoint PoseUpperBody[] = {
    POSE_NOSE, POSE_LEYE, POSE_REYE, POSE_LEAR, POSE_REAR, POSE_LSHOULDER, POSE_RSHOULDER, POSE_NECK,
    POSE_LELBOW, POSE_RELBOW, POSE_LWRIST, POSE_RWRIST
};

static const PoseJoint PoseHead[] = { POSE_NOSE, POSE_LEYE, POSE_REYE, POSE_LEAR, POSE_REAR };

static const float PoseBoneTorsoRatio[POSE_BONE_KINDS] = { 0.646f, 0.507f, 0.851f, 0.854f };

typedef struct {
    int count;
    float minX, maxX, meanX;
} PoseBand;

static const char *const PoseNames[POSE_JOINTS] = {
    "Nose", "REye", "LEye", "REar", "LEar", "RShoulder", "LShoulder",
    "RElbow", "LElbow", "RWrist", "LWrist", "RHip", "LHip",
    "RKnee", "LKnee", "RAnkle", "LAnkle", "Neck"
};

#define POSE_PELVIS POSE_JOINTS

static const int PoseBones[][2] = {
    {POSE_NECK, POSE_NOSE}, {POSE_NOSE, POSE_LEYE}, {POSE_LEYE, POSE_LEAR},
    {POSE_NOSE, POSE_REYE}, {POSE_REYE, POSE_REAR},
    {POSE_NECK, POSE_LSHOULDER}, {POSE_LSHOULDER, POSE_LELBOW}, {POSE_LELBOW, POSE_LWRIST},
    {POSE_NECK, POSE_RSHOULDER}, {POSE_RSHOULDER, POSE_RELBOW}, {POSE_RELBOW, POSE_RWRIST},
    {POSE_NECK, POSE_PELVIS}, {POSE_PELVIS, POSE_LHIP}, {POSE_LHIP, POSE_LKNEE}, {POSE_LKNEE, POSE_LANKLE},
    {POSE_PELVIS, POSE_RHIP}, {POSE_RHIP, POSE_RKNEE}, {POSE_RKNEE, POSE_RANKLE}
};

#define POSE_BONE_COUNT ((int)(sizeof(PoseBones) / sizeof(PoseBones[0])))

static const PoseJoint PoseArms[2][3] = {
    {POSE_LSHOULDER, POSE_LELBOW, POSE_LWRIST},
    {POSE_RSHOULDER, POSE_RELBOW, POSE_RWRIST}
};

static const PoseJoint PoseBoneChain[POSE_BONE_KINDS][2][2] = {
    {{POSE_LSHOULDER, POSE_LELBOW}, {POSE_RSHOULDER, POSE_RELBOW}},
    {{POSE_LELBOW, POSE_LWRIST}, {POSE_RELBOW, POSE_RWRIST}},
    {{POSE_LHIP, POSE_LKNEE}, {POSE_RHIP, POSE_RKNEE}},
    {{POSE_LKNEE, POSE_LANKLE}, {POSE_RKNEE, POSE_RANKLE}}
};

static const PoseJoint PoseLegs[2][3] = {
    {POSE_LHIP, POSE_LKNEE, POSE_LANKLE},
    {POSE_RHIP, POSE_RKNEE, POSE_RANKLE}
};

static int pose_on(const uint8_t *rgba, int w, int x, int y)
{
    return rgba[((size_t)y * w + x) * 4 + 3] > 127;
}

static float pose_limit(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static int pose_near(const uint8_t *p, const uint8_t *bg)
{
    return abs(p[0] - bg[0]) <= POSE_BG_TOLERANCE && abs(p[1] - bg[1]) <= POSE_BG_TOLERANCE &&
           abs(p[2] - bg[2]) <= POSE_BG_TOLERANCE;
}

static void pose_push(uint8_t *rgba, const uint8_t *bg, int *queue, int *tail, int idx)
{
    uint8_t *p = rgba + (size_t)idx * 4;
    if (p[3] == 0 || !pose_near(p, bg)) return;
    p[3] = 0;
    queue[(*tail)++] = idx;
}

static int pose_border_background(const uint8_t *rgba, int w, int h, uint8_t bg[3])
{
    int *index = malloc((size_t)2 * (w + h) * sizeof(int));
    if (!index) return 0;
    int n = 0;
    for (int x = 0; x < w; x++) {
        index[n++] = x;
        index[n++] = (h - 1) * w + x;
    }
    for (int y = 1; y < h - 1; y++) {
        index[n++] = y * w;
        index[n++] = y * w + w - 1;
    }
    int best = 0, bestVotes = 0;
    for (int i = 0; i < n; i++) {
        const uint8_t *c = rgba + (size_t)index[i] * 4;
        if (c[3] == 0) continue;
        int votes = 0;
        for (int j = 0; j < n; j++) {
            const uint8_t *d = rgba + (size_t)index[j] * 4;
            votes += d[3] > 0 && pose_near(d, c);
        }
        if (votes > bestVotes) {
            bestVotes = votes;
            best = index[i];
        }
    }
    free(index);
    if (bestVotes * 2 <= n) return 0;
    memcpy(bg, rgba + (size_t)best * 4, 3);
    return 1;
}

static void PoseKeyBackground(uint8_t *rgba, int w, int h)
{
    uint8_t bg[3];
    if (!pose_border_background(rgba, w, h, bg)) return;
    int *queue = malloc((size_t)w * h * sizeof(int));
    if (!queue) return;
    int head = 0, tail = 0;
    for (int x = 0; x < w; x++) {
        pose_push(rgba, bg, queue, &tail, x);
        pose_push(rgba, bg, queue, &tail, (h - 1) * w + x);
    }
    for (int y = 0; y < h; y++) {
        pose_push(rgba, bg, queue, &tail, y * w);
        pose_push(rgba, bg, queue, &tail, y * w + w - 1);
    }
    while (head < tail) {
        int idx = queue[head++];
        int x = idx % w, y = idx / w;
        if (x > 0) pose_push(rgba, bg, queue, &tail, idx - 1);
        if (x < w - 1) pose_push(rgba, bg, queue, &tail, idx + 1);
        if (y > 0) pose_push(rgba, bg, queue, &tail, idx - w);
        if (y < h - 1) pose_push(rgba, bg, queue, &tail, idx + w);
    }
    free(queue);
}

static PoseBand pose_band_x(const uint8_t *rgba, int w, int xa, int xb, int ya, int yb)
{
    PoseBand b = {0, (float)w, 0.0f, 0.0f};
    double sum = 0.0;
    for (int y = ya; y < yb; y++)
        for (int x = xa; x < xb; x++) {
            if (!pose_on(rgba, w, x, y)) continue;
            b.count++;
            sum += x;
            if (x < b.minX) b.minX = (float)x;
            if (x > b.maxX) b.maxX = (float)x;
        }
    if (b.count) b.meanX = (float)(sum / b.count);
    return b;
}

static PoseBand pose_band(const uint8_t *rgba, int w, int ya, int yb)
{
    return pose_band_x(rgba, w, 0, w, ya, yb);
}

static PoseBand pose_rows(const uint8_t *rgba, int w, int h, int y0, float H, float fa, float fb)
{
    int ya = (int)pose_limit(y0 + fa * H, 0.0f, (float)(h - 1));
    int yb = (int)pose_limit(y0 + fb * H + 1.0f, (float)(ya + 1), (float)h);
    return pose_band(rgba, w, ya, yb);
}

static float pose_head_center(const uint8_t *rgba, int w, int h, int y0, float H, float radius)
{
    float cx = pose_rows(rgba, w, h, y0, H, 0.0f, 0.03f).meanX;
    for (int it = 0; it < 2; it++) {
        int xa = (int)pose_limit(cx - 1.3f * radius, 0.0f, (float)(w - 1));
        int xb = (int)pose_limit(cx + 1.3f * radius + 1.0f, (float)(xa + 1), (float)w);
        int ya = (int)pose_limit(y0 + 0.3f * radius, 0.0f, (float)(h - 1));
        int yb = (int)pose_limit(y0 + 1.7f * radius + 1.0f, (float)(ya + 1), (float)h);
        PoseBand head = pose_band_x(rgba, w, xa, xb, ya, yb);
        if (head.count) cx = head.meanX;
    }
    return cx;
}

static float pose_head_x(float cx, float radius, float turn, float azimuth)
{
    return cx + radius * sinf(turn + azimuth);
}

static void pose_feet(const uint8_t *rgba, int w, int ya, int yb, float H, float x[2], float bottom[2])
{
    PoseBand b = pose_band(rgba, w, ya, yb);
    float c[2] = {b.minX, b.maxX};
    float low[2] = {(float)ya, (float)ya};
    int n[2] = {0, 0};
    for (int it = 0; it < 8; it++) {
        double sum[2] = {0.0, 0.0};
        n[0] = n[1] = 0;
        low[0] = low[1] = (float)ya;
        for (int y = ya; y < yb; y++)
            for (int px = 0; px < w; px++) {
                if (!pose_on(rgba, w, px, y)) continue;
                int k = fabsf(px - c[0]) > fabsf(px - c[1]);
                sum[k] += px;
                n[k]++;
                if (y > low[k]) low[k] = (float)y;
            }
        for (int k = 0; k < 2; k++)
            if (n[k]) c[k] = (float)(sum[k] / n[k]);
    }
    if (n[0] && n[1] && c[1] - c[0] > 0.06f * H) {
        x[0] = c[0];
        x[1] = c[1];
        bottom[0] = low[0];
        bottom[1] = low[1];
    } else {
        x[0] = b.meanX - 0.03f * H;
        x[1] = b.meanX + 0.03f * H;
        bottom[0] = bottom[1] = fmaxf(low[0], low[1]);
    }
}

static int pose_reach(const uint8_t *rgba, int w, int ya, int yb, int dir, float margin, PosePoint *tip, int *rows)
{
    int ext = -1;
    for (int y = ya; y < yb; y++)
        for (int x = 0; x < w; x++)
            if (pose_on(rgba, w, x, y) && (ext < 0 || (dir > 0 ? x > ext : x < ext))) ext = x;
    if (ext < 0) return 0;
    double sum = 0.0;
    int n = 0;
    *rows = 0;
    for (int y = ya; y < yb; y++) {
        int hit = 0;
        for (int x = 0; x < w; x++)
            if (pose_on(rgba, w, x, y) && abs(x - ext) <= 2.0f * margin) {
                hit = 1;
                if (abs(x - ext) <= margin) {
                    sum += y;
                    n++;
                }
            }
        *rows += hit;
    }
    if (!n) return 0;
    tip->x = ext - dir * margin * 0.5f;
    tip->y = (float)(sum / n);
    return 1;
}

static void pose_set(Pose *o, PoseJoint j, float x, float y)
{
    o->p[j].x = x;
    o->p[j].y = y;
}

static float pose_dist(PosePoint a, PosePoint b)
{
    return hypotf(b.x - a.x, b.y - a.y);
}

static void pose_knee(const uint8_t *rgba, int w, int h, float H, PosePoint hip, PosePoint ankle, PosePoint *knee)
{
    float mx = (hip.x + ankle.x) * 0.5f, my = (hip.y + ankle.y) * 0.5f;
    float lx = pose_limit(mx, 0.0f, (float)(w - 1));
    int y = (int)pose_limit(my, 0.0f, (float)(h - 1));
    int lo = (int)pose_limit(lx - 0.09f * H, 0.0f, (float)(w - 1));
    int hi = (int)pose_limit(lx + 0.09f * H, (float)lo, (float)(w - 1));
    double sum = 0.0;
    int n = 0;
    for (int x = lo; x <= hi; x++)
        if (pose_on(rgba, w, x, y)) {
            sum += x;
            n++;
        }
    knee->x = mx;
    knee->y = my;
    if (!n) return;
    float len = pose_dist(hip, ankle);
    if (len < 1.0f) return;
    float nx = (ankle.y - hip.y) / len, ny = -(ankle.x - hip.x) / len;
    float off = pose_limit(((float)(sum / n) - mx) * nx, -0.08f * H, 0.08f * H);
    knee->x = mx + nx * off;
    knee->y = my + ny * off;
}

static void PoseDetect(const uint8_t *rgba, int w, int h, Pose *out)
{
    const PoseJoint (*arm)[3] = PoseArms;
    const PoseJoint (*leg)[3] = PoseLegs;
    memset(out, 0, sizeof *out);
    int y0 = h, y1 = -1;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (pose_on(rgba, w, x, y)) {
                if (y < y0) y0 = y;
                if (y > y1) y1 = y;
            }
    if (y1 < 0) return;
    float H = (float)(y1 - y0 + 1);
    PoseBand torso = pose_rows(rgba, w, h, y0, H, 0.2f, 0.5f);
    PoseBand pelvis = pose_rows(rgba, w, h, y0, H, 0.45f, 0.55f);
    float headR = POSE_HEAD_RADIUS * H;
    float hx = pose_head_center(rgba, w, h, y0, H, headR);
    float tx = torso.count ? torso.meanX : hx;
    float px = pelvis.count ? pelvis.meanX : hx;
    float yaw = pose_limit((hx - px) / (POSE_YAW_RANGE * H), -1.0f, 1.0f);
    float turn = yaw * POSE_HALF_PI;
    int face = fabsf(yaw) > 0.15f ? (yaw > 0.0f ? 1 : -1) : 0;
    float headY = y0 + headR;
    float neckX = (hx + tx) * 0.5f, neckY = y0 + 2.0f * headR;
    float hipW = (0.075f - 0.03f * fabsf(yaw)) * H;
    float hipY = y0 + 0.5f * H;
    float eyeY = headY - 0.05f * headR;
    float earY = headY + 0.15f * headR;
    float noseY = headY + 0.3f * headR;

    pose_set(out, POSE_NOSE, pose_head_x(hx, 1.1f * headR, turn, 0.0f), noseY);
    pose_set(out, POSE_LEYE, pose_head_x(hx, 0.9f * headR, turn, POSE_EYE_AZIMUTH), eyeY);
    pose_set(out, POSE_REYE, pose_head_x(hx, 0.9f * headR, turn, -POSE_EYE_AZIMUTH), eyeY);
    pose_set(out, POSE_LEAR, pose_head_x(hx, 0.95f * headR, turn, POSE_EAR_AZIMUTH), earY);
    pose_set(out, POSE_REAR, pose_head_x(hx, 0.95f * headR, turn, -POSE_EAR_AZIMUTH), earY);
    pose_set(out, POSE_NECK, neckX, neckY);

    float footX[2], footBottom[2];
    int feetTop = (int)pose_limit(y1 - 0.12f * H, (float)y0, (float)y1);
    pose_feet(rgba, w, feetTop, y1 + 1, H, footX, footBottom);

    for (int k = 0; k < 2; k++) {
        int dir = k == 0 ? 1 : -1;
        int foot = 1 - k;
        pose_set(out, leg[k][0], tx + dir * hipW, hipY);
        float ankleY = fmaxf(footBottom[foot] - 0.05f * H, hipY + 0.1f * H);
        pose_set(out, leg[k][2], footX[foot], ankleY);
        PosePoint knee;
        pose_knee(rgba, w, h, H, out->p[leg[k][0]], out->p[leg[k][2]], &knee);
        pose_set(out, leg[k][1], knee.x, knee.y);

        float shoulderW = (0.17f - (dir == face ? 0.12f : 0.06f) * fabsf(yaw)) * H;
        PosePoint s = {neckX + dir * shoulderW, neckY + 0.02f * H, 0.0f, 0.0f}, tip;
        pose_set(out, arm[k][0], s.x, s.y);
        int ya = (int)pose_limit(neckY - 0.03f * H, 0.0f, (float)(h - 1));
        int yb = (int)pose_limit(hipY, (float)(ya + 1), (float)h);
        int rows = 0;
        int extended = pose_reach(rgba, w, ya, yb, dir, 0.04f * H, &tip, &rows) &&
                       dir * (tip.x - neckX) > shoulderW + 0.06f * H &&
                       rows <= 0.16f * H;
        out->solve[k] = extended;
        if (extended) {
            pose_set(out, arm[k][1], (s.x + tip.x) * 0.5f, (s.y + tip.y) * 0.5f + 0.03f * H);
            pose_set(out, arm[k][2], tip.x, tip.y);
        } else if (face) {
            float front = dir == face ? 1.0f : 0.0f;
            pose_set(out, arm[k][1], s.x + face * 0.04f * front * H, s.y + (0.17f + 0.05f * front) * H);
            pose_set(out, arm[k][2], neckX + face * (0.05f + 0.07f * front) * H, neckY + (0.05f + 0.1f * front) * H);
        } else {
            pose_set(out, arm[k][1], s.x + dir * 0.015f * H, s.y + 0.14f * H);
            pose_set(out, arm[k][2], s.x + dir * 0.025f * H, s.y + 0.27f * H);
        }
    }
    for (int j = 0; j < POSE_JOINTS; j++) {
        out->p[j].x = pose_limit(out->p[j].x, 0.0f, (float)(w - 1));
        out->p[j].y = pose_limit(out->p[j].y, 0.0f, (float)(h - 1));
    }
    out->yaw = yaw;
    out->height = H;
    out->valid = 1;
}

static void pose_solve_arm(Pose *o, const PoseJoint chain[3], float length)
{
    PosePoint root = o->p[chain[0]], end = o->p[chain[2]];
    float raw = pose_dist(root, end);
    float ux = raw > 0.001f ? (end.x - root.x) / raw : 0.0f;
    float uy = raw > 0.001f ? (end.y - root.y) / raw : 1.0f;
    float a = length * POSE_UPPER_ARM, b = length - a;
    float d = pose_limit(raw, fabsf(a - b) + 0.001f, length);
    float along = (d * d + a * a - b * b) / (2.0f * d);
    float off = sqrtf(fmaxf(a * a - along * along, 0.0f));
    float side = sinf(o->yaw * POSE_HALF_PI);
    float wx = -0.3f * side, wy = 1.0f;
    float px = -uy, py = ux;
    if (px * wx + py * wy < 0.0f) {
        px = -px;
        py = -py;
    }
    o->p[chain[2]].x = root.x + ux * d;
    o->p[chain[2]].y = root.y + uy * d;
    o->p[chain[1]].x = root.x + ux * along + px * off;
    o->p[chain[1]].y = root.y + uy * along + py * off;
}

static float pose_torso(const Pose *p)
{
    PosePoint pelvis = { (p->p[POSE_LHIP].x + p->p[POSE_RHIP].x) * 0.5f, (p->p[POSE_LHIP].y + p->p[POSE_RHIP].y) * 0.5f, 0.0f, 0.0f };
    return pose_dist(p->p[POSE_NECK], pelvis);
}

static float pose_torso_3d(const Pose *p)
{
    float dz = p->zModel[POSE_NECK] - (p->zModel[POSE_LHIP] + p->zModel[POSE_RHIP]) * 0.5f;
    return hypotf(pose_torso(p), dz);
}

static float pose_bone_3d(const Pose *p, PoseJoint a, PoseJoint b)
{
    return hypotf(pose_dist(p->p[a], p->p[b]), p->zModel[b] - p->zModel[a]);
}

static void pose_fit_spine(Pose *o, float length)
{
    float planar = pose_torso(o);
    float pelvis = (o->zModel[POSE_LHIP] + o->zModel[POSE_RHIP]) * 0.5f;
    float sign = o->zModel[POSE_NECK] >= pelvis ? 1.0f : -1.0f;
    float lean = sign * sqrtf(fmaxf(length * length - planar * planar, 0.0f));
    float shift = pelvis + lean - o->zModel[POSE_NECK];
    for (size_t i = 0; i < sizeof PoseUpperBody / sizeof PoseUpperBody[0]; i++)
        o->zModel[PoseUpperBody[i]] += shift;
}

static void pose_fit_head(Pose *o, float torso)
{
    float reach = POSE_HEAD_REACH * torso;
    float offset = o->zModel[POSE_NOSE] - o->zModel[POSE_NECK];
    float shift = pose_limit(offset, -reach, reach) - offset;
    for (size_t i = 0; i < sizeof PoseHead / sizeof PoseHead[0]; i++)
        o->zModel[PoseHead[i]] += shift;
}

static void pose_face_away(Pose *o)
{
    float ears = (o->zModel[POSE_LEAR] + o->zModel[POSE_REAR]) * 0.5f;
    if (o->zModel[POSE_NOSE] >= ears) return;
    float pelvis = (o->zModel[POSE_LHIP] + o->zModel[POSE_RHIP]) * 0.5f;
    for (int j = 0; j < POSE_JOINTS; j++)
        o->zModel[j] = 2.0f * pelvis - o->zModel[j];
}

static PoseVec pose_add(PoseVec a, PoseVec b)
{
    return (PoseVec){ a.x + b.x, a.y + b.y, a.z + b.z };
}

static PoseVec pose_sub(PoseVec a, PoseVec b)
{
    return (PoseVec){ a.x - b.x, a.y - b.y, a.z - b.z };
}

static PoseVec pose_mul(PoseVec a, float s)
{
    return (PoseVec){ a.x * s, a.y * s, a.z * s };
}

static float pose_dot(PoseVec a, PoseVec b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static PoseVec pose_cross(PoseVec a, PoseVec b)
{
    return (PoseVec){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

static PoseVec pose_unit(PoseVec v, PoseVec fallback)
{
    float n = sqrtf(pose_dot(v, v));
    return n > 0.001f ? pose_mul(v, 1.0f / n) : fallback;
}

static PoseVec pose_joint(const Pose *o, PoseJoint j)
{
    return (PoseVec){ o->p[j].x, o->p[j].y, o->zModel[j] };
}

static PoseVec pose_mid(const Pose *o, PoseJoint a, PoseJoint b)
{
    return pose_mul(pose_add(pose_joint(o, a), pose_joint(o, b)), 0.5f);
}

static void pose_place(Pose *o, PoseJoint j, PoseVec v)
{
    o->p[j].x = v.x;
    o->p[j].y = v.y;
    o->zModel[j] = v.z;
}

static PoseBody pose_body(const Pose *o)
{
    PoseVec down = pose_unit(pose_sub(pose_mid(o, POSE_LHIP, POSE_RHIP), pose_joint(o, POSE_NECK)), (PoseVec){ 0.0f, 1.0f, 0.0f });
    PoseVec across = pose_sub(pose_joint(o, POSE_LSHOULDER), pose_joint(o, POSE_RSHOULDER));
    PoseVec forward = pose_unit(pose_cross(across, down), (PoseVec){ 0.0f, 0.0f, 1.0f });
    PoseVec gaze = pose_sub(pose_joint(o, POSE_NOSE), pose_mid(o, POSE_LEAR, POSE_REAR));
    if (pose_dot(forward, gaze) < 0.0f) forward = pose_mul(forward, -1.0f);
    return (PoseBody){ forward, down, pose_unit(pose_cross(forward, down), (PoseVec){ 1.0f, 0.0f, 0.0f }) };
}

static PoseVec pose_swing(PoseVec v, PoseVec from, PoseVec to, PoseVec half)
{
    PoseVec axis = pose_cross(from, to);
    float w = 1.0f + pose_dot(from, to);
    if (w < 0.0001f) {
        axis = half;
        w = 0.0f;
    }
    float n = sqrtf(w * w + pose_dot(axis, axis));
    axis = pose_mul(axis, 1.0f / n);
    w /= n;
    PoseVec t = pose_mul(pose_cross(axis, v), 2.0f);
    return pose_add(pose_add(v, pose_mul(t, w)), pose_cross(axis, t));
}

static PoseVec pose_bone_end(PoseVec parent, PoseVec child, float length)
{
    float dx = child.x - parent.x, dy = child.y - parent.y;
    float planar = hypotf(dx, dy);
    if (planar >= length) {
        float s = length / planar;
        return (PoseVec){ parent.x + dx * s, parent.y + dy * s, parent.z };
    }
    float sign = child.z >= parent.z ? 1.0f : -1.0f;
    return (PoseVec){ child.x, child.y, parent.z + sign * sqrtf(length * length - planar * planar) };
}

static void pose_fit_limb(Pose *o, const PoseJoint chain[3], float upper, float lower, const PoseBody *body, float flexion, float twist)
{
    if (upper <= 0.0f || lower <= 0.0f) return;
    PoseVec root = pose_joint(o, chain[0]);
    PoseVec elbow = pose_bone_end(root, pose_joint(o, chain[1]), upper);
    PoseVec wrist = pose_bone_end(elbow, pose_joint(o, chain[2]), lower);
    PoseVec reach = pose_sub(wrist, root);
    float fold = sqrtf(upper * upper + lower * lower - 2.0f * upper * lower * POSE_FOLD_COS);
    float d = pose_limit(sqrtf(pose_dot(reach, reach)), fold, upper + lower);
    PoseVec u = pose_unit(reach, body->down);
    PoseVec nominal = pose_swing(pose_mul(body->forward, flexion), body->down, u, body->side);
    PoseVec lateral = pose_cross(u, nominal);
    PoseVec seen = pose_sub(elbow, root);
    float sx = pose_dot(seen, nominal), sy = pose_dot(seen, lateral);
    float angle = sx * sx + sy * sy > 0.000001f ? pose_limit(atan2f(sy, sx), -twist, twist) : 0.0f;
    PoseVec bend = pose_add(pose_mul(nominal, cosf(angle)), pose_mul(lateral, sinf(angle)));
    float along = (d * d + upper * upper - lower * lower) / (2.0f * d);
    float off = sqrtf(fmaxf(upper * upper - along * along, 0.0f));
    pose_place(o, chain[1], pose_add(root, pose_add(pose_mul(u, along), pose_mul(bend, off))));
    pose_place(o, chain[2], pose_add(root, pose_mul(u, d)));
}

static void pose_fit_bones(Pose *o, const PoseLimbs *limbs)
{
    pose_face_away(o);
    pose_fit_spine(o, limbs->torso);
    pose_fit_head(o, limbs->torso);
    PoseBody body = pose_body(o);
    for (int k = 0; k < 2; k++) {
        pose_fit_limb(o, PoseArms[k], limbs->bone[0], limbs->bone[1], &body, -1.0f, POSE_ARM_TWIST);
        pose_fit_limb(o, PoseLegs[k], limbs->bone[2], limbs->bone[3], &body, 1.0f, POSE_LEG_TWIST);
    }
    if (limbs->flips.head)
        for (size_t i = 0; i < sizeof PoseHead / sizeof PoseHead[0]; i++)
            o->zModel[PoseHead[i]] = 2.0f * o->zModel[POSE_NECK] - o->zModel[PoseHead[i]];
}

static void PoseFitLimbs(Pose *o, const PoseLimbs *limbs)
{
    if (!o->valid) return;
    if (o->hasZModel) {
        pose_fit_bones(o, limbs);
        return;
    }
    if (limbs->arm <= 0.0f) return;
    for (int k = 0; k < 2; k++)
        if (o->solve[k]) pose_solve_arm(o, PoseArms[k], limbs->arm);
}

static void PoseAdoptDepth(Pose *pose, int imageHeight)
{
    for (int j = 0; j < POSE_JOINTS; j++)
        pose->zModel[j] = (pose->p[j].z - 0.5f - pose->p[j].dz) * imageHeight;
    pose->hasZModel = 1;
}

static PoseFlips PoseLearnFlips(const Pose *edited, const Pose *fresh)
{
    PoseFlips toggled = { 0 };
    if (!edited->valid || !fresh->valid || !fresh->hasZModel) return toggled;
    float headBefore = fresh->p[POSE_NOSE].z - fresh->p[POSE_NECK].z;
    float headAfter = edited->p[POSE_NOSE].z - edited->p[POSE_NECK].z;
    toggled.head = headBefore * headAfter < 0.0f && fabsf(headAfter) > POSE_FLIP_MIN;
    return toggled;
}

static PoseLimbs PoseMeasureLimbs(const Pose *poses, int count)
{
    PoseLimbs l = { .arm = 0.0f };
    float tall = 0.0f;
    int modeled = 0;
    for (int i = 0; i < count; i++) {
        if (!poses[i].valid) continue;
        tall = fmaxf(tall, poses[i].height);
        for (int k = 0; k < 2; k++)
            l.arm = fmaxf(l.arm, pose_dist(poses[i].p[PoseArms[k][0]], poses[i].p[PoseArms[k][2]]));
        if (!poses[i].hasZModel) continue;
        modeled++;
        l.torso += pose_torso_3d(&poses[i]);
        for (int kind = 0; kind < POSE_BONE_KINDS; kind++)
            for (int k = 0; k < 2; k++)
                l.bone[kind] += 0.5f * pose_bone_3d(&poses[i], PoseBoneChain[kind][k][0], PoseBoneChain[kind][k][1]);
    }
    if (modeled) {
        l.torso /= modeled;
        for (int kind = 0; kind < POSE_BONE_KINDS; kind++) l.bone[kind] /= modeled;
    }
    for (int kind = 0; kind < POSE_BONE_KINDS; kind++)
        l.bone[kind] = fmaxf(l.bone[kind], PoseBoneTorsoRatio[kind] * l.torso);
    l.arm = fmaxf(l.arm * POSE_LIMB_MARGIN, POSE_LIMB_MIN * POSE_ARM_RATIO * tall);
    return l;
}

static PoseLimbs PoseReferenceLimbs(const Pose *pose)
{
    PoseLimbs l = { .arm = 0.0f };
    l.torso = pose_torso_3d(pose);
    for (int kind = 0; kind < POSE_BONE_KINDS; kind++)
        for (int k = 0; k < 2; k++)
            l.bone[kind] += 0.5f * pose_bone_3d(pose, PoseBoneChain[kind][k][0], PoseBoneChain[kind][k][1]);
    for (int k = 0; k < 2; k++)
        l.arm = fmaxf(l.arm, pose_dist(pose->p[PoseArms[k][0]], pose->p[PoseArms[k][2]]));
    l.arm = fmaxf(l.arm * POSE_LIMB_MARGIN, POSE_LIMB_MIN * POSE_ARM_RATIO * pose->height);
    return l;
}

static int PoseDepthSize(int n)
{
    return (n + POSE_DEPTH_STEP - 1) / POSE_DEPTH_STEP;
}

static size_t PoseDepthCells(int w, int h)
{
    return (size_t)PoseDepthSize(w) * PoseDepthSize(h);
}

static float PoseIntegrateDepth(const uint8_t *normals, const uint8_t *rgba, int w, int h, float *depth)
{
    int dw = PoseDepthSize(w);
    int dh = PoseDepthSize(h);
    size_t cells = (size_t)dw * dh;
    float *grad = calloc(cells * 4, sizeof(float));
    memset(depth, 0, cells * sizeof(float));
    if (!grad) return 0.0f;
    float *gx = grad, *gy = grad + cells, *div = grad + 2 * cells, *solid = grad + 3 * cells;
    for (int cy = 0; cy < dh; cy++)
        for (int cx = 0; cx < dw; cx++) {
            double sx = 0.0, sy = 0.0;
            int n = 0;
            for (int y = cy * POSE_DEPTH_STEP; y < (cy + 1) * POSE_DEPTH_STEP && y < h; y++)
                for (int x = cx * POSE_DEPTH_STEP; x < (cx + 1) * POSE_DEPTH_STEP && x < w; x++) {
                    if (!pose_on(rgba, w, x, y)) continue;
                    const uint8_t *p = normals + ((size_t)y * w + x) * 4;
                    float nz = fmaxf(p[2] / 255.0f * 2.0f - 1.0f, 0.1f);
                    sx += -(p[0] / 255.0f * 2.0f - 1.0f) / nz;
                    sy += (p[1] / 255.0f * 2.0f - 1.0f) / nz;
                    n++;
                }
            if (!n) continue;
            solid[cy * dw + cx] = 1.0f;
            gx[cy * dw + cx] = (float)(sx / n) * POSE_DEPTH_STEP;
            gy[cy * dw + cx] = (float)(sy / n) * POSE_DEPTH_STEP;
        }
    for (int cy = 0; cy < dh; cy++)
        for (int cx = 0; cx < dw; cx++) {
            int i = cy * dw + cx;
            float right = cx + 1 < dw ? (gx[i] + gx[i + 1]) * 0.5f : 0.0f;
            float left = cx > 0 ? (gx[i] + gx[i - 1]) * 0.5f : 0.0f;
            float down = cy + 1 < dh ? (gy[i] + gy[i + dw]) * 0.5f : 0.0f;
            float up = cy > 0 ? (gy[i] + gy[i - dw]) * 0.5f : 0.0f;
            div[i] = right - left + down - up;
        }
    for (int it = 0; it < POSE_DEPTH_ITERATIONS; it++)
        for (int cy = 0; cy < dh; cy++)
            for (int cx = 0; cx < dw; cx++) {
                int i = cy * dw + cx;
                float sum = (cx + 1 < dw ? depth[i + 1] : depth[i]) + (cx > 0 ? depth[i - 1] : depth[i]) +
                            (cy + 1 < dh ? depth[i + dw] : depth[i]) + (cy > 0 ? depth[i - dw] : depth[i]);
                depth[i] += POSE_DEPTH_RELAXATION * ((sum - div[i]) * 0.25f - depth[i]);
            }
    float lo = INFINITY, hi = -INFINITY;
    for (size_t i = 0; i < cells; i++) {
        if (solid[i] == 0.0f) continue;
        lo = fminf(lo, depth[i]);
        hi = fmaxf(hi, depth[i]);
    }
    free(grad);
    return hi > lo ? hi - lo : 0.0f;
}

static void PoseApplyDepth(Pose *pose, const float *depth, int w, int h, float span)
{
    if (!pose->valid) return;
    if (pose->hasZModel) {
        for (int j = 0; j < POSE_JOINTS; j++)
            pose->p[j].z = 0.5f + pose->zModel[j] / h + pose->p[j].dz;
        return;
    }
    if (span <= 0.0f) return;
    int dw = PoseDepthSize(w);
    int dh = PoseDepthSize(h);
    float d[POSE_JOINTS];
    for (int j = 0; j < POSE_JOINTS; j++) {
        int cx = (int)pose_limit(pose->p[j].x / POSE_DEPTH_STEP, 0.0f, (float)(dw - 1));
        int cy = (int)pose_limit(pose->p[j].y / POSE_DEPTH_STEP, 0.0f, (float)(dh - 1));
        d[j] = depth[cy * dw + cx];
    }
    float hip = (d[POSE_LHIP] + d[POSE_RHIP]) * 0.5f;
    for (int j = 0; j < POSE_JOINTS; j++)
        pose->p[j].z = 0.5f - POSE_Z_RANGE * (d[j] - hip) / span + pose->p[j].dz;
}

static void PoseDepthImage(const float *depth, const uint8_t *rgba, int w, int h, uint8_t *out)
{
    int dw = PoseDepthSize(w);
    int dh = PoseDepthSize(h);
    float lo = INFINITY, hi = -INFINITY;
    for (int cy = 0; cy < dh; cy++)
        for (int cx = 0; cx < dw; cx++) {
            if (!pose_on(rgba, w, cx * POSE_DEPTH_STEP, cy * POSE_DEPTH_STEP)) continue;
            lo = fminf(lo, depth[cy * dw + cx]);
            hi = fmaxf(hi, depth[cy * dw + cx]);
        }
    float span = hi > lo ? hi - lo : 1.0f;
    for (int cy = 0; cy < dh; cy++)
        for (int cx = 0; cx < dw; cx++) {
            int solid = pose_on(rgba, w, cx * POSE_DEPTH_STEP, cy * POSE_DEPTH_STEP);
            uint8_t g = solid ? (uint8_t)(pose_limit((depth[cy * dw + cx] - lo) / span, 0.0f, 1.0f) * 255.0f) : 0;
            uint8_t *p = out + ((size_t)cy * dw + cx) * 4;
            p[0] = p[1] = p[2] = g;
            p[3] = solid ? 255 : 0;
        }
}

#endif
