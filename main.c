
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "pose.h"
#include "pose_model.h"
#include "body_model.h"
#include "pose_export.h"
#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define WIN_W 1280
#define WIN_H 800

#define BOARD_HEIGHT 3.5f
#define MAX_BOARD_WIDTH 5.5f
#define LIGHT_RADIUS 3.8f
#define LIGHT_DEPTH 2.0f
#define DRAG_LIGHT_DISTANCE 2.0f
#define GIF_FPS 10.0f
#define EXPORT_EMPTY_FRAMES_DEFAULT 2
#define EXPORT_EMPTY_FRAMES_MAX 10
#define EXPORT_GAP_WIDTH 170.0f
#define EXPORT_OVERFLOW 0.12f
#define EXPORT_GAP_BUTTON 28.0f
#define EXPORT_TOGGLE_WIDTH 116.0f
#define EXPORT_TOGGLE_SPACING 6.0f
#define EXPORT_TOGGLE_MARGIN 12.0f
#define INSPECTOR_SPAN_DEFAULT 3
#define INSPECTOR_SPAN_MAX 30
#define ONION_MAX 8
#define ONION_DEFAULT 2
#define ONION_ALPHA 255
#define BOTTOM_ROW_HEIGHT 20.0f
#define BOTTOM_ROW_RIGHT 96.0f
#define BOTTOM_SPACING 8.0f
#define ONION_OPACITY_DEFAULT 60
#define ONION_OPACITY_MIN 10
#define ONION_OPACITY_MAX 100
#define ONION_OPACITY_STEP 10
#define POSE_PICK_RADIUS 12.0f
#define LIGHT_COUNT 3
#define OPPOSITE_LIGHT_ANGLE 3.14159265f
#define SCENE_TOP 112.0f
#define SCENE_BOTTOM 94.0f
#define SIDE_PANEL_RATIO 0.3f
#define SIDE_Z_MARGIN 1.1f
#define SIDE_ROTATE_SPEED 0.01f
#define SIDE_RETURN_RATE 14.0f
#define SIDE_YAW_EPSILON 0.01f
#define SIDE_HALF_TURN 3.14159265f
#define SIDE_BACK_TINT (Color){ 0, 0, 0, 255 }
#define BODY_ALPHA 110
#define BODY_UPPER_ARM_RADIUS 0.04f
#define BODY_FOREARM_RADIUS 0.032f
#define BODY_THIGH_RADIUS 0.06f
#define BODY_CALF_RADIUS 0.045f
#define BODY_TORSO_RADIUS 0.075f

#define TEXT_TITLE 28.0f
#define TEXT_HEADING 18.0f
#define TEXT_BODY 16.0f
#define TEXT_SMALL 14.0f
#define UI_SCALE_MIN 0.75f
#define UI_SCALE_MAX 4.0f
#define UI_FONT_BASE_SIZE 64
#define UI_TEXT_SPACING 0.05f

static Font uiFont = { 0 };
static float uiScale = 1.0f;

static Texture2D originalTexture = { 0 };
static Texture2D normalTexture = { 0 };
static Shader lightShader = { 0 };

static Image sprite = { 0 };
static unsigned char *normalFrames = NULL;
static float *depthMaps = NULL;
static float depthSpan = 0.0f;
static unsigned char *depthPixels = NULL;
static Texture2D depthTexture = { 0 };
static Pose *poses = NULL;
static PoseLimbs limbs = { 0 };
static PoseFlips *frameFlips = NULL;
static int frameCount = 0;
static int frameIndex = 0;
static float frameClock = 0.0f;
static int dragJoint = -1;
static bool dragSide = false;
static bool sideRotating = false;
static float sideYaw = 0.0f;

static float SideAngle(void)
{
    return sideYaw + SIDE_HALF_TURN;
}
static bool scrubbing = false;
static char spritePath[512] = "";

static bool hasImage = false;
static bool showNormals = false;
static bool autoLight = true;
static bool playing = true;
static bool poseMode = false;
static PoseExportOptions exportOptions = {
    .ground = 1,
    .aspect = 1,
    .smooth = 2,
    .keys = 0,
    .emptyFrames = EXPORT_EMPTY_FRAMES_DEFAULT
};
static bool showDepth = false;
static int exportPreview = 0;
static int poseRevision = 0;
static PoseExportResult exportResult = { 0 };
static PoseExportOptions exportBuilt = { 0 };
static int exportBuiltRevision = -1;

typedef enum {
    VIEW_ONION,
    VIEW_BONES,
    VIEW_BOTH,
    VIEW_COUNT
} ViewMode;

static ViewMode viewMode = VIEW_ONION;
static int onionOpacity = ONION_OPACITY_DEFAULT;
static bool inspectorOn = false;
static int impactFrame = -1;
static int inspectorBefore = INSPECTOR_SPAN_DEFAULT;
static int inspectorAfter = INSPECTOR_SPAN_DEFAULT;
static Texture2D onionTextures[2 * ONION_MAX];
static int onionCount = ONION_DEFAULT;
static int onionKeyFrame = -1;
static int onionKeyCount = -1;

static float lightStrength = 1.5f;
static float lightSpeed = 1.0f;
static float lightAngle = 0.0f;
static float verticalLightAngle = 0.0f;

static float cameraYaw = 0.65f;
static float cameraPitch = 0.32f;
static float cameraDistance = 11.5f;

static Vector2 imageSize = { 1.0f, 1.0f };
static Vector2 billboardSize = { 3.5f, 3.5f };

static const Vector3 BOARD_CENTER = { 0.0f, 2.0f, 0.0f };

static char statusText[256] =
    "Drag an image or GIF into the window or paste its path with Ctrl+V";

static PoseModel poseModel = { 0 };
static bool poseModelReady = false;
static BodyModel bodyModel = { 0 };
static bool bodyModelReady = false;

static const char *fragmentShader =
    "#version 330\n"
    "in vec2 fragTexCoord;\n"
    "in vec4 fragColor;\n"
    "uniform sampler2D texture0;\n"
    "uniform sampler2D normalMap;\n"
    "uniform vec3 lightLocal1;\n"
    "uniform vec3 lightLocal2;\n"
    "uniform vec3 lightColor1;\n"
    "uniform vec3 lightColor2;\n"
    "uniform vec3 lightLocal3;\n"
    "uniform vec3 lightColor3;\n"
    "uniform vec2 boardSize;\n"
    "uniform float strength;\n"
    "out vec4 finalColor;\n"
    "void main()\n"
    "{\n"
    "    vec4 albedo = texture(texture0, fragTexCoord);\n"
    "    if (albedo.a < 0.01) discard;\n"
    "    vec3 n = texture(normalMap, fragTexCoord).rgb * 2.0 - 1.0;\n"
    "    n = normalize(n);\n"
    "    vec3 surface = vec3(\n"
    "        (fragTexCoord.x - 0.5) * boardSize.x,\n"
    "        (0.5 - fragTexCoord.y) * boardSize.y,\n"
    "        0.0\n"
    "    );\n"
    "    vec3 delta1 = lightLocal1 - surface;\n"
    "    vec3 delta2 = lightLocal2 - surface;\n"
    "    float dist1 = max(dot(delta1, delta1), 0.001);\n"
    "    float dist2 = max(dot(delta2, delta2), 0.001);\n"
    "    vec3 L1 = normalize(delta1);\n"
    "    vec3 L2 = normalize(delta2);\n"
    "    float diffuse1 = max(dot(n, L1), 0.0);\n"
    "    float diffuse2 = max(dot(n, L2), 0.0);\n"
    "    float atten1 = 1.0 / (1.0 + 0.08 * dist1);\n"
    "    float atten2 = 1.0 / (1.0 + 0.08 * dist2);\n"
    "    vec3 delta3 = lightLocal3 - surface;\n"
    "    float dist3 = max(dot(delta3, delta3), 0.001);\n"
    "    vec3 L3 = normalize(delta3);\n"
    "    float diffuse3 = max(dot(n, L3), 0.0);\n"
    "    float atten3 = 1.0 / (1.0 + 0.08 * dist3);\n"
    "    vec3 lighting = vec3(0.18);\n"
    "    lighting += strength * diffuse1 * atten1 * lightColor1;\n"
    "    lighting += strength * diffuse2 * atten2 * lightColor2;\n"
    "    lighting += strength * diffuse3 * atten3 * lightColor3;\n"
    "    lighting = clamp(lighting, vec3(0.0), vec3(1.8));\n"
    "    finalColor = vec4(albedo.rgb * lighting, albedo.a) * fragColor;\n"
    "}\n";

static float Ui(float value)
{
    return value * uiScale;
}

static void UpdateUiScale(void)
{
    float scale = fminf(
        (float)GetScreenWidth() / WIN_W,
        (float)GetScreenHeight() / WIN_H
    );

    uiScale = Clamp(scale, UI_SCALE_MIN, UI_SCALE_MAX);
}

static bool UiFontIsCustom(void)
{
    return uiFont.texture.id != GetFontDefault().texture.id;
}

static void LoadUiFont(void)
{
    static const char *const candidates[] = {
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf"
    };

    uiFont = GetFontDefault();

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++)
    {
        if (!FileExists(candidates[i]))
            continue;

        Font font = LoadFontEx(candidates[i], UI_FONT_BASE_SIZE, NULL, 0);

        if (font.texture.id == GetFontDefault().texture.id)
            continue;

        GenTextureMipmaps(&font.texture);
        SetTextureFilter(font.texture, TEXTURE_FILTER_TRILINEAR);
        uiFont = font;
        return;
    }
}

static float MeasureUiText(const char *text, float size)
{
    float fontSize = Ui(size);

    return MeasureTextEx(uiFont, text, fontSize, fontSize * UI_TEXT_SPACING).x;
}

static void DrawUiText(const char *text, float x, float y, float size, Color color)
{
    float fontSize = Ui(size);

    DrawTextEx(
        uiFont, text, (Vector2){ floorf(x), floorf(y) },
        fontSize, fontSize * UI_TEXT_SPACING, color
    );
}

static void DrawUiTextRight(const char *text, float right, float y, float size, Color color)
{
    DrawUiText(text, right - MeasureUiText(text, size), y, size, color);
}

static float GetHeight(Color c)
{
    return (0.299f * c.r +
            0.587f * c.g +
            0.114f * c.b) / 255.0f;
}

static Image GenerateNormalMap(Image src)
{
    int w = src.width;
    int h = src.height;

    Image result = GenImageColor(
        w, h, (Color){ 128, 128, 255, 255 }
    );

    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < w; x++)
        {
            int xl = (x > 0) ? x - 1 : x;
            int xr = (x < w - 1) ? x + 1 : x;
            int yu = (y > 0) ? y - 1 : y;
            int yd = (y < h - 1) ? y + 1 : y;

            float left = GetHeight(GetImageColor(src, xl, y));
            float right = GetHeight(GetImageColor(src, xr, y));
            float up = GetHeight(GetImageColor(src, x, yu));
            float down = GetHeight(GetImageColor(src, x, yd));

            float nx = (left - right) * 7.0f;
            float ny = (down - up) * 7.0f;
            float nz = 1.0f;

            float len = sqrtf(nx * nx + ny * ny + nz * nz);

            nx /= len;
            ny /= len;
            nz /= len;

            Color c = {
                (unsigned char)((nx * 0.5f + 0.5f) * 255.0f),
                (unsigned char)((ny * 0.5f + 0.5f) * 255.0f),
                (unsigned char)((nz * 0.5f + 0.5f) * 255.0f),
                255
            };

            ImageDrawPixel(&result, x, y, c);
        }
    }

    return result;
}

static Image FrameView(void *base, int w, int h, int index)
{
    return (Image){
        (unsigned char *)base + (size_t)w * h * 4 * index,
        w, h, 1,
        PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
    };
}

static void UpdateDepthTexture(int index)
{
    PoseDepthImage(
        depthMaps + PoseDepthCells(sprite.width, sprite.height) * index,
        FrameView(sprite.data, sprite.width, sprite.height, index).data,
        sprite.width, sprite.height, depthPixels
    );

    UpdateTexture(depthTexture, depthPixels);
}

static void ShowFrame(int index)
{
    frameIndex = index;

    UpdateTexture(
        originalTexture,
        FrameView(sprite.data, sprite.width, sprite.height, index).data
    );

    UpdateTexture(
        normalTexture,
        FrameView(normalFrames, sprite.width, sprite.height, index).data
    );

    UpdateDepthTexture(index);
}

static Rectangle TimelineRect(void)
{
    return (Rectangle){ Ui(24.0f), Ui(98.0f), (float)GetScreenWidth() - Ui(48.0f), Ui(10.0f) };
}

static bool InspectorActive(void)
{
    return poseMode && inspectorOn && impactFrame >= 0 && frameCount > 1;
}

static void InspectorRange(int *lo, int *hi)
{
    *lo = impactFrame - inspectorBefore;
    *hi = impactFrame + inspectorAfter;

    if (*lo < 0)
        *lo = 0;

    if (*hi > frameCount - 1)
        *hi = frameCount - 1;
}

static bool InspectorContains(int index)
{
    int lo, hi;

    if (!InspectorActive())
        return false;

    InspectorRange(&lo, &hi);

    return index >= lo && index <= hi;
}

static int NextFrameIndex(int direction)
{
    if (!InspectorActive())
        return (frameIndex + direction + frameCount) % frameCount;

    int lo, hi;

    InspectorRange(&lo, &hi);

    if (frameIndex < lo || frameIndex > hi)
        return frameIndex < lo ? lo : hi;

    int next = frameIndex + direction;

    if (next > hi)
        return lo;

    if (next < lo)
        return hi;

    return next;
}

static void StepFrame(int direction)
{
    playing = false;
    ShowFrame(NextFrameIndex(direction));
}

static bool ExportMarks(void)
{
    return poseMode && exportPreview && exportResult.count == frameCount;
}

static int TimelineStep(void)
{
    return ExportMarks() ? exportOptions.emptyFrames + 1 : 1;
}

static int TimelineSlots(void)
{
    return (frameCount - 1) * TimelineStep() + 1;
}

static void UpdateTimeline(void)
{
    Rectangle bar = TimelineRect();
    Vector2 mouse = GetMousePosition();
    Rectangle hit = { bar.x, bar.y - Ui(6.0f), bar.width, bar.height + Ui(12.0f) };

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouse, hit))
        scrubbing = true;

    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        scrubbing = false;

    if (!scrubbing)
        return;

    int slots = TimelineSlots();
    int index = (int)((mouse.x - bar.x) / bar.width * slots) / TimelineStep();
    index = index < 0 ? 0 : (index >= frameCount ? frameCount - 1 : index);

    if (InspectorActive())
    {
        int lo, hi;

        InspectorRange(&lo, &hi);
        index = index < lo ? lo : (index > hi ? hi : index);
    }

    if (index != frameIndex)
    {
        playing = false;
        ShowFrame(index);
    }
}

static void DrawTimeline(void)
{
    Rectangle bar = TimelineRect();
    int slots = TimelineSlots();
    int step = TimelineStep();
    float cell = bar.width / slots;

    for (int i = 0; i < slots; i++)
    {
        bool filled = i % step == 0;
        int frame = i / step;
        Color color = (Color){ 60, 70, 90, 255 };

        if (ExportMarks())
            color = !filled ? (Color){ 28, 32, 42, 255 }
                : (exportResult.keep[frame] ? (Color){ 70, 160, 120, 255 } : (Color){ 42, 48, 62, 255 });

        if (!exportPreview && filled && frame == impactFrame)
            color = SKYBLUE;
        else if (!exportPreview && filled && InspectorContains(frame))
            color = (Color){ 90, 110, 140, 255 };

        DrawRectangleRec(
            (Rectangle){ bar.x + i * cell + Ui(1.0f), bar.y, cell - Ui(2.0f), bar.height },
            filled && frame == frameIndex ? GOLD : color
        );
    }
}

static void RefreshPoseDepth(int index)
{
    PoseApplyDepth(
        &poses[index],
        depthMaps + PoseDepthCells(sprite.width, sprite.height) * index,
        sprite.width, sprite.height, depthSpan
    );
}

static void UnloadSprite(void)
{
    UnloadTexture(originalTexture);
    UnloadTexture(normalTexture);
    UnloadTexture(depthTexture);
    UnloadImage(sprite);
    free(normalFrames);
    free(depthPixels);
    free(depthMaps);
    free(poses);
    free(frameFlips);
    PoseExportFree(&exportResult);
    for (int i = 0; i < 2 * ONION_MAX; i++)
    {
        if (onionTextures[i].id != 0)
            UnloadTexture(onionTextures[i]);

        onionTextures[i] = (Texture2D){ 0 };
    }
    onionKeyFrame = -1;
    onionKeyCount = -1;
    poseRevision++;
    sprite = (Image){ 0 };
    normalFrames = NULL;
    depthMaps = NULL;
    depthPixels = NULL;
    poses = NULL;
    frameFlips = NULL;
    hasImage = false;
}

static void DetectPose(const uint8_t *rgba, int w, int h, Pose *pose)
{
    if (!poseModelReady || !PoseModelDetect(&poseModel, rgba, w, h, pose))
        PoseDetect(rgba, w, h, pose);
}

static bool LoadDroppedImage(const char *path)
{
    if (!IsFileExtension(
            path, ".png;.jpg;.jpeg;.bmp;.gif;.tga"))
    {
        snprintf(statusText, sizeof(statusText),
                 "Unsupported format: %s", GetFileName(path));
        return false;
    }

    int frames = 0;
    Image anim = LoadImageAnim(path, &frames);

    if (anim.data == NULL || anim.width <= 0 || anim.height <= 0 || frames <= 0)
    {
        UnloadImage(anim);
        snprintf(statusText, sizeof(statusText),
                 "Could not open: %s", GetFileName(path));
        return false;
    }

    if (anim.format != PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
        ImageFormat(&anim, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);

    size_t bytes = (size_t)anim.width * anim.height * 4;
    unsigned char *newNormals = malloc(bytes * frames);
    Pose *newPoses = calloc(frames, sizeof(Pose));
    PoseFlips *newFlips = calloc(frames, sizeof(PoseFlips));
    size_t depthCells = PoseDepthCells(anim.width, anim.height);
    float *newDepth = malloc(depthCells * frames * sizeof(float));
    unsigned char *newDepthPixels = malloc(depthCells * 4);
    float newSpan = 0.0f;

    if (newNormals == NULL || newPoses == NULL || newDepth == NULL || newDepthPixels == NULL || newFlips == NULL)
    {
        free(newFlips);
        free(newNormals);
        free(newDepth);
        free(newDepthPixels);
        free(newPoses);
        UnloadImage(anim);
        snprintf(statusText, sizeof(statusText), "Not enough memory");
        return false;
    }

    for (int i = 0; i < frames; i++)
    {
        Image view = FrameView(anim.data, anim.width, anim.height, i);

        PoseKeyBackground(view.data, anim.width, anim.height);

        Image normals = GenerateNormalMap(view);

        memcpy(newNormals + bytes * i, normals.data, bytes);
        UnloadImage(normals);
        newSpan = fmaxf(newSpan, PoseIntegrateDepth(
            newNormals + bytes * i, view.data,
            anim.width, anim.height, newDepth + depthCells * i
        ));
        DetectPose(view.data, anim.width, anim.height, &newPoses[i]);
    }

    PoseLimbs newLimbs = PoseMeasureLimbs(newPoses, frames);

    for (int i = 0; i < frames; i++)
    {
        PoseFitLimbs(&newPoses[i], &newLimbs);
        PoseApplyDepth(
            &newPoses[i], newDepth + depthCells * i,
            anim.width, anim.height, newSpan
        );
    }

    PoseDepthImage(newDepth, anim.data, anim.width, anim.height, newDepthPixels);

    Texture2D newDepthTexture = LoadTextureFromImage((Image){
        newDepthPixels,
        PoseDepthSize(anim.width), PoseDepthSize(anim.height), 1,
        PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
    });
    Texture2D newOriginal = LoadTextureFromImage(
        FrameView(anim.data, anim.width, anim.height, 0)
    );
    Texture2D newNormalTexture = LoadTextureFromImage(
        FrameView(newNormals, anim.width, anim.height, 0)
    );

    if (newOriginal.id == 0 || newNormalTexture.id == 0 || newDepthTexture.id == 0)
    {
        if (newDepthTexture.id != 0)
            UnloadTexture(newDepthTexture);

        if (newOriginal.id != 0)
            UnloadTexture(newOriginal);

        if (newNormalTexture.id != 0)
            UnloadTexture(newNormalTexture);

        free(newNormals);
        free(newDepth);
        free(newDepthPixels);
        free(newPoses);
        free(newFlips);
        UnloadImage(anim);

        snprintf(statusText, sizeof(statusText),
                 "Could not create textures");
        return false;
    }

    SetTextureFilter(newOriginal, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(newNormalTexture, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(newDepthTexture, TEXTURE_FILTER_BILINEAR);

    if (hasImage)
        UnloadSprite();

    sprite = anim;
    normalFrames = newNormals;
    depthMaps = newDepth;
    depthSpan = newSpan;
    depthPixels = newDepthPixels;
    depthTexture = newDepthTexture;
    poses = newPoses;
    frameFlips = newFlips;
    limbs = newLimbs;
    frameCount = frames;
    frameIndex = 0;
    frameClock = 0.0f;
    impactFrame = -1;
    inspectorOn = false;
    dragJoint = -1;
    playing = frames > 1;
    originalTexture = newOriginal;
    normalTexture = newNormalTexture;
    snprintf(spritePath, sizeof(spritePath), "%s", path);

    imageSize = (Vector2){
        (float)originalTexture.width,
        (float)originalTexture.height
    };

    float aspect = imageSize.x / imageSize.y;

    billboardSize.y = BOARD_HEIGHT;
    billboardSize.x = BOARD_HEIGHT * aspect;

    if (billboardSize.x > MAX_BOARD_WIDTH)
    {
        billboardSize.x = MAX_BOARD_WIDTH;
        billboardSize.y = MAX_BOARD_WIDTH / aspect;
    }

    hasImage = true;
    showNormals = false;

    snprintf(statusText, sizeof(statusText),
             "%s | %d x %d | %d frames",
             GetFileName(path),
             originalTexture.width,
             originalTexture.height,
             frameCount);

    return true;
}

static void UpdateOrbitCamera(Camera3D *camera)
{
    Vector3 target = BOARD_CENTER;
    float cp = cosf(cameraPitch);

    camera->target = target;
    camera->position = (Vector3){
        target.x + sinf(cameraYaw) * cp * cameraDistance,
        target.y + sinf(cameraPitch) * cameraDistance,
        target.z + cosf(cameraYaw) * cp * cameraDistance
    };

    camera->up = (Vector3){ 0.0f, 1.0f, 0.0f };
}

static void GetCameraAxes(
    Camera3D camera,
    Vector3 *right,
    Vector3 *up,
    Vector3 *normal)
{
    Vector3 forward = Vector3Normalize(
        Vector3Subtract(camera.target, camera.position)
    );

    *right = Vector3Normalize(
        Vector3CrossProduct(forward, camera.up)
    );

    *up = Vector3Normalize(
        Vector3CrossProduct(*right, forward)
    );

    *normal = Vector3Scale(forward, -1.0f);
}

static Vector3 WorldLightToBillboard(
    Camera3D camera,
    Vector3 lightPosition)
{
    Vector3 right, up, normal;

    GetCameraAxes(camera, &right, &up, &normal);

    Vector3 delta = Vector3Subtract(lightPosition, BOARD_CENTER);

    return (Vector3){
        Vector3DotProduct(delta, right),
        Vector3DotProduct(delta, up),
        Vector3DotProduct(delta, normal)
    };
}

static Vector3 GetFirstLightPosition(
    Camera3D camera,
    float angle)
{
    Vector3 right, up, normal;

    GetCameraAxes(camera, &right, &up, &normal);

    float radius = fmaxf(
        LIGHT_RADIUS,
        billboardSize.x * 0.5f + 1.0f
    );

    Vector3 p = BOARD_CENTER;

    p = Vector3Add(
        p, Vector3Scale(right, cosf(angle) * radius)
    );

    p = Vector3Add(
        p, Vector3Scale(up, sinf(angle) * radius * 0.65f)
    );

    p = Vector3Add(
        p, Vector3Scale(normal,
            LIGHT_DEPTH + 0.4f * sinf(angle * 0.5f))
    );

    return p;
}

static Vector3 GetSecondLightPosition(
    Camera3D camera,
    float angle)
{
    Vector3 right, up, normal;

    GetCameraAxes(camera, &right, &up, &normal);

    float rightDistance = billboardSize.x * 0.5f + 1.2f;
    float verticalOffset = cosf(angle) * 2.6f;

    Vector3 p = BOARD_CENTER;

    p = Vector3Add(
        p, Vector3Scale(right, rightDistance)
    );

    p = Vector3Add(
        p, Vector3Scale(up, verticalOffset)
    );

    p = Vector3Add(
        p, Vector3Scale(normal, LIGHT_DEPTH)
    );

    return p;
}

static bool GetDraggedLightPosition(
    Camera3D camera,
    Vector2 mouse,
    Vector3 *result)
{
    Ray ray = GetScreenToWorldRay(mouse, camera);

    Vector3 right, up, normal;
    GetCameraAxes(camera, &right, &up, &normal);

    Vector3 toPlane = Vector3Subtract(
        BOARD_CENTER, ray.position
    );

    float denominator = Vector3DotProduct(
        ray.direction, normal
    );

    if (fabsf(denominator) < 0.0001f)
        return false;

    float t = Vector3DotProduct(toPlane, normal) / denominator;

    if (t < 0.0f)
        return false;

    Vector3 hit = Vector3Add(
        ray.position, Vector3Scale(ray.direction, t)
    );

    *result = Vector3Add(
        hit, Vector3Scale(normal, DRAG_LIGHT_DISTANCE)
    );

    return true;
}

static void DrawScene3D(
    Camera3D camera,
    Vector3 light1,
    Vector3 light2,
    Vector3 light3,
    int locNormalMap)
{
    BeginMode3D(camera);

    DrawPlane(
        (Vector3){ 0.0f, -0.03f, 0.0f },
        (Vector2){ 40.0f, 40.0f },
        (Color){ 37, 43, 51, 255 }
    );

    DrawGrid(40, 1.0f);

    DrawCylinder(
        (Vector3){ 0.0f, 0.12f, 0.0f },
        1.15f, 1.35f, 0.24f, 32,
        (Color){ 55, 63, 76, 255 }
    );

    DrawCylinderWires(
        (Vector3){ 0.0f, 0.12f, 0.0f },
        1.15f, 1.35f, 0.24f, 32,
        (Color){ 105, 123, 145, 255 }
    );

    DrawSphere(
        light1, 0.16f,
        (Color){ 255, 218, 145, 255 }
    );

    DrawSphere(
        light2, 0.16f,
        (Color){ 150, 195, 255, 255 }
    );

    DrawSphere(
        light3, 0.16f,
        (Color){ 255, 80, 80, 255 }
    );

    if (hasImage)
    {
        Rectangle source = {
            0.0f, 0.0f,
            (float)originalTexture.width,
            (float)originalTexture.height
        };

        if (showNormals)
        {
            DrawBillboardRec(
                camera, normalTexture, source,
                BOARD_CENTER, billboardSize, WHITE
            );
        }
        else
        {
            SetShaderValueTexture(
                lightShader, locNormalMap, normalTexture
            );

            BeginShaderMode(lightShader);

            DrawBillboardRec(
                camera, originalTexture, source,
                BOARD_CENTER, billboardSize, WHITE
            );

            EndShaderMode();
        }
    }
    else
    {
        DrawCube(
            BOARD_CENTER,
            2.4f, BOARD_HEIGHT, 0.12f,
            (Color){ 38, 85, 115, 255 }
        );

        DrawCubeWires(
            BOARD_CENTER,
            2.4f, BOARD_HEIGHT, 0.12f,
            SKYBLUE
        );
    }

    EndMode3D();
}

static Rectangle PoseViewRect(void)
{
    float areaW = (float)GetScreenWidth() * (1.0f - SIDE_PANEL_RATIO) - Ui(50.0f);
    float areaH = (float)GetScreenHeight() - Ui(SCENE_TOP) - Ui(SCENE_BOTTOM) - Ui(20.0f);
    float scale = fminf(areaW / sprite.width, areaH / sprite.height);
    float w = sprite.width * scale;
    float h = sprite.height * scale;

    return (Rectangle){
        Ui(20.0f) + (areaW - w) * 0.5f,
        Ui(SCENE_TOP) + Ui(10.0f) + (areaH - h) * 0.5f,
        w, h
    };
}

static Rectangle SidePanelRect(void)
{
    Rectangle view = PoseViewRect();
    float w = (float)GetScreenWidth() * SIDE_PANEL_RATIO;

    return (Rectangle){ (float)GetScreenWidth() - Ui(20.0f) - w, view.y, w, view.height };
}

static float SideHalfWidth(Rectangle side)
{
    return side.width * 0.5f - Ui(10.0f);
}

static Vector2 PoseToScreen(Rectangle view, PosePoint p)
{
    float scale = view.width / sprite.width;

    return (Vector2){ view.x + p.x * scale, view.y + p.y * scale };
}

static float SideZRange(Rectangle side, Rectangle view)
{
    float range = SideHalfWidth(side) / view.height;

    for (int i = 0; i < frameCount; i++)
    {
        if (!poses[i].valid)
            continue;

        for (int j = 0; j < POSE_JOINTS; j++)
            range = fmaxf(range, fabsf(poses[i].p[j].z - 0.5f) * SIDE_Z_MARGIN);
    }

    return range;
}

static float SideScale(Rectangle side, Rectangle view)
{
    return SideHalfWidth(side) / SideZRange(side, view);
}

static float SidePivot(const Pose *pose)
{
    return (pose->p[POSE_LHIP].x + pose->p[POSE_RHIP].x) * 0.5f;
}

static Vector2 PoseToSide(Rectangle side, Rectangle view, float pivot, PosePoint p)
{
    float depth = -(p.z - 0.5f) * SideScale(side, view);
    float lateral = (p.x - pivot) * view.width / sprite.width;

    return (Vector2){
        side.x + side.width * 0.5f + depth * cosf(SideAngle()) + lateral * sinf(SideAngle()),
        PoseToScreen(view, p).y
    };
}

static const Color JointColors[POSE_JOINTS + 1] = {
    [POSE_NOSE] = { 255, 215, 0, 255 },
    [POSE_NECK] = { 255, 150, 0, 255 },
    [POSE_PELVIS] = { 255, 150, 0, 255 },
    [POSE_LEYE] = { 140, 220, 255, 255 },
    [POSE_LEAR] = { 0, 100, 200, 255 },
    [POSE_LSHOULDER] = { 0, 70, 255, 255 },
    [POSE_LELBOW] = { 0, 160, 255, 255 },
    [POSE_LWRIST] = { 0, 235, 235, 255 },
    [POSE_LHIP] = { 90, 60, 255, 255 },
    [POSE_LKNEE] = { 150, 110, 255, 255 },
    [POSE_LANKLE] = { 205, 175, 255, 255 },
    [POSE_REYE] = { 255, 170, 170, 255 },
    [POSE_REAR] = { 150, 0, 0, 255 },
    [POSE_RSHOULDER] = { 255, 30, 30, 255 },
    [POSE_RELBOW] = { 255, 115, 0, 255 },
    [POSE_RWRIST] = { 255, 60, 160, 255 },
    [POSE_RHIP] = { 200, 0, 70, 255 },
    [POSE_RKNEE] = { 255, 95, 125, 255 },
    [POSE_RANKLE] = { 255, 165, 205, 255 }
};

static Color JointColor(int joint)
{
    return JointColors[joint];
}

static Color BoneColor(int bone)
{
    Color c = JointColor(PoseBones[bone][1]);

    return (Color){ (unsigned char)(c.r * 0.6f), (unsigned char)(c.g * 0.6f), (unsigned char)(c.b * 0.6f), 255 };
}

static void DrawBodyEllipse(Vector2 a, Vector2 b, float radius, Color color)
{
    Vector2 mid = Vector2Scale(Vector2Add(a, b), 0.5f);
    float angle = atan2f(b.y - a.y, b.x - a.x) * RAD2DEG;

    color.a = BODY_ALPHA;
    rlPushMatrix();
    rlTranslatef(mid.x, mid.y, 0.0f);
    rlRotatef(angle, 0.0f, 0.0f, 1.0f);
    DrawEllipse(0, 0, Vector2Distance(a, b) * 0.5f + radius * 0.5f, radius, color);
    rlPopMatrix();
}

typedef struct {
    Vector2 a, b;
    float radius;
    float depth;
    Color color;
} SideItem;

#define SIDE_ITEM_MAX 64

static float SideToward(Rectangle side, Rectangle view, float pivot, PosePoint p)
{
    float depth = -(p.z - 0.5f) * SideScale(side, view);
    float lateral = (p.x - pivot) * view.width / sprite.width;

    return lateral * cosf(SideAngle()) - depth * sinf(SideAngle());
}

static int CompareSideItems(const void *a, const void *b)
{
    float da = ((const SideItem *)a)->depth;
    float db = ((const SideItem *)b)->depth;

    return (da > db) - (da < db);
}

static int BuildSideItems(const Pose *pose, Rectangle side, Rectangle view, SideItem *items)
{
    float pivot = SidePivot(pose);
    float scale = view.width / sprite.width;
    Vector2 p[POSE_JOINTS + 1];
    float toward[POSE_JOINTS + 1];
    int n = 0;

    for (int j = 0; j < POSE_JOINTS; j++)
    {
        p[j] = PoseToSide(side, view, pivot, pose->p[j]);
        toward[j] = SideToward(side, view, pivot, pose->p[j]);
    }

    p[POSE_PELVIS] = Vector2Scale(Vector2Add(p[POSE_LHIP], p[POSE_RHIP]), 0.5f);
    toward[POSE_PELVIS] = (toward[POSE_LHIP] + toward[POSE_RHIP]) * 0.5f;

    float height = pose->height * scale;
    Vector2 pelvis = Vector2Scale(Vector2Add(p[POSE_LHIP], p[POSE_RHIP]), 0.5f);
    float pelvisDepth = (toward[POSE_LHIP] + toward[POSE_RHIP]) * 0.5f;
    Vector2 head = { 0.0f, 0.0f };
    float headDepth = 0.0f;
    const int headCount = (int)(sizeof PoseHead / sizeof PoseHead[0]);

    for (int i = 0; i < headCount; i++)
    {
        head = Vector2Add(head, Vector2Scale(p[PoseHead[i]], 1.0f / headCount));
        headDepth += toward[PoseHead[i]] / headCount;
    }

    const float armRadius[2] = { BODY_UPPER_ARM_RADIUS, BODY_FOREARM_RADIUS };
    const float legRadius[2] = { BODY_THIGH_RADIUS, BODY_CALF_RADIUS };

    items[n++] = (SideItem){ p[POSE_NECK], pelvis, BODY_TORSO_RADIUS * height, (toward[POSE_NECK] + pelvisDepth) * 0.5f, JointColor(POSE_NECK) };

    for (int k = 0; k < 2; k++)
        for (int s = 0; s < 2; s++)
        {
            int a = PoseArms[k][s], b = PoseArms[k][s + 1];
            items[n++] = (SideItem){ p[a], p[b], armRadius[s] * height, (toward[a] + toward[b]) * 0.5f, JointColor(b) };
            a = PoseLegs[k][s];
            b = PoseLegs[k][s + 1];
            items[n++] = (SideItem){ p[a], p[b], legRadius[s] * height, (toward[a] + toward[b]) * 0.5f, JointColor(b) };
        }

    items[n++] = (SideItem){ head, head, POSE_HEAD_RADIUS * height, headDepth, JointColor(POSE_NOSE) };

    return n;
}

static void DrawSideItems(const SideItem *items, int count)
{
    for (int i = 0; i < count; i++)
    {
        const SideItem *item = &items[i];

        DrawBodyEllipse(item->a, item->b, item->radius, item->color);
    }
}

static void ExportPoses(void)
{
    const char *out = TextFormat(
        "%s/%s.json",
        GetDirectoryPath(spritePath),
        GetFileNameWithoutExt(spritePath)
    );

    snprintf(statusText, sizeof(statusText),
             PoseExportJson(out, poses, frameCount, sprite.width, sprite.height, &exportOptions)
                 ? "Exported: %s" : "Could not write: %s",
             GetFileName(out));
}

static Rectangle ExportGapRect(void)
{
    return (Rectangle){
        (float)GetScreenWidth() - Ui(24.0f) - Ui(EXPORT_GAP_WIDTH),
        Ui(58.0f), Ui(EXPORT_GAP_WIDTH), Ui(EXPORT_GAP_BUTTON)
    };
}

static Rectangle ExportGapButton(bool plus)
{
    Rectangle rect = ExportGapRect();

    return (Rectangle){
        plus ? rect.x + rect.width - Ui(EXPORT_GAP_BUTTON) : rect.x,
        rect.y, Ui(EXPORT_GAP_BUTTON), rect.height
    };
}

static void UpdateExportGap(void)
{
    if (!IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        return;

    Vector2 mouse = GetMousePosition();

    if (CheckCollisionPointRec(mouse, ExportGapButton(false)) && exportOptions.emptyFrames > 0)
        exportOptions.emptyFrames--;

    if (CheckCollisionPointRec(mouse, ExportGapButton(true)) && exportOptions.emptyFrames < EXPORT_EMPTY_FRAMES_MAX)
        exportOptions.emptyFrames++;
}

static void DrawExportGapButton(Rectangle rect, const char *text)
{
    Color background = CheckCollisionPointRec(GetMousePosition(), rect)
        ? (Color){ 70, 80, 100, 255 }
        : (Color){ 45, 52, 68, 255 };

    DrawRectangleRec(rect, background);
    DrawUiText(
        text,
        rect.x + (rect.width - MeasureUiText(text, TEXT_HEADING)) * 0.5f,
        rect.y + Ui(2.0f),
        TEXT_HEADING, RAYWHITE
    );
}

static void DrawExportGap(void)
{
    Rectangle minus = ExportGapButton(false);
    Rectangle plus = ExportGapButton(true);
    const char *label = TextFormat("Gaps: %d", exportOptions.emptyFrames);
    float left = minus.x + minus.width;
    float labelX = left + (plus.x - left - MeasureUiText(label, TEXT_BODY)) * 0.5f;

    DrawExportGapButton(minus, "-");
    DrawExportGapButton(plus, "+");
    DrawUiText(label, labelX, minus.y + Ui(4.0f), TEXT_BODY, RAYWHITE);
}

typedef struct {
    const char *name;
    int *level;
    int max;
    int key;
} ExportToggle;

static const ExportToggle exportToggles[] = {
    { "Ground", &exportOptions.ground, 1, KEY_ONE },
    { "Aspect", &exportOptions.aspect, 1, KEY_TWO },
    { "Smoothing", &exportOptions.smooth, POSE_EXPORT_LEVELS, KEY_THREE },
    { "Keyframes", &exportOptions.keys, POSE_EXPORT_LEVELS, KEY_FOUR },
    { "Result", &exportPreview, 1, KEY_FIVE }
};

#define EXPORT_TOGGLE_COUNT ((int)(sizeof exportToggles / sizeof exportToggles[0]))

static Rectangle ExportToggleRect(int index)
{
    Rectangle gap = ExportGapRect();
    int fromRight = EXPORT_TOGGLE_COUNT - index;

    return (Rectangle){
        gap.x - Ui(EXPORT_TOGGLE_MARGIN) - fromRight * Ui(EXPORT_TOGGLE_WIDTH) - (fromRight - 1) * Ui(EXPORT_TOGGLE_SPACING),
        gap.y, Ui(EXPORT_TOGGLE_WIDTH), gap.height
    };
}

static void UpdateExportToggles(void)
{
    Vector2 mouse = GetMousePosition();

    for (int i = 0; i < EXPORT_TOGGLE_COUNT; i++)
    {
        const ExportToggle *toggle = &exportToggles[i];
        bool clicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
                       CheckCollisionPointRec(mouse, ExportToggleRect(i));

        if (clicked || IsKeyPressed(toggle->key))
            *toggle->level = (*toggle->level + 1) % (toggle->max + 1);
    }
}

static void DrawExportToggles(void)
{
    for (int i = 0; i < EXPORT_TOGGLE_COUNT; i++)
    {
        const ExportToggle *toggle = &exportToggles[i];
        Rectangle rect = ExportToggleRect(i);
        int level = *toggle->level;
        const char *label = level == 0
            ? TextFormat("%s: no", toggle->name)
            : (toggle->max > 1 ? TextFormat("%s: %d", toggle->name, level) : TextFormat("%s: yes", toggle->name));

        DrawRectangleRec(rect, CheckCollisionPointRec(GetMousePosition(), rect)
            ? (Color){ 70, 80, 100, 255 }
            : (Color){ 45, 52, 68, 255 });
        DrawUiText(
            label,
            rect.x + (rect.width - MeasureUiText(label, TEXT_SMALL)) * 0.5f,
            rect.y + Ui(6.0f),
            TEXT_SMALL, level > 0 ? GOLD : LIGHTGRAY
        );
    }
}

static bool ViewControlsVisible(void)
{
    return poseMode && hasImage && !exportPreview;
}

static const char *ViewModeName(void)
{
    static const char *const names[VIEW_COUNT] = { "Onion", "Bones", "Both" };

    return names[viewMode];
}

typedef enum {
    CTRL_INSPECTOR,
    CTRL_IMPACT,
    CTRL_BEFORE_DOWN, CTRL_BEFORE_VALUE, CTRL_BEFORE_UP,
    CTRL_AFTER_DOWN, CTRL_AFTER_VALUE, CTRL_AFTER_UP,
    CTRL_MODE,
    CTRL_ONION_DOWN, CTRL_ONION_VALUE, CTRL_ONION_UP,
    CTRL_OPACITY_DOWN, CTRL_OPACITY_VALUE, CTRL_OPACITY_UP,
    CTRL_COUNT
} BottomControl;

static const float BottomControlWidth[CTRL_COUNT] = {
    100.0f, 96.0f,
    22.0f, 80.0f, 22.0f,
    22.0f, 80.0f, 22.0f,
    130.0f,
    22.0f, 84.0f, 22.0f,
    22.0f, 104.0f, 22.0f
};

static void LayoutBottomControls(Rectangle rects[CTRL_COUNT])
{
    float right = (float)GetScreenWidth() - Ui(BOTTOM_ROW_RIGHT);

    for (int i = CTRL_COUNT - 1; i >= 0; i--)
    {
        float width = Ui(BottomControlWidth[i]);

        rects[i] = (Rectangle){
            right - width, (float)GetScreenHeight() - Ui(BOTTOM_ROW_HEIGHT),
            width, Ui(BOTTOM_ROW_HEIGHT)
        };
        right = rects[i].x - Ui(BOTTOM_SPACING);
    }
}

static void InspectorToggle(void)
{
    inspectorOn = !inspectorOn;

    if (!inspectorOn)
        return;

    viewMode = VIEW_BOTH;

    if (impactFrame < 0)
        impactFrame = frameIndex;
}

static void UpdateBottomControls(void)
{
    if (!ViewControlsVisible() || !IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        return;

    Rectangle r[CTRL_COUNT];
    Vector2 mouse = GetMousePosition();

    LayoutBottomControls(r);

    if (CheckCollisionPointRec(mouse, r[CTRL_INSPECTOR]))
        InspectorToggle();

    if (CheckCollisionPointRec(mouse, r[CTRL_IMPACT]))
        impactFrame = frameIndex;

    if (CheckCollisionPointRec(mouse, r[CTRL_BEFORE_DOWN]) && inspectorBefore > 0)
        inspectorBefore--;

    if (CheckCollisionPointRec(mouse, r[CTRL_BEFORE_UP]) && inspectorBefore < INSPECTOR_SPAN_MAX)
        inspectorBefore++;

    if (CheckCollisionPointRec(mouse, r[CTRL_AFTER_DOWN]) && inspectorAfter > 0)
        inspectorAfter--;

    if (CheckCollisionPointRec(mouse, r[CTRL_AFTER_UP]) && inspectorAfter < INSPECTOR_SPAN_MAX)
        inspectorAfter++;

    if (CheckCollisionPointRec(mouse, r[CTRL_MODE]))
    {
        viewMode = (ViewMode)((viewMode + 1) % VIEW_COUNT);
        dragJoint = -1;
    }

    if (CheckCollisionPointRec(mouse, r[CTRL_ONION_DOWN]) && onionCount > 1)
        onionCount--;

    if (CheckCollisionPointRec(mouse, r[CTRL_ONION_UP]) && onionCount < ONION_MAX)
        onionCount++;

    if (CheckCollisionPointRec(mouse, r[CTRL_OPACITY_DOWN]) && onionOpacity > ONION_OPACITY_MIN)
        onionOpacity -= ONION_OPACITY_STEP;

    if (CheckCollisionPointRec(mouse, r[CTRL_OPACITY_UP]) && onionOpacity < ONION_OPACITY_MAX)
        onionOpacity += ONION_OPACITY_STEP;
}

static void DrawBottomButton(Rectangle rect, const char *text)
{
    Color background = CheckCollisionPointRec(GetMousePosition(), rect)
        ? (Color){ 70, 80, 100, 255 }
        : (Color){ 45, 52, 68, 255 };

    DrawRectangleRec(rect, background);
    DrawUiText(
        text,
        rect.x + (rect.width - MeasureUiText(text, TEXT_SMALL)) * 0.5f,
        rect.y + Ui(3.0f),
        TEXT_SMALL, RAYWHITE
    );
}

static void DrawBottomValue(Rectangle rect, const char *text)
{
    DrawUiText(
        text,
        rect.x + (rect.width - MeasureUiText(text, TEXT_SMALL)) * 0.5f,
        rect.y + Ui(3.0f),
        TEXT_SMALL, RAYWHITE
    );
}

static void DrawBottomControls(void)
{
    Rectangle r[CTRL_COUNT];

    LayoutBottomControls(r);

    DrawBottomButton(r[CTRL_INSPECTOR], inspectorOn ? "Inspector: on" : "Inspector: off");
    DrawBottomButton(r[CTRL_IMPACT], impactFrame < 0 ? "Impact: -" : TextFormat("Impact: %d", impactFrame + 1));
    DrawBottomButton(r[CTRL_BEFORE_DOWN], "-");
    DrawBottomValue(r[CTRL_BEFORE_VALUE], TextFormat("Before: %d", inspectorBefore));
    DrawBottomButton(r[CTRL_BEFORE_UP], "+");
    DrawBottomButton(r[CTRL_AFTER_DOWN], "-");
    DrawBottomValue(r[CTRL_AFTER_VALUE], TextFormat("After: %d", inspectorAfter));
    DrawBottomButton(r[CTRL_AFTER_UP], "+");
    DrawBottomButton(r[CTRL_MODE], TextFormat("View: %s", ViewModeName()));
    DrawBottomButton(r[CTRL_ONION_DOWN], "-");
    DrawBottomValue(r[CTRL_ONION_VALUE], TextFormat("Onion: %d", onionCount));
    DrawBottomButton(r[CTRL_ONION_UP], "+");
    DrawBottomButton(r[CTRL_OPACITY_DOWN], "-");
    DrawBottomValue(r[CTRL_OPACITY_VALUE], TextFormat("Opacity: %d%%", onionOpacity));
    DrawBottomButton(r[CTRL_OPACITY_UP], "+");
}

static void RedetectPose(int index)
{
    Pose *pose = &poses[index];
    PoseLimbs frameLimbs = limbs;

    poseRevision++;

    frameLimbs.flips = frameFlips[index];
    DetectPose(
        FrameView(sprite.data, sprite.width, sprite.height, index).data,
        sprite.width, sprite.height, pose
    );
    PoseFitLimbs(pose, &frameLimbs);
    RefreshPoseDepth(index);
}

static void ReloadPoses(void)
{
    poseRevision++;

    if (!poses[frameIndex].valid)
    {
        snprintf(statusText, sizeof(statusText),
                 "Frame %d has no reference pose", frameIndex + 1);
        return;
    }

    PoseAdoptDepth(&poses[frameIndex], sprite.height);
    limbs = PoseReferenceLimbs(&poses[frameIndex]);

    for (int i = frameIndex + 1; i < frameCount; i++)
    {
        frameFlips[i] = (PoseFlips){ 0 };
        RedetectPose(i);
        frameFlips[i] = PoseLearnFlips(&poses[i - 1], &poses[i]);
        RedetectPose(i);
    }

    snprintf(statusText, sizeof(statusText),
             "Animation recalculated from frame %d", frameIndex + 1);
}

static void MoveJoints(Pose *pose, int joint, float dx, float dy, float dz, float range)
{
    poseRevision++;

    bool head = joint == POSE_NOSE;
    int count = head ? (int)(sizeof PoseHead / sizeof PoseHead[0]) : 1;

    for (int i = 0; i < count; i++)
    {
        PosePoint *p = &pose->p[head ? (int)PoseHead[i] : joint];

        p->x = Clamp(p->x + dx, 0.0f, sprite.width - 1.0f);
        p->y = Clamp(p->y + dy, 0.0f, sprite.height - 1.0f);

        if (dz != 0.0f)
        {
            float z = Clamp(p->z + dz, 0.5f - range, 0.5f + range);

            p->dz += z - p->z;
            p->z = z;
        }
    }
}

static void UpdatePoseEditor(void)
{
    Rectangle view = PoseViewRect();
    Vector2 mouse = GetMousePosition();
    Pose *pose = &poses[frameIndex];

    if (IsKeyPressed(KEY_R))
        ReloadPoses();

    if (IsKeyPressed(KEY_J))
        ExportPoses();

    if (IsKeyPressed(KEY_Z))
        showDepth = !showDepth;

    Rectangle side = SidePanelRect();

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && pose->valid && !exportPreview && viewMode != VIEW_ONION)
    {
        float best = Ui(POSE_PICK_RADIUS);

        dragJoint = -1;
        playing = false;

        for (int j = 0; j < POSE_JOINTS; j++)
        {
            float dMain = Vector2Distance(mouse, PoseToScreen(view, pose->p[j]));
            float dSide = fabsf(sideYaw) < SIDE_YAW_EPSILON
                ? Vector2Distance(mouse, PoseToSide(side, view, SidePivot(pose), pose->p[j]))
                : INFINITY;

            if (dMain < best)
            {
                best = dMain;
                dragJoint = j;
                dragSide = false;
            }

            if (dSide < best)
            {
                best = dSide;
                dragJoint = j;
                dragSide = true;
            }
        }
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && pose->valid && dragJoint < 0)
        sideRotating = CheckCollisionPointRec(mouse, side);

    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
        dragJoint = -1;
        sideRotating = false;
    }

    if (sideRotating)
        sideYaw -= GetMouseDelta().x * SIDE_ROTATE_SPEED;
    else
        sideYaw *= expf(-SIDE_RETURN_RATE * GetFrameTime());

    if (dragJoint >= 0 && dragSide)
    {
        float range = SideZRange(side, view);
        float z = 0.5f + (mouse.x - side.x - side.width * 0.5f) / SideScale(side, view);

        MoveJoints(pose, dragJoint, 0.0f, 0.0f, Clamp(z, 0.5f - range, 0.5f + range) - pose->p[dragJoint].z, range);
    }
    else if (dragJoint >= 0)
    {
        float scale = view.width / sprite.width;
        float x = Clamp((mouse.x - view.x) / scale, 0.0f, sprite.width - 1.0f);
        float y = Clamp((mouse.y - view.y) / scale, 0.0f, sprite.height - 1.0f);

        MoveJoints(pose, dragJoint, x - pose->p[dragJoint].x, y - pose->p[dragJoint].y, 0.0f, 0.0f);
        RefreshPoseDepth(frameIndex);
    }
}

static void DrawSideBillboard(Rectangle side, Rectangle view, float pivot, Rectangle source)
{
    float scale = view.width / sprite.width;
    float shear = sinf(SideAngle());
    float left = side.x + side.width * 0.5f - pivot * scale * shear;
    float right = left + view.width * shear;
    Rectangle dest = { fminf(left, right), view.y, fabsf(right - left), view.height };

    if (dest.width < 1.0f)
        return;

    Color tint = WHITE;

    if (shear < 0.0f)
    {
        source.width = -source.width;
        tint = SIDE_BACK_TINT;
    }

    BeginScissorMode((int)side.x, (int)side.y, (int)side.width, (int)side.height);
    DrawTexturePro(originalTexture, source, dest, (Vector2){ 0.0f, 0.0f }, 0.0f, tint);
    EndScissorMode();
}

static void RefreshExportPreview(void)
{
    if (exportBuiltRevision == poseRevision && !memcmp(&exportBuilt, &exportOptions, sizeof exportBuilt))
        return;

    PoseExportBuild(&exportResult, poses, frameCount, sprite.width, sprite.height, &exportOptions);
    exportBuilt = exportOptions;
    exportBuiltRevision = poseRevision;
}

static float ExportScaleX(Rectangle view)
{
    return exportBuilt.aspect ? view.height : view.width;
}

static Vector2 ExportToStage(Rectangle view, const float *p)
{
    return (Vector2){ view.x + view.width * 0.5f + (p[0] - 0.5f) * ExportScaleX(view), view.y + p[1] * view.height };
}

static Vector2 ExportToSide(Rectangle side, Rectangle view, float pivot, const float *p)
{
    float depth = -(p[2] - 0.5f) * SideScale(side, view);
    float lateral = (p[0] - pivot) * ExportScaleX(view);

    return (Vector2){
        side.x + side.width * 0.5f + depth * cosf(SideAngle()) + lateral * sinf(SideAngle()),
        view.y + p[1] * view.height
    };
}

static float ExportSideToward(Rectangle side, Rectangle view, float pivot, const float *p)
{
    float depth = -(p[2] - 0.5f) * SideScale(side, view);
    float lateral = (p[0] - pivot) * ExportScaleX(view);

    return lateral * cosf(SideAngle()) - depth * sinf(SideAngle());
}

static void FillPelvis3D(Vector3 *k)
{
    k[POSE_PELVIS] = Vector3Scale(Vector3Add(k[POSE_LHIP], k[POSE_RHIP]), 0.5f);
}

static void DrawBones3D(const Vector3 *k, Rectangle clip, float boneRadius, float jointRadius, int highlight, bool withBody)
{
    bool drawBody = withBody && bodyModelReady;

    if (drawBody)
        body_pose(&bodyModel, k);

    BeginScissorMode((int)clip.x, (int)clip.y, (int)clip.width, (int)clip.height);
    BeginMode3D(BodyCamera());

    if (drawBody)
        DrawMesh(bodyModel.mesh, bodyModel.material, MatrixIdentity());

    for (int i = 0; i < POSE_BONE_COUNT; i++)
        DrawCylinderEx(k[PoseBones[i][0]], k[PoseBones[i][1]], boneRadius, boneRadius, 8, BoneColor(i));

    for (int j = 0; j < POSE_JOINTS; j++)
        DrawSphere(k[j], j == highlight ? jointRadius * 1.4f : jointRadius, JointColor(j));

    EndMode3D();
    EndScissorMode();
}

static void DrawExportGhost(const Vector2 *joints)
{
    const Color ghostColor = { 150, 155, 170, 120 };
    Vector2 p[POSE_JOINTS + 1];

    for (int j = 0; j < POSE_JOINTS; j++)
        p[j] = joints[j];
    p[POSE_PELVIS] = Vector2Scale(Vector2Add(p[POSE_LHIP], p[POSE_RHIP]), 0.5f);

    for (int i = 0; i < POSE_BONE_COUNT; i++)
        DrawLineEx(p[PoseBones[i][0]], p[PoseBones[i][1]], Ui(1.5f), ghostColor);

    for (int j = 0; j < POSE_JOINTS; j++)
        DrawCircleV(p[j], Ui(3.0f), ghostColor);
}

static void DrawExportPreview(Rectangle view, Rectangle side)
{
    RefreshExportPreview();

    float floorY = view.y + view.height;
    float centerX = view.x + view.width * 0.5f;
    int sent = 0;

    for (int i = 0; i < frameCount; i++)
        sent += exportResult.count == frameCount && exportResult.keep[i];

    DrawRectangleRec(view, (Color){ 22, 26, 36, 255 });
    DrawRectangleLinesEx(view, Ui(1.0f), (Color){ 70, 80, 100, 255 });
    DrawLine((int)centerX, (int)view.y, (int)centerX, (int)floorY, (Color){ 52, 58, 74, 255 });
    DrawLineEx((Vector2){ view.x, floorY }, (Vector2){ view.x + view.width, floorY }, Ui(2.0f), (Color){ 70, 160, 120, 255 });
    DrawUiText("ground", view.x + Ui(8.0f), floorY - Ui(20.0f), TEXT_SMALL, (Color){ 70, 160, 120, 255 });

    DrawUiText(
        TextFormat("Export result | %d of %d frames | %d with gaps", sent, frameCount, TimelineSlots()),
        view.x + Ui(10.0f), view.y + Ui(8.0f), TEXT_BODY, RAYWHITE
    );
    DrawUiText("gray: detection | color: exported", view.x + Ui(10.0f), view.y + Ui(30.0f), TEXT_SMALL, GRAY);

    int k = PoseExportFind(&exportResult, frameIndex);
    float shown[POSE_JOINTS][3];

    if (!PoseExportSample(&exportResult, frameIndex, shown))
    {
        DrawUiText("No pose for this frame", view.x + Ui(10.0f), view.y + Ui(50.0f), TEXT_SMALL, GOLD);
        return;
    }

    DrawUiText(
        exportResult.keep[frameIndex]
            ? TextFormat("frame_%04d | exported", frameIndex * (exportOptions.emptyFrames + 1))
            : "interpolated between keyframes",
        view.x + Ui(10.0f), view.y + Ui(50.0f), TEXT_SMALL, exportResult.keep[frameIndex] ? GOLD : SKYBLUE
    );

    Vector2 rawFront[POSE_JOINTS], rawLateral[POSE_JOINTS];
    float pivot = (shown[POSE_LHIP][0] + shown[POSE_RHIP][0]) * 0.5f;
    Vector3 frontK[POSE_JOINTS + 1], sideK[POSE_JOINTS + 1];

    for (int j = 0; j < POSE_JOINTS; j++)
    {
        Vector2 front = ExportToStage(view, shown[j]);
        Vector2 lateral = ExportToSide(side, view, pivot, shown[j]);

        frontK[j] = (Vector3){ front.x, -front.y, (shown[j][2] - 0.5f) * view.height };
        sideK[j] = (Vector3){ lateral.x, -lateral.y, ExportSideToward(side, view, pivot, shown[j]) };
    }

    FillPelvis3D(frontK);
    FillPelvis3D(sideK);
    float margin = fminf(view.height * EXPORT_OVERFLOW, fminf(fminf(view.x, side.x), fminf(view.y, side.y)));
    Rectangle frontClip = { view.x - margin, view.y - margin, view.width + 2.0f * margin, view.height + margin };
    Rectangle sideClip = { side.x - margin, side.y - margin, side.width + 2.0f * margin, side.height + margin };

    DrawBones3D(frontK, frontClip, Ui(2.5f), Ui(5.0f), -1, true);
    DrawBones3D(sideK, sideClip, Ui(2.5f), Ui(5.0f), -1, true);

    if (k >= 0)
    {
        const PoseExportFrame *raw = &exportResult.raw[k];
        float rawPivot = (raw->p[POSE_LHIP][0] + raw->p[POSE_RHIP][0]) * 0.5f;

        for (int j = 0; j < POSE_JOINTS; j++)
        {
            rawFront[j] = ExportToStage(view, raw->p[j]);
            rawLateral[j] = ExportToSide(side, view, rawPivot, raw->p[j]);
        }

        DrawExportGhost(rawLateral);
        DrawExportGhost(rawFront);
    }
}

static void DrawSideSkeleton(const Pose *pose, Rectangle side, Rectangle view, int highlight)
{
    Vector3 sideK[POSE_JOINTS + 1];
    float pivot = SidePivot(pose);

    for (int j = 0; j < POSE_JOINTS; j++)
    {
        Vector2 lateral = PoseToSide(side, view, pivot, pose->p[j]);

        sideK[j] = (Vector3){ lateral.x, -lateral.y, SideToward(side, view, pivot, pose->p[j]) };
    }

    FillPelvis3D(sideK);
    DrawBones3D(sideK, side, Ui(3.0f), Ui(7.0f), highlight, false);
}

static void DrawFrontSkeleton(const Pose *pose, Rectangle view, int highlight)
{
    Vector3 frontK[POSE_JOINTS + 1];

    for (int j = 0; j < POSE_JOINTS; j++)
    {
        Vector2 screen = PoseToScreen(view, pose->p[j]);

        frontK[j] = (Vector3){ screen.x, -screen.y, (pose->p[j].z - 0.5f) * view.height };
    }

    FillPelvis3D(frontK);
    DrawBones3D(frontK, view, Ui(2.0f), Ui(5.0f), highlight, false);
}

static void DrawSideBody(const Pose *pose, Rectangle side, Rectangle view)
{
    SideItem sideItems[SIDE_ITEM_MAX];
    int sideCount = BuildSideItems(pose, side, view, sideItems);

    qsort(sideItems, sideCount, sizeof *sideItems, CompareSideItems);
    DrawSideItems(sideItems, sideCount);
}

static void OnionLoad(int slot, int frame)
{
    Image image = FrameView(sprite.data, sprite.width, sprite.height, frame);

    if (onionTextures[slot].id == 0)
    {
        onionTextures[slot] = LoadTextureFromImage(image);
        SetTextureFilter(onionTextures[slot], TEXTURE_FILTER_BILINEAR);
        return;
    }

    UpdateTexture(onionTextures[slot], image.data);
}

static void OnionRefresh(void)
{
    if (onionKeyFrame == frameIndex && onionKeyCount == onionCount)
        return;

    onionKeyFrame = frameIndex;
    onionKeyCount = onionCount;

    for (int k = 1; k <= onionCount; k++)
    {
        OnionLoad(k - 1, ((frameIndex - k) % frameCount + frameCount) % frameCount);
        OnionLoad(ONION_MAX + k - 1, (frameIndex + k) % frameCount);
    }
}

static void DrawOnionFrames(Rectangle view, Rectangle source)
{
    if (frameCount < 2)
        return;

    OnionRefresh();

    for (int k = onionCount; k >= 1; k--)
    {
        unsigned char alpha = (unsigned char)(ONION_ALPHA * (onionCount + 1 - k) * onionOpacity / (onionCount * 100));
        Color earlier = { 110, 255, 150, alpha };
        Color later = { 110, 170, 255, alpha };

        DrawTexturePro(onionTextures[k - 1], source, view, (Vector2){ 0.0f, 0.0f }, 0.0f, earlier);
        DrawTexturePro(onionTextures[ONION_MAX + k - 1], source, view, (Vector2){ 0.0f, 0.0f }, 0.0f, later);
    }
}

static void DrawPoseView(void)
{
    Rectangle view = PoseViewRect();
    Rectangle source = {
        0.0f, 0.0f,
        (float)originalTexture.width,
        (float)originalTexture.height
    };

    DrawRectangleRec(view, (Color){ 30, 34, 46, 255 });

    if (!exportPreview)
    {
        if (viewMode != VIEW_BONES && !showDepth)
            DrawOnionFrames(view, source);

        DrawTexturePro(
            showDepth ? depthTexture : originalTexture,
            showDepth ? (Rectangle){ 0.0f, 0.0f, (float)depthTexture.width, (float)depthTexture.height } : source,
            view, (Vector2){ 0.0f, 0.0f }, 0.0f, WHITE
        );
    }

    Rectangle side = SidePanelRect();
    float sideCenter = side.x + side.width * 0.5f;

    DrawRectangleRec(side, (Color){ 30, 34, 46, 255 });

    for (int t = -2; t <= 2; t++)
    {
        float x = sideCenter + t * 0.5f * SideHalfWidth(side);

        DrawLine((int)x, (int)side.y, (int)x, (int)(side.y + side.height),
                 t == 0 ? (Color){ 90, 100, 125, 255 } : (Color){ 52, 58, 74, 255 });
    }

    DrawUiText("Side view (z) | background: rotate", side.x + Ui(10.0f), side.y + Ui(8.0f), TEXT_BODY, RAYWHITE);
    DrawUiText("far", side.x + Ui(10.0f), side.y + Ui(32.0f), TEXT_SMALL, GRAY);
    DrawUiTextRight("near", side.x + side.width - Ui(10.0f), side.y + Ui(32.0f), TEXT_SMALL, GRAY);

    if (exportPreview)
    {
        DrawExportPreview(view, side);
        return;
    }

    const Pose *pose = &poses[frameIndex];

    if (!pose->valid)
        return;

    DrawSideBillboard(side, view, SidePivot(pose), source);

    if (viewMode == VIEW_ONION)
        return;

    DrawSideBody(pose, side, view);
    DrawSideSkeleton(pose, side, view, dragSide ? dragJoint : -1);

    if (dragJoint >= 0)
        DrawUiText(
            TextFormat("%s z=%.3f", PoseNames[dragJoint], pose->p[dragJoint].z),
            side.x + Ui(10.0f), side.y + side.height - Ui(28.0f), TEXT_BODY, RAYWHITE
        );

    DrawFrontSkeleton(pose, view, dragJoint);
}

static bool LoadImageFromPathText(const char *text)
{
    char path[1024];

    if (text == NULL)
        return false;

    while (*text != '\0' && (isspace((unsigned char)*text) || *text == '"' || *text == '\''))
        text++;

    size_t offset = 0;

#ifndef _WIN32
    if (isalpha((unsigned char)text[0]) && text[1] == ':' && (text[2] == '\\' || text[2] == '/'))
    {
        offset = snprintf(path, sizeof(path), "/mnt/%c", tolower((unsigned char)text[0]));
        text += 2;
    }
#endif

    size_t length = offset + (size_t)snprintf(path + offset, sizeof(path) - offset, "%s", text);

    if (length >= sizeof(path))
        length = sizeof(path) - 1;

    while (length > 0 && (isspace((unsigned char)path[length - 1]) || path[length - 1] == '"' || path[length - 1] == '\''))
        length--;

    path[length] = '\0';

#ifndef _WIN32
    for (size_t i = 0; i < length; i++)
        if (path[i] == '\\')
            path[i] = '/';
#endif

    if (length == 0)
        return false;

    return LoadDroppedImage(path);
}

int main(int argc, char **argv)
{
    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE |
        FLAG_VSYNC_HINT |
        FLAG_MSAA_4X_HINT
    );

    InitWindow(WIN_W, WIN_H, "Normals 3D - Dual Lights");
    MaximizeWindow();
    SetTargetFPS(60);
    LoadUiFont();

    Camera3D camera = { 0 };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    UpdateOrbitCamera(&camera);

    lightShader = LoadShaderFromMemory(NULL, fragmentShader);

    if (!IsShaderValid(lightShader))
    {
        if (UiFontIsCustom())
            UnloadFont(uiFont);

        CloseWindow();
        return 1;
    }

    poseModelReady = PoseModelOpen(
        &poseModel,
        TextFormat("%s" POSE_MODEL_TASK, GetApplicationDirectory())
    );

    if (!poseModelReady)
        snprintf(statusText, sizeof(statusText), "Pose model not found: using heuristic");

    bodyModelReady = BodyModelOpen(
        &bodyModel,
        TextFormat("%s" BODY_MODEL_FILE, GetApplicationDirectory())
    );

    if (!bodyModelReady)
        snprintf(statusText, sizeof(statusText), "Body model not found: " BODY_MODEL_FILE);

    int locLight1 = GetShaderLocation(lightShader, "lightLocal1");
    int locLight2 = GetShaderLocation(lightShader, "lightLocal2");
    int locLight3 = GetShaderLocation(lightShader, "lightLocal3");
    int locColor1 = GetShaderLocation(lightShader, "lightColor1");
    int locColor2 = GetShaderLocation(lightShader, "lightColor2");
    int locColor3 = GetShaderLocation(lightShader, "lightColor3");
    int locBoardSize = GetShaderLocation(lightShader, "boardSize");
    int locStrength = GetShaderLocation(lightShader, "strength");
    int locNormalMap = GetShaderLocation(lightShader, "normalMap");

    float lightColor1[3] = { 1.0f, 0.82f, 0.55f };
    float lightColor2[3] = { 0.55f, 0.75f, 1.0f };
    float lightColor3[3] = { 1.0f, 0.2f, 0.2f };

    if (argc > 1)
        LoadImageFromPathText(argv[1]);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        UpdateUiScale();

        if (IsFileDropped())
        {
            FilePathList files = LoadDroppedFiles();

            if (files.count > 0)
                LoadDroppedImage(files.paths[0]);

            UnloadDroppedFiles(files);
        }

        if (IsKeyPressed(KEY_V) && (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)))
            LoadImageFromPathText(GetClipboardText());

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            Vector2 delta = GetMouseDelta();

            cameraYaw += delta.x * 0.005f;
            cameraPitch += delta.y * 0.005f;

            if (cameraPitch > 1.35f) cameraPitch = 1.35f;
            if (cameraPitch < -1.35f) cameraPitch = -1.35f;
        }

        cameraDistance -= GetMouseWheelMove() * 0.8f;

        if (cameraDistance < 4.5f) cameraDistance = 4.5f;
        if (cameraDistance > 24.0f) cameraDistance = 24.0f;

        UpdateOrbitCamera(&camera);

        if (IsKeyPressed(KEY_SPACE) && hasImage)
            showNormals = !showNormals;

        if (IsKeyPressed(KEY_P) && hasImage)
        {
            poseMode = !poseMode;
            dragJoint = -1;
        }

        if (IsKeyPressed(KEY_G) && frameCount > 1)
            playing = !playing;

        if (hasImage && frameCount > 1)
        {
            if (IsKeyPressed(KEY_COMMA) || IsKeyPressed(KEY_A) ||
                (poseMode && IsKeyPressed(KEY_LEFT)))
                StepFrame(-1);

            if (IsKeyPressed(KEY_PERIOD) || IsKeyPressed(KEY_D) ||
                (poseMode && IsKeyPressed(KEY_RIGHT)))
                StepFrame(1);

            UpdateTimeline();

            if (playing)
            {
                frameClock += dt;

                if (frameClock >= 1.0f / GIF_FPS)
                {
                    frameClock = fmodf(frameClock, 1.0f / GIF_FPS);
                    ShowFrame(NextFrameIndex(1));
                }
            }
        }

        if (poseMode && hasImage)
        {
            UpdateExportGap();
            UpdateExportToggles();
            UpdatePoseEditor();
            UpdateBottomControls();
        }

        if (IsKeyPressed(KEY_L))
            autoLight = !autoLight;

        if (IsKeyDown(KEY_UP))
            lightStrength += 3.0f * dt;

        if (IsKeyDown(KEY_DOWN))
            lightStrength -= 3.0f * dt;

        if (IsKeyDown(KEY_RIGHT))
            lightSpeed += 0.5f * dt;

        if (IsKeyDown(KEY_LEFT))
            lightSpeed -= 0.5f * dt;

        if (lightStrength < 0.0f) lightStrength = 0.0f;
        if (lightStrength > 15.0f) lightStrength = 15.0f;
        if (lightSpeed < 0.0f) lightSpeed = 0.0f;
        if (lightSpeed > 5.0f) lightSpeed = 5.0f;

        Vector2 mouse = GetMousePosition();

        bool mouseOverScene =
            mouse.y >= Ui(SCENE_TOP) &&
            mouse.y < (float)GetScreenHeight() - Ui(SCENE_BOTTOM);

        bool draggingLights =
            IsMouseButtonDown(MOUSE_BUTTON_LEFT) && mouseOverScene && !poseMode;

        if (autoLight && !draggingLights)
        {
            lightAngle += dt * lightSpeed;
            verticalLightAngle += dt * lightSpeed * 1.2f;
        }

        Vector3 lightOrbit[LIGHT_COUNT] = {
            GetFirstLightPosition(camera, lightAngle),
            GetSecondLightPosition(camera, verticalLightAngle),
            GetFirstLightPosition(camera, lightAngle + OPPOSITE_LIGHT_ANGLE)
        };

        Vector3 lightPosition[LIGHT_COUNT] = {
            lightOrbit[0], lightOrbit[1], lightOrbit[2]
        };

        if (draggingLights)
        {
            Vector3 draggedPosition;

            if (GetDraggedLightPosition(camera, mouse, &draggedPosition))
            {
                Vector3 center = Vector3Scale(
                    Vector3Add(Vector3Add(lightOrbit[0], lightOrbit[1]), lightOrbit[2]),
                    1.0f / LIGHT_COUNT
                );
                Vector3 offset = Vector3Subtract(draggedPosition, center);

                for (int i = 0; i < LIGHT_COUNT; i++)
                    lightPosition[i] = Vector3Add(lightOrbit[i], offset);
            }
        }

        Vector3 lightLocal1 = WorldLightToBillboard(camera, lightPosition[0]);
        Vector3 lightLocal2 = WorldLightToBillboard(camera, lightPosition[1]);
        Vector3 lightLocal3 = WorldLightToBillboard(camera, lightPosition[2]);

        SetShaderValue(
            lightShader, locLight1,
            &lightLocal1, SHADER_UNIFORM_VEC3
        );

        SetShaderValue(
            lightShader, locLight2,
            &lightLocal2, SHADER_UNIFORM_VEC3
        );

        SetShaderValue(
            lightShader, locLight3,
            &lightLocal3, SHADER_UNIFORM_VEC3
        );

        SetShaderValue(
            lightShader, locColor1,
            lightColor1, SHADER_UNIFORM_VEC3
        );

        SetShaderValue(
            lightShader, locColor2,
            lightColor2, SHADER_UNIFORM_VEC3
        );

        SetShaderValue(
            lightShader, locColor3,
            lightColor3, SHADER_UNIFORM_VEC3
        );

        SetShaderValue(
            lightShader, locBoardSize,
            &billboardSize, SHADER_UNIFORM_VEC2
        );

        SetShaderValue(
            lightShader, locStrength,
            &lightStrength, SHADER_UNIFORM_FLOAT
        );

        BeginDrawing();
        ClearBackground((Color){ 17, 20, 29, 255 });

        if (poseMode)
            DrawPoseView();
        else
            DrawScene3D(camera, lightPosition[0], lightPosition[1], lightPosition[2], locNormalMap);

        float screenW = (float)GetScreenWidth();
        float screenH = (float)GetScreenHeight();

        DrawRectangleRec(
            (Rectangle){ 0.0f, 0.0f, screenW, Ui(SCENE_TOP) },
            (Color){ 9, 12, 19, 225 }
        );

        DrawUiText("NORMALS 3D", Ui(24.0f), Ui(16.0f), TEXT_TITLE, RAYWHITE);

        DrawUiText(
            hasImage
                ? (poseMode
                    ? "Pose editor | OpenPose 18"
                    : (showNormals
                        ? "3D billboard | Normal map"
                        : "3D billboard | Two lights in real time"))
                : "Drag an image or GIF into this window",
            Ui(24.0f), Ui(52.0f), TEXT_HEADING, SKYBLUE
        );

        DrawUiText(statusText, Ui(24.0f), Ui(80.0f), TEXT_SMALL, GRAY);

        if (poseMode)
        {
            DrawExportGap();
            DrawExportToggles();
        }

        if (hasImage && frameCount > 1)
            DrawTimeline();

        if (hasImage)
            DrawUiTextRight(
                TextFormat("FRAME %d/%d", frameIndex + 1, frameCount),
                screenW - Ui(24.0f), Ui(52.0f), TEXT_HEADING, RAYWHITE
            );

        DrawUiTextRight(
            draggingLights ? "LIGHTS: MANUAL" :
                (autoLight ? "LIGHTS: AUTO" : "LIGHTS: PAUSED"),
            screenW - Ui(24.0f), Ui(22.0f), TEXT_HEADING,
            draggingLights ? WHITE : (autoLight ? GOLD : LIGHTGRAY)
        );

        DrawRectangleRec(
            (Rectangle){ 0.0f, screenH - Ui(SCENE_BOTTOM), screenW, Ui(SCENE_BOTTOM) },
            (Color){ 9, 12, 19, 225 }
        );

        DrawUiText(
            "Drag or Ctrl+V a path | SPACE: normals | L: lights | P: poses | G: animate | A D or , . : frame | top bar",
            Ui(20.0f), screenH - Ui(82.0f), TEXT_BODY, RAYWHITE
        );

        DrawUiText(
            poseMode
                ? "Left click + drag: move joint | R: re-detect | J: export JSON | 1-4: options | 5: show result | Z: depth map"
                : "Left click + drag: move the three lights",
            Ui(20.0f), screenH - Ui(59.0f), TEXT_BODY, LIGHTGRAY
        );

        DrawUiText(
            "Right click + drag: camera | Wheel: zoom",
            Ui(20.0f), screenH - Ui(36.0f), TEXT_BODY, LIGHTGRAY
        );

        DrawUiText(
            TextFormat(
                "Intensity %.1f | Speed %.1f",
                lightStrength, lightSpeed
            ),
            Ui(20.0f), screenH - Ui(17.0f), TEXT_SMALL, GRAY
        );

        DrawUiTextRight(
            TextFormat("%d FPS", GetFPS()),
            screenW - Ui(24.0f), screenH - Ui(17.0f), TEXT_SMALL, GRAY
        );

        if (ViewControlsVisible())
            DrawBottomControls();

        EndDrawing();
    }

    if (hasImage)
        UnloadSprite();

    if (UiFontIsCustom())
        UnloadFont(uiFont);

    if (bodyModelReady)
        BodyModelClose(&bodyModel);

    UnloadShader(lightShader);

    PoseModelClose(&poseModel);
    CloseWindow();

    return 0;
}
