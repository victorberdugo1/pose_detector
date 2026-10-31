# Normals 3D

https://github.com/user-attachments/assets/648bfc9e-e568-4bc9-8b0a-a347465f21f6

A C desktop tool that turns 2D sprites and animated images into relightable 3D assets, with pose estimation and skeletal animation export.

Normals 3D takes an image or GIF, generates normal and depth maps, presents them as a billboard with real-time dynamic lighting, detects an 18-joint skeleton (OpenPose format) using a locally executed machine learning model, and exports the animation as clean, smoothed, keyframe-reduced JSON.

---

## Features

### Lighting and normals
- Normal map generated automatically from each image or GIF frame.
- Three real-time dynamic lights, with automatic mode, pause, and manual drag control.
- Light intensity and speed adjustable live.
- 3D orbital camera with zoom.
- Automatic background removal by detecting the color along the image border.

### Pose estimation
- 18-joint detection (nose, eyes, ears, neck, shoulders, elbows, wrists, hips, knees, ankles).
- MediaPipe Pose Landmarker (*heavy* variant) running on CPU through LiteRT, offline and with no external services.
- Silhouette-based heuristic fallback when the model is unavailable.
- Per-joint depth reconstruction and bone proportion fitting (torso, arms, legs) kept consistent across the whole animation.
- Interactive editor: joint dragging, per-frame redetection, and learning of head orientation corrections.
- Synthetic side view for reviewing and correcting depth.
- Onion and Bones checkboxes, both on by default, with an inspector for frames around an impact. Bones draws the Raw skeleton in joint colors and the Smoothed skeleton in blue.
- Raw vs Smoothed comparison: both 3D bodies overlaid on the original GIF with adjustable transparency.

### 3D body model
- Loads a rigged character (`body.glb`, binary glTF) and deforms it by skinning, driven by the detected pose.
- Supports Mixamo and Unreal bone naming conventions.

### Export
- Per-frame JSON output (`frame_XXXX` → `person_0` → joint → `x`, `y`, `z`).
- Export options:
  - **Ground**: anchors the animation to the ground plane and centers the character.
  - **Aspect**: normalizes coordinates while preserving aspect ratio.
  - **Smoothing** (3 levels): bidirectional Kalman filter per joint and axis, with bone length restoration.
  - **Keyframes** (3 levels): adaptive frame reduction with eased interpolation and configurable tolerance.
  - **Empty frames** (0–10): blank frames interleaved between exported frames.
- Preview of the exported result overlaid on the original detection before writing the file.

---

## Requirements

| Component | Detail |
|---|---|
| Compiler | C99-compatible GCC |
| Graphics | OpenGL 3.3 |
| Operating system | Linux (X11) or Windows |
| Build dependencies | `make`, `git` |
| Inference library | LiteRT (`libLiteRt.so` on Linux, `libLiteRt.dll` on Windows) placed next to the sources |
| Cross-compiling to Windows from Linux | `x86_64-w64-mingw32-gcc` |

raylib is downloaded and built automatically from its official repository during the first build.

### Runtime files

These must be placed next to the executable:

| File | Purpose | If missing |
|---|---|---|
| `pose_landmarker_heavy.task` | MediaPipe pose model | Heuristic detector is used |
| `body.glb` | Rigged 3D character | Body model is not displayed |
| `libLiteRt.so` / `libLiteRt.dll` | Inference runtime | Executable does not start |

---

## Build

```sh
make            # Linux: builds normals and normals.exe | Windows: builds normals.exe
make linux      # Linux executable only
make windows    # Windows executable only
make clean      # remove executables
make distclean  # also remove the downloaded raylib
```

The Linux executable embeds `-Wl,-rpath,'$ORIGIN'`, so it locates `libLiteRt.so` in its own directory.

---

## Usage

1. Run `normals`.
2. Drag an image or GIF onto the window, or copy its path and paste it with `Ctrl+V`.
3. Supported formats: `.png`, `.jpg`, `.jpeg`, `.bmp`, `.gif`, `.tga`.

### Controls

| Action | Control |
|---|---|
| Load file | Drag and drop, or `Ctrl+V` with the path |
| Toggle normals view | `Space` |
| Automatic / paused lights | `L` |
| Light intensity | `↑` / `↓` |
| Light speed | `←` / `→` |
| Move all three lights | Left click + drag |
| Orbit camera | Right click + drag |
| Zoom | Mouse wheel |
| Play / pause animation | `G` |
| Previous / next frame | `A` `D` or `,` `.` |
| Pose editor mode | `P` |

### Pose editor

| Action | Control |
|---|---|
| Move joint | Left click + drag |
| Redetect pose for the frame | `R` |
| Export JSON | `J` |
| Ground / Aspect / Smoothing / Keyframes | `1` / `2` / `3` / `4` |
| View export result | `5` |
| Depth map | `Z` |
| Change frame | `←` / `→` |

### Raw vs Smoothed comparison

The pose editor compares the original pose data with the smoothed data in two ways, both over the original GIF (never modified) and both without Ground or any floor normalization, so position and framing match the GIF:

| Type | What it shows | Toggle |
|---|---|---|
| 3D Models | A Raw and a Smoothed 3D body with adjustable alpha (10% to 100%, default 40%) | **3D Models** checkbox (off by default) |
| Bones | Raw skeleton in its joint colors (the editable one) and Smoothed skeleton in blue, in the front and side views | **Bones** checkbox (on by default) |

Each layer groups its model and its bones. The **Raw** and **Smoothed** checkboxes (second row of controls) show or hide that layer's 3D model and bones together, so you can view only Raw, only Smoothed, or both. The **Alpha** `-` / `+` controls set each layer's model transparency.

- To see only the bones, leave **Bones** on and turn **3D Models** off. To see only the models, do the opposite. To see neither, turn both off for an overlay-off view of the GIF and the side body.
- Raw uses the unsmoothed pose data, including manual edits; Smoothed uses the current **Smoothing** level (`3`). With **Smoothing: no**, Smoothed is identical to Raw.
- Raw bones keep their joint colors so moving joints stays readable; dragging a joint edits the Raw skeleton. Joints can be dragged only while Bones is on and the Raw layer is shown.
- All layers follow the same frame and stay synchronized during playback and while stepping.
- Playback runs at the application's GIF frame rate (10 FPS). Compare matching frames to see which poses changed, then play to judge pauses, impacts and timing.
- The **Onion** checkbox works independently of the comparison.

| Control | Action |
|---|---|
| **Onion** / **Bones** checkboxes | Show or hide onion skin and the skeleton overlays |
| **3D Models** checkbox | Show or hide the Raw and Smoothed 3D bodies |
| **Raw** / **Smoothed** checkboxes | Show or hide each layer's model and bones |
| **Play** button or `G` | Play or pause at the original frame rate |
| `A` `D` / `,` `.` / `←` `→` / timeline | Inspect individual frames |
| **Alpha** `-` / `+` | Adjust each layer's model transparency |

Exported JSON is not affected by the comparison: the **Result** view (`5`) remains dedicated to previewing the export.

The JSON is written next to the source image, with the same name and a `.json` extension.

---

## Output format

```json
{
  "frame_0000": {
    "person_0": {
      "Nose":  { "x": 0.512345, "y": 0.134567, "z": 0.498765 },
      "Neck":  { "x": 0.508901, "y": 0.231234, "z": 0.501122 }
    }
  }
}
```

The 18 exported joints are: `Nose`, `REye`, `LEye`, `REar`, `LEar`, `RShoulder`, `LShoulder`, `RElbow`, `LElbow`, `RWrist`, `LWrist`, `RHip`, `LHip`, `RKnee`, `LKnee`, `RAnkle`, `LAnkle`, `Neck`. Coordinates are normalized; the frame index reflects the configured empty frames.

---

## Architecture

| File | Responsibility |
|---|---|
| `main.c` | Application, UI, rendering, lighting, and pose editor |
| `pose.h` | Pose types, heuristic detector, bone fitting, and depth integration |
| `pose_model.h` | Pose inference with LiteRT and model extraction from the `.task` bundle |
| `body_model.h` | In-house glTF/GLB reader, skinning, and character deformation |
| `pose_export.h` | Smoothing, bone length restoration, keyframe selection, and JSON writing |
| `Makefile` | Linux and Windows builds, including the raylib build |

Dependency design: the project code is C99 with no external dependencies other than raylib and LiteRT. GLB and `.task` bundle parsing is implemented in-house, with no additional parsing libraries.

---

## Privacy

All processing happens locally. Images, poses, and exports never leave the user's machine.
