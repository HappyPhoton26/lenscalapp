# lenscalapp — Lens Align

An OpenXR app for the Steam Frame (SteamVR on Linux, arm64). You use it to adjust **lens centering**
so the left and right images fuse.

It shows one photo to both eyes, always 10 m away. You move each eye's image in 0.25-pixel steps
until the photo looks single and sharp. **The offsets only change this app's own rendering. Nothing in
SteamVR or the headset settings is changed.**

## Build and run

```
./build.sh      # needs gcc, libjpeg, EGL/OpenGL; links SteamVR's OpenXR loader in /opt/steamvr
./run.sh        # or: ./run.sh /path/to/your/picture.jpg
```

SteamVR must be running. You can also add `run.sh` to Steam as a non-Steam VR app, so it shows up
in SteamVR under **Non-Steam**.

## Controls

| Control | What it does |
|---|---|
| **A** tap (either controller) | Mode: BOTH eyes → LEFT eye → RIGHT eye |
| **A** hold ½ s (either controller) | View: PHOTO → 3D SCENE → CROSSHAIR → WARP EDIT. The 3D scene is world-locked (floor grid, pillars from 1 m to 40 m, a table, a far wall). The crosshair is 1 px wide through each eye's lens center, turned by that eye's roll (left blue, right red). All views use the current centering and roll |
| **Stick** (either) | Moves lens centering in 0.25 px steps. BOTH: X moves the eyes in/out (mirrored), Y moves both up/down together. LEFT/RIGHT: X/Y move that eye only |
| **Trigger** (hold) | Fine, slower steps |
| **B** hold ½ s | Warp ON/OFF, in any view (see Warp edit below) |
| **B** tap | Stick mode: MOVE (lens centering) ↔ ROTATE (clocking). In ROTATE, stick X rolls the image about the lens center in 0.01° steps. BOTH = relative (left and right turn opposite ways), LEFT/RIGHT = that eye only. + = counter-clockwise |
| **Grip** | Turns the center pixel on/off at each eye's lens center (left eye **blue**, right eye **red**) |
| **Stick click** | Back to the values the app started with (centering in MOVE, roll in ROTATE) |

The headset shows each eye's lens center (px and degrees), the mode and the red-pixel state.
Values are saved to `~/.config/lensalign/offsets.txt`:

```
left  <lens_px_x> <lens_px_y> <pos_mm_x> <pos_mm_y> <roll_deg>
right <lens_px_x> <lens_px_y> <pos_mm_x> <pos_mm_y> <roll_deg>
```

(`pos` is not used.)

## Warp (distortion) edit

Hold **A** until the view reads **WARP EDIT**. The eye's crosshair shows 7 control points on its horizontal line and 7 on its
vertical line. The middle one is the crosshair center, shared by both lines. The points are spaced 1/8 of the image apart, and the
lines bend through them with a smooth spline.

| Control | What it does |
|---|---|
| **A** tap | Edit LEFT eye ↔ RIGHT eye |
| **Left stick** push | Select a point: left/right walks along the horizontal line, up/down along the vertical line. The line you are on wins unless you push clearly along the other one; hold to keep stepping |
| **Right stick** | Move the selected point in 0.25 px steps (hold to repeat, trigger = fine) |
| **B** | Apply the warp ON/OFF. When ON, the warp bends every view: photo, 3D scene and crosshair |
| **Right stick click** | Selected point back to 0 |
| **Left stick click** | This eye's warp back to the values the app started with |

The warp is stored in `offsets.txt` as `warp_on <0|1>` and one `warp_left` / `warp_right` line each: 7 horizontal then 7 vertical
`dx dy` pairs, in px, in the rolled crosshair frame.

## How it works

Each eye is rendered with its view rolled about its optical axis by the roll value, and its projection shifted by the lens-centering offset, so the roll pivots on the lens center. When the warp is applied, each eye is first drawn to a texture, then drawn again with each pixel moved by the spline through the control points. The composition layer
still reports the true eye pose and FOV, so the compositor shows the image shifted on the display by
exactly that amount.

## Credits

`assets/photo.jpg` is Upsala Glacier, Argentina, photographed from the International Space Station
(NASA). It is in the public domain, via Wikimedia Commons.
OpenXR headers (`include/openxr`) are © The Khronos Group, Apache-2.0.
