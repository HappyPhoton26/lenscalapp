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
| **A** | Mode: BOTH eyes → LEFT eye → RIGHT eye |
| **Stick** (either) | Moves lens centering in 0.25 px steps. BOTH: X moves the eyes in/out (mirrored), Y moves both up/down together. LEFT/RIGHT: X/Y move that eye only |
| **Trigger** (hold) | Fine, slower steps |
| **Grip** | Turns one red pixel at each eye's image center on/off |
| **Stick click** | Back to the values the app started with |

The headset shows each eye's lens center (px and degrees), the mode and the red-pixel state.
Values are saved to `~/.config/lensalign/offsets.txt`:

```
left  <lens_px_x> <lens_px_y> <pos_mm_x> <pos_mm_y>
right <lens_px_x> <lens_px_y> <pos_mm_x> <pos_mm_y>
```

(`pos` is not used.)

## How it works

Each eye is rendered with its projection shifted by the lens-centering offset. The composition layer
still reports the true eye pose and FOV, so the compositor shows the image shifted on the display by
exactly that amount.

## Credits

`assets/photo.jpg` is Upsala Glacier, Argentina, photographed from the International Space Station
(NASA). It is in the public domain, via Wikimedia Commons.
OpenXR headers (`include/openxr`) are © The Khronos Group, Apache-2.0.
