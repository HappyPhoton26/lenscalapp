// lensalign - OpenXR stereo-fusion alignment tool (Steam Frame / SteamVR).
//
// Shows a head-locked photo (assets/photo.jpg, or a .jpg given as argument) to both eyes.
// (old:) Shared parts (white grid, rings) are seen by both eyes,
// nonius lines are RED = left eye only, GREEN = right eye only. When the eyes are aligned the
// red line above the centre lines up with the green line below it (horizontal alignment) and
// the red line on the left lines up with the green one on the right (vertical alignment).
//
// The offsets only change how THIS app renders. Nothing in SteamVR / the headset is changed.
//
// Controls (Frame / Index controllers):
//   A               mode: BOTH eyes / LEFT eye / RIGHT eye
//   either stick    move lens centering in 0.25 px steps. BOTH: X = in/out (mirrored), Y = up/down together
//   trigger (hold)  fine (slower)
//   B               stick mode: MOVE (lens centering) / ROTATE (per-eye roll about the lens centre, 0.01 deg steps)
//   grip            centre pixel on/off (left eye BLUE, right eye RED; at each eye's lens centre)
//   (the photo is always 10 m away)
//   stick click     back to the start values (for the eye(s) of the current mode)
// Values are saved to ~/.config/lensalign/offsets.txt and loaded on the next start.
#define GL_GLEXT_PROTOTYPES
#include <EGL/egl.h>
#include <GL/gl.h>
#include <GL/glext.h>
#define XR_USE_PLATFORM_EGL
#define XR_USE_GRAPHICS_API_OPENGL
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <jpeglib.h>

#define CHK(x) do { XrResult r_ = (x); if (XR_FAILED(r_)) { fprintf(stderr, "%s failed: %d (line %d)\n", #x, r_, __LINE__); exit(1); } } while (0)
static volatile int g_quit;
static void on_sig(int s) { (void)s; g_quit = 1; }
static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

/* ---------- math (column-major) ---------- */
typedef struct { float m[16]; } M4;
static M4 mmul(M4 a, M4 b) {
    M4 r;
    for (int c = 0; c < 4; c++) for (int rr = 0; rr < 4; rr++) {
        float s = 0; for (int k = 0; k < 4; k++) s += a.m[k*4 + rr] * b.m[c*4 + k]; r.m[c*4 + rr] = s;
    }
    return r;
}
static M4 m_proj(XrFovf f, float n, float fa, float tx, float ty) {
    float l = tanf(f.angleLeft), r = tanf(f.angleRight), u = tanf(f.angleUp), d = tanf(f.angleDown);
    float w = r - l, h = u - d; M4 m = {{0}};
    m.m[0] = 2 / w; m.m[5] = 2 / h; m.m[8] = (r + l) / w; m.m[9] = (u + d) / h;
    m.m[10] = -(fa + n) / (fa - n); m.m[11] = -1; m.m[14] = -2 * fa * n / (fa - n);
    // image-space shift in NDC (lens centering): x' = x + tx*w  ->  add tx * (-z) column
    m.m[8] -= tx; m.m[9] -= ty;
    return m;
}
// roll about the view axis (CCW in the image, x right / y up)
static M4 m_roll(float deg) {
    float c = cosf(deg * 0.0174533f), s = sinf(deg * 0.0174533f); M4 m = {{0}};
    m.m[0] = c; m.m[1] = s; m.m[4] = -s; m.m[5] = c; m.m[10] = 1; m.m[15] = 1;
    return m;
}
// inverse of rigid pose (q, p)
static M4 m_view(XrQuaternionf q, XrVector3f p) {
    float x = q.x, y = q.y, z = q.z, w = q.w; M4 r = {{0}};
    // R^T
    r.m[0] = 1 - 2*(y*y + z*z); r.m[4] = 2*(x*y + z*w);     r.m[8] = 2*(x*z - y*w);
    r.m[1] = 2*(x*y - z*w);     r.m[5] = 1 - 2*(x*x + z*z); r.m[9] = 2*(y*z + x*w);
    r.m[2] = 2*(x*z + y*w);     r.m[6] = 2*(y*z - x*w);     r.m[10] = 1 - 2*(x*x + y*y);
    r.m[12] = -(r.m[0]*p.x + r.m[4]*p.y + r.m[8]*p.z);
    r.m[13] = -(r.m[1]*p.x + r.m[5]*p.y + r.m[9]*p.z);
    r.m[14] = -(r.m[2]*p.x + r.m[6]*p.y + r.m[10]*p.z);
    r.m[15] = 1;
    return r;
}

/* ---------- geometry lists (x,y in tangent units, rgb) ---------- */
typedef struct { float *v; int n, cap; } List;
static void push(List *L, float x, float y, float r, float g, float b) {
    if (L->n + 1 > L->cap) { L->cap = L->cap ? L->cap * 2 : 4096; L->v = realloc(L->v, L->cap * 5 * sizeof(float)); }
    float *p = L->v + L->n * 5; p[0] = x; p[1] = y; p[2] = r; p[3] = g; p[4] = b; L->n++;
}
static void quad(List *L, float x0, float y0, float x1, float y1, float r, float g, float b) {
    push(L, x0, y0, r, g, b); push(L, x1, y0, r, g, b); push(L, x1, y1, r, g, b);
    push(L, x0, y0, r, g, b); push(L, x1, y1, r, g, b); push(L, x0, y1, r, g, b);
}
#define DEG 0.0174533f
static float T(float deg) { return tanf(deg * DEG); }
static void seg(List *L, float x0, float y0, float x1, float y1, float w, float r, float g, float b) {
    float dx = x1 - x0, dy = y1 - y0, l = sqrtf(dx*dx + dy*dy); if (l <= 0) return;
    float nx = -dy / l * w / 2, ny = dx / l * w / 2;
    push(L, x0+nx, y0+ny, r, g, b); push(L, x1+nx, y1+ny, r, g, b); push(L, x1-nx, y1-ny, r, g, b);
    push(L, x0+nx, y0+ny, r, g, b); push(L, x1-nx, y1-ny, r, g, b); push(L, x0-nx, y0-ny, r, g, b);
}
// line given in degrees
static void segd(List *L, float x0, float y0, float x1, float y1, float wdeg, float r, float g, float b) {
    seg(L, T(x0), T(y0), T(x1), T(y1), T(wdeg), r, g, b);
}
static void circle(List *L, float rdeg, float wdeg, float r, float g, float b) {
    float R = T(rdeg), w = T(wdeg);
    for (int i = 0; i < 96; i++) {
        float a0 = i * 6.2831853f / 96, a1 = (i + 1) * 6.2831853f / 96;
        seg(L, R*cosf(a0), R*sinf(a0), R*cosf(a1), R*sinf(a1), w, r, g, b);
    }
}

/* ---------- 3D scene geometry (x,y,z, rgb) in LOCAL space, metres, floor at y = FLOOR ---------- */
#define FLOOR (-1.6f)
typedef struct { float *v; int n, cap; } List3;
static void push3(List3 *L, float x, float y, float z, float r, float g, float b) {
    if (L->n + 1 > L->cap) { L->cap = L->cap ? L->cap * 2 : 4096; L->v = realloc(L->v, L->cap * 6 * sizeof(float)); }
    float *p = L->v + L->n * 6; p[0] = x; p[1] = y; p[2] = z; p[3] = r; p[4] = g; p[5] = b; L->n++;
}
static void quad3(List3 *L, const float *a, const float *b, const float *c, const float *d, float r, float g, float bl) {
    push3(L, a[0], a[1], a[2], r, g, bl); push3(L, b[0], b[1], b[2], r, g, bl); push3(L, c[0], c[1], c[2], r, g, bl);
    push3(L, a[0], a[1], a[2], r, g, bl); push3(L, c[0], c[1], c[2], r, g, bl); push3(L, d[0], d[1], d[2], r, g, bl);
}
// axis-aligned box, faces shaded by a fixed light so edges stay visible
static void box3(List3 *L, float x0, float y0, float z0, float x1, float y1, float z1, float r, float g, float b) {
    float v[8][3] = {{x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},{x0,y0,z1},{x1,y0,z1},{x1,y1,z1},{x0,y1,z1}};
    struct { int i[4]; float k; } f[6] = {{{4,5,6,7},0.85f},{{1,0,3,2},0.55f},{{5,1,2,6},0.7f},{{0,4,7,3},0.65f},{{7,6,2,3},1.0f},{{0,1,5,4},0.3f}};
    for (int i = 0; i < 6; i++) quad3(L, v[f[i].i[0]], v[f[i].i[1]], v[f[i].i[2]], v[f[i].i[3]], r*f[i].k, g*f[i].k, b*f[i].k);
}
static void build_scene(List3 *L) {
    float y = FLOOR;
    { float a[3] = {-60,y,-80}, b[3] = {60,y,-80}, c[3] = {60,y,40}, d[3] = {-60,y,40}; quad3(L, d, c, b, a, 0.08f, 0.09f, 0.1f); }
    // floor grid, 1 m
    for (int i = -40; i <= 40; i++) {
        float w = (i % 5 == 0) ? 0.03f : 0.012f, k = (i % 5 == 0) ? 0.6f : 0.35f;
        float a[3] = {i - w, y + 0.002f, -80}, b[3] = {i + w, y + 0.002f, -80}, c[3] = {i + w, y + 0.002f, 40}, d[3] = {i - w, y + 0.002f, 40};
        quad3(L, d, c, b, a, k, k, k);
        float e[3] = {-60, y + 0.003f, -i - w}, f[3] = {60, y + 0.003f, -i - w}, g[3] = {60, y + 0.003f, -i + w}, h[3] = {-60, y + 0.003f, -i + w};
        quad3(L, h, g, f, e, k, k, k);
    }
    // pillars at increasing distance, alternating sides (good for near/far fusion)
    static const float dist[] = {1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f, 12.0f, 18.0f, 26.0f, 40.0f};
    static const float col[][3] = {{0.9f,0.3f,0.3f},{0.3f,0.8f,0.3f},{0.3f,0.5f,1.0f},{1.0f,0.8f,0.2f},{0.8f,0.3f,0.9f},{0.2f,0.9f,0.9f}};
    for (int i = 0; i < 11; i++) {
        float d = dist[i], x = (i & 1 ? 1 : -1) * (0.35f + d * 0.25f), s = 0.08f + d * 0.02f; const float *c = col[i % 6];
        box3(L, x - s, y, -d - s, x + s, y + 1.6f + d * 0.15f, -d + s, c[0], c[1], c[2]);
        box3(L, x - s * 1.6f, y + 1.6f + d * 0.15f, -d - s * 1.6f, x + s * 1.6f, y + 1.6f + d * 0.15f + s, -d + s * 1.6f, 0.9f, 0.9f, 0.9f);
    }
    // small cubes on a table straight ahead, eye height-ish
    box3(L, -0.4f, y, -1.4f, 0.4f, y + 0.75f, -0.9f, 0.45f, 0.32f, 0.2f);
    for (int i = 0; i < 4; i++) box3(L, -0.3f + i * 0.18f, y + 0.75f, -1.2f, -0.22f + i * 0.18f, y + 0.83f + i * 0.03f, -1.12f, col[i][0], col[i][1], col[i][2]);
    // far wall with a grid of tiles, 50 m ahead
    for (int i = -10; i < 10; i++) for (int j = 0; j < 8; j++) {
        float k = ((i + j) & 1) ? 0.55f : 0.25f;
        box3(L, i * 3.0f + 0.1f, y + j * 3.0f + 0.1f, -50.5f, i * 3.0f + 2.9f, y + j * 3.0f + 2.9f, -50, k, k, k * 1.1f);
    }
    // horizontal + vertical bars at 5 m (sharp edges for checking rotation)
    box3(L, -3, -0.02f, -5.05f, 3, 0.02f, -5, 1, 1, 1);
    box3(L, -0.02f, -1.5f, -5.05f, 0.02f, 1.5f, -5, 1, 1, 1);
}

/* ---------- 5x7 font ---------- */
static const unsigned char *glyph(char c) {
    static const unsigned char digits[10][7] = {
        {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},{0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},{0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
        {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E},{0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},{0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
        {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},{0x1F,0x01,0x02,0x04,0x08,0x08,0x08},{0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
        {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}};
    static const unsigned char letters[26][7] = {
        {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},{0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},{0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
        {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C},{0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},{0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
        {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},{0x11,0x11,0x11,0x1F,0x11,0x11,0x11},{0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
        {0x07,0x02,0x02,0x02,0x02,0x12,0x0C},{0x11,0x12,0x14,0x18,0x14,0x12,0x11},{0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
        {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},{0x11,0x11,0x19,0x15,0x13,0x11,0x11},{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
        {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},{0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},{0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
        {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},{0x1F,0x04,0x04,0x04,0x04,0x04,0x04},{0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
        {0x11,0x11,0x11,0x11,0x11,0x0A,0x04},{0x11,0x11,0x11,0x15,0x15,0x15,0x0A},{0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
        {0x11,0x11,0x11,0x0A,0x04,0x04,0x04},{0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}};
    static const unsigned char plus[7] = {0,0x04,0x04,0x1F,0x04,0x04,0}, minus[7] = {0,0,0,0x1F,0,0,0},
        dot[7] = {0,0,0,0,0,0x0C,0x0C}, colon[7] = {0,0x0C,0x0C,0,0x0C,0x0C,0}, slash[7] = {0,0x01,0x02,0x04,0x08,0x10,0},
        gt[7] = {0x08,0x04,0x02,0x01,0x02,0x04,0x08}, eq[7] = {0,0,0x1F,0,0x1F,0,0},
        lp[7] = {0x02,0x04,0x08,0x08,0x08,0x04,0x02}, rp[7] = {0x08,0x04,0x02,0x02,0x02,0x04,0x08};
    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c >= 'A' && c <= 'Z') return letters[c - 'A'];
    switch (c) { case '+': return plus; case '-': return minus; case '.': return dot; case ':': return colon;
                 case '/': return slash; case '>': return gt; case '=': return eq; case '(': return lp; case ')': return rp; }
    return NULL;
}
// text centred at x=0, top at ydeg, pixel cell = cdeg degrees
static void text(List *L, const char *s, float ydeg, float cdeg, float r, float g, float b) {
    int n = strlen(s); float c = T(cdeg), x = -n * 6 * c / 2, y = T(ydeg);
    for (; *s; s++, x += 6 * c) {
        const unsigned char *gl = glyph(*s); if (!gl) continue;
        for (int row = 0; row < 7; row++) for (int col = 0; col < 5; col++)
            if (gl[row] & (0x10 >> col)) quad(L, x + col*c, y - (row+1)*c, x + (col+1)*c, y - row*c, r, g, b);
    }
}

/* ---------- settings ---------- */
typedef struct { float lens[2], pos[2], roll; } EyeOff;   // lens px (x right, y up), pos mm (x right, y up), roll deg (CCW +)
static EyeOff off[2];
// Distortion control points per eye: 7 along the crosshair's horizontal axis (h) and 7 along its vertical axis (v),
// index 3 = the crosshair centre (shared: h[3] == v[3]). Values = displacement in px in the rolled crosshair frame.
// Nominal positions: h[i] at ((i-3)*W/8, 0), v[i] at (0, (i-3)*H/8) from the lens centre.
typedef struct { float h[7][2], v[7][2]; } Warp;
static Warp wp[2];
static int warp_on;
static void cr7(float P[7][2], float u, float o[2]) {   // Catmull-Rom through the 7 points, constant beyond the ends
    u = u < 0 ? 0 : u > 6 ? 6 : u;
    int i = (int)floorf(u); if (i > 5) i = 5; float t = u - i;
    int i0 = i > 0 ? i - 1 : 0, i3 = i + 2 < 6 ? i + 2 : 6;
    for (int k = 0; k < 2; k++) {
        float p0 = P[i0][k], p1 = P[i][k], p2 = P[i + 1][k], p3 = P[i3][k];
        o[k] = 0.5f * (2*p1 + (-p0 + p2)*t + (2*p0 - 5*p1 + 4*p2 - p3)*t*t + (-p0 + 3*p1 - 3*p2 + p3)*t*t*t);
    }
}
static void warp_disp(Warp *w, float lx, float ly, float sx, float sy, float o[2]) {   // displacement at local (lx,ly)
    float a[2], b[2]; cr7(w->h, lx / sx + 3, a); cr7(w->v, ly / sy + 3, b);
    o[0] = a[0] + b[0] - w->h[3][0]; o[1] = a[1] + b[1] - w->h[3][1];
}
static char cfgpath[1024];
static void save_cfg(void) {
    char dir[1024]; snprintf(dir, sizeof dir, "%s/.config/lensalign", getenv("HOME")); mkdir(dir, 0755);
    FILE *f = fopen(cfgpath, "w"); if (!f) return;
    fprintf(f, "# eye lens_px_x lens_px_y pos_mm_x pos_mm_y roll_deg   (lensalign; in-app only)\n");
    for (int e = 0; e < 2; e++) fprintf(f, "%s %.2f %.2f %.3f %.3f %.3f\n", e ? "right" : "left", off[e].lens[0], off[e].lens[1], off[e].pos[0], off[e].pos[1], off[e].roll);
    fprintf(f, "warp_on %d\n", warp_on);
    for (int e = 0; e < 2; e++) {
        fprintf(f, "warp_%s", e ? "right" : "left");
        for (int i = 0; i < 7; i++) fprintf(f, " %.2f %.2f", wp[e].h[i][0], wp[e].h[i][1]);
        for (int i = 0; i < 7; i++) fprintf(f, " %.2f %.2f", wp[e].v[i][0], wp[e].v[i][1]);
        fprintf(f, "\n");
    }
    fclose(f);
    printf("saved: L lens %+.2f %+.2f px roll %+.3f deg | R lens %+.2f %+.2f px roll %+.3f deg\n",
           off[0].lens[0], off[0].lens[1], off[0].roll, off[1].lens[0], off[1].lens[1], off[1].roll);
    fflush(stdout);
}
static void load_cfg(void) {
    FILE *f = fopen(cfgpath, "r"); if (!f) return;
    char line[256], name[16]; EyeOff o;
    while (fgets(line, sizeof line, f)) {
        if (!strncmp(line, "warp_on ", 8)) { warp_on = atoi(line + 8) != 0; continue; }
        if (!strncmp(line, "warp_left ", 10) || !strncmp(line, "warp_right ", 11)) {
            Warp w; float *dst = &w.h[0][0]; char *q = strchr(line, ' '); int n = 0;
            for (; n < 28; n++) { char *end; float x = strtof(q, &end); if (end == q) break; q = end; dst[n] = x; }
            if (n == 28) { w.v[3][0] = w.h[3][0]; w.v[3][1] = w.h[3][1]; wp[line[5] == 'r'] = w; }
            continue;
        }
        o.roll = 0;   // older files have no roll column
        if (sscanf(line, "%15s %f %f %f %f %f", name, &o.lens[0], &o.lens[1], &o.pos[0], &o.pos[1], &o.roll) >= 5) {
            if (!strcmp(name, "left")) off[0] = o; else if (!strcmp(name, "right")) off[1] = o;
        }
    }
    fclose(f);
}

static const char *VS =
    "#version 430 core\n"
    "layout(location=0) in vec2 p; layout(location=1) in vec3 c;\n"
    "uniform mat4 mvp; uniform float D; out vec3 vc;\n"
    "void main(){ vc=c; gl_Position = mvp*vec4(p*D, -D, 1.0); }\n";
static const char *FS =
    "#version 430 core\n in vec3 vc; out vec4 o; void main(){ o = vec4(vc,1.0); }\n";
static const char *SVS =
    "#version 430 core\n"
    "layout(location=0) in vec3 p; layout(location=1) in vec3 c;\n"
    "uniform mat4 mvp; out vec3 vc;\n"
    "void main(){ vc=c; gl_Position = mvp*vec4(p, 1.0); }\n";
static const char *WVS =
    "#version 430 core\n"
    "void main(){ const vec2 P[3] = vec2[](vec2(-1,-1), vec2(3,-1), vec2(-1,3)); gl_Position = vec4(P[gl_VertexID], 0, 1); }\n";
static const char *WFS =
    "#version 430 core\n"
    "uniform sampler2D tex; uniform vec2 c, S, size, cs; uniform vec2 h[7], v[7]; out vec4 o;\n"
    "vec2 pt(int a, int i){ return a == 0 ? h[i] : v[i]; }\n"
    "vec2 cr(int a, float u){ u = clamp(u, 0.0, 6.0); int i = min(int(floor(u)), 5); float t = u - float(i);\n"
    "  vec2 p0 = pt(a, max(i-1, 0)), p1 = pt(a, i), p2 = pt(a, i+1), p3 = pt(a, min(i+2, 6));\n"
    "  return 0.5*(2.0*p1 + (-p0+p2)*t + (2.0*p0-5.0*p1+4.0*p2-p3)*t*t + (-p0+3.0*p1-3.0*p2+p3)*t*t*t); }\n"
    "vec2 D(vec2 l){ return cr(0, l.x/S.x + 3.0) + cr(1, l.y/S.y + 3.0) - h[3]; }\n"
    "void main(){ vec2 d = gl_FragCoord.xy - c;\n"
    "  vec2 q = vec2(cs.x*d.x + cs.y*d.y, -cs.y*d.x + cs.x*d.y);\n"          /* into the rolled crosshair frame */
    "  vec2 s = q; for (int k = 0; k < 5; k++) s = q - D(s);\n"              /* invert: find s with s + D(s) = q */
    "  vec2 w = c + vec2(cs.x*s.x - cs.y*s.y, cs.y*s.x + cs.x*s.y); vec2 uv = w / size;\n"
    "  o = (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) ? vec4(0,0,0,1) : texture(tex, uv); }\n";
static const char *TVS =
    "#version 430 core\n"
    "layout(location=0) in vec2 p; layout(location=1) in vec2 uv;\n"
    "uniform mat4 mvp; uniform float D; out vec2 vuv;\n"
    "void main(){ vuv=uv; gl_Position = mvp*vec4(p*D, -D, 1.0); }\n";
static const char *TFS =
    "#version 430 core\n in vec2 vuv; uniform sampler2D tex; out vec4 o; void main(){ o = texture(tex, vuv); }\n";

// decode a JPEG to RGB (top row first)
static unsigned char *load_jpeg(const char *path, int *w, int *h) {
    FILE *f = fopen(path, "rb"); if (!f) return NULL;
    struct jpeg_decompress_struct ci; struct jpeg_error_mgr err;
    ci.err = jpeg_std_error(&err); jpeg_create_decompress(&ci); jpeg_stdio_src(&ci, f);
    jpeg_read_header(&ci, TRUE); ci.out_color_space = JCS_RGB; jpeg_start_decompress(&ci);
    *w = ci.output_width; *h = ci.output_height;
    unsigned char *px = malloc((size_t)*w * *h * 3);
    while (ci.output_scanline < ci.output_height) {
        unsigned char *row = px + (size_t)ci.output_scanline * *w * 3; jpeg_read_scanlines(&ci, &row, 1);
    }
    jpeg_finish_decompress(&ci); jpeg_destroy_decompress(&ci); fclose(f);
    return px;
}

static GLuint mk_shader(GLenum t, const char *src) {
    GLuint s = glCreateShader(t); glShaderSource(s, 1, &src, NULL); glCompileShader(s);
    GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) { char log[1024]; glGetShaderInfoLog(s, 1024, NULL, log); fprintf(stderr, "shader: %s\n", log); exit(1); }
    return s;
}

static int has_ext(XrExtensionProperties *p, uint32_t n, const char *name) {
    for (uint32_t i = 0; i < n; i++) if (!strcmp(p[i].extensionName, name)) return 1;
    return 0;
}

int main(int argc, char **argv) {
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
    snprintf(cfgpath, sizeof cfgpath, "%s/.config/lensalign/offsets.txt", getenv("HOME"));
    load_cfg();

    // EGL / GL
    EGLDisplay dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY); eglInitialize(dpy, NULL, NULL); eglBindAPI(EGL_OPENGL_API);
    EGLint cattr[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
                      EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_NONE};
    EGLConfig cfg; EGLint ncfg;
    if (!eglChooseConfig(dpy, cattr, &cfg, 1, &ncfg) || !ncfg) { fprintf(stderr, "no EGL config\n"); return 1; }
    EGLint ctxattr[] = {EGL_CONTEXT_MAJOR_VERSION, 4, EGL_CONTEXT_MINOR_VERSION, 3,
                        EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT, EGL_NONE};
    EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctxattr);
    if (!ctx) { fprintf(stderr, "eglCreateContext failed 0x%x\n", eglGetError()); return 1; }
    eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx);
    printf("GL: %s\n", glGetString(GL_RENDERER));

    // OpenXR instance
    uint32_t next_n = 0; xrEnumerateInstanceExtensionProperties(NULL, 0, &next_n, NULL);
    XrExtensionProperties *ep = calloc(next_n, sizeof *ep);
    for (uint32_t i = 0; i < next_n; i++) ep[i].type = XR_TYPE_EXTENSION_PROPERTIES;
    xrEnumerateInstanceExtensionProperties(NULL, next_n, &next_n, ep);
    const char *exts[3]; uint32_t nexts = 0;
    exts[nexts++] = "XR_MNDX_egl_enable"; exts[nexts++] = "XR_KHR_opengl_enable";
    int have_frame = has_ext(ep, next_n, "XR_VALVE_frame_controller_interaction");
    if (have_frame) exts[nexts++] = "XR_VALVE_frame_controller_interaction";
    XrInstanceCreateInfo ici = {XR_TYPE_INSTANCE_CREATE_INFO};
    strcpy(ici.applicationInfo.applicationName, "Lens Align");
    ici.applicationInfo.apiVersion = XR_API_VERSION_1_0;
    ici.enabledExtensionCount = nexts; ici.enabledExtensionNames = exts;
    XrInstance inst; CHK(xrCreateInstance(&ici, &inst));
    XrSystemGetInfo sgi = {XR_TYPE_SYSTEM_GET_INFO}; sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XrSystemId sys; CHK(xrGetSystem(inst, &sgi, &sys));

    PFN_xrGetOpenGLGraphicsRequirementsKHR glreq;
    CHK(xrGetInstanceProcAddr(inst, "xrGetOpenGLGraphicsRequirementsKHR", (PFN_xrVoidFunction *)&glreq));
    XrGraphicsRequirementsOpenGLKHR req = {XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR}; CHK(glreq(inst, sys, &req));
    XrGraphicsBindingEGLMNDX gb = {XR_TYPE_GRAPHICS_BINDING_EGL_MNDX};
    gb.getProcAddress = eglGetProcAddress; gb.display = dpy; gb.config = cfg; gb.context = ctx;
    XrSessionCreateInfo sci = {XR_TYPE_SESSION_CREATE_INFO}; sci.next = &gb; sci.systemId = sys;
    XrSession sess; CHK(xrCreateSession(inst, &sci, &sess));

    XrReferenceSpaceCreateInfo rsi = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rsi.poseInReferenceSpace.orientation.w = 1;
    rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW; XrSpace vspace; CHK(xrCreateReferenceSpace(sess, &rsi, &vspace));
    rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL; XrSpace lspace; CHK(xrCreateReferenceSpace(sess, &rsi, &lspace));

    // swapchains
    XrViewConfigurationView vcv[2] = {{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    uint32_t nv; CHK(xrEnumerateViewConfigurationViews(inst, sys, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &nv, vcv));
    int64_t fmts[64]; uint32_t nf; CHK(xrEnumerateSwapchainFormats(sess, 64, &nf, fmts));
    int64_t fmt = fmts[0];
    for (uint32_t i = 0; i < nf; i++) if (fmts[i] == GL_SRGB8_ALPHA8) { fmt = GL_SRGB8_ALPHA8; break; }
    XrSwapchain sc[2]; XrSwapchainImageOpenGLKHR img[2][8]; uint32_t nimg[2]; GLuint fbo[2], msfbo[2], msrb[2], msdb[2], rtex[2], rfbo[2]; int W[2], H[2];
    for (int e = 0; e < 2; e++) {
        W[e] = vcv[e].recommendedImageRectWidth; H[e] = vcv[e].recommendedImageRectHeight;
        XrSwapchainCreateInfo ci = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
        ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
        ci.format = fmt; ci.sampleCount = 1; ci.width = W[e]; ci.height = H[e]; ci.faceCount = 1; ci.arraySize = 1; ci.mipCount = 1;
        CHK(xrCreateSwapchain(sess, &ci, &sc[e]));
        for (int i = 0; i < 8; i++) { img[e][i].type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR; img[e][i].next = NULL; }
        CHK(xrEnumerateSwapchainImages(sc[e], 8, &nimg[e], (XrSwapchainImageBaseHeader *)img[e]));
        glGenFramebuffers(1, &fbo[e]);
        // 4x MSAA render target, resolved into the swapchain image
        glGenFramebuffers(1, &msfbo[e]); glGenRenderbuffers(1, &msrb[e]);
        glBindRenderbuffer(GL_RENDERBUFFER, msrb[e]);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, (GLenum)fmt, W[e], H[e]);
        glBindFramebuffer(GL_FRAMEBUFFER, msfbo[e]);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, msrb[e]);
        glGenRenderbuffers(1, &msdb[e]); glBindRenderbuffer(GL_RENDERBUFFER, msdb[e]);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT24, W[e], H[e]);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, msdb[e]);
        // resolve target for the distortion pass
        glGenTextures(1, &rtex[e]); glBindTexture(GL_TEXTURE_2D, rtex[e]);
        glTexStorage2D(GL_TEXTURE_2D, 1, (GLenum)fmt, W[e], H[e]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glGenFramebuffers(1, &rfbo[e]); glBindFramebuffer(GL_FRAMEBUFFER, rfbo[e]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, rtex[e], 0);
        printf("eye %d: %dx%d\n", e, W[e], H[e]);
    }

    GLuint prog = glCreateProgram();
    glAttachShader(prog, mk_shader(GL_VERTEX_SHADER, VS)); glAttachShader(prog, mk_shader(GL_FRAGMENT_SHADER, FS));
    glLinkProgram(prog);
    GLint u_mvp = glGetUniformLocation(prog, "mvp"), u_D = glGetUniformLocation(prog, "D");

    // photo target (argument = another .jpg)
    char photo[1024];
    if (argc > 1) snprintf(photo, sizeof photo, "%s", argv[1]);
    else { snprintf(photo, sizeof photo, "%s", argv[0]); char *sl = strrchr(photo, '/');
           snprintf(sl ? sl + 1 : photo, sizeof photo - (sl ? sl + 1 - photo : 0), "assets/photo.jpg"); }
    int pw = 0, ph = 0; unsigned char *ppx = load_jpeg(photo, &pw, &ph);
    if (!ppx) { fprintf(stderr, "cannot load %s\n", photo); return 1; }
    printf("photo %s %dx%d\n", photo, pw, ph);
    GLuint tex; glGenTextures(1, &tex); glBindTexture(GL_TEXTURE_2D, tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8, pw, ph, 0, GL_RGB, GL_UNSIGNED_BYTE, ppx); free(ppx);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    GLuint tprog = glCreateProgram();
    glAttachShader(tprog, mk_shader(GL_VERTEX_SHADER, TVS)); glAttachShader(tprog, mk_shader(GL_FRAGMENT_SHADER, TFS));
    glLinkProgram(tprog);
    GLint ut_mvp = glGetUniformLocation(tprog, "mvp"), ut_D = glGetUniformLocation(tprog, "D");
    // photo quad: 45 deg wide, centred 3 deg up
    float qx = tanf(22.5f * DEG), qy = qx * ph / pw, qc = tanf(3 * DEG);
    float pq[] = {-qx, qc - qy, 0, 1,  qx, qc - qy, 1, 1,  qx, qc + qy, 1, 0,
                  -qx, qc - qy, 0, 1,  qx, qc + qy, 1, 0, -qx, qc + qy, 0, 0};
    GLuint pvao, pvbo; glGenVertexArrays(1, &pvao); glGenBuffers(1, &pvbo); glBindVertexArray(pvao); glBindBuffer(GL_ARRAY_BUFFER, pvbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof pq, pq, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 16, (void *)0);
    glEnableVertexAttribArray(1); glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 16, (void *)8);
    float text_top = atanf(qc - qy) / DEG - 1.5f;
    GLuint sprog = glCreateProgram();
    glAttachShader(sprog, mk_shader(GL_VERTEX_SHADER, SVS)); glAttachShader(sprog, mk_shader(GL_FRAGMENT_SHADER, FS));
    glLinkProgram(sprog);
    GLint us_mvp = glGetUniformLocation(sprog, "mvp");
    List3 scene = {0}; build_scene(&scene);
    GLuint svao, svbo; glGenVertexArrays(1, &svao); glGenBuffers(1, &svbo); glBindVertexArray(svao); glBindBuffer(GL_ARRAY_BUFFER, svbo);
    glBufferData(GL_ARRAY_BUFFER, (size_t)scene.n * 24, scene.v, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, (void *)0);
    glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 24, (void *)12);
    printf("3d scene: %d vertices\n", scene.n);
    GLuint wprog = glCreateProgram();
    glAttachShader(wprog, mk_shader(GL_VERTEX_SHADER, WVS)); glAttachShader(wprog, mk_shader(GL_FRAGMENT_SHADER, WFS));
    glLinkProgram(wprog);
    { GLint ok; glGetProgramiv(wprog, GL_LINK_STATUS, &ok); if (!ok) { char lg[1024]; glGetProgramInfoLog(wprog, 1024, NULL, lg); fprintf(stderr, "warp link: %s\n", lg); return 1; } }
    GLint uw_c = glGetUniformLocation(wprog, "c"), uw_S = glGetUniformLocation(wprog, "S"), uw_size = glGetUniformLocation(wprog, "size"),
          uw_cs = glGetUniformLocation(wprog, "cs"), uw_h = glGetUniformLocation(wprog, "h"), uw_v = glGetUniformLocation(wprog, "v");
    GLuint evao; glGenVertexArrays(1, &evao);
    static float cvb[8192 * 5];
    // crosshair: 2 lines in pixel space per eye
    GLuint cvao, cvbo; glGenVertexArrays(1, &cvao); glGenBuffers(1, &cvbo); glBindVertexArray(cvao); glBindBuffer(GL_ARRAY_BUFFER, cvbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof cvb, NULL, GL_STREAM_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 20, (void *)0);
    glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 20, (void *)8);
    GLuint vao, vbo; glGenVertexArrays(1, &vao); glGenBuffers(1, &vbo); glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 20, (void *)0);
    glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 20, (void *)8);

    // actions
    XrActionSetCreateInfo asci = {XR_TYPE_ACTION_SET_CREATE_INFO};
    strcpy(asci.actionSetName, "align"); strcpy(asci.localizedActionSetName, "Align");
    XrActionSet aset; CHK(xrCreateActionSet(inst, &asci, &aset));
    XrPath hand[2]; xrStringToPath(inst, "/user/hand/left", &hand[0]); xrStringToPath(inst, "/user/hand/right", &hand[1]);
    XrAction a_stick, a_click, a_eye, a_dist, a_fine, a_grip;
    struct { XrAction *a; const char *n; XrActionType t; } adef[] = {
        {&a_stick, "stick", XR_ACTION_TYPE_VECTOR2F_INPUT}, {&a_click, "reset", XR_ACTION_TYPE_BOOLEAN_INPUT},
        {&a_eye, "switch_eye", XR_ACTION_TYPE_BOOLEAN_INPUT}, {&a_dist, "distance", XR_ACTION_TYPE_BOOLEAN_INPUT},
        {&a_fine, "fine", XR_ACTION_TYPE_FLOAT_INPUT}, {&a_grip, "position_mode", XR_ACTION_TYPE_FLOAT_INPUT}};
    for (unsigned i = 0; i < sizeof adef / sizeof adef[0]; i++) {
        XrActionCreateInfo aci = {XR_TYPE_ACTION_CREATE_INFO};
        strcpy(aci.actionName, adef[i].n); strcpy(aci.localizedActionName, adef[i].n);
        aci.actionType = adef[i].t; aci.countSubactionPaths = 2; aci.subactionPaths = hand;
        CHK(xrCreateAction(aset, &aci, adef[i].a));
    }
    const char *profs[] = {"/interaction_profiles/valve/frame_controller", "/interaction_profiles/valve/index_controller"};
    for (int p = 0; p < 2; p++) {
        if (p == 0 && !have_frame) continue;
        // add bindings one by one and keep only the paths this runtime accepts (Frame left hand may use x/y)
        XrActionSuggestedBinding sb[32]; uint32_t n = 0;
        XrInteractionProfileSuggestedBinding s = {XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
        xrStringToPath(inst, profs[p], &s.interactionProfile);
        const char *hn[2] = {"left", "right"};
        for (int h = 0; h < 2; h++) {
            struct { XrAction a; const char *s; } b[] = {{a_stick, "thumbstick"}, {a_click, "thumbstick/click"},
                {a_eye, "a/click"}, {a_eye, "x/click"}, {a_dist, "b/click"}, {a_dist, "y/click"}, {a_fine, "trigger/value"}, {a_grip, "squeeze/value"}};
            for (int i = 0; i < 8; i++) {
                char path[128]; snprintf(path, sizeof path, "/user/hand/%s/input/%s", hn[h], b[i].s);
                sb[n].action = b[i].a;
                if (XR_FAILED(xrStringToPath(inst, path, &sb[n].binding))) continue;
                s.suggestedBindings = sb; s.countSuggestedBindings = n + 1;
                if (XR_SUCCEEDED(xrSuggestInteractionProfileBindings(inst, &s))) { n++; printf("  bound %s\n", path); }
            }
        }
        s.countSuggestedBindings = n;
        XrResult r = n ? xrSuggestInteractionProfileBindings(inst, &s) : XR_ERROR_PATH_UNSUPPORTED;
        printf("%s: %u bindings%s\n", profs[p], n, XR_FAILED(r) ? " (rejected)" : "");
    }
    XrSessionActionSetsAttachInfo att = {XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO}; att.countActionSets = 1; att.actionSets = &aset;
    CHK(xrAttachSessionActionSets(sess, &att));

    int running = 0, prev_dist[2] = {0}, prev_click[2] = {0}, dirty = 0, rot = 0;   // rot: stick adjusts roll instead of centering
    double last_t = now_s(), last_change = 0;
    int stick_active[2] = {-1, -1};
    int a_long[2] = {0}, b_long[2] = {0}; double a_down_t[2] = {0}, b_down_t[2] = {0};
    int view = 0, mode = 0, red_px = 0, prev_eye[2] = {0}, prev_grip[2] = {0}, prev_stick[2] = {0};   // mode 0 both, 1 left, 2 right
    float acc[2] = {0, 0}, base[2][2], base_roll[2];      // base = values at start (stick click returns to them)
    for (int e = 0; e < 2; e++) base_roll[e] = off[e].roll = roundf(off[e].roll * 100) / 100;
    for (int e = 0; e < 2; e++) for (int k = 0; k < 2; k++) base[e][k] = off[e].lens[k] = roundf(off[e].lens[k] * 4) / 4;
    List shared = {0}, only[2] = {{0}, {0}};
    Warp wbase[2] = {wp[0], wp[1]};                       // warp at start (left stick click in WARP EDIT returns to it)
    int sel_ax = 0, sel_k = 0, sel_held = 0; double sel_t = 0;              // selected control point: axis 0 = horizontal, 1 = vertical; k = -3..3
    float hs[2][2];                                       // stick per hand (WARP EDIT uses them separately)

    printf("lensalign running; config %s\n", cfgpath); fflush(stdout);
    while (!g_quit) {
        XrEventDataBuffer ev = {XR_TYPE_EVENT_DATA_BUFFER};
        while (xrPollEvent(inst, &ev) == XR_SUCCESS) {
            if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                XrSessionState st = ((XrEventDataSessionStateChanged *)&ev)->state;
                printf("session state %d\n", st); fflush(stdout);
                if (st == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo bi = {XR_TYPE_SESSION_BEGIN_INFO}; bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                    CHK(xrBeginSession(sess, &bi)); running = 1;
                } else if (st == XR_SESSION_STATE_STOPPING) { xrEndSession(sess); running = 0; }
                else if (st == XR_SESSION_STATE_EXITING || st == XR_SESSION_STATE_LOSS_PENDING) g_quit = 1;
            } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) g_quit = 1;
            ev.type = XR_TYPE_EVENT_DATA_BUFFER;
        }
        if (!running) { struct timespec ts = {0, 50000000}; nanosleep(&ts, NULL); continue; }

        XrFrameState fs = {XR_TYPE_FRAME_STATE}; CHK(xrWaitFrame(sess, NULL, &fs));
        CHK(xrBeginFrame(sess, NULL));
        double t = now_s(), dt = t - last_t; last_t = t; if (dt > 0.1) dt = 0.1;

        // input
        XrActiveActionSet aas = {aset, XR_NULL_PATH};
        XrActionsSyncInfo si = {XR_TYPE_ACTIONS_SYNC_INFO}; si.countActiveActionSets = 1; si.activeActionSets = &aas;
        xrSyncActions(sess, &si);
        float fine = 0;
        for (int h = 0; h < 2; h++) {
            XrActionStateGetInfo gi = {XR_TYPE_ACTION_STATE_GET_INFO}; gi.subactionPath = hand[h];
            XrActionStateFloat fv = {XR_TYPE_ACTION_STATE_FLOAT}; gi.action = a_fine; xrGetActionStateFloat(sess, &gi, &fv);
            if (fv.isActive && fv.currentState > fine) fine = fv.currentState;
        }
        float rate = fine > 0.5f ? 0.1f : 1.0f;
        // A = mode BOTH / LEFT / RIGHT, grip = red centre pixel on/off, B = distance, stick click = back to start value
        float stick[2] = {0, 0}; memset(hs, 0, sizeof hs);
        for (int h = 0; h < 2; h++) {
            XrActionStateGetInfo gi = {XR_TYPE_ACTION_STATE_GET_INFO}; gi.subactionPath = hand[h];
            XrActionStateBoolean bs = {XR_TYPE_ACTION_STATE_BOOLEAN};
            gi.action = a_eye; xrGetActionStateBoolean(sess, &gi, &bs);
            int v = bs.isActive && bs.currentState;
            // A/X on either hand: tap = eye mode, hold 0.5 s = next view
            if (v && !prev_eye[h]) { a_down_t[h] = t; a_long[h] = 0; printf("%s A/X down\n", h ? "right" : "left"); fflush(stdout); }
            if (v && !a_long[h] && t - a_down_t[h] >= 0.5) { a_long[h] = 1; view = (view + 1) % 4; if (view == 3 && mode == 0) mode = 1; acc[0] = acc[1] = 0; printf("view -> %d\n", view); fflush(stdout); }
            if (!v && prev_eye[h] && !a_long[h]) { mode = view == 3 ? (mode == 1 ? 2 : 1) : (mode + 1) % 3; acc[0] = acc[1] = 0; printf("mode -> %d\n", mode); fflush(stdout); }
            prev_eye[h] = v;
            gi.action = a_dist; xrGetActionStateBoolean(sess, &gi, &bs);
            v = bs.isActive && bs.currentState;
            // B: tap = MOVE/ROTATE (WARP EDIT: warp on/off), hold 0.5 s = warp on/off in any view
            if (v && !prev_dist[h]) { b_down_t[h] = t; b_long[h] = 0; }
            if (v && !b_long[h] && t - b_down_t[h] >= 0.5) { b_long[h] = 1; warp_on ^= 1; dirty = 1; last_change = t; printf("warp %s (hold B)\n", warp_on ? "APPLIED" : "OFF"); fflush(stdout); }
            if (!v && prev_dist[h] && !b_long[h] && view == 3) { warp_on ^= 1; dirty = 1; last_change = t; printf("warp %s\n", warp_on ? "APPLIED" : "OFF"); fflush(stdout); }
            else if (!v && prev_dist[h] && !b_long[h]) { rot ^= 1; acc[0] = acc[1] = 0; printf("%s B/Y -> %s\n", h ? "right" : "left", rot ? "ROTATE" : "MOVE"); fflush(stdout); }
            prev_dist[h] = v;
            gi.action = a_click; xrGetActionStateBoolean(sess, &gi, &bs);
            v = bs.isActive && bs.currentState;
            if (v && !prev_click[h] && view == 3) {   // right click = zero selected point, left click = this eye's warp back to start
                Warp *w = &wp[mode - 1];
                if (h == 1) { float *pt = sel_ax ? w->v[sel_k + 3] : w->h[sel_k + 3]; pt[0] = pt[1] = 0; }
                else *w = wbase[mode - 1];
                w->v[3][0] = w->h[3][0] = (sel_k == 0 && sel_ax) ? w->v[3][0] : w->h[3][0];
                w->v[3][1] = w->h[3][1] = (sel_k == 0 && sel_ax) ? w->v[3][1] : w->h[3][1];
                dirty = 1; last_change = t;
            } else if (v && !prev_click[h]) {
                for (int e = 0; e < 2; e++) if (mode == 0 || mode == e + 1) {
                    if (rot) off[e].roll = base_roll[e]; else { off[e].lens[0] = base[e][0]; off[e].lens[1] = base[e][1]; }
                }
                dirty = 1; last_change = t;
            }
            prev_click[h] = v;
            XrActionStateFloat fv = {XR_TYPE_ACTION_STATE_FLOAT}; gi.action = a_grip; xrGetActionStateFloat(sess, &gi, &fv);
            v = fv.isActive && fv.currentState > 0.6f; if (v && !prev_grip[h]) red_px ^= 1; prev_grip[h] = v;
            XrActionStateVector2f vs = {XR_TYPE_ACTION_STATE_VECTOR2F}; gi.action = a_stick; xrGetActionStateVector2f(sess, &gi, &vs);
            if (vs.isActive != stick_active[h]) {
                printf("%s stick %s\n", h ? "right" : "left", vs.isActive ? "active" : "NOT active"); fflush(stdout);
                stick_active[h] = vs.isActive;
            }
            if (!vs.isActive) continue;
            hs[h][0] = vs.currentState.x; hs[h][1] = vs.currentState.y;
            if (fabsf(vs.currentState.x) > fabsf(stick[0])) stick[0] = vs.currentState.x;
            if (fabsf(vs.currentState.y) > fabsf(stick[1])) stick[1] = vs.currentState.y;
        }
        // stick -> 0.25 px steps. BOTH: X = out(+)/in(-) mirrored, Y = both up/down together. LEFT/RIGHT: that eye only.
        // ROTATE: stick X clocks the image about the lens center in 0.01 deg steps.
        //   BOTH = relative (left +, right -), LEFT/RIGHT = that eye only. + = counter-clockwise.
        if (view == 3) {
            // WARP EDIT: left stick flick = select point along the cross, right stick = move it (0.25 px steps)
            int e = mode - 1; float lx = hs[0][0], ly = hs[0][1];
            // forgiving selection: any push past 0.45 counts; the axis you are on wins unless you push clearly
            // along the other one (> ~63 deg off); holding repeats every 0.25 s after 0.45 s
            float mag = sqrtf(lx * lx + ly * ly);
            int ax = -1, dir = 0;
            if (mag > 0.45f) {
                float bias = sel_k != 0 ? 2.0f : 1.0f;            // stick to the current line
                int horiz = sel_ax == 0 ? fabsf(lx) * bias >= fabsf(ly) : fabsf(lx) > fabsf(ly) * bias;
                ax = horiz ? 0 : 1; dir = (horiz ? lx : ly) > 0 ? 1 : -1;
            }
            int step = 0;
            if (ax >= 0 && !sel_held) { step = 1; sel_t = t + 0.45; }
            else if (ax >= 0 && t >= sel_t) { step = 1; sel_t = t + 0.25; }
            if (step) {
                int k = sel_ax == ax ? sel_k : 0; k += dir; if (k < -3) k = -3; if (k > 3) k = 3;
                sel_ax = ax; sel_k = k;
            }
            sel_held = mag > 0.25f;
            for (int k = 0; k < 2; k++) {
                float sv = hs[1][k];
                if (fabsf(sv) < 0.15f) { acc[k] = 0; prev_stick[k] = 0; continue; }
                float m = (fabsf(sv) - 0.15f) / 0.85f; m = m * m * (sv < 0 ? -1 : 1);
                acc[k] += m * (rate < 1 ? 1.0f : 10.0f) * dt;
                if (!prev_stick[k]) acc[k] += sv < 0 ? -0.25f : 0.25f;
                prev_stick[k] = 1;
                while (fabsf(acc[k]) >= 0.25f) {
                    float d = acc[k] > 0 ? 0.25f : -0.25f; acc[k] -= d;
                    if (sel_k == 0) { wp[e].h[3][k] += d; wp[e].v[3][k] = wp[e].h[3][k]; }
                    else if (sel_ax) wp[e].v[sel_k + 3][k] += d; else wp[e].h[sel_k + 3][k] += d;
                    dirty = 1; last_change = t;
                }
            }
        }
        else if (rot) {
            const float RS = 0.01f;
            if (fabsf(stick[0]) < 0.15f) acc[0] = 0;
            else {
                float m = (fabsf(stick[0]) - 0.15f) / 0.85f; m = m * m * (stick[0] < 0 ? -1 : 1);
                acc[0] += m * (rate < 1 ? 0.1f : 1.0f) * dt;       // deg per second at full deflection
                if (prev_stick[0] == 0) acc[0] += stick[0] < 0 ? -RS : RS;
                while (fabsf(acc[0]) >= RS) {
                    float d = acc[0] > 0 ? RS : -RS; acc[0] -= d;
                    if (mode == 0) { off[0].roll += d / 2; off[1].roll -= d / 2; }
                    else off[mode - 1].roll += d;
                    dirty = 1; last_change = t;
                }
            }
            prev_stick[0] = fabsf(stick[0]) >= 0.15f; prev_stick[1] = 0;
        }
        else for (int k = 0; k < 2; k++) {
            if (fabsf(stick[k]) < 0.15f) { acc[k] = 0; continue; }
            float m = (fabsf(stick[k]) - 0.15f) / 0.85f; m = m * m * (stick[k] < 0 ? -1 : 1);
            acc[k] += m * (rate < 1 ? 1.0f : 10.0f) * dt;          // px per second
            if (prev_stick[k] == 0) acc[k] += stick[k] < 0 ? -0.25f : 0.25f;   // first touch = one step at once
            while (fabsf(acc[k]) >= 0.25f) {
                float d = acc[k] > 0 ? 0.25f : -0.25f; acc[k] -= d;
                if (mode == 0) { if (k == 0) { off[0].lens[0] -= d; off[1].lens[0] += d; } else { off[0].lens[1] += d; off[1].lens[1] += d; } }
                else off[mode - 1].lens[k] += d;
                dirty = 1; last_change = t;
            }
        }
        if (!rot && view != 3) for (int k = 0; k < 2; k++) prev_stick[k] = fabsf(stick[k]) >= 0.15f;
        if (dirty && t - last_change > 1.0) { save_cfg(); dirty = 0; }

        XrViewLocateInfo vli = {XR_TYPE_VIEW_LOCATE_INFO};
        vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO; vli.displayTime = fs.predictedDisplayTime; vli.space = vspace;
        XrViewState vst = {XR_TYPE_VIEW_STATE}; XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}}; uint32_t nvo;
        CHK(xrLocateViews(sess, &vli, &vst, 2, &nvo, views));
        XrViewState vstl = {XR_TYPE_VIEW_STATE}; XrView lviews[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
        vli.space = lspace; CHK(xrLocateViews(sess, &vli, &vstl, 2, &nvo, lviews));
        int world = view == 1 && (vstl.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) && (vstl.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT);
        int render = fs.shouldRender && (vst.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT);

        XrCompositionLayerProjectionView pv[2];
        if (render) {
            shared.n = only[0].n = only[1].n = 0;
            // text
            char line[128]; float y = text_top, cs = 0.16f;
            const char *mn[3] = {"BOTH EYES", "LEFT EYE ONLY", "RIGHT EYE ONLY"};
            const char *vn[4] = {"PHOTO", "3D SCENE", "CROSSHAIR", "WARP EDIT"};
            snprintf(line, sizeof line, "VIEW: %s   WARP: %s", vn[view], warp_on ? "APPLIED" : "OFF");
            text(&shared, line, y + 11 * cs * 1.3f, cs * 1.3f, 0.5f, 0.9f, 1.0f);
            if (view == 3) {
                Warp *w = &wp[mode - 1]; float *pt = sel_ax ? w->v[sel_k + 3] : w->h[sel_k + 3];
                snprintf(line, sizeof line, "WARP EDIT  %s EYE   POINT %s %+d   DX %+.2f  DY %+.2f PX", mode == 1 ? "LEFT" : "RIGHT",
                         sel_k == 0 ? "CENTER" : (sel_ax ? "V" : "H"), sel_k, pt[0], pt[1]);
                text(&shared, line, y, cs * 1.3f, 1, 1, 0.4f); y -= 11 * cs * 1.3f;
                if (rate < 1) text(&shared, "FINE", y, cs, 0.8f, 0.8f, 0.8f);
                y -= 12 * cs;
                text(&shared, "LEFT STICK = SELECT POINT   RIGHT STICK = MOVE POINT   TRIGGER = FINE   A TAP = EYE", y, cs * 0.8f, 0.6f, 0.6f, 0.6f); y -= 9 * cs * 0.8f;
                text(&shared, "B = WARP ON / OFF   RIGHT CLICK = ZERO POINT   LEFT CLICK = EYE BACK TO START   A HOLD = VIEW", y, cs * 0.8f, 0.6f, 0.6f, 0.6f);
            } else {
            snprintf(line, sizeof line, "MODE: %s   STICK: %s", mn[mode], rot ? (mode ? "ROTATE (X)" : "ROTATE RELATIVE (X)") : (mode ? "MOVE" : "MOVE (X = IN/OUT, Y = UP/DOWN)"));
            text(&shared, line, y, cs * 1.3f, 1, 1, 0.4f); y -= 11 * cs * 1.3f;
            for (int e = 0; e < 2; e++) {
                float pdx = (tanf(views[e].fov.angleRight) - tanf(views[e].fov.angleLeft)) / W[e];
                snprintf(line, sizeof line, "%s LENS CENTER  X %+7.2f  Y %+7.2f PX   (%+.3f %+.3f DEG)", e ? "RIGHT" : "LEFT ",
                         off[e].lens[0], off[e].lens[1], atanf(off[e].lens[0] * pdx) / DEG, atanf(off[e].lens[1] * pdx) / DEG);
                int act = mode == 0 || mode == e + 1;
                text(&shared, line, y, cs, act ? 1 : 0.5f, act ? 1 : 0.5f, act ? 1 : 0.5f); y -= 9 * cs;
            }
            snprintf(line, sizeof line, "ROLL  LEFT %+.2f DEG   RIGHT %+.2f DEG   (DIFF %+.2f)", off[0].roll, off[1].roll, off[0].roll - off[1].roll);
            text(&shared, line, y, cs, rot ? 1 : 0.7f, rot ? 1 : 0.7f, rot ? 0.4f : 0.7f); y -= 9 * cs;
            snprintf(line, sizeof line, "CENTER PIXELS (L BLUE / R RED): %s", red_px ? "ON" : "OFF");
            text(&shared, line, y, cs, 1, red_px ? 0.3f : 0.8f, red_px ? 0.3f : 0.8f); y -= 9 * cs;
            if (rate < 1) { text(&shared, "FINE", y, cs, 0.8f, 0.8f, 0.8f); }
            y -= 12 * cs;
            text(&shared, "A TAP = BOTH / LEFT / RIGHT   A HOLD = PHOTO / 3D / CROSSHAIR / WARP EDIT   TRIGGER = FINE", y, cs * 0.8f, 0.6f, 0.6f, 0.6f); y -= 9 * cs * 0.8f;
            text(&shared, "B TAP = MOVE / ROTATE   B HOLD = WARP ON / OFF   GRIP = CENTER PIXELS   STICK CLICK = BACK TO START", y, cs * 0.8f, 0.6f, 0.6f, 0.6f);
            }

            int nsh = shared.n;
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, (size_t)(nsh + only[0].n + only[1].n) * 20, NULL, GL_STREAM_DRAW);
            glBufferSubData(GL_ARRAY_BUFFER, 0, (size_t)nsh * 20, shared.v);
            glBufferSubData(GL_ARRAY_BUFFER, (size_t)nsh * 20, (size_t)only[0].n * 20, only[0].v);
            glBufferSubData(GL_ARRAY_BUFFER, (size_t)(nsh + only[0].n) * 20, (size_t)only[1].n * 20, only[1].v);

            float D = 10.0f;     // photo is 10 m away (user asked)
            for (int e = 0; e < 2; e++) {
                uint32_t ii; XrSwapchainImageAcquireInfo ai = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
                CHK(xrAcquireSwapchainImage(sc[e], &ai, &ii));
                XrSwapchainImageWaitInfo wi = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO}; wi.timeout = XR_INFINITE_DURATION;
                CHK(xrWaitSwapchainImage(sc[e], &wi));

                glBindFramebuffer(GL_FRAMEBUFFER, msfbo[e]);
                glViewport(0, 0, W[e], H[e]);
                glEnable(GL_FRAMEBUFFER_SRGB); glEnable(GL_MULTISAMPLE); glDisable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE);
                glClearColor(0, 0, 0, 1); glClearDepth(1); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                // eye pose shifted by the position offset (head frame), projection shifted by lens centering
                XrPosef ep2 = views[e].pose;
                // (eye position offsets are not used)
                float tx = 2.0f * off[e].lens[0] / W[e], ty = 2.0f * off[e].lens[1] / H[e];
                M4 mvp = mmul(m_proj(views[e].fov, 0.05f, 3000.0f, tx, ty), mmul(m_roll(off[e].roll), m_view(ep2.orientation, ep2.position)));
                if (world) {   // world-locked 3D scene, same lens shift + roll
                    M4 smvp = mmul(m_proj(lviews[e].fov, 0.05f, 3000.0f, tx, ty), mmul(m_roll(off[e].roll), m_view(lviews[e].pose.orientation, lviews[e].pose.position)));
                    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
                    glUseProgram(sprog); glUniformMatrix4fv(us_mvp, 1, GL_FALSE, smvp.m);
                    glBindVertexArray(svao); glDrawArrays(GL_TRIANGLES, 0, scene.n);
                    glDisable(GL_DEPTH_TEST);
                } else if (view == 0) {
                    glUseProgram(tprog); glUniformMatrix4fv(ut_mvp, 1, GL_FALSE, mvp.m); glUniform1f(ut_D, D);
                    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, tex); glBindVertexArray(pvao);
                    glDrawArrays(GL_TRIANGLES, 0, 6);
                }
                glUseProgram(prog); glUniformMatrix4fv(u_mvp, 1, GL_FALSE, mvp.m); glUniform1f(u_D, D);
                glBindVertexArray(vao);
                glDrawArrays(GL_TRIANGLES, 0, nsh);   // HUD text, head-locked, on top

                // lens centre of this eye in swapchain pixels (pixel centre), roll, control-point spacing
                float fl = tanf(views[e].fov.angleLeft), fr = tanf(views[e].fov.angleRight);
                float fu = tanf(views[e].fov.angleUp), fd = tanf(views[e].fov.angleDown);
                float cx = floorf((-fl / (fr - fl)) * W[e] + off[e].lens[0]) + 0.5f, cy = floorf((-fd / (fu - fd)) * H[e] + off[e].lens[1]) + 0.5f;
                float rc = cosf(off[e].roll * DEG), rs = sinf(off[e].roll * DEG), Sx = W[e] / 8.0f, Sy = H[e] / 8.0f;
                int warped = warp_on || view == 3;

                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo[e]);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, img[e][ii].image, 0);
                glBindFramebuffer(GL_READ_FRAMEBUFFER, msfbo[e]);
                if (warp_on) {   // resolve, then distort into the swapchain image
                    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, rfbo[e]);
                    glBlitFramebuffer(0, 0, W[e], H[e], 0, 0, W[e], H[e], GL_COLOR_BUFFER_BIT, GL_NEAREST);
                    glBindFramebuffer(GL_FRAMEBUFFER, fbo[e]); glViewport(0, 0, W[e], H[e]); glDisable(GL_MULTISAMPLE);
                    glUseProgram(wprog);
                    glUniform2f(uw_c, cx, cy); glUniform2f(uw_S, Sx, Sy); glUniform2f(uw_size, (float)W[e], (float)H[e]); glUniform2f(uw_cs, rc, rs);
                    glUniform2fv(uw_h, 7, &wp[e].h[0][0]); glUniform2fv(uw_v, 7, &wp[e].v[0][0]);
                    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, rtex[e]);
                    glBindVertexArray(evao); glDrawArrays(GL_TRIANGLES, 0, 3);
                    glEnable(GL_MULTISAMPLE);
                } else glBlitFramebuffer(0, 0, W[e], H[e], 0, 0, W[e], H[e], GL_COLOR_BUFFER_BIT, GL_NEAREST);
                if (view >= 2) {   // 1-pixel crosshair through the lens centre, rotated by the roll, bent by the warp; left blue, right red
                    float L = (float)(W[e] + H[e]), cr = e ? 1 : 0, cb = e ? 0 : 1; int n = 0;
                    #define CPUSH(lx_, ly_, R_, G_, B_) do { float *q_ = cvb + n * 5; q_[0] = cx + rc*(lx_) - rs*(ly_); q_[1] = cy + rs*(lx_) + rc*(ly_); \
                                                            q_[2] = R_; q_[3] = G_; q_[4] = B_; n++; } while (0)
                    for (int ax = 0; ax < 2; ax++)
                        for (float a = -L; a <= L; a += 4) {
                            float lx = ax ? 0 : a, ly = ax ? a : 0, dd[2] = {0, 0};
                            if (warped) warp_disp(&wp[e], lx, ly, Sx, Sy, dd);
                            CPUSH(lx + dd[0], ly + dd[1], cr, 0, cb);
                        }
                    int nline = n / 2, nmark = 0;
                    if (view == 3 && e == mode - 1)   // control points of the eye being edited
                        for (int ax = 0; ax < 2; ax++) for (int k = -3; k <= 3; k++) {
                            if (ax && k == 0) continue;
                            float *pt = ax ? wp[e].v[k + 3] : wp[e].h[k + 3];
                            float px = (ax ? 0 : k * Sx) + pt[0], py = (ax ? k * Sy : 0) + pt[1];
                            int sel = (k == 0 && sel_k == 0) || (ax == sel_ax && k == sel_k);
                            float hz = sel ? 9 : 5, g = sel ? 1 : 0.6f, R = sel ? 1 : cr * 0.6f + 0.4f, B = sel ? 1 : cb * 0.6f + 0.4f;
                            CPUSH(px - hz, py - hz, R, g, B); CPUSH(px + hz, py - hz, R, g, B); CPUSH(px + hz, py + hz, R, g, B); CPUSH(px - hz, py + hz, R, g, B);
                            nmark++;
                        }
                    #undef CPUSH
                    M4 ortho = {{0}}; ortho.m[0] = 2.0f / W[e]; ortho.m[5] = 2.0f / H[e]; ortho.m[12] = -1; ortho.m[13] = -1; ortho.m[15] = 1;
                    glBindFramebuffer(GL_FRAMEBUFFER, fbo[e]); glViewport(0, 0, W[e], H[e]); glDisable(GL_MULTISAMPLE);
                    glBindBuffer(GL_ARRAY_BUFFER, cvbo); glBufferSubData(GL_ARRAY_BUFFER, 0, (size_t)n * 20, cvb);
                    glUseProgram(prog); glUniformMatrix4fv(u_mvp, 1, GL_FALSE, ortho.m); glUniform1f(u_D, 1);
                    glBindVertexArray(cvao); glLineWidth(1);
                    glDrawArrays(GL_LINE_STRIP, 0, nline); glDrawArrays(GL_LINE_STRIP, nline, nline);
                    for (int i = 0; i < nmark; i++) glDrawArrays(GL_LINE_LOOP, 2 * nline + 4 * i, 4);
                    glEnable(GL_MULTISAMPLE);
                }
                if (red_px) {   // one pixel at the (shifted) centre of this eye's image: left blue, right red
                    float l = tanf(views[e].fov.angleLeft), r = tanf(views[e].fov.angleRight);
                    float u = tanf(views[e].fov.angleUp), d = tanf(views[e].fov.angleDown);
                    float cx = (-l / (r - l)) * W[e] + off[e].lens[0], cy = (-d / (u - d)) * H[e] + off[e].lens[1];
                    if (warped) { cx += rc * wp[e].h[3][0] - rs * wp[e].h[3][1]; cy += rs * wp[e].h[3][0] + rc * wp[e].h[3][1]; }
                    glBindFramebuffer(GL_FRAMEBUFFER, fbo[e]);
                    glEnable(GL_SCISSOR_TEST); glScissor((int)floorf(cx), (int)floorf(cy), 1, 1);
                    if (e == 0) glClearColor(0, 0, 1, 1); else glClearColor(1, 0, 0, 1);   // left = blue, right = red
                    glClear(GL_COLOR_BUFFER_BIT); glDisable(GL_SCISSOR_TEST);
                }
                glBindFramebuffer(GL_FRAMEBUFFER, 0);

                XrSwapchainImageReleaseInfo ri = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                CHK(xrReleaseSwapchainImage(sc[e], &ri));
                // the layer declares the TRUE eye pose/fov, so the offsets above really move the image
                pv[e] = (XrCompositionLayerProjectionView){XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};
                pv[e].pose = world ? lviews[e].pose : views[e].pose; pv[e].fov = world ? lviews[e].fov : views[e].fov;
                pv[e].subImage.swapchain = sc[e]; pv[e].subImage.imageRect.extent.width = W[e]; pv[e].subImage.imageRect.extent.height = H[e];
            }
            glFlush();
        }
        XrCompositionLayerProjection layer = {XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        int world_layer = render && world; layer.space = world_layer ? lspace : vspace; layer.viewCount = 2; layer.views = pv;
        const XrCompositionLayerBaseHeader *layers[] = {(XrCompositionLayerBaseHeader *)&layer};
        XrFrameEndInfo fei = {XR_TYPE_FRAME_END_INFO};
        fei.displayTime = fs.predictedDisplayTime; fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        fei.layerCount = render ? 1 : 0; fei.layers = layers;
        CHK(xrEndFrame(sess, &fei));
    }
    save_cfg();
    if (running) xrEndSession(sess);
    xrDestroySession(sess); xrDestroyInstance(inst);
    return 0;
}
