// GlPresenter.cpp - see GlPresenter.hpp
#include "GlPresenter.hpp"

#include <algorithm>
#include <cmath>

#include <GL/gl.h>

#include "uevr/API.hpp"

namespace fpv {

namespace {

// --- Minimal OpenGL 2.0 / 3.0 loader (only what we use) -----------------------------------------
using GLchar = char;
constexpr GLenum kFragmentShader = 0x8B30, kVertexShader = 0x8B31, kCompileStatus = 0x8B81, kLinkStatus = 0x8B82;
constexpr GLenum kBgra = 0x80E1, kClampToEdge = 0x812F, kTexture0 = 0x84C0;

using PFNCreateShader = GLuint(WINAPI*)(GLenum);
using PFNShaderSource = void(WINAPI*)(GLuint, GLsizei, const GLchar* const*, const GLint*);
using PFNCompileShader = void(WINAPI*)(GLuint);
using PFNGetShaderiv = void(WINAPI*)(GLuint, GLenum, GLint*);
using PFNGetShaderInfoLog = void(WINAPI*)(GLuint, GLsizei, GLsizei*, GLchar*);
using PFNCreateProgram = GLuint(WINAPI*)();
using PFNAttachShader = void(WINAPI*)(GLuint, GLuint);
using PFNLinkProgram = void(WINAPI*)(GLuint);
using PFNGetProgramiv = void(WINAPI*)(GLuint, GLenum, GLint*);
using PFNUseProgram = void(WINAPI*)(GLuint);
using PFNGetUniformLocation = GLint(WINAPI*)(GLuint, const GLchar*);
using PFNUniform1f = void(WINAPI*)(GLint, GLfloat);
using PFNUniform2f = void(WINAPI*)(GLint, GLfloat, GLfloat);
using PFNUniform1i = void(WINAPI*)(GLint, GLint);
using PFNGenVertexArrays = void(WINAPI*)(GLsizei, GLuint*);
using PFNBindVertexArray = void(WINAPI*)(GLuint);
using PFNActiveTexture = void(WINAPI*)(GLenum);
using PFNSwapInterval = BOOL(WINAPI*)(int);

struct Gl {
    PFNCreateShader CreateShader{};
    PFNShaderSource ShaderSource{};
    PFNCompileShader CompileShader{};
    PFNGetShaderiv GetShaderiv{};
    PFNGetShaderInfoLog GetShaderInfoLog{};
    PFNCreateProgram CreateProgram{};
    PFNAttachShader AttachShader{};
    PFNLinkProgram LinkProgram{};
    PFNGetProgramiv GetProgramiv{};
    PFNUseProgram UseProgram{};
    PFNGetUniformLocation GetUniformLocation{};
    PFNUniform1f Uniform1f{};
    PFNUniform2f Uniform2f{};
    PFNUniform1i Uniform1i{};
    PFNGenVertexArrays GenVertexArrays{};
    PFNBindVertexArray BindVertexArray{};
    PFNActiveTexture ActiveTexture{};
    PFNSwapInterval SwapInterval{};

    template <typename T>
    static bool load(T& fn, const char* name) {
        auto p = wglGetProcAddress(name);
        const auto v = reinterpret_cast<intptr_t>(p);
        if (v == 0 || v == 1 || v == 2 || v == 3 || v == -1) return false;
        fn = reinterpret_cast<T>(p);
        return true;
    }

    bool load_all() {
        return load(CreateShader, "glCreateShader") && load(ShaderSource, "glShaderSource") &&
               load(CompileShader, "glCompileShader") && load(GetShaderiv, "glGetShaderiv") &&
               load(GetShaderInfoLog, "glGetShaderInfoLog") && load(CreateProgram, "glCreateProgram") &&
               load(AttachShader, "glAttachShader") && load(LinkProgram, "glLinkProgram") &&
               load(GetProgramiv, "glGetProgramiv") && load(UseProgram, "glUseProgram") &&
               load(GetUniformLocation, "glGetUniformLocation") && load(Uniform1f, "glUniform1f") &&
               load(Uniform2f, "glUniform2f") && load(Uniform1i, "glUniform1i") &&
               load(ActiveTexture, "glActiveTexture") &&
               (load(GenVertexArrays, "glGenVertexArrays") || true) && (load(BindVertexArray, "glBindVertexArray") || true) &&
               (load(SwapInterval, "wglSwapIntervalEXT") || true);
    }
};

Gl g;

constexpr const char* kVertex = R"(#version 130
out vec2 uv;
void main() {
    // Fullscreen triangle; uv (0,0) = top-left of the image.
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    uv = vec2(p.x, p.y);
    gl_Position = vec4(p.x * 2.0 - 1.0, 1.0 - p.y * 2.0, 0.0, 1.0);
}
)";

constexpr const char* kFragment = R"(#version 130
in vec2 uv;
out vec4 frag;
uniform sampler2D tex;
uniform vec2 src_size;   // texels
uniform vec2 dst_size;   // pixels of the letterboxed viewport
uniform float sharpness;
uniform float gain;      // exposure gain (linear light); 1 when the capture is exposed at the source
uniform float inv_gamma;
uniform float analog;    // 0 / 1
uniform float noise, softness, scanlines;
uniform float lines;     // video lines (scanlines / grain size)
uniform float time;

vec3 tap(vec2 p) { return texture(tex, p).rgb; }

// Catmull-Rom (sharp, smooth upscale) with 9 bilinear taps.
vec3 catmull(vec2 p) {
    vec2 pos = p * src_size;
    vec2 tc = floor(pos - 0.5) + 0.5;
    vec2 f = pos - tc;
    vec2 w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
    vec2 w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
    vec2 w2 = f * (0.5 + f * (2.0 - 1.5 * f));
    vec2 w3 = f * f * (-0.5 + 0.5 * f);
    vec2 w12 = w1 + w2;
    vec2 t0 = (tc - 1.0) / src_size;
    vec2 t3 = (tc + 2.0) / src_size;
    vec2 t12 = (tc + w2 / w12) / src_size;
    vec3 c = tap(vec2(t0.x, t0.y)) * w0.x * w0.y + tap(vec2(t12.x, t0.y)) * w12.x * w0.y + tap(vec2(t3.x, t0.y)) * w3.x * w0.y
           + tap(vec2(t0.x, t12.y)) * w0.x * w12.y + tap(vec2(t12.x, t12.y)) * w12.x * w12.y + tap(vec2(t3.x, t12.y)) * w3.x * w12.y
           + tap(vec2(t0.x, t3.y)) * w0.x * w3.y + tap(vec2(t12.x, t3.y)) * w12.x * w3.y + tap(vec2(t3.x, t3.y)) * w3.x * w3.y;
    return max(c, vec3(0.0));
}

// Image sample at p: box filter when the capture is larger than the window (supersampling),
// Catmull-Rom when it is smaller.
vec3 sample_img(vec2 p) {
    vec2 r = src_size / dst_size; // texels per pixel
    if (max(r.x, r.y) > 1.05) {
        vec2 o = 0.25 * r / src_size;
        return 0.25 * (tap(p + vec2(-o.x, -o.y)) + tap(p + vec2(o.x, -o.y)) + tap(p + vec2(-o.x, o.y)) + tap(p + vec2(o.x, o.y)));
    }
    return catmull(p);
}

float luma(vec3 c) { return dot(c, vec3(0.299, 0.587, 0.114)); }

float hash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }

void main() {
    vec2 px = 1.0 / dst_size;
    vec3 c = sample_img(uv);

    // Sharpening (contrast-limited unsharp mask, in window pixels).
    if (sharpness > 0.001) {
        vec2 o = max(px, 1.0 / src_size); // one source texel when upscaling, one window pixel otherwise
        vec3 n = 0.25 * (tap(uv + vec2(o.x, 0.0)) + tap(uv - vec2(o.x, 0.0)) + tap(uv + vec2(0.0, o.y)) + tap(uv - vec2(0.0, o.y)));
        vec3 d = clamp(c - n, -0.12, 0.12);
        c = max(c + d * sharpness * 1.6, vec3(0.0));
    }

    if (analog > 0.5) {
        // Analog bandwidth: chroma smears horizontally, luma stays.
        if (softness > 0.001) {
            float k = softness * 1.5 / (lines * src_size.x / src_size.y);
            vec3 s = (tap(uv + vec2(k, 0.0)) + tap(uv - vec2(k, 0.0)) + tap(uv + vec2(2.0 * k, 0.0)) + tap(uv - vec2(2.0 * k, 0.0)) + c) * 0.2;
            c = max(s - vec3(luma(s)) + vec3(luma(c)), vec3(0.0));
        }
        // Grain at the video line resolution.
        vec2 cell = floor(uv * vec2(lines * dst_size.x / dst_size.y, lines));
        float g = hash(cell + vec2(time * 17.0, time * 31.0)) - 0.5;
        c += vec3(g * noise * 0.16);
        // Scanlines and vignette.
        float s = 0.5 + 0.5 * cos(6.2831853 * uv.y * lines);
        c *= 1.0 - scanlines * 0.18 * (1.0 - s);
        vec2 v = uv * 2.0 - 1.0;
        c *= 1.0 - dot(v, v) * 0.055;
    }

    // Exposure (linear light, soft shoulder) and gamma.
    vec3 lin = pow(clamp(c, 0.0, 1.0), vec3(2.2)) * gain;
    lin = mix(lin, 0.6 + (lin - 0.6) / (1.0 + (lin - 0.6) / 0.4), step(0.6, lin));
    c = pow(clamp(lin, 0.0, 1.0), vec3(1.0 / 2.2));
    c = pow(c, vec3(inv_gamma));
    frag = vec4(c, 1.0);
}
)";

GLuint compile(GLenum type, const char* src) {
    const GLuint s = g.CreateShader(type);
    g.ShaderSource(s, 1, &src, nullptr);
    g.CompileShader(s);
    GLint ok = 0;
    g.GetShaderiv(s, kCompileStatus, &ok);
    if (!ok) {
        char log[1024]{};
        g.GetShaderInfoLog(s, sizeof(log), nullptr, log);
        uevr::API::get()->log_error("[FPV Drone] second window shader: %s", log);
        return 0;
    }
    return s;
}

} // namespace

GlPresenter::~GlPresenter() { shutdown(); }

bool GlPresenter::init(HWND hwnd) {
    m_hwnd = hwnd;
    m_dc = GetDC(hwnd);
    if (m_dc == nullptr) return false;

    PIXELFORMATDESCRIPTOR pfd{};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.iLayerType = PFD_MAIN_PLANE;
    const int format = ChoosePixelFormat(m_dc, &pfd);
    if (format == 0 || !SetPixelFormat(m_dc, format, &pfd)) {
        shutdown();
        return false;
    }
    m_rc = wglCreateContext(m_dc);
    if (m_rc == nullptr || !wglMakeCurrent(m_dc, m_rc)) {
        shutdown();
        return false;
    }
    if (!g.load_all()) {
        uevr::API::get()->log_warn("[FPV Drone] second window: OpenGL 3 functions missing, using GDI");
        shutdown();
        return false;
    }
    if (g.SwapInterval) g.SwapInterval(1); // our own thread: waiting for vsync costs the game nothing

    const GLuint vs = compile(kVertexShader, kVertex);
    const GLuint fs = compile(kFragmentShader, kFragment);
    if (vs == 0 || fs == 0) {
        shutdown();
        return false;
    }
    m_program = g.CreateProgram();
    g.AttachShader(m_program, vs);
    g.AttachShader(m_program, fs);
    g.LinkProgram(m_program);
    GLint linked = 0;
    g.GetProgramiv(m_program, kLinkStatus, &linked);
    if (!linked) {
        uevr::API::get()->log_error("[FPV Drone] second window: shader link failed, using GDI");
        shutdown();
        return false;
    }
    u_tex = g.GetUniformLocation(m_program, "tex");
    u_src_size = g.GetUniformLocation(m_program, "src_size");
    u_dst_size = g.GetUniformLocation(m_program, "dst_size");
    u_sharp = g.GetUniformLocation(m_program, "sharpness");
    u_gain = g.GetUniformLocation(m_program, "gain");
    u_inv_gamma = g.GetUniformLocation(m_program, "inv_gamma");
    u_analog = g.GetUniformLocation(m_program, "analog");
    u_noise = g.GetUniformLocation(m_program, "noise");
    u_soft = g.GetUniformLocation(m_program, "softness");
    u_scan = g.GetUniformLocation(m_program, "scanlines");
    u_lines = g.GetUniformLocation(m_program, "lines");
    u_time = g.GetUniformLocation(m_program, "time");

    if (g.GenVertexArrays && g.BindVertexArray) {
        GLuint vao = 0;
        g.GenVertexArrays(1, &vao);
        m_vao = vao;
    }
    GLuint tex = 0;
    glGenTextures(1, &tex);
    m_tex = tex;
    glBindTexture(GL_TEXTURE_2D, m_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, kClampToEdge);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, kClampToEdge);

    const char* version = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    const char* renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    uevr::API::get()->log_info("[FPV Drone] second window: OpenGL %s on %s", version ? version : "?", renderer ? renderer : "?");
    m_ok = true;
    return true;
}

void GlPresenter::draw(const uint32_t* pixels, int w, int h, bool upload, const FeedEffect& fx, float gain,
                       float video_lines, int client_w, int client_h) {
    if (!m_ok || client_w <= 0 || client_h <= 0 || w <= 0 || h <= 0) return;
    wglMakeCurrent(m_dc, m_rc);

    glBindTexture(GL_TEXTURE_2D, m_tex);
    if (upload && pixels != nullptr) {
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        if (w != m_tex_w || h != m_tex_h) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, kBgra, GL_UNSIGNED_BYTE, pixels);
            m_tex_w = w;
            m_tex_h = h;
        } else {
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, kBgra, GL_UNSIGNED_BYTE, pixels);
        }
        ++m_frame_counter;
    }
    if (m_tex_w <= 0) return;

    // Letterbox to the image aspect.
    const double aspect = static_cast<double>(m_tex_w) / m_tex_h;
    int dw = client_w, dh = static_cast<int>(client_w / aspect);
    if (dh > client_h) { dh = client_h; dw = static_cast<int>(client_h * aspect); }
    const int dx = (client_w - dw) / 2, dy = (client_h - dh) / 2;

    glViewport(0, 0, client_w, client_h);
    glClearColor(0.f, 0.f, 0.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(dx, dy, dw, dh);

    g.UseProgram(m_program);
    if (m_vao != 0) g.BindVertexArray(m_vao);
    g.ActiveTexture(kTexture0);
    glBindTexture(GL_TEXTURE_2D, m_tex);
    g.Uniform1i(u_tex, 0);
    g.Uniform2f(u_src_size, static_cast<float>(m_tex_w), static_cast<float>(m_tex_h));
    g.Uniform2f(u_dst_size, static_cast<float>(dw), static_cast<float>(dh));
    g.Uniform1f(u_sharp, std::clamp(fx.sharpness, 0.0f, 1.0f));
    g.Uniform1f(u_gain, gain);
    g.Uniform1f(u_inv_gamma, 1.0f / std::max(fx.gamma, 0.1f));
    g.Uniform1f(u_analog, fx.analog ? 1.0f : 0.0f);
    g.Uniform1f(u_noise, fx.noise);
    g.Uniform1f(u_soft, fx.softness);
    g.Uniform1f(u_scan, fx.scanlines);
    g.Uniform1f(u_lines, std::max(video_lines, 1.0f));
    g.Uniform1f(u_time, static_cast<float>(m_frame_counter % 997u));
    glDrawArrays(GL_TRIANGLES, 0, 3);

    SwapBuffers(m_dc);
}

void GlPresenter::shutdown() {
    if (m_rc != nullptr) {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(m_rc); // deletes the texture / program with it
        m_rc = nullptr;
    }
    if (m_dc != nullptr && m_hwnd != nullptr) {
        ReleaseDC(m_hwnd, m_dc);
        m_dc = nullptr;
    }
    m_ok = false;
    m_tex = m_program = m_vao = 0;
    m_tex_w = m_tex_h = 0;
}

} // namespace fpv
