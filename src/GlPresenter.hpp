// GlPresenter.hpp - draws the second window's frames with OpenGL on the window's own thread.
//
// Why OpenGL: the game's DXGI swapchains are hooked by UEVR (and often ReShade). Our window never
// touches DXGI, so nothing of ours runs inside those hooks. OpenGL gives GPU scaling (Catmull-Rom
// upscale, box downscale for supersampled captures), sharpening and the analog look for free on
// the CPU side. If OpenGL cannot start, SecondWindow falls back to its GDI path.
#pragma once

#include <cstdint>

#include <windows.h>

#include "FeedEffect.hpp"

namespace fpv {

class GlPresenter {
public:
    ~GlPresenter();

    bool init(HWND hwnd);
    bool ok() const { return m_ok; }

    // Uploads `pixels` (BGRA, top-down) when `upload` is true, then draws letterboxed into the
    // client area and swaps. `video_lines` = lines of the simulated video signal (scanlines / grain).
    void draw(const uint32_t* pixels, int w, int h, bool upload, const FeedEffect& fx, float gain, float video_lines,
              int client_w, int client_h);

    void shutdown();

private:
    HWND m_hwnd{nullptr};
    HDC m_dc{nullptr};
    HGLRC m_rc{nullptr};
    bool m_ok{false};
    unsigned m_tex{0};
    unsigned m_program{0};
    unsigned m_vao{0};
    int m_tex_w{0}, m_tex_h{0};
    uint32_t m_frame_counter{0};

    // Uniform locations
    int u_src_size{-1}, u_dst_size{-1}, u_sharp{-1}, u_gain{-1}, u_inv_gamma{-1}, u_analog{-1}, u_noise{-1},
        u_soft{-1}, u_scan{-1}, u_lines{-1}, u_time{-1}, u_tex{-1};
};

} // namespace fpv
