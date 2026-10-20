// SecondWindow.hpp - a separate Windows window showing the SceneCaptureView texture.
//
// No swapchain, no Present: presenting from inside the game's Present (hooked by UEVR, and often
// wrapped by ReShade) is fragile. Instead, on the game's render thread (UEVR on_present) the
// capture texture is copied into CPU-readable memory with a plain GPU copy on the game's own
// device / queue, and the window's own thread applies the look (optional analog feed: grain,
// soft chroma, scanlines, vignette, gamma) and draws it with GDI (smooth HALFTONE upscale).
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <windows.h>

#include "FeedEffect.hpp"
#include "GlPresenter.hpp"

namespace fpv {

class SecondWindow {
public:
    ~SecondWindow();

    // Any thread.
    void set_visible(bool visible);
    bool visible() const { return m_visible.load(); }
    bool closed_by_user() { return m_closed_by_user.exchange(false); }
    std::string status() const;

    // Render thread (UEVR on_present). Schedules a GPU copy of `texture` (native ID3D11Texture2D* /
    // ID3D12Resource*) and hands finished copies to the window thread. Never blocks.
    void present(int renderer_type, void* texture, uint64_t generation, const FeedEffect& fx);
    // Render thread: the game's device was reset; drop every GPU object.
    void on_device_reset();

    // Game thread: exposure correction (EV) to apply to the capture itself, and whether the capture's
    // exposure could be driven (otherwise the window brightens the 8-bit image).
    float source_exposure_ev() const { return m_source_ev.load(); }
    void set_source_exposure_available(bool ok) { m_source_exposure_ok = ok; }

private:
    struct Gpu;

    void window_thread();
    static LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);
    void toggle_fullscreen();
    void draw_latest_frame(HWND hwnd, bool new_frame);
    float update_exposure(const FeedEffect& fx, bool measure);
    void draw_gdi(HWND hwnd, int w, int h, const FeedEffect& fx, float gain);
    void set_status(const char* s);

    // Window thread
    std::thread m_thread{};
    std::atomic<HWND> m_hwnd{nullptr};
    std::atomic<bool> m_visible{false};
    std::atomic<bool> m_closed_by_user{false};
    bool m_fullscreen{false};
    RECT m_windowed_rect{};
    LONG m_windowed_style{0};

    // Frame hand-off (render thread -> window thread)
    std::mutex m_frame_mutex{};
    std::vector<uint32_t> m_frame{}; // BGRA, top-down
    int m_frame_w{0}, m_frame_h{0};
    FeedEffect m_frame_fx{};
    std::atomic<bool> m_frame_posted{false};

    // Window-thread processing buffers
    std::vector<uint32_t> m_work{};
    int m_work_w{0}, m_work_h{0};
    GlPresenter m_gl{};
    bool m_gl_tried{false};
    float m_ev{0.0f};                       // auto exposure state (window thread)
    std::atomic<float> m_source_ev{0.0f};
    std::atomic<bool> m_source_exposure_ok{false};
    std::vector<uint32_t> m_out{};
    uint32_t m_rng{0x9E3779B9u};

    // Render thread GPU state
    Gpu* m_gpu{nullptr};

    mutable std::mutex m_status_mutex{};
    std::string m_status{"closed"};
};

} // namespace fpv
