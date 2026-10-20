// SecondWindow.cpp - see SecondWindow.hpp
#include "SecondWindow.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <d3d11.h>
#include <d3d12.h>
#include <wrl/client.h>

#include "uevr/API.hpp"

using Microsoft::WRL::ComPtr;

namespace fpv {

namespace {

constexpr wchar_t kClassName[] = L"UEVR_FPV_Drone_View";
constexpr wchar_t kTitle[] = L"FPV Drone - second view (double-click: fullscreen)";
constexpr UINT WM_APP_FRAME = WM_APP + 1;
constexpr int kSlots = 3;

bool is_bgra(DXGI_FORMAT f) {
    switch (f) {
    case DXGI_FORMAT_B8G8R8A8_TYPELESS:
    case DXGI_FORMAT_B8G8R8A8_UNORM:
    case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
    case DXGI_FORMAT_B8G8R8X8_TYPELESS:
    case DXGI_FORMAT_B8G8R8X8_UNORM:
    case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
        return true;
    default:
        return false;
    }
}

bool is_rgba8(DXGI_FORMAT f) {
    switch (f) {
    case DXGI_FORMAT_R8G8B8A8_TYPELESS:
    case DXGI_FORMAT_R8G8B8A8_UNORM:
    case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
        return true;
    default:
        return false;
    }
}

} // namespace

// ------------------------------------------------------------------------------------------------
// GPU side (render thread): copy the capture into CPU-readable memory, harvest finished copies.
// ------------------------------------------------------------------------------------------------
struct SecondWindow::Gpu {
    int api{-1};
    bool failed{false};

    // D3D11
    ComPtr<ID3D11Device> d11;
    struct Slot11 {
        ComPtr<ID3D11Texture2D> staging;
        bool busy{false};
        int w{0}, h{0};
        bool bgra{true};
    } s11[kSlots];

    // D3D12
    ComPtr<ID3D12Device> d12;
    ComPtr<ID3D12CommandQueue> queue12;
    ComPtr<ID3D12Fence> fence12;
    uint64_t fence_counter{0};
    struct Slot12 {
        ComPtr<ID3D12Resource> readback;
        ComPtr<ID3D12CommandAllocator> alloc;
        ComPtr<ID3D12GraphicsCommandList> list;
        uint64_t fence_value{0};
        bool busy{false};
        int w{0}, h{0};
        UINT row_pitch{0};
        UINT64 size{0};
        bool bgra{true};
    } s12[kSlots];

    int next{0};

    bool init(int renderer_type) {
        api = renderer_type;
        const auto renderer = uevr::API::get()->param()->renderer;
        if (api == UEVR_RENDERER_D3D11) {
            d11 = static_cast<ID3D11Device*>(renderer->device);
            return d11 != nullptr;
        }
        if (api == UEVR_RENDERER_D3D12) {
            d12 = static_cast<ID3D12Device*>(renderer->device);
            queue12 = static_cast<ID3D12CommandQueue*>(renderer->command_queue);
            if (!d12 || !queue12) return false;
            return SUCCEEDED(d12->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence12)));
        }
        return false;
    }

    // Copies a finished slot into `out` (BGRA). Returns true if a new frame was produced.
    bool harvest(std::vector<uint32_t>& out, int& w, int& h) {
        bool got = false;
        if (api == UEVR_RENDERER_D3D11) {
            ComPtr<ID3D11DeviceContext> ctx;
            d11->GetImmediateContext(&ctx);
            for (auto& s : s11) {
                if (!s.busy) continue;
                D3D11_MAPPED_SUBRESOURCE m{};
                const HRESULT hr = ctx->Map(s.staging.Get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &m);
                if (hr == DXGI_ERROR_WAS_STILL_DRAWING) continue;
                s.busy = false;
                if (FAILED(hr)) continue;
                copy_out(static_cast<const uint8_t*>(m.pData), m.RowPitch, s.w, s.h, s.bgra, out);
                w = s.w;
                h = s.h;
                got = true;
                ctx->Unmap(s.staging.Get(), 0);
            }
        } else {
            const uint64_t done = fence12->GetCompletedValue();
            for (auto& s : s12) {
                if (!s.busy || s.fence_value > done) continue;
                s.busy = false;
                void* p = nullptr;
                const D3D12_RANGE range{0, static_cast<SIZE_T>(s.size)};
                if (FAILED(s.readback->Map(0, &range, &p)) || p == nullptr) continue;
                copy_out(static_cast<const uint8_t*>(p), s.row_pitch, s.w, s.h, s.bgra, out);
                const D3D12_RANGE none{0, 0};
                s.readback->Unmap(0, &none);
                w = s.w;
                h = s.h;
                got = true;
            }
        }
        return got;
    }

    static void copy_out(const uint8_t* src, UINT pitch, int w, int h, bool bgra, std::vector<uint32_t>& out) {
        out.resize(static_cast<size_t>(w) * h);
        for (int y = 0; y < h; ++y) {
            const auto row = reinterpret_cast<const uint32_t*>(src + static_cast<size_t>(y) * pitch);
            uint32_t* dst = out.data() + static_cast<size_t>(y) * w;
            if (bgra) {
                std::memcpy(dst, row, static_cast<size_t>(w) * 4);
            } else {
                for (int x = 0; x < w; ++x) {
                    const uint32_t c = row[x]; // RGBA in memory -> swap R and B
                    dst[x] = (c & 0xFF00FF00u) | ((c & 0xFFu) << 16) | ((c >> 16) & 0xFFu);
                }
            }
        }
    }

    // Schedules a copy of the texture into a free slot (skipped if all slots are busy).
    bool schedule(void* tex) {
        if (api == UEVR_RENDERER_D3D11) {
            auto t = static_cast<ID3D11Texture2D*>(tex);
            D3D11_TEXTURE2D_DESC td{};
            t->GetDesc(&td);
            if (!is_bgra(td.Format) && !is_rgba8(td.Format)) return false;
            for (int k = 0; k < kSlots; ++k) {
                auto& s = s11[(next + k) % kSlots];
                if (s.busy) continue;
                if (!s.staging || s.w != static_cast<int>(td.Width) || s.h != static_cast<int>(td.Height)) {
                    D3D11_TEXTURE2D_DESC sd = td;
                    sd.MipLevels = 1;
                    sd.ArraySize = 1;
                    sd.SampleDesc = {1, 0};
                    sd.Usage = D3D11_USAGE_STAGING;
                    sd.BindFlags = 0;
                    sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
                    sd.MiscFlags = 0;
                    s.staging.Reset();
                    if (FAILED(d11->CreateTexture2D(&sd, nullptr, &s.staging))) return false;
                    s.w = static_cast<int>(td.Width);
                    s.h = static_cast<int>(td.Height);
                }
                s.bgra = is_bgra(td.Format);
                ComPtr<ID3D11DeviceContext> ctx;
                d11->GetImmediateContext(&ctx);
                ctx->CopySubresourceRegion(s.staging.Get(), 0, 0, 0, 0, t, 0, nullptr);
                s.busy = true;
                next = (next + k + 1) % kSlots;
                return true;
            }
            return false;
        }

        auto res = static_cast<ID3D12Resource*>(tex);
        const auto td = res->GetDesc();
        if (td.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D || (!is_bgra(td.Format) && !is_rgba8(td.Format))) return false;
        for (int k = 0; k < kSlots; ++k) {
            auto& s = s12[(next + k) % kSlots];
            if (s.busy) continue;

            D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp{};
            UINT64 total = 0;
            d12->GetCopyableFootprints(&td, 0, 1, 0, &fp, nullptr, nullptr, &total);

            if (!s.readback || s.size < total) {
                D3D12_HEAP_PROPERTIES hp{};
                hp.Type = D3D12_HEAP_TYPE_READBACK;
                D3D12_RESOURCE_DESC bd{};
                bd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
                bd.Width = total;
                bd.Height = 1;
                bd.DepthOrArraySize = 1;
                bd.MipLevels = 1;
                bd.SampleDesc.Count = 1;
                bd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
                s.readback.Reset();
                if (FAILED(d12->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &bd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                        IID_PPV_ARGS(&s.readback)))) {
                    return false;
                }
                s.size = total;
            }
            if (!s.alloc) {
                if (FAILED(d12->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&s.alloc)))) return false;
                if (FAILED(d12->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, s.alloc.Get(), nullptr, IID_PPV_ARGS(&s.list)))) return false;
                s.list->Close();
            }

            s.alloc->Reset();
            s.list->Reset(s.alloc.Get(), nullptr);

            // Unreal leaves render targets in a shader-readable state after a scene capture.
            const auto kSrv = static_cast<D3D12_RESOURCE_STATES>(D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE |
                                                                     D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
            D3D12_RESOURCE_BARRIER b{};
            b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
            b.Transition.pResource = res;
            b.Transition.Subresource = 0;
            b.Transition.StateBefore = kSrv;
            b.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
            s.list->ResourceBarrier(1, &b);

            D3D12_TEXTURE_COPY_LOCATION dst{};
            dst.pResource = s.readback.Get();
            dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
            dst.PlacedFootprint = fp;
            D3D12_TEXTURE_COPY_LOCATION src{};
            src.pResource = res;
            src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
            src.SubresourceIndex = 0;
            s.list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);

            b.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
            b.Transition.StateAfter = kSrv;
            s.list->ResourceBarrier(1, &b);
            s.list->Close();

            ID3D12CommandList* lists[]{s.list.Get()};
            queue12->ExecuteCommandLists(1, lists);
            s.fence_value = ++fence_counter;
            queue12->Signal(fence12.Get(), s.fence_value);

            s.w = static_cast<int>(td.Width);
            s.h = static_cast<int>(td.Height);
            s.row_pitch = fp.Footprint.RowPitch;
            s.bgra = is_bgra(td.Format);
            s.busy = true;
            next = (next + k + 1) % kSlots;
            return true;
        }
        return false;
    }

    // Waits for in-flight GPU copies (device reset / shutdown).
    void wait() {
        if (api == UEVR_RENDERER_D3D12 && fence12) {
            HANDLE e = CreateEventW(nullptr, FALSE, FALSE, nullptr);
            if (e != nullptr) {
                if (fence12->GetCompletedValue() < fence_counter) {
                    fence12->SetEventOnCompletion(fence_counter, e);
                    WaitForSingleObject(e, 1000);
                }
                CloseHandle(e);
            }
        }
    }
};

// ------------------------------------------------------------------------------------------------
// Window
// ------------------------------------------------------------------------------------------------
SecondWindow::~SecondWindow() {
    if (HWND h = m_hwnd.load()) PostMessageW(h, WM_CLOSE, 1, 0);
    if (m_thread.joinable()) m_thread.detach(); // process is shutting down
    // m_gpu is intentionally leaked: at process exit the game's device may already be gone.
}

std::string SecondWindow::status() const {
    std::scoped_lock _{m_status_mutex};
    return m_status;
}

void SecondWindow::set_status(const char* s) {
    std::scoped_lock _{m_status_mutex};
    m_status = s;
}

void SecondWindow::set_visible(bool visible) {
    m_visible = visible;
    if (visible && !m_thread.joinable()) {
        m_thread = std::thread{[this] { window_thread(); }};
    }
    if (HWND h = m_hwnd.load()) {
        ShowWindow(h, visible ? SW_SHOWNOACTIVATE : SW_HIDE);
    }
}

void SecondWindow::window_thread() {
    const HINSTANCE inst = GetModuleHandleW(nullptr);
    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.style = CS_DBLCLKS;
    wc.lpfnWndProc = &SecondWindow::wnd_proc;
    wc.hInstance = inst;
    wc.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512)); // IDC_ARROW
    wc.hbrBackground = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
    wc.lpszClassName = kClassName;
    RegisterClassExW(&wc);

    RECT r{0, 0, 960, 720};
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    // WS_EX_NOACTIVATE: clicking the view never takes the focus away from the game.
    HWND hwnd = CreateWindowExW(WS_EX_NOACTIVATE | WS_EX_APPWINDOW, kClassName, kTitle, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT,
                                CW_USEDEFAULT, r.right - r.left, r.bottom - r.top, nullptr, nullptr, inst, this);
    if (hwnd == nullptr) {
        set_status("window creation failed");
        return;
    }
    m_hwnd = hwnd;
    ShowWindow(hwnd, m_visible.load() ? SW_SHOWNOACTIVATE : SW_HIDE);
    set_status("waiting for frames");

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    m_hwnd = nullptr;
}

LRESULT CALLBACK SecondWindow::wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    if (msg == WM_NCCREATE) {
        auto cs = reinterpret_cast<CREATESTRUCTW*>(lp);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
    }
    auto self = reinterpret_cast<SecondWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    switch (msg) {
    case WM_APP_FRAME:
        if (self != nullptr) {
            self->m_frame_posted = false;
            self->draw_latest_frame(hwnd, true);
        }
        return 0;
    case WM_PAINT: {
        PAINTSTRUCT ps{};
        BeginPaint(hwnd, &ps);
        EndPaint(hwnd, &ps);
        if (self != nullptr) self->draw_latest_frame(hwnd, false);
        return 0;
    }
    case WM_LBUTTONDBLCLK:
        if (self != nullptr) self->toggle_fullscreen();
        return 0;
    case WM_MOUSEACTIVATE:
        return MA_NOACTIVATE;
    case WM_CLOSE:
        if (wp == 1) { // shutting down
            DestroyWindow(hwnd);
            return 0;
        }
        if (self != nullptr) {
            self->m_visible = false;
            self->m_closed_by_user = true;
        }
        ShowWindow(hwnd, SW_HIDE);
        return 0;
    case WM_SIZE:
        if (self != nullptr) self->draw_latest_frame(hwnd, false);
        return 0;
    case WM_DESTROY:
        if (self != nullptr) self->m_gl.shutdown();
        PostQuitMessage(0);
        return 0;
    case WM_ERASEBKGND:
        return 1;
    default:
        break;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

void SecondWindow::toggle_fullscreen() {
    HWND hwnd = m_hwnd.load();
    if (hwnd == nullptr) return;
    if (!m_fullscreen) {
        GetWindowRect(hwnd, &m_windowed_rect);
        m_windowed_style = GetWindowLongW(hwnd, GWL_STYLE);
        MONITORINFO mi{};
        mi.cbSize = sizeof(mi);
        GetMonitorInfoW(MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST), &mi);
        SetWindowLongW(hwnd, GWL_STYLE, m_windowed_style & ~WS_OVERLAPPEDWINDOW);
        SetWindowPos(hwnd, HWND_TOP, mi.rcMonitor.left, mi.rcMonitor.top, mi.rcMonitor.right - mi.rcMonitor.left,
                     mi.rcMonitor.bottom - mi.rcMonitor.top, SWP_NOACTIVATE | SWP_FRAMECHANGED);
        m_fullscreen = true;
    } else {
        SetWindowLongW(hwnd, GWL_STYLE, m_windowed_style);
        SetWindowPos(hwnd, nullptr, m_windowed_rect.left, m_windowed_rect.top, m_windowed_rect.right - m_windowed_rect.left,
                     m_windowed_rect.bottom - m_windowed_rect.top, SWP_NOACTIVATE | SWP_NOZORDER | SWP_FRAMECHANGED);
        m_fullscreen = false;
    }
}

// Window thread: picks up the latest frame (if any), runs the auto exposure and draws it.
void SecondWindow::draw_latest_frame(HWND hwnd, bool new_frame) {
    FeedEffect fx{};
    {
        std::scoped_lock _{m_frame_mutex};
        if (m_frame.empty()) return;
        if (new_frame || m_work.empty()) {
            m_work = m_frame;
            m_work_w = m_frame_w;
            m_work_h = m_frame_h;
        } else {
            new_frame = false;
        }
        fx = m_frame_fx;
    }
    const int w = m_work_w, h = m_work_h;
    if (w <= 0 || h <= 0 || m_work.size() < static_cast<size_t>(w) * h) return;

    const float gain = update_exposure(fx, new_frame);

    if (!m_gl_tried) {
        m_gl_tried = true;
        if (!m_gl.init(hwnd)) {
            uevr::API::get()->log_warn("[FPV Drone] second window: OpenGL unavailable, using GDI");
        }
    }
    RECT cr{};
    GetClientRect(hwnd, &cr);
    if (m_gl.ok()) {
        const float lines = static_cast<float>(h) / std::max(fx.supersample, 1.0f);
        m_gl.draw(m_work.data(), w, h, new_frame, fx, gain, lines, cr.right - cr.left, cr.bottom - cr.top);
        return;
    }
    draw_gdi(hwnd, w, h, fx, gain);
}

// Auto exposure. Scene captures often miss the game's eye adaptation and come out dark. When the
// capture's own exposure can be driven (PostProcessSettings.AutoExposureBias, applied in HDR before
// tone mapping: no banding, no amplified noise), the correction is published for the game thread;
// otherwise it becomes a gain on the 8-bit image here. Returns that gain (1 when exposed at source).
float SecondWindow::update_exposure(const FeedEffect& fx, bool measure) {
    const bool source = m_source_exposure_ok.load();
    if (!fx.auto_exposure) {
        m_ev = 0.0f;
        m_source_ev = fx.exposure;
        return source ? 1.0f : std::exp2(fx.exposure);
    }
    if (measure) {
        const int w = m_work_w, h = m_work_h;
        uint64_t sum = 0, n = 0;
        for (int y = 2; y < h; y += 6) {
            const uint32_t* row = m_work.data() + static_cast<size_t>(y) * w;
            for (int x = 2; x < w; x += 6) {
                const uint32_t c = row[x];
                sum += (((c >> 16) & 0xFF) * 77 + ((c >> 8) & 0xFF) * 150 + (c & 0xFF) * 29) >> 8;
                ++n;
            }
        }
        const float mean = n > 0 ? static_cast<float>(sum) / n / 255.0f : 0.4f; // display-encoded
        const float mean_lin = std::pow(std::max(mean, 0.01f), 2.2f);
        constexpr float kTargetLin = 0.13f; // ~0.40 display-encoded: a normally exposed scene
        const float err = std::log2(kTargetLin * std::exp2(fx.exposure) / mean_lin);
        if (source) {
            m_ev = std::clamp(m_ev + std::clamp(err, -1.0f, 1.0f) * 0.06f, -4.0f, 6.0f); // integrates (closed loop)
        } else {
            m_ev += (std::clamp(err, -1.5f, 4.0f) - m_ev) * 0.08f;
        }
    }
    m_source_ev = source ? m_ev : 0.0f;
    return source ? 1.0f : std::exp2(m_ev);
}

// GDI fallback (no OpenGL): the look is applied on the CPU, then StretchDIBits (HALFTONE) scales it.
void SecondWindow::draw_gdi(HWND hwnd, int w, int h, const FeedEffect& fx, float gain) {
    // Output has two rows per captured line, so scanlines survive the upscale.
    const bool analog = fx.analog;
    const int out_h = analog ? h * 2 : h;
    m_out.resize(static_cast<size_t>(w) * out_h);


    uint8_t gamma_lut[256];
    const float inv_gamma = 1.0f / std::max(fx.gamma, 0.1f);
    for (int i = 0; i < 256; ++i) {
        float lin = std::pow(i / 255.0f, 2.2f) * gain;
        if (lin > 0.6f) lin = 0.6f + (lin - 0.6f) / (1.0f + (lin - 0.6f) / 0.4f); // soft shoulder, no hard clip
        const float v = std::pow(std::min(lin, 1.0f), 1.0f / 2.2f);
        gamma_lut[i] = static_cast<uint8_t>(std::clamp(std::pow(v, inv_gamma) * 255.0f + 0.5f, 0.0f, 255.0f));
    }

    const int radius = analog ? std::clamp(static_cast<int>(std::lround(fx.softness * 1.0f)), 0, 2) : 0;
    const int noise_amp = analog ? static_cast<int>(fx.noise * 20.0f) : 0;
    const int scan = analog ? static_cast<int>(256.0f - fx.scanlines * 0.18f * 256.0f) : 256; // odd rows factor /256
    const float cx = (w - 1) * 0.5f, cy = (h - 1) * 0.5f;

    auto R = [](uint32_t c) { return static_cast<int>((c >> 16) & 0xFF); };
    auto G = [](uint32_t c) { return static_cast<int>((c >> 8) & 0xFF); };
    auto B = [](uint32_t c) { return static_cast<int>(c & 0xFF); };

    for (int y = 0; y < h; ++y) {
        const uint32_t* row = m_work.data() + static_cast<size_t>(y) * w;
        uint32_t* o0 = m_out.data() + static_cast<size_t>(analog ? 2 * y : y) * w;
        uint32_t* o1 = analog ? o0 + w : nullptr;
        const float dy = (y - cy) / std::max(cy, 1.0f);
        for (int x = 0; x < w; ++x) {
            int r, g, b;
            if (radius > 0) {
                // Wide chroma (analog bandwidth) + slightly soft luma.
                int sr = 0, sg = 0, sb = 0;
                for (int k = -radius; k <= radius; ++k) {
                    const uint32_t c = row[std::clamp(x + k, 0, w - 1)];
                    sr += R(c); sg += G(c); sb += B(c);
                }
                const int n = 2 * radius + 1;
                sr /= n; sg /= n; sb /= n;
                const uint32_t c0 = row[x], cl = row[std::max(x - 1, 0)], cr = row[std::min(x + 1, w - 1)];
                const int lr = (R(c0) * 2 + R(cl) + R(cr)) >> 2;
                const int lg = (G(c0) * 2 + G(cl) + G(cr)) >> 2;
                const int lb = (B(c0) * 2 + B(cl) + B(cr)) >> 2;
                const int luma = (lr * 77 + lg * 150 + lb * 29) >> 8;
                const int cluma = (sr * 77 + sg * 150 + sb * 29) >> 8;
                r = sr - cluma + luma;
                g = sg - cluma + luma;
                b = sb - cluma + luma;
            } else {
                const uint32_t c = row[x];
                r = R(c); g = G(c); b = B(c);
            }
            if (noise_amp > 0) {
                m_rng ^= m_rng << 13; m_rng ^= m_rng >> 17; m_rng ^= m_rng << 5;
                const int n = static_cast<int>(m_rng & 0xFF) - 128;
                const int d = n * noise_amp / 128;
                r += d; g += d; b += d;
            }
            if (analog) {
                const float dx = (x - cx) / std::max(cx, 1.0f);
                const int v = static_cast<int>((1.0f - (dx * dx + dy * dy) * 0.055f) * 256.0f);
                r = r * v >> 8; g = g * v >> 8; b = b * v >> 8;
            }
            r = gamma_lut[std::clamp(r, 0, 255)];
            g = gamma_lut[std::clamp(g, 0, 255)];
            b = gamma_lut[std::clamp(b, 0, 255)];
            o0[x] = 0xFF000000u | (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
            if (o1 != nullptr) {
                o1[x] = 0xFF000000u | (static_cast<uint32_t>(r * scan >> 8) << 16) |
                        (static_cast<uint32_t>(g * scan >> 8) << 8) | static_cast<uint32_t>(b * scan >> 8);
            }
        }
    }

    RECT cr{};
    GetClientRect(hwnd, &cr);
    const int cw = cr.right - cr.left, ch = cr.bottom - cr.top;
    if (cw <= 0 || ch <= 0) return;

    // Letterbox to the capture's aspect.
    const double src_aspect = static_cast<double>(w) / h;
    int dw = cw, dh = static_cast<int>(cw / src_aspect);
    if (dh > ch) { dh = ch; dw = static_cast<int>(ch * src_aspect); }
    const int dx = (cw - dw) / 2, dy = (ch - dh) / 2;

    BITMAPINFO bi{};
    bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth = w;
    bi.bmiHeader.biHeight = -out_h; // top-down
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;

    HDC dc = GetDC(hwnd);
    if (dc == nullptr) return;
    HBRUSH black = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
    if (dy > 0) {
        RECT top{0, 0, cw, dy}, bottom{0, dy + dh, cw, ch};
        FillRect(dc, &top, black);
        FillRect(dc, &bottom, black);
    }
    if (dx > 0) {
        RECT left{0, 0, dx, ch}, right{dx + dw, 0, cw, ch};
        FillRect(dc, &left, black);
        FillRect(dc, &right, black);
    }
    SetStretchBltMode(dc, HALFTONE);
    SetBrushOrgEx(dc, 0, 0, nullptr);
    StretchDIBits(dc, dx, dy, dw, dh, 0, 0, w, out_h, m_out.data(), &bi, DIB_RGB_COLORS, SRCCOPY);
    ReleaseDC(hwnd, dc);
}

// ------------------------------------------------------------------------------------------------
// Render thread
// ------------------------------------------------------------------------------------------------
void SecondWindow::on_device_reset() {
    if (m_gpu != nullptr) {
        m_gpu->wait();
        delete m_gpu;
        m_gpu = nullptr;
    }
}

void SecondWindow::present(int renderer_type, void* texture, uint64_t /*generation*/, const FeedEffect& fx) {
    HWND hwnd = m_hwnd.load();
    if (!m_visible.load() || hwnd == nullptr || texture == nullptr) return;

    if (m_gpu == nullptr || m_gpu->api != renderer_type) {
        delete m_gpu;
        m_gpu = new Gpu{};
        if (!m_gpu->init(renderer_type)) {
            m_gpu->failed = true;
            set_status("GPU setup failed");
            uevr::API::get()->log_error("[FPV Drone] second window: GPU setup failed");
        } else {
            uevr::API::get()->log_info("[FPV Drone] second window: GPU readback ready (%s)",
                                       renderer_type == UEVR_RENDERER_D3D11 ? "D3D11" : "D3D12");
        }
    }
    if (m_gpu->failed) return;

    // 1) Hand the most recent finished copy to the window thread (never block the render thread).
    {
        std::unique_lock lk{m_frame_mutex, std::try_to_lock};
        if (lk.owns_lock()) {
            int w = 0, h = 0;
            if (m_gpu->harvest(m_frame, w, h)) {
                m_frame_w = w;
                m_frame_h = h;
                m_frame_fx = fx;
                lk.unlock();
                if (!m_frame_posted.exchange(true)) {
                    PostMessageW(hwnd, WM_APP_FRAME, 0, 0);
                }
                set_status("ok");
            }
        }
    }

    // 2) Schedule the copy of this frame's capture.
    if (!m_gpu->schedule(texture)) {
        static bool logged = false;
        if (!logged) {
            logged = true;
            uevr::API::get()->log_warn("[FPV Drone] second window: capture copy skipped (format or busy)");
        }
    }
}

} // namespace fpv
