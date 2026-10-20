// GameThreadTicker.hpp - a per-frame callback on Unreal's game thread, without UEVR's VR hooks.
//
// UEVR only installs its UGameEngine::Tick / viewport hooks when a VR runtime is loaded, so in
// flat-screen mode plugins never get on_pre_engine_tick. Unreal's game thread, however, pumps
// the Windows message queue of the game window at the start of every frame
// (FEngineLoop::Tick -> PumpMessages). We install a WH_GETMESSAGE hook on that thread and keep
// exactly one private "tick" message posted to the window: each time the game thread pumps
// messages, the hook swallows it and runs our callback, before the world ticks and before the
// camera manager computes the view.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>

#include <windows.h>

namespace fpv {

class GameThreadTicker {
public:
    using Callback = std::function<void(double /*dt seconds*/)>;

    ~GameThreadTicker();

    // Safe to call repeatedly; does nothing once installed. Returns true when installed.
    bool install(HWND game_window, Callback cb);
    bool installed() const { return m_hook != nullptr; }

    DWORD game_thread_id() const { return m_thread_id; }
    double measured_hz() const { return m_hz.load(); }
    uint64_t tick_count() const { return m_ticks.load(); }

private:
    static LRESULT CALLBACK hook_proc(int code, WPARAM wparam, LPARAM lparam);
    void poster_loop();
    void on_tick_message();

    HWND m_window{nullptr};
    DWORD m_thread_id{0};
    HHOOK m_hook{nullptr};
    UINT m_message{0};
    Callback m_callback{};

    std::thread m_poster{};
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_pending{false};

    LARGE_INTEGER m_freq{};
    LARGE_INTEGER m_last{};
    std::atomic<double> m_hz{0.0};
    std::atomic<uint64_t> m_ticks{0};
    std::atomic<int64_t> m_last_tick_end{0}; // QPC
    bool m_in_tick{false};                    // game thread only
    static constexpr int64_t kMinGapUs = 2000; // >= 2 ms between a tick's end and the next post

    static inline GameThreadTicker* s_instance{nullptr};
};

} // namespace fpv
