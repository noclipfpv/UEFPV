// GameThreadTicker.cpp - see GameThreadTicker.hpp
#include "GameThreadTicker.hpp"

#include <algorithm>

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

namespace fpv {

GameThreadTicker::~GameThreadTicker() {
    m_running = false;
    if (m_poster.joinable()) m_poster.join();
    if (m_hook != nullptr) UnhookWindowsHookEx(m_hook);
    if (s_instance == this) s_instance = nullptr;
}

bool GameThreadTicker::install(HWND game_window, Callback cb) {
    if (m_hook != nullptr) return true;
    if (game_window == nullptr || s_instance != nullptr) return false;

    m_thread_id = GetWindowThreadProcessId(game_window, nullptr);
    if (m_thread_id == 0) return false;

    m_message = RegisterWindowMessageW(L"UEVR_FPV_DRONE_TICK");
    if (m_message == 0) return false;

    m_window = game_window;
    m_callback = std::move(cb);
    s_instance = this;

    // hMod = nullptr is valid for a hook on a thread of the current process.
    m_hook = SetWindowsHookExW(WH_GETMESSAGE, &GameThreadTicker::hook_proc, nullptr, m_thread_id);
    if (m_hook == nullptr) {
        // Some setups want the module that contains the hook procedure.
        HMODULE self_module{};
        GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           reinterpret_cast<LPCWSTR>(&GameThreadTicker::hook_proc), &self_module);
        m_hook = SetWindowsHookExW(WH_GETMESSAGE, &GameThreadTicker::hook_proc, self_module, m_thread_id);
    }
    if (m_hook == nullptr) {
        s_instance = nullptr;
        return false;
    }

    QueryPerformanceFrequency(&m_freq);
    m_last.QuadPart = 0;
    m_last_tick_end = 0;

    m_running = true;
    m_poster = std::thread{[this] { poster_loop(); }};
    return true;
}

// Keeps one tick message waiting in the game thread's queue, re-posted only once the previous
// tick has finished AND a minimum gap has passed. The gap is essential: Unreal drains the queue
// with `while (PeekMessage(...))`; if a new tick message arrived while the previous tick was still
// running, the queue would never be empty and the game thread would never leave the loop.
void GameThreadTicker::poster_loop() {
    HANDLE timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    if (timer == nullptr) {
        timer = CreateWaitableTimerExW(nullptr, nullptr, 0, TIMER_ALL_ACCESS);
    }

    while (m_running) {
        LARGE_INTEGER now{};
        QueryPerformanceCounter(&now);
        const int64_t gap = m_freq.QuadPart > 0 ? m_freq.QuadPart * kMinGapUs / 1000000 : 0;
        if (!m_pending.load() && now.QuadPart - m_last_tick_end.load() >= gap) {
            m_pending = true;
            if (!PostMessageW(m_window, m_message, 0, 0)) {
                m_pending = false; // window gone or queue full; try again later
            }
        }

        if (timer != nullptr) {
            LARGE_INTEGER due{};
            due.QuadPart = -5000; // 0.5 ms, relative, in 100 ns units
            SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE);
            WaitForSingleObject(timer, 10);
        } else {
            Sleep(1);
        }
    }

    if (timer != nullptr) CloseHandle(timer);
}

LRESULT CALLBACK GameThreadTicker::hook_proc(int code, WPARAM wparam, LPARAM lparam) {
    auto self = s_instance;
    if (self != nullptr && code == HC_ACTION && wparam == PM_REMOVE && lparam != 0) {
        auto msg = reinterpret_cast<MSG*>(lparam);
        if (msg->message == self->m_message && self->m_message != 0) {
            msg->message = WM_NULL; // swallow: the game never sees it
            self->on_tick_message();
        }
    }
    return CallNextHookEx(nullptr, code, wparam, lparam);
}

void GameThreadTicker::on_tick_message() {
    if (m_in_tick) {
        // Re-entered from a nested message pump inside our own tick: skip, and keep m_pending set
        // so nothing new is posted before the outer tick has finished.
        return;
    }
    m_in_tick = true;

    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);
    double dt = 0.0;
    if (m_last.QuadPart != 0 && m_freq.QuadPart != 0) {
        dt = static_cast<double>(now.QuadPart - m_last.QuadPart) / static_cast<double>(m_freq.QuadPart);
    }
    m_last = now;
    dt = std::clamp(dt, 0.0, 0.1);

    if (dt > 0.0) {
        const double hz = 1.0 / std::max(dt, 1e-4);
        m_hz = m_hz.load() * 0.95 + hz * 0.05;
    }
    ++m_ticks;

    if (m_callback) m_callback(dt);

    // Only now may the next tick message be posted (after the minimum gap).
    LARGE_INTEGER end{};
    QueryPerformanceCounter(&end);
    m_last_tick_end = end.QuadPart;
    m_pending = false;
    m_in_tick = false;
}

} // namespace fpv
