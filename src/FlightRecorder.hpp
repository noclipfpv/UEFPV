// FlightRecorder.hpp - records the drone's path and replays it (Dual OP "replay" mode):
// fly first, then replay the same flight (real time or slow motion) while operating the gimbal.
#pragma once

#include <vector>

#include "MathTypes.hpp"

namespace fpv {

class FlightRecorder {
public:
    struct Frame {
        double t;
        Vec3 pos;  // meters, relative to the launch point
        Vec3 f, u; // body forward / up
    };

    void clear();
    // Appends a frame `dt` seconds after the previous one (ignored while playing or frozen).
    void record(double dt, const Vec3& pos_m, const Basis& body);

    bool empty() const { return m_frames.size() < 2; }
    double duration() const { return m_frames.empty() ? 0.0 : m_frames.back().t; }
    bool frozen() const { return m_frozen; } // a replay happened: keep this take until cleared

    void start_playback();
    void stop_playback() { m_playing = false; }
    bool playing() const { return m_playing; }
    double playback_time() const { return m_time; }

    // Advances the replay (dt already scaled by the replay speed). Loops or stops at the end.
    void advance(double dt, bool loop);
    // Interpolated pose at the current replay time.
    Vec3 position() const;
    Basis body() const;

private:
    size_t find(double t) const;

    std::vector<Frame> m_frames{};
    bool m_playing{false};
    bool m_frozen{false};
    double m_time{0.0};
    size_t m_index{0};
    static constexpr size_t kMaxFrames = 250000; // ~ 30 min at 144 fps
};

} // namespace fpv
