// FlightRecorder.cpp - see FlightRecorder.hpp
#include "FlightRecorder.hpp"

namespace fpv {

void FlightRecorder::clear() {
    m_frames.clear();
    m_playing = false;
    m_frozen = false;
    m_time = 0.0;
    m_index = 0;
}

void FlightRecorder::record(double dt, const Vec3& pos_m, const Basis& body) {
    if (m_playing || m_frozen || m_frames.size() >= kMaxFrames) return;
    const double t = m_frames.empty() ? 0.0 : m_frames.back().t + std::max(dt, 0.0);
    if (!m_frames.empty() && t <= m_frames.back().t) return; // paused frame
    m_frames.push_back({t, pos_m, body.f, body.u});
}

void FlightRecorder::start_playback() {
    if (empty()) return;
    m_playing = true;
    m_frozen = true;
    m_time = 0.0;
    m_index = 0;
}

size_t FlightRecorder::find(double t) const {
    size_t i = m_index;
    if (i >= m_frames.size() || m_frames[i].t > t) i = 0;
    while (i + 1 < m_frames.size() && m_frames[i + 1].t <= t) ++i;
    return i;
}

void FlightRecorder::advance(double dt, bool loop) {
    if (!m_playing || empty()) return;
    m_time += std::max(dt, 0.0);
    if (m_time >= duration()) {
        m_time = loop ? std::fmod(m_time, duration()) : duration();
    }
    m_index = find(m_time);
}

Vec3 FlightRecorder::position() const {
    if (m_frames.empty()) return {};
    const auto& a = m_frames[m_index];
    if (m_index + 1 >= m_frames.size()) return a.pos;
    const auto& b = m_frames[m_index + 1];
    const double k = clampv((m_time - a.t) / std::max(b.t - a.t, 1e-6), 0.0, 1.0);
    return a.pos + (b.pos - a.pos) * k;
}

Basis FlightRecorder::body() const {
    Basis out;
    if (m_frames.empty()) return out;
    const auto& a = m_frames[m_index];
    Vec3 f = a.f, u = a.u;
    if (m_index + 1 < m_frames.size()) {
        const auto& b = m_frames[m_index + 1];
        const double k = clampv((m_time - a.t) / std::max(b.t - a.t, 1e-6), 0.0, 1.0);
        f = a.f + (b.f - a.f) * k;
        u = a.u + (b.u - a.u) * k;
    }
    out.f = f.normalized();
    out.u = u.normalized();
    out.r = cross_lh(out.u, out.f);
    out.orthonormalize();
    return out;
}

} // namespace fpv
