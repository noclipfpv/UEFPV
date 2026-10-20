// FeedEffect.hpp - look of the second window's video feed.
#pragma once

namespace fpv {

struct FeedEffect {
    bool analog{false};
    float noise{0.35f};     // grain amount
    float softness{1.0f};   // chroma smear / luma softening
    float scanlines{0.35f}; // scanline darkening
    float gamma{1.0f};
    bool auto_exposure{true}; // brighten / darken the feed toward a normal average level
    float exposure{0.0f};     // EV, on top of auto exposure
    float sharpness{0.4f};    // window: contrast-limited sharpening
    float supersample{1.5f};  // capture rendered at this multiple of the window resolution
};

} // namespace fpv
