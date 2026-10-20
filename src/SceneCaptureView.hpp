// SceneCaptureView.hpp - a second in-game camera rendered into a texture (for the second window).
//
// Spawns an ASceneCapture2D actor with a UTextureRenderTarget2D (KismetRenderingLibrary::
// CreateRenderTarget2D), moves it every frame and triggers the capture. The render target's GPU
// texture is not exposed through reflection, so it is located by walking the object's memory
// (UTexture -> FTextureResource -> FRHITexture) and recognized by its width / height, with every
// read checked with VirtualQuery. UEVR then gives the native D3D11 / D3D12 resource.
//
// All methods: game thread, except native_texture() (any thread).
#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "uevr/API.hpp"

#include "GameCamera.hpp"

namespace fpv {

class SceneCaptureView {
public:
    // Creates (or re-creates after a resize / level change) the capture. Needs a world context
    // (any actor of the current world, e.g. the drone camera actor).
    bool ensure(uevr::API::UObject* world_context, int width, int height);
    // Moves the capture camera and renders it (every `interval` calls).
    void update(const CameraPov& pov, int interval);
    // Drops everything without touching the objects (they may be gone after a level change).
    void forget();
    // Exposure compensation (EV) of the capture itself (PostProcessSettings.AutoExposureBias, applied
    // in HDR before tone mapping). Returns false if this engine build does not expose it.
    bool set_exposure_bias(float ev);

    bool ready() const { return m_rhi_texture.load() != nullptr; }
    int width() const { return m_width; }
    int height() const { return m_height; }
    const std::string& status() const { return m_status; }

    // Native ID3D11Texture2D* / ID3D12Resource* of the capture (nullptr until found).
    void* native_texture() const;
    uint64_t generation() const { return m_generation.load(); }

private:
    bool spawn(uevr::API::UObject* world_context);
    bool create_target(uevr::API::UObject* world_context, int width, int height);
    void* find_rhi_texture() const;

    uevr::API::UObject* m_world_context{nullptr};
    uevr::API::UObject* m_actor{nullptr};
    uevr::API::UObject* m_component{nullptr};
    uevr::API::UObject* m_target{nullptr};
    int m_width{0}, m_height{0};
    int m_frame{0};
    int m_find_attempts{0};
    std::atomic<void*> m_rhi_texture{nullptr};
    std::atomic<uint64_t> m_generation{0};
    std::string m_status{"off"};

    // PostProcessSettings.AutoExposureBias access (resolved once per component)
    uevr::API::UObject* m_pp_component{nullptr};
    uint8_t* m_pp_base{nullptr};
    float* m_pp_bias{nullptr};
    uevr::API::FBoolProperty* m_pp_override{nullptr};
    bool m_pp_failed{false};
};

} // namespace fpv
