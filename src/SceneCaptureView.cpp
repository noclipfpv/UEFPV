// SceneCaptureView.cpp - see SceneCaptureView.hpp
#include "SceneCaptureView.hpp"

#include <algorithm>
#include <cstring>

#include <windows.h>

#include "Reflection.hpp"

using namespace uevr;

namespace fpv {

namespace {

constexpr uint8_t kSpawnAlways = 1;      // ESpawnActorCollisionHandlingMethod::AlwaysSpawn
constexpr uint8_t kRtfRgba8 = 2;         // ETextureRenderTargetFormat::RTF_RGBA8
constexpr uint8_t kScsFinalColorLdr = 2; // ESceneCaptureSource::SCS_FinalColorLDR

bool plausible_pointer(uint64_t p) {
    return p > 0x10000 && p < 0x00007FFFFFFFFFFFull && (p & 7) == 0;
}

// True if [p, p + size) is committed, readable memory.
bool readable(const void* p, size_t size) {
    auto addr = reinterpret_cast<uintptr_t>(p);
    const uintptr_t end = addr + size;
    while (addr < end) {
        MEMORY_BASIC_INFORMATION mbi{};
        if (VirtualQuery(reinterpret_cast<const void*>(addr), &mbi, sizeof(mbi)) == 0) return false;
        if (mbi.State != MEM_COMMIT) return false;
        if (mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) return false;
        const DWORD ok = PAGE_READONLY | PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READ |
                         PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
        if ((mbi.Protect & ok) == 0) return false;
        addr = reinterpret_cast<uintptr_t>(mbi.BaseAddress) + mbi.RegionSize;
    }
    return true;
}

bool inside_module(uint64_t p) {
    HMODULE mod{};
    return GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                              reinterpret_cast<LPCWSTR>(p), &mod) != FALSE;
}

uint64_t read_u64(const uint8_t* base, size_t off) {
    uint64_t v{};
    std::memcpy(&v, base + off, sizeof(v));
    return v;
}

} // namespace

bool SceneCaptureView::spawn(API::UObject* world_context) {
    const auto& api = API::get();
    const auto capture_class = api->find_uobject<API::UClass>(L"Class /Script/Engine.SceneCapture2D");
    const auto statics_class = api->find_uobject<API::UClass>(L"Class /Script/Engine.GameplayStatics");
    if (capture_class == nullptr || statics_class == nullptr) {
        m_status = "SceneCapture2D class not found";
        return false;
    }
    const auto statics = statics_class->get_class_default_object();

    refl::Call begin{statics, L"BeginDeferredActorSpawnFromClass"};
    begin.set(L"WorldContextObject", world_context);
    begin.set(L"ActorClass", capture_class);
    begin.set_transform(L"SpawnTransform", {});
    begin.set(L"CollisionHandlingOverride", kSpawnAlways);
    if (!begin.invoke()) return false;
    const auto actor = begin.get<API::UObject*>(L"ReturnValue");
    if (actor == nullptr) {
        m_status = "capture spawn failed";
        return false;
    }
    refl::Call finish{statics, L"FinishSpawningActor"};
    finish.set(L"Actor", actor);
    finish.set_transform(L"SpawnTransform", {});
    finish.invoke();

    m_actor = actor;
    m_component = refl::get_object_property(actor, L"CaptureComponent2D");
    if (m_component == nullptr) {
        m_status = "CaptureComponent2D not found";
        return false;
    }

    // We trigger the captures ourselves (rate control); keep temporal effects between captures.
    m_component->set_bool_property(L"bCaptureEveryFrame", false);
    m_component->set_bool_property(L"bCaptureOnMovement", false);
    m_component->set_bool_property(L"bAlwaysPersistRenderingState", true);
    if (auto src = m_component->get_property_data<uint8_t>(L"CaptureSource")) {
        *src = kScsFinalColorLdr; // tonemapped, post-processed, display-ready
    }
    return true;
}

bool SceneCaptureView::create_target(API::UObject* world_context, int width, int height) {
    const auto lib_class = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetRenderingLibrary");
    if (lib_class == nullptr) {
        m_status = "KismetRenderingLibrary not found";
        return false;
    }
    refl::Call create{lib_class->get_class_default_object(), L"CreateRenderTarget2D"};
    create.set(L"WorldContextObject", world_context);
    create.set(L"Width", static_cast<int32_t>(width));
    create.set(L"Height", static_cast<int32_t>(height));
    create.set(L"Format", kRtfRgba8);
    create.set(L"bAutoGenerateMipMaps", false);
    if (!create.invoke()) return false;
    const auto rt = create.get<API::UObject*>(L"ReturnValue");
    if (rt == nullptr) {
        m_status = "render target creation failed";
        return false;
    }

    if (auto target = m_component->get_property_data<API::UObject*>(L"TextureTarget")) {
        *target = rt; // UPROPERTY reference: also keeps the render target alive
    }
    m_target = rt;
    m_width = width;
    m_height = height;
    m_rhi_texture = nullptr;
    m_find_attempts = 0;
    ++m_generation;
    m_status = "waiting for the GPU texture";
    return true;
}

bool SceneCaptureView::ensure(API::UObject* world_context, int width, int height) {
    if (world_context != m_world_context) {
        forget(); // new world (or first use)
        m_world_context = world_context;
    }
    if (world_context == nullptr) return false;

    if (m_actor == nullptr && !spawn(world_context)) {
        API::get()->log_error("[FPV Drone] second view: %s", m_status.c_str());
        m_world_context = nullptr; // retry later
        return false;
    }
    if (m_target == nullptr || width != m_width || height != m_height) {
        if (!create_target(world_context, width, height)) {
            API::get()->log_error("[FPV Drone] second view: %s", m_status.c_str());
            return false;
        }
        API::get()->log_info("[FPV Drone] second view: render target %dx%d created", width, height);
    }

    if (m_rhi_texture.load() == nullptr && m_find_attempts < 600) {
        ++m_find_attempts;
        if (void* tex = find_rhi_texture()) {
            m_rhi_texture = tex;
            ++m_generation;
            m_status = "ok";
            API::get()->log_info("[FPV Drone] second view: GPU texture found after %d frame(s)", m_find_attempts);
        } else if (m_find_attempts == 600) {
            m_status = "GPU texture not found (see log)";
            API::get()->log_error("[FPV Drone] second view: could not locate the render target's GPU texture");
        }
    }
    return true;
}

void* SceneCaptureView::find_rhi_texture() const {
    if (m_target == nullptr) return nullptr;
    const auto rt = reinterpret_cast<const uint8_t*>(m_target);
    if (!readable(rt, 0x400)) return nullptr;

    const uint32_t w = static_cast<uint32_t>(m_width);
    const uint32_t h = static_cast<uint32_t>(m_height);

    // UTexture::Resource / PrivateResource -> FTextureResource -> TextureRHI (FRHITexture*).
    for (size_t off = 0x28; off + 8 <= 0x400; off += 8) {
        const uint64_t res = read_u64(rt, off);
        if (!plausible_pointer(res) || !readable(reinterpret_cast<const void*>(res), 0x200)) continue;
        const auto r = reinterpret_cast<const uint8_t*>(res);
        if (!inside_module(read_u64(r, 0))) continue; // FTextureResource has a vtable

        for (size_t off2 = 0x8; off2 + 8 <= 0x200; off2 += 8) {
            const uint64_t tex = read_u64(r, off2);
            if (!plausible_pointer(tex) || !readable(reinterpret_cast<const void*>(tex), 0x140)) continue;
            const auto t = reinterpret_cast<const uint8_t*>(tex);
            const uint64_t vtable = read_u64(t, 0);
            if (!plausible_pointer(vtable) || !inside_module(vtable)) continue;

            // FRHITexture2D::SizeX / SizeY (UE4) or FRHITextureDesc::Extent (UE5): adjacent.
            for (size_t o = 0x8; o + 8 <= 0x140; o += 4) {
                uint32_t a{}, b{};
                std::memcpy(&a, t + o, 4);
                std::memcpy(&b, t + o + 4, 4);
                if (a == w && b == h) {
                    return reinterpret_cast<void*>(tex);
                }
            }
        }
    }
    return nullptr;
}

void SceneCaptureView::update(const CameraPov& pov, int interval) {
    if (m_actor == nullptr || m_component == nullptr) return;

    refl::Call move{m_actor, L"K2_SetActorLocationAndRotation"};
    move.set_vector(L"NewLocation", pov.location);
    move.set_rotator(L"NewRotation", pov.rotation);
    move.set(L"bTeleport", true);
    move.invoke();

    if (auto fov = m_component->get_property_data<float>(L"FOVAngle")) {
        *fov = pov.fov;
    }

    if (++m_frame >= std::max(interval, 1)) {
        m_frame = 0;
        refl::Call capture{m_component, L"CaptureScene"};
        capture.invoke();
    }
}

bool SceneCaptureView::set_exposure_bias(float ev) {
    if (m_component == nullptr) return false;
    if (m_pp_component != m_component) {
        m_pp_component = m_component;
        m_pp_base = nullptr;
        m_pp_bias = nullptr;
        m_pp_override = nullptr;
        m_pp_failed = true;
        auto pp_prop = refl::find_property(m_component->get_class(), L"PostProcessSettings");
        auto pp_struct = pp_prop ? static_cast<API::FStructProperty*>(pp_prop)->get_struct() : nullptr;
        auto bias = pp_struct ? refl::find_property(pp_struct, L"AutoExposureBias") : nullptr;
        auto ovr = pp_struct ? refl::find_property(pp_struct, L"bOverride_AutoExposureBias") : nullptr;
        if (bias != nullptr && ovr != nullptr) {
            m_pp_base = reinterpret_cast<uint8_t*>(m_component) + pp_prop->get_offset();
            m_pp_bias = reinterpret_cast<float*>(m_pp_base + bias->get_offset());
            m_pp_override = static_cast<API::FBoolProperty*>(ovr);
            m_pp_failed = false;
            if (auto w = m_component->get_property_data<float>(L"PostProcessBlendWeight")) *w = 1.0f;
            API::get()->log_info("[FPV Drone] second view: exposure driven at the source (AutoExposureBias)");
        } else {
            API::get()->log_warn("[FPV Drone] second view: AutoExposureBias not found, exposure applied in the window");
        }
    }
    if (m_pp_failed) return false;
    m_pp_override->set_value_in_object(m_pp_base, true);
    *m_pp_bias = std::clamp(ev, -6.0f, 8.0f);
    return true;
}

void SceneCaptureView::forget() {
    m_world_context = nullptr;
    m_actor = nullptr;
    m_component = nullptr;
    m_pp_component = nullptr;
    m_target = nullptr;
    m_width = m_height = 0;
    m_rhi_texture = nullptr;
    ++m_generation;
    m_status = "off";
}

void* SceneCaptureView::native_texture() const {
    void* tex = m_rhi_texture.load();
    if (tex == nullptr) return nullptr;
    return reinterpret_cast<API::FRHITexture2D*>(tex)->get_native_resource();
}

} // namespace fpv
