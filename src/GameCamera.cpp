// GameCamera.cpp - see GameCamera.hpp
#include "GameCamera.hpp"

#include <cstring>
#include <cwctype>
#include <string>

#include "Reflection.hpp"

using namespace uevr;

namespace fpv {

namespace {

template <typename T>
Vec3 read_vec(const uint8_t* src) {
    T tmp[3]{};
    std::memcpy(tmp, src, sizeof(tmp));
    return {static_cast<double>(tmp[0]), static_cast<double>(tmp[1]), static_cast<double>(tmp[2])};
}

constexpr uint8_t kSpawnAlways = 1; // ESpawnActorCollisionHandlingMethod::AlwaysSpawn

} // namespace

bool GameCamera::resolve() {
    if (m_resolved) return true;

    const auto& api = API::get();
    m_camera_actor_class = api->find_uobject<API::UClass>(L"Class /Script/Engine.CameraActor");
    const auto statics_class = api->find_uobject<API::UClass>(L"Class /Script/Engine.GameplayStatics");
    const auto pcm_class = api->find_uobject<API::UClass>(L"Class /Script/Engine.PlayerCameraManager");

    if (m_camera_actor_class == nullptr || statics_class == nullptr || pcm_class == nullptr) {
        m_status = "waiting for engine classes";
        return false;
    }

    m_gameplay_statics = statics_class->get_class_default_object();
    if (const auto ksl = api->find_uobject<API::UClass>(L"Class /Script/Engine.KismetSystemLibrary")) {
        m_kismet_system = ksl->get_class_default_object();
    }
    if (m_kismet_system == nullptr) {
        api->log_warn("[FPV Drone] KismetSystemLibrary not found: collisions disabled");
    }
    if (m_gameplay_statics == nullptr) {
        m_status = "GameplayStatics default object not found";
        return false;
    }

    // Camera cache layout: PlayerCameraManager::CameraCachePrivate.POV.{Location,Rotation,FOV}
    auto cache_prop = refl::find_property(pcm_class, L"CameraCachePrivate");
    if (cache_prop == nullptr) cache_prop = refl::find_property(pcm_class, L"CameraCache");
    const auto cache_struct = cache_prop ? static_cast<API::FStructProperty*>(cache_prop)->get_struct() : nullptr;
    const auto pov_prop = cache_struct ? refl::find_property(cache_struct, L"POV") : nullptr;
    const auto pov_struct = pov_prop ? static_cast<API::FStructProperty*>(pov_prop)->get_struct() : nullptr;
    const auto loc = pov_struct ? refl::find_property(pov_struct, L"Location") : nullptr;
    const auto rot = pov_struct ? refl::find_property(pov_struct, L"Rotation") : nullptr;
    const auto fov = pov_struct ? refl::find_property(pov_struct, L"FOV") : nullptr;

    if (loc != nullptr && rot != nullptr && fov != nullptr) {
        m_cache_ok = true;
        m_cache_double = refl::struct_size(loc) >= 24;
        m_pov_offset = cache_prop->get_offset() + pov_prop->get_offset();
        m_location_offset = loc->get_offset();
        m_rotation_offset = rot->get_offset();
        m_fov_offset = fov->get_offset();
    } else {
        api->log_warn("[FPV Drone] camera cache layout not found; the drone will spawn at the pawn instead");
    }

    m_resolved = true;
    m_status = "ready";
    api->log_info("[FPV Drone] engine resolved (camera cache: %s, %s precision)",
                  m_cache_ok ? "ok" : "missing", m_cache_double ? "double" : "single");
    return true;
}

bool GameCamera::read_game_pov(CameraPov& out) const {
    const auto pc = API::get()->get_player_controller(0);
    if (pc == nullptr) return false;

    const auto pcm = refl::get_object_property(pc, L"PlayerCameraManager");
    if (m_cache_ok && pcm != nullptr) {
        const auto pov = reinterpret_cast<const uint8_t*>(pcm) + m_pov_offset;
        Vec3 r{};
        if (m_cache_double) {
            out.location = read_vec<double>(pov + m_location_offset);
            r = read_vec<double>(pov + m_rotation_offset);
        } else {
            out.location = read_vec<float>(pov + m_location_offset);
            r = read_vec<float>(pov + m_rotation_offset);
        }
        out.rotation = {r.x, r.y, r.z};
        std::memcpy(&out.fov, pov + m_fov_offset, sizeof(float));
        return true;
    }

    // Fallback: the pawn's location and the controller's rotation.
    const auto pawn = API::get()->get_local_pawn(0);
    if (pawn == nullptr) return false;
    refl::Call loc{pawn, L"K2_GetActorLocation"};
    if (!loc.invoke()) return false;
    API::FProperty* prop{};
    const auto data = loc.param(L"ReturnValue", &prop);
    if (data == nullptr) return false;
    out.location = refl::struct_size(prop) >= 24 ? read_vec<double>(data) : read_vec<float>(data);
    out.rotation = {};
    out.fov = 90.0f;
    return true;
}

API::UObject* GameCamera::spawn_camera(API::UObject* pc, const Vec3& location) {
    refl::Call begin{m_gameplay_statics, L"BeginDeferredActorSpawnFromClass"};
    if (!begin.valid()) {
        m_status = "BeginDeferredActorSpawnFromClass not found";
        return nullptr;
    }
    begin.set(L"WorldContextObject", pc);
    begin.set(L"ActorClass", m_camera_actor_class);
    begin.set_transform(L"SpawnTransform", location);
    begin.set(L"CollisionHandlingOverride", kSpawnAlways);
    begin.invoke();

    const auto actor = begin.get<API::UObject*>(L"ReturnValue");
    if (actor == nullptr) {
        m_status = "camera spawn failed";
        return nullptr;
    }

    refl::Call finish{m_gameplay_statics, L"FinishSpawningActor"};
    finish.set(L"Actor", actor);
    finish.set_transform(L"SpawnTransform", location);
    finish.invoke();

    const auto spawned = finish.get<API::UObject*>(L"ReturnValue");
    return spawned != nullptr ? spawned : actor;
}

API::UObject* GameCamera::get_view_target(API::UObject* pc) const {
    refl::Call call{pc, L"GetViewTarget"};
    if (!call.invoke()) return nullptr;
    return call.get<API::UObject*>(L"ReturnValue");
}

bool GameCamera::set_view_target(API::UObject* pc, API::UObject* target) {
    refl::Call call{pc, L"SetViewTargetWithBlend"};
    if (!call.valid()) return false;
    call.set(L"NewViewTarget", target); // blend time 0 = immediate cut
    return call.invoke();
}

void GameCamera::set_player_input_ignored(API::UObject* pc, bool ignored) {
    // These are counters in Unreal: every "true" must be balanced by a "false".
    struct Fn { const wchar_t* name; const wchar_t* param; };
    for (const Fn& fn : {Fn{L"SetIgnoreMoveInput", L"bNewMoveInput"}, Fn{L"SetIgnoreLookInput", L"bNewLookInput"}}) {
        refl::Call call{pc, fn.name};
        if (!call.valid()) continue;
        call.set(fn.param, ignored);
        call.invoke();
    }
}

void GameCamera::forget() {
    m_actor = nullptr;
    m_camera_component = nullptr;
    m_pc = nullptr;
    m_pc_outer = nullptr;
    m_input_frozen = false;
    m_last_fov = -1.0f;
}

bool GameCamera::activate(const CameraPov& pov, bool freeze_player) {
    if (!resolve()) return false;
    if (m_actor != nullptr) deactivate();

    const auto pc = API::get()->get_player_controller(0);
    if (pc == nullptr) {
        m_status = "no player controller";
        return false;
    }

    const auto actor = spawn_camera(pc, pov.location);
    if (actor == nullptr) {
        API::get()->log_error("[FPV Drone] %s", m_status.c_str());
        return false;
    }

    m_actor = actor;
    m_camera_component = refl::get_object_property(actor, L"CameraComponent");
    if (m_camera_component != nullptr) {
        m_camera_component->set_bool_property(L"bConstrainAspectRatio", false); // no black bars
    }
    m_pc = pc;
    m_pc_outer = pc->get_outer();
    m_freeze_player = freeze_player;
    m_reasserts = 0;

    set_view_target(pc, actor);
    if (freeze_player) {
        set_player_input_ignored(pc, true);
        m_input_frozen = true;
    }

    const bool ok = get_view_target(pc) == actor;
    m_status = ok ? "active" : "active (view target not confirmed)";
    API::get()->log_info("[FPV Drone] camera actor spawned @ %p, view target %s", actor, ok ? "set" : "NOT set");
    return true;
}

bool GameCamera::update(const CameraPov& pov, bool write_fov) {
    if (m_actor == nullptr) return false;

    const auto pc = API::get()->get_player_controller(0);
    if (pc == nullptr) {
        m_status = "no player controller (loading?)";
        return false;
    }

    // Level change / seamless travel: our actor died with the old world. Spawn a new one.
    if (pc != m_pc || pc->get_outer() != m_pc_outer) {
        API::get()->log_info("[FPV Drone] world changed, re-spawning the camera");
        const bool freeze = m_freeze_player;
        forget();
        return activate(pov, freeze);
    }

    if (get_view_target(pc) != m_actor) {
        set_view_target(pc, m_actor);
        ++m_reasserts;
    }

    refl::Call move{m_actor, L"K2_SetActorLocationAndRotation"};
    move.set_vector(L"NewLocation", pov.location);
    move.set_rotator(L"NewRotation", pov.rotation);
    move.set(L"bSweep", false);
    move.set(L"bTeleport", true);
    if (!move.invoke()) {
        m_status = "K2_SetActorLocationAndRotation not found";
        return false;
    }

    if (write_fov && m_camera_component != nullptr && pov.fov != m_last_fov) {
        if (auto fov = m_camera_component->get_property_data<float>(L"FieldOfView")) {
            *fov = pov.fov;
            m_last_fov = pov.fov;
        }
    }

    m_status = "active";
    return true;
}

bool GameCamera::sweep(const Vec3& from, const Vec3& to, double radius, int channel, SweepHit& out) {
    if (m_kismet_system == nullptr || m_actor == nullptr) return false;

    refl::Call trace{m_kismet_system, L"SphereTraceSingle"};
    if (!trace.valid()) return false;

    // The camera actor is the world context; bIgnoreSelf skips it.
    trace.set(L"WorldContextObject", m_actor);
    trace.set_vector(L"Start", from);
    trace.set_vector(L"End", to);
    trace.set(L"Radius", static_cast<float>(radius));
    trace.set(L"TraceChannel", static_cast<uint8_t>(channel == 1 ? 1 : 0)); // TraceTypeQuery1/2
    trace.set(L"bTraceComplex", false);
    trace.set(L"bIgnoreSelf", true);
    // ActorsToIgnore stays an empty TArray, DrawDebugType = None (zero).
    if (!trace.invoke()) return false;
    if (!trace.get<bool>(L"ReturnValue")) return false;

    API::FProperty* hit_prop{};
    const auto hit = trace.param(L"OutHit", &hit_prop);
    if (hit == nullptr) return false;

    if (m_hit_layout == HitLayout::Unknown && !resolve_hit_layout(hit_prop, hit, from)) {
        return false; // try again on the next hit
    }
    if (m_hit_layout == HitLayout::Failed) {
        return false;
    }

    auto vec_at = [&](int32_t off) -> Vec3 {
        if (m_hit_double) {
            double t[3]{};
            std::memcpy(t, hit + off, sizeof(t));
            return {t[0], t[1], t[2]};
        }
        float t[3]{};
        std::memcpy(t, hit + off, sizeof(t));
        return {t[0], t[1], t[2]};
    };

    out.location = vec_at(m_off_location);
    out.normal = vec_at(m_off_normal);

    // Sanity checks: a garbage read must never teleport the drone.
    const double nlen = out.normal.length();
    if (nlen < 0.5 || nlen > 1.5) return false;
    out.normal = out.normal / nlen;

    // Time along the sweep: computed from the location (works for every layout).
    const Vec3 seg = to - from;
    const double seg_len2 = dot(seg, seg);
    out.time = seg_len2 > 1e-9 ? clampv(dot(out.location - from, seg) / seg_len2, 0.0, 1.0) : 0.0;
    if ((out.location - from).length() > std::sqrt(seg_len2) + 2.0 * radius + 1.0) return false;

    float pen{};
    if (m_off_penetration >= 0) {
        std::memcpy(&pen, hit + m_off_penetration, sizeof(float));
    }
    out.start_penetrating = pen > 0.0f;
    return true;
}

namespace {

std::string narrow(const std::wstring& w) {
    std::string out;
    out.reserve(w.size());
    for (wchar_t c : w) out.push_back(c < 128 ? static_cast<char>(c) : '?');
    return out;
}

std::wstring lower(std::wstring w) {
    for (auto& c : w) c = static_cast<wchar_t>(towlower(c));
    return w;
}

// Case-insensitive lookup walking the property list (and logs what it saw).
API::FProperty* find_property_ci(API::UStruct* s, const wchar_t* name, std::string* seen) {
    const auto want = lower(name);
    for (auto c = s; c != nullptr; c = c->get_super()) {
        for (auto f = c->get_child_properties(); f != nullptr; f = f->get_next()) {
            const auto fname = f->get_fname();
            if (fname == nullptr) continue;
            const auto n = fname->to_string();
            if (seen != nullptr && seen->size() < 600) {
                *seen += narrow(n) + " ";
            }
            if (lower(n) == want) return reinterpret_cast<API::FProperty*>(f);
        }
    }
    return nullptr;
}

} // namespace

bool GameCamera::resolve_hit_layout(API::FProperty* hit_prop, const uint8_t* hit, const Vec3& from) {
    const auto& api = API::get();
    const auto hs = hit_prop != nullptr ? static_cast<API::FStructProperty*>(hit_prop)->get_struct() : nullptr;
    const int32_t size = hs != nullptr ? hs->get_struct_size() : 0;

    // 1) Reflection, exact then case-insensitive names.
    if (hs != nullptr) {
        auto loc = refl::find_property(hs, L"Location");
        auto nrm = refl::find_property(hs, L"Normal");
        auto pen = refl::find_property(hs, L"PenetrationDepth");
        std::string seen;
        if (loc == nullptr) loc = find_property_ci(hs, L"Location", &seen);
        if (nrm == nullptr) nrm = find_property_ci(hs, L"Normal", nullptr);
        if (pen == nullptr) pen = find_property_ci(hs, L"PenetrationDepth", nullptr);
        if (loc != nullptr && nrm != nullptr) {
            m_off_location = loc->get_offset();
            m_off_normal = nrm->get_offset();
            m_off_penetration = pen != nullptr ? pen->get_offset() : -1;
            m_hit_double = refl::struct_size(loc) >= 24;
            m_hit_layout = HitLayout::Reflection;
            api->log_info("[FPV Drone] FHitResult via reflection (size %d, loc 0x%X, normal 0x%X, %s)", size,
                          m_off_location, m_off_normal, m_hit_double ? "double" : "float");
            return true;
        }
        api->log_warn("[FPV Drone] FHitResult reflection incomplete (size %d), fields seen: %s", size, seen.c_str());
    }

    // 2) Scan for TraceStart, which equals our sweep start. In every UE4/UE5 FHitResult the
    //    vectors are laid out Location, ImpactPoint, Normal, ImpactNormal, TraceStart, TraceEnd,
    //    followed by PenetrationDepth.
    const int32_t limit = size > 0 ? size : 256;
    for (int pass = 0; pass < 2 && m_hit_layout == HitLayout::Unknown; ++pass) {
        const bool dbl = (pass == 1);
        const int32_t vsize = dbl ? 24 : 12;
        const int32_t step = dbl ? 8 : 4;
        for (int32_t off = 4 * vsize; off + vsize <= limit; off += step) {
            Vec3 v{};
            if (dbl) {
                double t[3]{};
                std::memcpy(t, hit + off, sizeof(t));
                v = {t[0], t[1], t[2]};
            } else {
                float t[3]{};
                std::memcpy(t, hit + off, sizeof(t));
                v = {t[0], t[1], t[2]};
            }
            if ((v - from).length() < 0.01 * (1.0 + from.length() * 1e-4)) {
                m_off_location = off - 4 * vsize;
                m_off_normal = off - 2 * vsize;
                m_off_penetration = off + 2 * vsize;
                m_hit_double = dbl;
                m_hit_layout = dbl ? HitLayout::ScannedDouble : HitLayout::ScannedFloat;
                api->log_info("[FPV Drone] FHitResult layout found by scan (size %d, trace start 0x%X, %s)", size, off,
                              dbl ? "double" : "float");
                return true;
            }
        }
    }

    // A hit without a recognizable TraceStart can happen (e.g. start penetrating); give up only
    // after several attempts.
    if (++m_hit_scan_failures >= 20) {
        m_hit_layout = HitLayout::Failed;
        api->log_error("[FPV Drone] FHitResult layout not found (size %d): collisions disabled", size);
    }
    return false;
}

void GameCamera::deactivate() {
    if (m_actor == nullptr) return;

    const auto pc = API::get()->get_player_controller(0);
    const bool same_world = pc != nullptr && pc == m_pc && pc->get_outer() == m_pc_outer;

    if (same_world) {
        if (m_input_frozen) set_player_input_ignored(pc, false);

        auto pawn = API::get()->get_local_pawn(0);
        set_view_target(pc, pawn != nullptr ? pawn : pc);

        refl::Call destroy{m_actor, L"K2_DestroyActor"};
        destroy.invoke();
    }

    API::get()->log_info("[FPV Drone] camera released");
    forget();
    m_status = "ready";
}

} // namespace fpv
