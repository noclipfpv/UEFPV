// GameCamera.hpp - game-agnostic camera takeover for flat-screen play.
//
// A standard ACameraActor is spawned in the player's world and made the player's view target
// (APlayerController::SetViewTargetWithBlend). Every frame, before the world ticks, the actor is
// moved to the drone camera; the engine's own camera manager then renders from it. Everything
// goes through Unreal's reflection system, so no per-game offsets are needed.
//
// All methods must be called on the game thread.
#pragma once

#include <cstdint>
#include <string>

#include "uevr/API.hpp"

#include "MathTypes.hpp"

namespace fpv {

struct CameraPov {
    Vec3 location{};   // Unreal units
    Rotator rotation{};
    float fov{90.0f};
};

struct SweepHit {
    Vec3 location{};      // sphere center at impact (Unreal units)
    Vec3 normal{};        // unit normal pointing away from the surface
    double time{1.0};     // 0..1 along the sweep
    bool start_penetrating{false};
};

class GameCamera {
public:
    // Finds the classes / functions / camera cache layout. Returns false while the engine is not
    // ready yet (call again later).
    bool resolve();
    bool resolved() const { return m_resolved; }

    // The view the game is currently rendering (last frame), from the camera manager cache.
    bool read_game_pov(CameraPov& out) const;

    // Spawns the drone camera at `pov` and makes it the view target.
    bool activate(const CameraPov& pov, bool freeze_player);
    // Moves the drone camera. Re-spawns it after a level change and re-asserts the view target if
    // the game switched it away. Returns false if the camera is not available this frame.
    bool update(const CameraPov& pov, bool write_fov);
    // Gives the view back to the player and destroys the drone camera.
    void deactivate();

    // Sphere sweep in the drone's world (Unreal units). Returns true on a blocking hit.
    // channel: 0 = Visibility, 1 = Camera.
    bool sweep(const Vec3& from, const Vec3& to, double radius, int channel, SweepHit& out);
    bool sweep_available() const { return m_kismet_system != nullptr && m_hit_layout != HitLayout::Failed; }

    bool active() const { return m_actor != nullptr; }
    uevr::API::UObject* actor() const { return m_actor; }
    const std::string& status() const { return m_status; }
    uint32_t view_target_reasserts() const { return m_reasserts; }

private:
    uevr::API::UObject* spawn_camera(uevr::API::UObject* pc, const Vec3& location);
    uevr::API::UObject* get_view_target(uevr::API::UObject* pc) const;
    bool set_view_target(uevr::API::UObject* pc, uevr::API::UObject* target);
    void set_player_input_ignored(uevr::API::UObject* pc, bool ignored);
    void forget(); // drop pointers without touching the objects (they may be gone)

    bool m_resolved{false};
    std::string m_status{"not resolved"};

    uevr::API::UClass* m_camera_actor_class{nullptr};
    uevr::API::UObject* m_gameplay_statics{nullptr}; // CDO, used to call static functions
    uevr::API::UObject* m_kismet_system{nullptr};    // CDO of KismetSystemLibrary (traces)

    // FHitResult layout (resolved on the first sweep). Offsets inside the struct.
    enum class HitLayout { Unknown, Reflection, ScannedFloat, ScannedDouble, Failed };
    HitLayout m_hit_layout{HitLayout::Unknown};
    int32_t m_off_location{-1};
    int32_t m_off_normal{-1};
    int32_t m_off_penetration{-1};
    bool m_hit_double{false};
    int m_hit_scan_failures{0};
    bool resolve_hit_layout(uevr::API::FProperty* hit_prop, const uint8_t* hit, const Vec3& from);

    // Camera cache layout (only used to read the spawn POV)
    bool m_cache_ok{false};
    bool m_cache_double{false};
    int32_t m_pov_offset{-1};
    int32_t m_location_offset{-1};
    int32_t m_rotation_offset{-1};
    int32_t m_fov_offset{-1};

    // Live state
    uevr::API::UObject* m_actor{nullptr};
    uevr::API::UObject* m_camera_component{nullptr};
    uevr::API::UObject* m_pc{nullptr};
    uevr::API::UObject* m_pc_outer{nullptr};
    bool m_freeze_player{false};
    bool m_input_frozen{false};
    float m_last_fov{-1.0f};
    uint32_t m_reasserts{0};
};

} // namespace fpv
