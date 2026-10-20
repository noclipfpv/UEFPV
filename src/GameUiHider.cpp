// GameUiHider.cpp - see GameUiHider.hpp
#include "GameUiHider.hpp"

#include <unordered_set>

#include "Reflection.hpp"

namespace fpv {

using uevr::API;

namespace {
constexpr uint8_t kVisible = 0, kCollapsed = 1, kHidden = 2;

API::UClass* user_widget_class() {
    static API::UClass* c = API::get()->find_uobject<API::UClass>(L"Class /Script/UMG.UserWidget");
    return c;
}

bool in_viewport(API::UObject* w) {
    refl::Call c{w, L"IsInViewport"};
    return c.valid() && c.invoke() && c.get<bool>(L"ReturnValue");
}

int get_visibility(API::UObject* w) {
    refl::Call c{w, L"GetVisibility"};
    if (!c.valid() || !c.invoke()) return -1;
    return c.get<uint8_t>(L"ReturnValue");
}

void set_visibility(API::UObject* w, uint8_t v) {
    refl::Call c{w, L"SetVisibility"};
    if (c.valid() && c.set<uint8_t>(L"InVisibility", v)) c.invoke();
}
} // namespace

void GameUiHider::update(bool hide, double dt) {
    if (hide) {
        m_timer -= dt;
        if (!m_active || m_timer <= 0.0) {
            m_timer = 0.5;
            m_active = true;
            scan_and_hide();
        }
    } else if (m_active) {
        restore();
        m_active = false;
    }
}

void GameUiHider::scan_and_hide() {
    // Legacy HUD (AHUD::bShowHUD)
    if (m_hud == nullptr) {
        if (auto pc = API::get()->get_player_controller(0)) {
            if (auto hud = refl::get_object_property(pc, L"MyHUD")) {
                m_hud = hud;
                m_hud_was_shown = hud->get_bool_property(L"bShowHUD");
                hud->set_bool_property(L"bShowHUD", false);
            }
        }
    } else {
        m_hud->set_bool_property(L"bShowHUD", false);
    }

    auto cls = user_widget_class();
    if (cls == nullptr) return;
    for (auto w : cls->get_objects_matching(false)) {
        if (w == nullptr || m_hidden.count(w) != 0) continue;
        if (!in_viewport(w)) continue;
        const int v = get_visibility(w);
        if (v < 0 || v == kHidden || v == kCollapsed) continue;
        m_hidden[w] = static_cast<uint8_t>(v);
        set_visibility(w, kHidden);
    }
}

void GameUiHider::restore() {
    if (!m_hidden.empty()) {
        // Only touch widgets that still exist (the game may have destroyed some meanwhile).
        std::unordered_set<API::UObject*> alive;
        if (auto cls = user_widget_class()) {
            for (auto w : cls->get_objects_matching(false)) alive.insert(w);
        }
        for (const auto& [w, original] : m_hidden) {
            if (alive.count(w) == 0) continue;
            if (get_visibility(w) == kHidden) set_visibility(w, original);
        }
        m_hidden.clear();
    }
    if (m_hud != nullptr) {
        // The HUD object may have been replaced (level change): only restore the current one.
        auto pc = API::get()->get_player_controller(0);
        if (pc != nullptr && refl::get_object_property(pc, L"MyHUD") == m_hud) {
            m_hud->set_bool_property(L"bShowHUD", m_hud_was_shown);
        }
        m_hud = nullptr;
    }
}

} // namespace fpv
