// GameUiHider.hpp - hides the game's own UI (UMG widgets on the viewport, legacy AHUD) while the
// drone flies, and puts it back afterwards. Game thread only.
//
// UMG widgets are hidden with UUserWidget::SetVisibility(Hidden) and restored to the visibility
// they had. The scan repeats twice a second so widgets the game adds later are hidden too.
// Restoring only touches widgets that still exist and are still hidden by us.
#pragma once

#include <unordered_map>

#include "uevr/API.hpp"

namespace fpv {

class GameUiHider {
public:
    void update(bool hide, double dt);
    int hidden_count() const { return static_cast<int>(m_hidden.size()); }

private:
    void scan_and_hide();
    void restore();

    std::unordered_map<uevr::API::UObject*, uint8_t> m_hidden{}; // widget -> original visibility
    uevr::API::UObject* m_hud{nullptr};
    bool m_hud_was_shown{false};
    double m_timer{0.0};
    bool m_active{false};
};

} // namespace fpv
