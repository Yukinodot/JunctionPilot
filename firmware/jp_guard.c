#include "jp_guard.h"
#include <limits.h>

int jp_motion_allowed(jp_guard *g, uint32_t revision, uint8_t line_visible,
                      uint8_t obstacle, uint8_t recognition_confirmed, uint16_t limit) {
    if (!g || limit == 0) return JP_HOLD;
    if (line_visible) g->lost_line_ticks = 0;
    else if (g->lost_line_ticks < UINT16_MAX) ++g->lost_line_ticks;
    if (g->plan_revision != revision || obstacle || !recognition_confirmed ||
        !g->pose_confirmed || g->lost_line_ticks >= limit) g->latched_hold = 1;
    if (g->latched_hold) return JP_HOLD;
    return JP_MOVE;
}

int jp_guard_arm(jp_guard *g, uint32_t plan_revision, uint8_t line_visible,
                 uint8_t obstacle, uint8_t recognition_confirmed, uint8_t pose_confirmed) {
    if (!g) return JP_HOLD;
    if (!line_visible || obstacle || !recognition_confirmed || !pose_confirmed) {
        g->latched_hold = 1; return JP_HOLD;
    }
    g->plan_revision = plan_revision; g->pose_confirmed = 1;
    g->lost_line_ticks = 0; g->latched_hold = 0;
    return JP_MOVE;
}
