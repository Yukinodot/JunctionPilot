#ifndef JP_GUARD_H
#define JP_GUARD_H
#include <stdint.h>

enum { JP_HOLD=0, JP_MOVE=1 };
typedef struct {
    uint32_t plan_revision;
    uint16_t lost_line_ticks;
    uint8_t pose_confirmed;
    uint8_t latched_hold;
} jp_guard;

/* No HAL dependencies: caller must actually apply zero PWM when this returns HOLD. */
int jp_motion_allowed(jp_guard *guard, uint32_t map_revision,
                      uint8_t line_visible, uint8_t obstacle,
                      uint8_t recognition_confirmed, uint16_t lost_line_limit);
/* Explicit acknowledgement after fresh localization/sensor checks. */
int jp_guard_arm(jp_guard *guard, uint32_t plan_revision,
                 uint8_t line_visible, uint8_t obstacle,
                 uint8_t recognition_confirmed, uint8_t pose_confirmed);
#endif
