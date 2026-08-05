#ifndef _CORE_H_
#define _CORE_H_
#include <stdint.h>

/*
 * Transport-agnostic command core (MERGE_PLAN §5).
 *
 * Each function takes already-normalized values (gains 0-indexed, real widths)
 * and operates on the single global config owned by PAM
 * (adpd_current_config / adpd_gains_config / adpd_mode). It does NO wire I/O:
 * no Serial / printf / JSON. The frontends (text do_command, binary run_esp,
 * and later the openJII json adapter) parse their own wire format, resolve
 * their own conventions (sentinels, 1-indexing), call these, then format output.
 *
 * Increment 1 covers the operations whose orchestration was duplicated across
 * the text parser's arrun / arrun1 / arrun2 / q / r / w and set_currents /
 * set_gains cases. The binary frontend adopts the same functions in a later
 * increment, gated on the §13 wire-conformance check.
 */

// Config setters. Values are final/normalized; the caller has already resolved
// its wire conventions. Marks the config dirty (adpd_mode = MPF_MODE) so the
// next array run re-applies it — the text path's "set, then apply on run".
void core_set_currents(uint8_t i620, uint8_t i720, uint8_t ir);
void core_set_gains(uint8_t fluo, uint8_t fluo_ref, uint8_t ir, uint8_t ir_ref,
                    uint8_t sun, uint8_t leaf);

// Apply the current global config to the detector. core_config_detector applies
// unconditionally + enters array mode (binary cmd 10). core_ensure_array_config
// applies only if not already in array mode — for callers (binary cmd 21) that
// interleave wire I/O between the config step and the run step.
void core_config_detector(void);
void core_ensure_array_config(void);

// Array-mode run: the single copy of "ensure config, then run". `persist` is
// treated as a bool (nonzero = LEDs persist), matching the legacy run_arr_type1.
int core_run_array(uint8_t len, uint8_t* arr, uint8_t persist, bool allow_interrupt);

// Multi-phase (trigger-spacer) run; always interruptible (the legacy constant).
int core_run_mpf(uint16_t length, uint8_t interval, bool change_act, uint8_t act);

#endif
