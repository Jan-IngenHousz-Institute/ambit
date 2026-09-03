#ifndef _PAM_H_
#define _PAM_H_
#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/devices_init.h"
#include "src/mlx90632/u_mlx.h"
#include "data_utils.h"
#include <Arduino.h>


extern ADPD6 adpd;
extern uint8_t adpd_mode;

typedef struct{
    bool init = false;
    uint8_t I620 = 110;
    uint8_t I720 = 40;
    uint8_t IR = 20;
} adpd_current_config_t;

typedef struct {
    bool init = false;
    uint8_t Fluo = 1;
    uint8_t FluoRef = 5;
    uint8_t IR = 5;
    uint8_t IRRef = 1;
    uint8_t Sun = 5;
    uint8_t Leaf = 5;
} adpd_gains_config_t;


extern adpd_current_config_t adpd_current_config;
extern adpd_gains_config_t adpd_gains_config;



int conf_slow_FR_1(uint8_t I620, uint8_t I730, uint8_t I_FR, uint8_t G_Fluor, uint8_t G_FluorRef, uint8_t G_Sun, uint8_t G_IR, uint8_t G_FR, uint8_t G_FRref);
int conf_slow_FR_1(void);
int fluor_offset_test(uint8_t current, uint8_t num_integ, uint8_t lit_offset, uint8_t dark1_offset, uint8_t dark2_offset, uint8_t pulse_offset, uint8_t pulse_duration);

int MPF(uint16_t mode, uint16_t current, uint16_t dc_current, uint8_t sign_gain, uint8_t ref_gain);
int MPF(uint16_t mode, uint16_t dc_current);
int run_arr_type1(uint8_t length, uint8_t* arr, bool led_persist);
// retain=true: keep the result buffers alive in the async holder (no FSM/serial
// send, no delete) so a later FETCH can stream them — used by the parallel
// trigger/poll/fetch protocol. Default false = legacy synchronous send+free.
int run_arr_type1(uint8_t length, uint8_t* arr, bool led_persist, bool allow_interrupt, bool json_output = false, bool retain = false);
int run_trigger_spacer(uint16_t length, uint8_t interval, bool change_act, uint8_t act, bool interrrupt);

// ── Exact-N triggered acquisition (plans/DETERMINISTIC_ADPD.md) ──────────────
// Deterministic sibling of run_arr_type1: same frozen 8-byte line protocol, same
// calibration / storage / sinks (shared helpers), but the ADPD runs in EXT_SYNC so
// every sample is one GPIO10 edge -> exactly one timeslot sequence. Emitted LED
// sequences == edges == stored == num_ptx at every rate. `freq` is the software
// pacing target (period 1/freq), not a free-run divider. Additive: arrun / cmd 21
// keep the free-run engine until the calibration-gated default swap (plan §6 Ph.5).
// Returns ARR_TRIG_OK or a negative ArrTriggerResult. Logging is compiled out
// (-DCORE_DEBUG_LEVEL=0), so every adapter must reply explicitly on failure.
enum ArrTriggerResult {
    ARR_TRIG_OK           =  0,
    ARR_TRIG_ABORT        = -1,   // generic / internal
    ARR_TRIG_BAD_LINE     = -2,   // total N outside 1..MAX_DATACLASS_SIZE-1, or freq == 0
    ARR_TRIG_NOMEM        = -3,   // dataclass allocation failed
    ARR_TRIG_LOST_TRIGGER = -4,   // an edge produced no full readout within the per-sample bound
    ARR_TRIG_FIFO_DESYNC  = -5,   // FIFO held more than the one sequence the edge asked for (or overflowed)
    ARR_TRIG_IO_ERROR     = -6,   // SPI/driver error on FIFO_BYTE_COUNT or the FIFO read
};
int run_arr_trigger_validate(uint8_t length, uint8_t* arr);
// Owned by ambit-1.ino (the ISR lives there): the BOOT-pin reset gesture is paused for
// the duration of a triggered run, see the definition for why.
void ambit_boot_gesture_pause(void);
void ambit_boot_gesture_resume(void);
int run_arr_trigger(uint8_t length, uint8_t* arr, bool led_persist, bool allow_interrupt,
                    bool json_output = false, bool retain = false);

#ifdef AMBIT_DIAG_TRIGGER
// Bench diagnostics (plan §6 Phase 0, gates V0/V1f). Console-only, branch-only:
// they print on UART0 and are compiled out without -DAMBIT_DIAG_TRIGGER.
// tseq,<reps>,<farred>,<integ>[,<frrep>]: edge -> full-FIFO time, min/mean/max.
int measure_tseq(uint16_t reps, bool farred, uint8_t integ, uint8_t frrep);
// tidle,<farred>,<integ>,<gate>: arm EXT_SYNC, send NO edges for 2 s, count bytes.
int measure_idle(bool farred, uint8_t integ, uint8_t gate);
// tratio,<reps>,<N>,<freq>,<integ>: first-sample s/r vs steady state (plan §4.6).
int measure_first_ratio(uint16_t reps, uint16_t N, uint16_t freq, uint8_t integ);
// tstat: pacing statistics of the last run_arr_trigger() (gate V1j).
void print_trig_stats(void);
// tdrop,<n>: skip the n-th edge of the next run_arr_trigger() (gates V1g/V1h). 0 disarms.
void diag_drop_edge(uint32_t n);
// tpark,<hz>: parked internal period used while EXT_SYNC drives the chip (default 10).
void diag_set_park_hz(uint32_t hz);
// tquiet,<us>: SPI-silent wait after each edge before the first FIFO poll (0 = off).
void diag_set_quiet_us(uint32_t us);
// tsleepq,<us>: light-sleep the core for <us> after each edge (0 = off).
void diag_set_sleepq_us(uint32_t us);
// twfi,<us>: block on a one-shot timer (idle task / WFI) for <us> after each edge (0 = off).
void diag_set_wfi_us(uint32_t us);
// tslotc,<lit>,<width>,<dark2>,<period>: re-time slot C for both engines (production 72,19,90,58).
void diag_set_slotc_timing(uint16_t lit, uint16_t width, uint16_t dark2, uint16_t period);
// tinteg,<n>: NUM_INT of slots A and C for both engines (production 4).
void diag_set_integ(uint8_t n);
// tovf,<n>: before sample n of the next run_arr_trigger(), fire 31 extra edges unread so
// the 640-byte FIFO overflows — the run must abort with -5, never hang (gate V2).
void diag_overflow_at(uint32_t n);
// tblk,<N>,<freq>: N EXT_SYNC sequences, alternating per-value readfifo() and
// readfifo_block(); reports per-column stats for both and the leftover-byte count (gate V2).
int measure_block_read(uint16_t N, uint16_t freq);
// traw,<mode>,<N>,<freq>: dump N raw readouts (sun, leaf, s_dark, s_lit, r_dark, r_lit,
// s730, r730) as CSV; mode 0 = free-run at freq, 1 = EXT_SYNC paced at freq. For the
// "why is s_630 9 % higher in EXT_SYNC" question: which raw term moves.
int measure_raw(uint8_t mode, uint16_t N, uint16_t freq);
#endif

// ── Async (trigger/poll/fetch) result holder — parallel measurement protocol ──
// A run started via ambit_async_run_start() blocks the C3 to completion (as the
// synchronous run does today) but RETAINS its result arrays instead of streaming
// them, so the host can trigger all sensors, then poll state and fetch later.
enum { AMBIT_ASYNC_IDLE = 0, AMBIT_ASYNC_DONE = 1, AMBIT_ASYNC_ERROR = 2 };
int     ambit_async_run_start(uint8_t length, uint8_t* arr, bool led_persist, bool allow_interrupt);
uint8_t ambit_async_get_state(void);   // AMBIT_ASYNC_IDLE | DONE | ERROR
int     ambit_async_fetch(void);       // stream retained arrays (AMBYTE FSM), then clear; 0 ok
void    ambit_async_clear(void);       // free any retained buffers, state -> IDLE
uint32_t PAM_get_env(uint8_t mode, unsigned int t0);
uint32_t PAM_retrieve_env(uint32_t r, uint8_t* mode, float_t* data_f = NULL, int16_t* data_i = NULL);
enum FluorOffsetResult {
    FLUOR_OFFSET_OK = 0,
    FLUOR_OFFSET_INVALID_ARGUMENT = -1,
    FLUOR_OFFSET_CONFIG_ERROR = -2,
    FLUOR_OFFSET_ADPD_ERROR = -3,
    FLUOR_OFFSET_TIMEOUT = -4,
};
int fluor_offset(uint32_t* ret);
enum ADPD_CONFIG_MODE {
    MPF_MODE,
    IDLE,
    ARRAY_MODE1,
    ARRAY_SLOW,
    FUTURE
};

#endif
