// SPDX-License-Identifier: MIT
// NEC IR transmit as a ZMK keymap behavior. Two selectable backends emit the
// NEC envelope over a 38 kHz carrier:
//   * bit-bang (default): k_busy_wait mark/space on the `irpwm` Zephyr PWM,
//     portable but sensitive to interrupt jitter (see IR_TX_THREAD_PRIORITY).
//   * hardware PWM (CONFIG_IR_TX_HW_PWM): the nRF PWM peripheral plays the whole
//     frame from a RAM duty-cycle buffer via EasyDMA, so the carrier is clocked
//     in hardware and immune to interrupt jitter. nRF-only.
// The high-level per-key sequencing (repeat codes / frame counts / gaps) is
// shared; only the low-level ir_send_frame()/ir_send_repeat() differ.

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

/* nrfx_pwm.h pulls in nrf_pwm.h which references PSEL.OUT; include it before the
   ZMK headers, which #define OUT as a keycode macro that would clobber it. */
#if IS_ENABLED(CONFIG_IR_TX_HW_PWM)
#include <nrfx_pwm.h>
#endif

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/keys.h>

LOG_MODULE_REGISTER(zmk_behavior_ir_tx, LOG_LEVEL_INF);

/* Called around the NEC send with active=true then false. Weakly a no-op;
   another module can override it to quiet its own RF activity so it doesn't
   jitter the (bit-bang) IR waveform. Harmless but unnecessary with HW PWM. */
__weak void zmk_ir_tx_active(bool active) { ARG_UNUSED(active); }

/* 38kHz period in ns */
#define IR_CARRIER_PERIOD_NS 26316U
#define IR_DUTY_NS(period_ns) ((period_ns) / 3U)

/* NEC timing (us) */
#define NEC_LEADER_MARK_US 8992U
#define NEC_LEADER_SPACE_US 4496U
#define NEC_BIT_MARK_US 562U
#define NEC_ONE_SPACE_US 1686U
#define NEC_ZERO_SPACE_US 562U
#define NEC_TRAILER_MARK_US 562U
#define NEC_REPEAT_SPACE_US 2248U
#define NEC_INTERFRAME_GAP_US 58544U
#define NEC_GAP_ADJ_PER_ONE_US 1124U
/* NEC repeat codes are sent every 108ms, measured from the start of the frame. */
#define NEC_REPEAT_PERIOD_MS 108

/* ========================================================================== */
/*  Backend: hardware PWM (nRF EasyDMA)                                        */
/* ========================================================================== */
#if IS_ENABLED(CONFIG_IR_TX_HW_PWM)

#ifndef NRFX_PWM_PIN_NOT_USED
#define NRFX_PWM_PIN_NOT_USED 0xFF
#endif

/* A valid Zephyr Cortex-M IRQ priority (0/1 may be reserved for zero-latency).
   IR is not latency-critical, so a middle priority is fine. */
#define IR_PWM_IRQ_PRIO 2

#define IR_BACKEND_AVAILABLE 1

/* PWM COUNTERTOP for the 38kHz carrier from the 16MHz base clock, and the ~1/3
   duty compare value. Each sequence entry is one carrier period. */
#define IR_PWM_TOP  ((uint16_t)((16000000U + 19000U) / 38000U))  /* ~421 */
#define IR_PWM_DUTY ((uint16_t)(IR_PWM_TOP / 3U))                /* ~140, mark */
#define IR_MARK_VAL  IR_PWM_DUTY  /* carrier at ~33% duty */
#define IR_SPACE_VAL 0U           /* output stays low */

/* Worst case one frame: leader (342+171) + 32*(mark 21 + one-space 64) + trailer
   21 = ~3254 carrier periods. Buffer is only allocated in this backend. */
#define IR_SEQ_MAX 3400U
static nrf_pwm_values_common_t ir_seq[IR_SEQ_MAX];
static uint16_t ir_seq_len;

static nrfx_pwm_t ir_pwm_inst = NRFX_PWM_INSTANCE(0);
static struct k_sem ir_done_sem;
static bool ir_hw_ok;

static void ir_pwm_evt_handler(nrfx_pwm_evt_type_t type, void *ctx) {
    ARG_UNUSED(ctx);
    if (type == NRFX_PWM_EVT_FINISHED) {
        k_sem_give(&ir_done_sem);
    }
}

static bool ir_backend_ready(void) { return ir_hw_ok; }

/* No explicit "carrier off" needed: the PWM output idles low between plays. */
static inline int ir_carrier_off(void) { return 0; }

static void seq_reset(void) { ir_seq_len = 0; }

/* Append `us` microseconds worth of carrier periods, each holding `val`. */
static void seq_add(uint16_t val, uint32_t us) {
    uint32_t periods = (us * 1000U + IR_CARRIER_PERIOD_NS / 2U) / IR_CARRIER_PERIOD_NS;
    while (periods-- > 0U && ir_seq_len < IR_SEQ_MAX) {
        ir_seq[ir_seq_len++] = val;
    }
}

static void seq_add_word(uint16_t b) {
    for (int i = 0; i < 16; i++) {
        seq_add(IR_MARK_VAL, NEC_BIT_MARK_US);
        seq_add(IR_SPACE_VAL, (b & 0x8000) ? NEC_ONE_SPACE_US : NEC_ZERO_SPACE_US);
        b <<= 1;
    }
}

static void ir_play(void) {
    if (!ir_hw_ok || ir_seq_len == 0U) {
        return;
    }
    nrf_pwm_sequence_t seq = {
        .values = { .p_common = ir_seq },
        .length = ir_seq_len,
        .repeats = 0,
        .end_delay = 0,
    };
    k_sem_reset(&ir_done_sem);
    (void)nrfx_pwm_simple_playback(&ir_pwm_inst, &seq, 1, NRFX_PWM_FLAG_STOP);
    /* Frame is <~90ms; wait out the DMA play with a generous ceiling. */
    (void)k_sem_take(&ir_done_sem, K_MSEC(300));
}

static void ir_send_frame(uint16_t customer, uint16_t data) {
    seq_reset();
    seq_add(IR_MARK_VAL, NEC_LEADER_MARK_US);
    seq_add(IR_SPACE_VAL, NEC_LEADER_SPACE_US);
    seq_add_word(customer);
    seq_add_word(data);
    seq_add(IR_MARK_VAL, NEC_TRAILER_MARK_US);
    ir_play();
}

static void ir_send_repeat(void) {
    seq_reset();
    seq_add(IR_MARK_VAL, NEC_LEADER_MARK_US);
    seq_add(IR_SPACE_VAL, NEC_REPEAT_SPACE_US);
    seq_add(IR_MARK_VAL, NEC_TRAILER_MARK_US);
    ir_play();
}

static int ir_backend_init(void) {
    nrfx_pwm_config_t config = {
        .output_pins = {
            (uint8_t)CONFIG_IR_TX_HW_PWM_PIN,
            NRFX_PWM_PIN_NOT_USED,
            NRFX_PWM_PIN_NOT_USED,
            NRFX_PWM_PIN_NOT_USED,
        },
        .pin_inverted = { false, false, false, false },
        .irq_priority = IR_PWM_IRQ_PRIO,
        .base_clock = NRF_PWM_CLK_16MHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value = IR_PWM_TOP,
        .load_mode = NRF_PWM_LOAD_COMMON,
        .step_mode = NRF_PWM_STEP_AUTO,
        .skip_gpio_cfg = false,
        .skip_psel_cfg = false,
    };

    k_sem_init(&ir_done_sem, 0, 1);

    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM0), IR_PWM_IRQ_PRIO,
                nrfx_isr, nrfx_pwm_0_irq_handler, 0);

    nrfx_err_t err = nrfx_pwm_init(&ir_pwm_inst, &config, ir_pwm_evt_handler, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("IR HW PWM init failed: 0x%08x", err);
        return -EIO;
    }
    ir_hw_ok = true;
    LOG_INF("IR HW PWM ready on nRF pin %d (top=%u)", CONFIG_IR_TX_HW_PWM_PIN, IR_PWM_TOP);
    return 0;
}

/* ========================================================================== */
/*  Backend: bit-bang (default, portable)                                     */
/* ========================================================================== */
#else /* !CONFIG_IR_TX_HW_PWM */

#include <zephyr/drivers/pwm.h>

/* DT alias irpwm から PWM を取る（定義がない場合は送信を無効化） */
#define IRPWM_NODE DT_ALIAS(irpwm)
#if DT_NODE_EXISTS(IRPWM_NODE)
static const struct pwm_dt_spec ir_pwm = PWM_DT_SPEC_GET(IRPWM_NODE);
#define IR_BACKEND_AVAILABLE 1
#else
#define IR_BACKEND_AVAILABLE 0
#endif

static bool ir_backend_ready(void) {
#if IR_BACKEND_AVAILABLE
    return device_is_ready(ir_pwm.dev);
#else
    return false;
#endif
}

/* PWM helper: enable carrier (mark) */
static int ir_carrier_on(void) {
#if IR_BACKEND_AVAILABLE
    if (!device_is_ready(ir_pwm.dev)) {
        return -ENODEV;
    }
    return pwm_set_dt(&ir_pwm, IR_CARRIER_PERIOD_NS, IR_DUTY_NS(IR_CARRIER_PERIOD_NS));
#else
    return -ENODEV;
#endif
}

/* PWM helper: stop carrier (space) */
static int ir_carrier_off(void) {
#if IR_BACKEND_AVAILABLE
    if (!device_is_ready(ir_pwm.dev)) {
        return -ENODEV;
    }
    return pwm_set_dt(&ir_pwm, IR_CARRIER_PERIOD_NS, 0);
#else
    return -ENODEV;
#endif
}

static void ir_mark(uint32_t us) {
    (void)ir_carrier_on();
    k_busy_wait(us);
}

static void ir_space(uint32_t us) {
    (void)ir_carrier_off();
    k_busy_wait(us);
}

static void ir_send_word(uint16_t b) {
    for (int i = 0; i < 16; i++) {
        ir_mark(NEC_BIT_MARK_US);
        ir_space((b & 0x8000) ? NEC_ONE_SPACE_US : NEC_ZERO_SPACE_US);
        b <<= 1;
    }
}

static void ir_send_frame(uint16_t customer, uint16_t data) {
    ir_mark(NEC_LEADER_MARK_US);
    ir_space(NEC_LEADER_SPACE_US);
    ir_send_word(customer);
    ir_send_word(data);
    ir_mark(NEC_TRAILER_MARK_US);
    (void)ir_carrier_off();
}

/* NEC repeat code: 9ms leader mark + 2.25ms space + 562us trailer mark. A real
   remote sends these (not the whole frame) while a button is held. */
static void ir_send_repeat(void) {
    ir_mark(NEC_LEADER_MARK_US);
    ir_space(NEC_REPEAT_SPACE_US);
    ir_mark(NEC_TRAILER_MARK_US);
    (void)ir_carrier_off();
}

static int ir_backend_init(void) {
#if IR_BACKEND_AVAILABLE
    if (device_is_ready(ir_pwm.dev)) {
        (void)ir_carrier_off();
    } else {
        LOG_WRN("IR PWM not ready at init; will check again on send");
    }
#endif
    return 0;
}

#endif /* CONFIG_IR_TX_HW_PWM */

/* ========================================================================== */
/*  Shared core: per-key sequencing + behavior driver                         */
/* ========================================================================== */

struct ir_tx_job {
    struct k_work work;
    uint16_t customer;
    uint16_t data;
};

static struct ir_tx_job job;
static atomic_t busy = ATOMIC_INIT(0);

/* Dedicated, cooperative-priority work queue: the bit-bang backend needs it so
   its busy-wait never blocks the shared system work queue or higher-priority
   threads; the HW-PWM backend only sleeps on it, so it is harmless there too. */
#if IR_BACKEND_AVAILABLE
K_THREAD_STACK_DEFINE(ir_tx_q_stack, CONFIG_IR_TX_THREAD_STACK_SIZE);
static struct k_work_q ir_tx_work_q;
#endif

static uint8_t __maybe_unused popcount(uint16_t x) {
    uint8_t c = 0;
    while (x) {
        x &= (uint16_t)(x - 1);
        c++;
    }
    return c;
}

/* Transmit for a single key press. Either the full frame CONFIG_IR_TX_REPEAT_COUNT
   times (redundant, default), or the frame once + CONFIG_IR_TX_REPEAT_COUNT NEC
   repeat codes (held-button behaviour). Gaps use k_sleep (not busy-wait) so key
   scanning / BLE stay responsive between transmissions. */
static void ir_send_frames(uint16_t customer, uint16_t data) {
#if IS_ENABLED(CONFIG_IR_TX_USE_REPEAT_CODE)
    /* IR_TX_FRAME_COUNT sets of (full frame + IR_TX_REPEAT_COUNT repeat codes),
       every transmission spaced at the ~108ms NEC period. `slot` counts all
       transmissions so each is timed from t0. */
    const int64_t t0 = k_uptime_get();
    int slot = 0;
    for (int set = 0; set < CONFIG_IR_TX_FRAME_COUNT; set++) {
        if (slot > 0) {
            (void)ir_carrier_off();
            int64_t wait = (t0 + (int64_t)NEC_REPEAT_PERIOD_MS * slot) - k_uptime_get();
            if (wait > 0) {
                k_sleep(K_MSEC(wait));
            }
        }
        ir_send_frame(customer, data);
        slot++;
        for (int r = 0; r < CONFIG_IR_TX_REPEAT_COUNT; r++) {
            (void)ir_carrier_off();
            int64_t wait = (t0 + (int64_t)NEC_REPEAT_PERIOD_MS * slot) - k_uptime_get();
            if (wait > 0) {
                k_sleep(K_MSEC(wait));
            }
            ir_send_repeat();
            slot++;
        }
    }
#else
    const uint32_t ones = (uint32_t)(popcount(customer) + popcount(data));
    const uint32_t gap_us = NEC_INTERFRAME_GAP_US - NEC_GAP_ADJ_PER_ONE_US * ones;
    for (int n = 0; n < CONFIG_IR_TX_REPEAT_COUNT; n++) {
        if (n > 0) {
            (void)ir_carrier_off();
            k_sleep(K_USEC(gap_us));
        }
        ir_send_frame(customer, data);
    }
#endif
}

static void ir_work_handler(struct k_work *work) {
    struct ir_tx_job *j = CONTAINER_OF(work, struct ir_tx_job, work);

    if (!IR_BACKEND_AVAILABLE || !ir_backend_ready()) {
        LOG_WRN("IR TX: backend not ready on this side, skipping");
        atomic_set(&busy, 0);
        return;
    }

    LOG_INF("IR TX NEC32 customer=0x%04x data=0x%04x x%d", j->customer, j->data,
            CONFIG_IR_TX_REPEAT_COUNT);

    /* Bit-bang is jittered by another radio user's interrupts; signal start/end
       so such a module can go quiet (weak no-op if none, unnecessary for HW PWM). */
    zmk_ir_tx_active(true);
    ir_send_frames(j->customer, j->data);
    zmk_ir_tx_active(false);

    atomic_set(&busy, 0);
}

/* behavior driver: pressed */
static int on_ir_tx_binding_pressed(struct zmk_behavior_binding *binding,
                                   struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    if (atomic_cas(&busy, 0, 1) == false) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    job.customer = (uint16_t)binding->param1;
    job.data = (uint16_t)(binding->param2);

#if IR_BACKEND_AVAILABLE
    k_work_submit_to_queue(&ir_tx_work_q, &job.work);
#else
    k_work_submit(&job.work);
#endif

    return ZMK_BEHAVIOR_OPAQUE;
}

/* behavior driver: release */
static int on_ir_tx_binding_released(struct zmk_behavior_binding *binding,
                                    struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_ir_tx_driver_api = {
    .binding_pressed = on_ir_tx_binding_pressed,
    .binding_released = on_ir_tx_binding_released,
};

static int behavior_ir_tx_init(const struct device *dev) {
    ARG_UNUSED(dev);

    k_work_init(&job.work, ir_work_handler);

#if IR_BACKEND_AVAILABLE
    static const struct k_work_queue_config q_cfg = {.name = "ir_tx"};
    k_work_queue_start(&ir_tx_work_q, ir_tx_q_stack, K_THREAD_STACK_SIZEOF(ir_tx_q_stack),
                       CONFIG_IR_TX_THREAD_PRIORITY, &q_cfg);
    (void)ir_backend_init();
#endif

    return 0;
}

/*
 * DT インスタンス化：このマクロは app/include/drivers/behavior.h に存在し、
 * 他の behavior 実装（例 app/src/behaviors/behavior_key_press.c）も使っているのと同じ流儀。
 */
#define DT_DRV_COMPAT zmk_behavior_ir_tx

BEHAVIOR_DT_INST_DEFINE(0, behavior_ir_tx_init, NULL, NULL, NULL,
                        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_ir_tx_driver_api);
