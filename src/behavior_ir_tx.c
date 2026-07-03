// SPDX-License-Identifier: MIT
// NEC IR transmit as a ZMK keymap behavior. Bit-bangs the NEC envelope over a
// 38 kHz PWM carrier (the `irpwm` alias) and sends the frame
// CONFIG_IR_TX_REPEAT_COUNT times on a dedicated cooperative-priority work queue
// so the waveform is not preempted.

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/keys.h>

LOG_MODULE_REGISTER(zmk_behavior_ir_tx, LOG_LEVEL_INF);

/* DT alias irpwm から PWM を取る（定義がない場合は送信を無効化） */
#define IRPWM_NODE DT_ALIAS(irpwm)
#if DT_NODE_EXISTS(IRPWM_NODE)
static const struct pwm_dt_spec ir_pwm = PWM_DT_SPEC_GET(IRPWM_NODE);
#define IR_PWM_AVAILABLE 1
#else
#define IR_PWM_AVAILABLE 0
#endif

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

struct ir_tx_job {
    struct k_work work;
    uint16_t customer;
    uint16_t data;
};

static struct ir_tx_job job;
static atomic_t busy = ATOMIC_INIT(0);

/* IR is sent only where the PWM carrier exists (the central / devkit). Give it a
   dedicated, preemptible work queue so its busy-wait never blocks the shared
   system work queue (CH559 mouse, etc.) or higher-priority threads (keys, BLE). */
#if IR_PWM_AVAILABLE
K_THREAD_STACK_DEFINE(ir_tx_q_stack, CONFIG_IR_TX_THREAD_STACK_SIZE);
static struct k_work_q ir_tx_work_q;
#endif

/* PWM helper: enable carrier (mark) */
static int ir_carrier_on(void) {
#if IR_PWM_AVAILABLE
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
#if IR_PWM_AVAILABLE
    if (!device_is_ready(ir_pwm.dev)) {
        return -ENODEV;
    }
    return pwm_set_dt(&ir_pwm, IR_CARRIER_PERIOD_NS, 0);
#else
    return -ENODEV;
#endif
}

static inline void ir_delay_us(uint32_t us) {
    k_busy_wait(us);
}

static void ir_mark(uint32_t us) {
    (void)ir_carrier_on();
    ir_delay_us(us);
}

static void ir_space(uint32_t us) {
    (void)ir_carrier_off();
    ir_delay_us(us);
}

static void ir_send_word(uint16_t b) {
    for (int i = 0; i < 16; i++) {
        ir_mark(NEC_BIT_MARK_US);
        if (b & 0x8000) {
            ir_space(NEC_ONE_SPACE_US);
        } else {
            ir_space(NEC_ZERO_SPACE_US);
        }
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

static uint8_t popcount(uint16_t x) {
    uint8_t c = 0;
    while (x) {
        x &= (uint16_t)(x - 1);
        c++;
    }
    return c;
}

/* Send the complete NEC frame CONFIG_IR_TX_REPEAT_COUNT times. The codes are
   idempotent direct-select commands, so repeating the whole frame just adds
   redundancy against a missed signal. The inter-frame gap uses k_sleep (rather
   than a busy-wait) so key scanning / BLE stay responsive between frames. */
static void ir_send_frames(uint16_t customer, uint16_t data) {
    const uint32_t ones = (uint32_t)(popcount(customer) + popcount(data));
    const uint32_t gap_us = NEC_INTERFRAME_GAP_US - NEC_GAP_ADJ_PER_ONE_US * ones;
    for (int n = 0; n < CONFIG_IR_TX_REPEAT_COUNT; n++) {
        if (n > 0) {
            (void)ir_carrier_off();
            k_sleep(K_USEC(gap_us));
        }
        ir_send_frame(customer, data);
    }
}

static void ir_work_handler(struct k_work *work) {
    struct ir_tx_job *j = CONTAINER_OF(work, struct ir_tx_job, work);

#if !IR_PWM_AVAILABLE
    LOG_WRN("IR TX: no PWM hardware on this side, skipping");
    atomic_set(&busy, 0);
    return;
#else
    if (!device_is_ready(ir_pwm.dev)) {
        LOG_ERR("IR PWM device not ready (dev not ready at send time)");
        atomic_set(&busy, 0);
        return;
    }
#endif

    LOG_INF("IR TX NEC32 customer=0x%04x data=0x%04x x%d", j->customer, j->data,
            CONFIG_IR_TX_REPEAT_COUNT);

    ir_send_frames(j->customer, j->data);

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

#if IR_PWM_AVAILABLE
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

#if IR_PWM_AVAILABLE
    static const struct k_work_queue_config q_cfg = {.name = "ir_tx"};
    k_work_queue_start(&ir_tx_work_q, ir_tx_q_stack, K_THREAD_STACK_SIZEOF(ir_tx_q_stack),
                       CONFIG_IR_TX_THREAD_PRIORITY, &q_cfg);

    if (device_is_ready(ir_pwm.dev)) {
        (void)ir_carrier_off();
    } else {
        LOG_WRN("IR PWM not ready at init; will check again on send");
    }
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