// SPDX-License-Identifier: MIT
// 38kHz IR receiver (demodulated output) capture + NEC32 decode -> debug log.
//
// The receiver (e.g. OSRB38C9AA) idles high and pulls low while the 38kHz
// carrier is present, so a "mark" (burst) is a LOW level and a "space" is HIGH.
// We time every edge with the cycle counter, and after an idle gap decode the
// NEC framing used by the IR TX behavior: leader + customer(16, MSB first) +
// data(16, MSB first) + trailer.

#define DT_DRV_COMPAT zmk_ir_rx

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ir_rx, CONFIG_IR_RX_LOG_LEVEL);

#define IR_RX_MAX_EDGES 200
#define IR_RX_IDLE_MS   15     // gap that marks the end of a frame

struct ir_rx_config {
    struct gpio_dt_spec in;
};

struct ir_rx_data {
    const struct device *dev;
    struct gpio_callback cb;
    struct k_timer idle;
    struct k_work work;
    uint32_t last_cycle;
    uint16_t durs[IR_RX_MAX_EDGES];  // pulse durations in us (durs[0] = leader mark)
    volatile uint16_t count;
    volatile bool capturing;
};

// Dump the raw pulse widths (us) so non-NEC / undecodable frames can be
// inspected: "m" = mark (IR burst, line low), "s" = space (gap, line high).
static void ir_rx_dump_raw(struct ir_rx_data *data, uint16_t n) {
    char line[120];
    int pos = 0;

    line[0] = '\0';
    for (uint16_t i = 0; i < n; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%c%u ",
                        (i & 1) ? 's' : 'm', data->durs[i]);
        if ((i % 12) == 11 || i == n - 1) {
            LOG_INF("  %s", line);
            pos = 0;
            line[0] = '\0';
        }
    }
}

// Known pulse-distance / pulse-width protocols. Classified by leader timing and
// confirmed by the decoded bit count (NEC and JVC have similar leaders but 32 vs
// 16 bits). All bits are packed first-received-bit = MSB.
//   from_mark = false : bit value is the SPACE length  (NEC family)
//   from_mark = true  : bit value is the MARK length   (Sony SIRC)
struct ir_proto {
    const char *name;
    uint16_t lmark_lo, lmark_hi;    // leader mark (us)
    uint16_t lspace_lo, lspace_hi;  // leader space (us)
    uint8_t  nbits;                 // expected bits (0 = Sony: accept 12/15/20)
    bool     from_mark;
    uint16_t one_thresh;            // duration above this => '1'
    bool     nec_fmt;               // print as customer/data
};

static const struct ir_proto ir_protos[] = {
    /* name        lmark        lspace       bits  fromMark thr   nec  */
    { "NEC",       8000, 10000, 4000, 5000,  32, false, 1000, true  },
    { "Samsung",   3900,  5100, 3900, 5100,  32, false, 1000, false },
    { "JVC",       7700,  9000, 3700, 4700,  16, false, 1000, false },
    { "Panasonic", 3000,  3900, 1450, 2050,  48, false,  900, false },
    { "SONY",      2000,  2800,  350,  850,   0, true,   900, false },
};

static void ir_rx_decode(struct ir_rx_data *data) {
    uint16_t n = data->count;
    data->capturing = false;
    data->count = 0;

    if (n < 4) {
        LOG_DBG("IR: %u edges (noise)", n);
        return;
    }

    uint16_t lmark = data->durs[0];   // leader mark (low)
    uint16_t lspace = data->durs[1];  // leader space (high)

    // NEC "repeat" code: ~9ms mark + ~2.25ms space.
    if (lmark >= 8000 && lmark <= 10000 && lspace >= 1800 && lspace <= 2800) {
        LOG_INF("IR NEC repeat");
        return;
    }

    for (int p = 0; p < (int)ARRAY_SIZE(ir_protos); p++) {
        const struct ir_proto *pr = &ir_protos[p];
        if (lmark < pr->lmark_lo || lmark > pr->lmark_hi ||
            lspace < pr->lspace_lo || lspace > pr->lspace_hi) {
            continue;
        }

        uint64_t val = 0;
        int bits = 0;
        for (int i = 0;; i++) {
            int idx = pr->from_mark ? (2 + 2 * i) : (3 + 2 * i);
            if (idx >= n) {
                break;
            }
            val = (val << 1) | (data->durs[idx] > pr->one_thresh ? 1ULL : 0ULL);
            bits++;
            if (pr->nbits && bits >= pr->nbits) {
                break;
            }
        }

        bool ok = pr->nbits ? (bits == pr->nbits)
                            : (bits == 12 || bits == 15 || bits == 20);
        if (!ok) {
            continue;
        }

        if (pr->nec_fmt) {
            LOG_INF("IR NEC32 customer=0x%04x data=0x%04x (raw=0x%08x)",
                    (uint16_t)(val >> 16), (uint16_t)val, (uint32_t)val);
        } else if (bits > 32) {
            LOG_INF("IR %s bits=%d raw=0x%x%08x", pr->name, bits,
                    (uint32_t)(val >> 32), (uint32_t)val);
        } else {
            LOG_INF("IR %s bits=%d raw=0x%x", pr->name, bits, (uint32_t)val);
        }
        return;
    }

    LOG_INF("IR: unknown frame (%u pulses, leader %u/%u us):", n, lmark, lspace);
    ir_rx_dump_raw(data, n);
}

static void ir_rx_work(struct k_work *work) {
    ir_rx_decode(CONTAINER_OF(work, struct ir_rx_data, work));
}

static void ir_rx_idle_expiry(struct k_timer *timer) {
    struct ir_rx_data *data = k_timer_user_data_get(timer);
    k_work_submit(&data->work);
}

static void ir_rx_cb(const struct device *port, struct gpio_callback *cb,
                     gpio_port_pins_t pins) {
    struct ir_rx_data *data = CONTAINER_OF(cb, struct ir_rx_data, cb);
    const struct ir_rx_config *cfg = data->dev->config;
    uint32_t now = k_cycle_get_32();
    int lvl = gpio_pin_get_dt(&cfg->in);   // raw level (spec is ACTIVE_HIGH)

    if (!data->capturing) {
        if (lvl != 0) {
            return;                        // wait for a falling edge (start of mark)
        }
        data->capturing = true;
        data->count = 0;
        data->last_cycle = now;
        k_timer_start(&data->idle, K_MSEC(IR_RX_IDLE_MS), K_NO_WAIT);
        return;
    }

    uint32_t dus = k_cyc_to_us_floor32(now - data->last_cycle);
    data->last_cycle = now;
    if (dus > 65535) {
        dus = 65535;
    }
    if (data->count < IR_RX_MAX_EDGES) {
        data->durs[data->count++] = (uint16_t)dus;
    }
    k_timer_start(&data->idle, K_MSEC(IR_RX_IDLE_MS), K_NO_WAIT);
}

static int ir_rx_init(const struct device *dev) {
    const struct ir_rx_config *cfg = dev->config;
    struct ir_rx_data *data = dev->data;
    int err;

    data->dev = dev;
    k_work_init(&data->work, ir_rx_work);
    k_timer_init(&data->idle, ir_rx_idle_expiry, NULL);
    k_timer_user_data_set(&data->idle, data);

    if (!gpio_is_ready_dt(&cfg->in)) {
        LOG_ERR("IR RX GPIO not ready");
        return -ENODEV;
    }
    // Pull-up so the line idles high if the receiver output is open-drain or the
    // pin is disconnected (avoids spurious edges); harmless with a push-pull
    // receiver since its active-low drive easily overcomes the internal pull-up.
    err = gpio_pin_configure_dt(&cfg->in, GPIO_INPUT | GPIO_PULL_UP);
    if (err) {
        LOG_ERR("GPIO configure failed: %d", err);
        return err;
    }

    gpio_init_callback(&data->cb, ir_rx_cb, BIT(cfg->in.pin));
    err = gpio_add_callback(cfg->in.port, &data->cb);
    if (err) {
        LOG_ERR("GPIO add callback failed: %d", err);
        return err;
    }

    err = gpio_pin_interrupt_configure_dt(&cfg->in, GPIO_INT_EDGE_BOTH);
    if (err) {
        LOG_ERR("GPIO interrupt configure failed: %d", err);
        return err;
    }

    LOG_INF("IR RX initialized");
    return 0;
}

#define IR_RX_DEFINE(n)                                                          \
    static struct ir_rx_data ir_rx_data_##n;                                     \
    static const struct ir_rx_config ir_rx_config_##n = {                        \
        .in = GPIO_DT_SPEC_INST_GET(n, in_gpios),                                \
    };                                                                           \
    DEVICE_DT_INST_DEFINE(n, ir_rx_init, NULL,                                   \
                          &ir_rx_data_##n, &ir_rx_config_##n,                    \
                          POST_KERNEL, CONFIG_IR_RX_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IR_RX_DEFINE)
