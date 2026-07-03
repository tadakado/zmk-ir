# zmk-ir

NEC infrared **transmit** and **receive** for [ZMK](https://zmk.dev):

- **IR TX** — a keymap behavior (`&ir_tx`) that sends a NEC frame, e.g. to drive
  a TV / HDMI switch / AC from a key. 38 kHz PWM carrier, sent a few times for
  reliability, on its own work queue so the waveform stays clean.
- **IR RX** — a driver that reads a 38 kHz demodulated IR receiver (e.g.
  OSRB38C9AA), decodes the NEC32 frame, and logs it — handy for capturing the
  codes your remotes send.

Both are optional; enable whichever you need. Built and tested on the Seeed XIAO
nRF52840, but nothing is board-specific beyond the pins you wire up.

---

## IR TX

### Devicetree

Set up a 38 kHz PWM feeding your IR LED, alias it as `irpwm`, and declare the
behavior. Example for nRF52 (`&pwm0`, output on P1.01):

```dts
&pinctrl {
    ir_pwm0_default: ir_pwm0_default {
        group1 { psels = <NRF_PSEL(PWM_OUT0, 1, 1)>; };
    };
};

&pwm0 {
    status = "okay";
    pinctrl-0 = <&ir_pwm0_default>;
    pinctrl-names = "default";
};

/ {
    ir_pwm: ir_pwm {
        compatible = "pwm-leds";
        ir_pwm_led0: ir_pwm_led0 {
            pwms = <&pwm0 0 26316 PWM_POLARITY_NORMAL>;  /* 38 kHz = 26316 ns */
        };
    };

    aliases { irpwm = &ir_pwm_led0; };   /* the behavior finds the PWM here */

    behaviors {
        ir_tx: ir_tx {
            compatible = "zmk,behavior-ir-tx";
            #binding-cells = <2>;         /* param1 = customer code, param2 = data */
        };
    };
};
```

Enable PWM in your `.conf`:

```ini
CONFIG_PWM=y
CONFIG_NRFX_PWM0=y
```

If the `irpwm` alias is absent the behavior compiles to a no-op (logs a warning),
so it's safe on halves/boards without an IR LED.

### Keymap

`&ir_tx <customer-code> <data-code>` — both 16-bit. For example, an Anker HDMI
switch "input 1":

```dts
&ir_tx 0x01FE 0xE01F
```

Define your own codes (capture them with IR RX below) and wrap them in macros if
you like.

### Configuration

| Kconfig | Default | Description |
|---|---|---|
| `CONFIG_ZMK_BEHAVIOR_IR_TX` | `y` when the node exists | Enable the TX behavior. |
| `CONFIG_IR_TX_REPEAT_COUNT` | `3` | Full NEC frames sent per press (idempotent codes → redundancy against a missed frame). |
| `CONFIG_IR_TX_THREAD_STACK_SIZE` | `1024` | Dedicated work-queue stack. |
| `CONFIG_IR_TX_THREAD_PRIORITY` | `-1` | Cooperative priority (the bit-bang waveform must not be preempted). |

---

## IR RX

### Devicetree

Point it at the demodulated output of a 38 kHz receiver (idle high, active low):

```dts
/ {
    ir_rx: ir_rx {
        compatible = "zmk,ir-rx";
        in-gpios = <&gpio0 19 GPIO_ACTIVE_HIGH>;   /* your receiver's OUT pin */
    };
};
```

### Configuration

```ini
CONFIG_IR_RX=y
CONFIG_IR_RX_LOG_LEVEL_DBG=y   # print decoded frames
```

Decoded frames appear in the log:

```
IR NEC32 customer=0x01fe data=0xe01f (raw=0x01fee01f)
IR NEC repeat
IR: non-NEC frame (...)
```

---

## Installation

Add the module to your `config/west.yml`:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: tadakado
      url-base: https://github.com/tadakado
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-ir
      remote: tadakado
      revision: main
  self:
    path: config
```

Each half is gated on its devicetree node, so it's safe to add the project
unconditionally — TX builds only where a `zmk,behavior-ir-tx` node exists, RX
only where a `zmk,ir-rx` node exists.

## License

MIT — see [LICENSE](LICENSE).
