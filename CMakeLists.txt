zephyr_library()
# The IR TX behavior needs ZMK's app headers (drivers/behavior.h, zmk/behavior.h).
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
zephyr_library_sources_ifdef(CONFIG_ZMK_BEHAVIOR_IR_TX src/behavior_ir_tx.c)
zephyr_library_sources_ifdef(CONFIG_IR_RX             src/ir_rx.c)
