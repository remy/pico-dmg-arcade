#include "./gb.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
// #include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "common_dvi_pin_configs.h"
#include "dvi.h"
#include "dvi_serialiser.h"
#include "gfx.h"
#include "sprite.h"

// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

#define LED_PIN 25

#define PIN_BASE 6
#define PIN_COUNT 4
#define SM 0 // State machine index

#define TO_RGB565(rgb888)                                                      \
  (((rgb888 >> 19) & 0x1F) << 11) | (((rgb888 >> 10) & 0x3F) << 5) |           \
      ((rgb888 >> 3) & 0x1F)

uint16_t pal[] = {
    // Switch DMG
    TO_RGB565(0xC0D635),
    TO_RGB565(0x869625),
    TO_RGB565(0x4D5615),
    TO_RGB565(0x131505),

    // SameBoy DMG
    TO_RGB565(0xC6DE8C),
    TO_RGB565(0x84A563),
    TO_RGB565(0x39613A),
    TO_RGB565(0x081711),
};

uint8_t pal_offset = 1 * 4;

uint16_t g_framebuf[FRAME_WIDTH * FRAME_HEIGHT];
struct dvi_inst dvi0;
PIO pio = pio1;

void __not_in_flash("core1_main") core1_main() {
  dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
  dvi_start(&dvi0);
  dvi_scanbuf_main_16bpp(&dvi0);
  __builtin_unreachable();
}

void core1_scanline_callback() {
  // Discard any scanline pointers passed back
  uint16_t *bufptr;
  while (queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
    ;
  // // Note first two scanlines are pushed before DVI start
  static uint scanline = 2;
  bufptr = &g_framebuf[FRAME_WIDTH * scanline];
  queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
  scanline = (scanline + 1) % FRAME_HEIGHT;
}

void init_lcd_capture(PIO pio, uint sm, uint offset, uint pin_base) {
  pio_gpio_init(pio, pin_base + 0); // d0
  pio_gpio_init(pio, pin_base + 1); // d1
  pio_gpio_init(pio, pin_base + 2); // vsync
  pio_gpio_init(pio, pin_base + 3); // hsync
  pio_gpio_init(pio, pin_base + 4); // clock

  // Adjusted to the new program name
  pio_sm_config c = gb_program_get_default_config(offset);
  // when pin 3 (hsync) goes high, allow `jmp pin, <label>`
  sm_config_set_jmp_pin(&c, pin_base + 3);
  // Set base pin for 'in' operations
  sm_config_set_in_pins(&c, pin_base);
  // Shift bits into ISR, autopush
  sm_config_set_in_shift(&c, true, true, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);
}

int main() {
  vreg_set_voltage(VREG_VSEL);
  sleep_ms(10);

  set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

  stdio_init_all();
  setup_default_uart();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  // gfx_init();

  dvi0.timing = &DVI_TIMING;
  dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
  dvi0.scanline_callback = core1_scanline_callback;
  dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

  // kick off the render core
  sprite_fill16(g_framebuf, 0xf099, FRAME_WIDTH * FRAME_HEIGHT);
  uint16_t *bufptr = g_framebuf;
  queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
  bufptr += FRAME_WIDTH;
  queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);

  multicore_launch_core1(core1_main);

  uint offset = pio_add_program(pio, &gb_program);
  init_lcd_capture(pio, SM, offset, PIN_BASE);

  // uint heartbeat = 0;
  uint i = 0;
  uint8_t ddvh = 0;
  uint32_t data = 0;

  uint8_t vsync = 0;
  uint8_t hsync = 0;
  uint buffer_ptr = 0;
  // uint ptr = 0;

  uint scan_x = 0;
  uint scan_y = 0;

  // uint8_t vsync_count = 0;

  // clear the g_framebuf with a gradient
  for (int y = 0; y < FRAME_HEIGHT; y++) {
    for (int x = 0; x < FRAME_WIDTH; x++) {
      g_framebuf[y * FRAME_WIDTH + x] = 0x0;
    }
  }

  while (true) {

    if (!pio_sm_is_rx_fifo_empty(pio, SM)) {
      data = pio_sm_get(pio, SM);

      for (i = 0; i < 8; i++) {

        // 4bits: d0, d1, hsync, vsync
        ddvh = (data >> (i * 4)) & 0b1111;

        vsync = (ddvh >> 2) & 0b1;
        hsync = (ddvh >> 3);
        ddvh = ddvh & 0b11;

        if (vsync == 1) {
          scan_y = 0;
        }

        if (hsync == 1) {
          scan_x = 0;
          if (vsync == 0) {
            scan_y = (scan_y + 1) % 144;
          }
        } else {
          scan_x = (scan_x + 1) % 160;
        }

        // add 80 to x and then add 48 to y
        // when display is 160x144, get the correct buffer_ptr for scan_x and
        // scan_y:
        buffer_ptr = (scan_y + 48) * FRAME_WIDTH + (scan_x + 80);

        g_framebuf[buffer_ptr] = pal[pal_offset + ddvh];
      }
    }
  }
  __builtin_unreachable();
}
