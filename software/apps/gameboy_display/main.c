#include "./gb.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/pll.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

// #define DEBUG 0

#ifdef DEBUG
#include "pico/stdlib.h"
#include "hardware/adc.h"
#endif

#include "common_dvi_pin_configs.h"
#include "dvi.h"
#include "dvi_serialiser.h"
#include "sprite.h"

// DVDD 1.2V (1.1V seems ok too)
// #define FRAME_WIDTH 800
// #define FRAME_HEIGHT 150
#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 180
#define VREG_VSEL VREG_VOLTAGE_1_30 // needs to be 1v3 for 800x60
#define DVI_TIMING dvi_timing_1280x720p_30hz

#define LED_PIN 25

// LCD pins
#define PIN_BASE 2
#define PIN_COUNT 4

// Order of vsync, hsync and clock are important
// D0    15
// D1    16
// VSYNC 12 (60Hz)
// HSYNC 17 (9.2kHz)
// CLOCK 14 (4Mhz)

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
    TO_RGB565(0x39613A),
    TO_RGB565(0x84A563),
    TO_RGB565(0x081711),

    // "softbreeze"
    // https://www.deviantart.com/advancedfan2020/art/Game-Boy-Palette-Set-Soft-Breeze-1013214618
    TO_RGB565(0xFFD895),
    TO_RGB565(0xFDBF68),
    TO_RGB565(0xFB8A9C),
    TO_RGB565(0x776DBD),

    // switch pocket
    TO_RGB565(0xb5c69c),
    TO_RGB565(0x8d9c7b),
    TO_RGB565(0x637251),
    TO_RGB565(0x303820),

    // famicon
    TO_RGB565(0xefecda),
    TO_RGB565(0xd9be72),
    TO_RGB565(0xa32135),
    TO_RGB565(0x231916),
};

uint8_t pal_offset = 1;

#define DMG_WIDTH 160
#define DMG_HEIGHT 144

uint16_t g_framebuf[DMG_WIDTH * DMG_HEIGHT];
uint16_t scanline_buffer[FRAME_WIDTH];
uint16_t bg_color = 0;
struct dvi_inst dvi0;
static PIO pio = pio1;

static uint lcd_state_machine = 0;

static uint pio_out_value = 0;

void __not_in_flash("core1_main") core1_main() {
  dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
  dvi_start(&dvi0);
  dvi_scanbuf_main_16bpp(&dvi0);
  __builtin_unreachable();
}

void core1_scanline_callback() {
  static uint scanline = 2;
  uint idx = 0;
  uint border_horz = ((FRAME_WIDTH/2)-(DMG_WIDTH*2))/2;
  uint border_vert = (FRAME_HEIGHT-DMG_HEIGHT)/2;
  static uint frame_idx = 0;
  static uint dmg_line_idx = 0;

  if (scanline < border_vert || scanline >= FRAME_HEIGHT - border_vert) {
    for (uint i = 0; i < FRAME_WIDTH; i++) {
      scanline_buffer[idx++] = bg_color;
    }

    dmg_line_idx = 0;
  } else {
    dmg_line_idx = scanline - border_vert;

    for (uint i = 0; i < border_horz; i++)
      scanline_buffer[idx++] = bg_color;

    for (uint i = 0; i < DMG_WIDTH; i++) {
      frame_idx = dmg_line_idx * DMG_WIDTH + i;
      scanline_buffer[idx++] = pal[(pal_offset * 4) + g_framebuf[frame_idx]];
      scanline_buffer[idx++] = pal[(pal_offset * 4) + g_framebuf[frame_idx]];
    }

    for (uint i = 0; i < border_horz; i++)
      scanline_buffer[idx++] = bg_color;
  }

  const uint32_t *bufptr = (uint32_t *)scanline_buffer;

  queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
  while (queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
    ;

  scanline = (scanline + 1) % FRAME_HEIGHT;
}

void init_overclock() {
  // Configure the PLL for a VCO of 540 MHz and a system clock of 270 MHz
  pll_init(pll_sys, 1, 1600, 6, 2);

  clock_configure(clk_sys,
                  0,              // No glitchless mux
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                  200 * MHZ,      // Target frequency 200MHz
                  200 * MHZ);     // Match the target frequency
}


void init_lcd_capture() {
  static uint pin_base = PIN_BASE;
  uint offset = pio_add_program(pio, &gb_program);
  lcd_state_machine = pio_claim_unused_sm(pio, true);

  uint sm = lcd_state_machine;

  pio_gpio_init(pio, pin_base + 0); // d0 - green
  pio_gpio_init(pio, pin_base + 1); // d1 - yellow
  pio_gpio_init(pio, pin_base + 2); // vsync - blue
  pio_gpio_init(pio, pin_base + 3); // hsync - red
  pio_gpio_init(pio, pin_base + 4); // clock - white

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




int main(void) {

  #ifdef DEBUG
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);
  #endif

  vreg_set_voltage(VREG_VSEL);
  sleep_ms(10);

  set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

  stdio_init_all();
  setup_default_uart();

  // overclocking doesn't work for me.
  // init_overclock();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  dvi0.timing = &DVI_TIMING;
  dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
  dvi0.scanline_callback = core1_scanline_callback;
  dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

  // kick off the render core
  sprite_fill16(scanline_buffer, 0xf099, FRAME_WIDTH);
  uint16_t *bufptr = scanline_buffer;
  queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
  bufptr += FRAME_WIDTH;
  queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);

  multicore_launch_core1(core1_main);

  init_lcd_capture();

  sleep_ms(200);

  // board_init();


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

  // // clear the g_framebuf with a gradient
  // for (int y = 0; y < DMG_HEIGHT; y++) {
  //   for (int x = 0; x < DMG_WIDTH; x++) {
  //     g_framebuf[y * DMG_WIDTH + x] = 0x0;
  //   }
  // }

#ifdef DEBUG
  uint heartbeat = 0;
#endif

  while (true) {

    #ifdef DEBUG
    if (++heartbeat >= 65530) {
			heartbeat = 0;
      uint16_t result = adc_read();
      const float conversion_factor = 3.3f / (1 << 12);
      float voltage = result * conversion_factor;
      float temperature = 27 - (voltage - 0.706) / 0.001721;
      printf("CPU temp: %.2f C\n", temperature);
      printf("pio_report: %x\n", pio_out_value);
		}
    #endif

    if (!pio_sm_is_rx_fifo_empty(pio, SM)) {
      data = pio_sm_get(pio, SM);

      for (i = 0; i < 8; i++) {

        // 4bits: d0, d1, vsync, hsync
        ddvh = (data >> (i * 4)) & 0b1111;

        vsync = (ddvh >> 2) & 0b1;
        hsync = (ddvh >> 3);
        ddvh = ddvh & 0b11;

        if (vsync == 1) {
          scan_y = 0;
          // printf("vsync\n");
        }

        if (hsync == 1) {
          scan_x = 0;
          if (vsync == 0) {
            scan_y = (scan_y + 1) % DMG_HEIGHT;
          }
        } else {
          scan_x = (scan_x + 1) % DMG_WIDTH;
        }

        // add 80 to x and then add 48 to y
        // when display is 160x144, get the correct buffer_ptr for scan_x and
        // scan_y:
        buffer_ptr = scan_y * DMG_WIDTH + scan_x;

        g_framebuf[buffer_ptr] = ddvh;
      }
    } else {
      // sleep_ms(10);
    }
  }
  __builtin_unreachable();
}
