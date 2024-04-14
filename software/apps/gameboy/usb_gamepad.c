#include <stdio.h>
#include <stdlib.h>

#include "bsp/board_api.h"
#include "tusb.h"

void toggle_led(void);

typedef struct {
  uint8_t reportId;
  uint8_t id;
  uint8_t : 8; // unknown

  uint8_t BTN_Y : 1;
  uint8_t BTN_X : 1;
  uint8_t BTN_B : 1;
  uint8_t BTN_A : 1;

  uint8_t : 4; // skip next 4 bits / nibble (turbo and 8bitdo button)

  uint8_t BTN_Select : 1;
  uint8_t BTN_Start : 1;

  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 4;
  uint8_t X : 4;
  uint8_t Y : 4;

} my_gamepad_report_t;

// check if different than 2
bool diff_than_2(uint8_t x, uint8_t y) { return (x - y > 2) || (y - x > 2); }

void tuh_mount_cb(uint8_t dev_addr) {
  // application set-up
  printf("tuh_mount_cb / A device with address %d is mounted\r\n", dev_addr);
}

void tuh_umount_cb(uint8_t dev_addr) {
  // application tear-down
  printf("tuh_umount_cb / A device with address %d is unmounted \r\n",
         dev_addr);
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                uint8_t const *report, uint16_t len) {

  static uint8_t prev_report[64] = {0};

  static uint32_t poll_ms = 0;
  const uint32_t interval_ms = 16;

  // Blink every interval ms
  if (board_millis() - poll_ms < interval_ms)
    return; // not enough time
  poll_ms += interval_ms;



  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    printf("Error: cannot request to receive report\r\n");
    return;
  }

  // Checking if there are changes in report since the method was last called
  bool match = true;
  for (uint8_t i = 2; i < len; i++) {
    if (prev_report[i] != report[i]) {
      // printf("change on %d (%02X <> %02X)\r\n", i, prev_report[i],
      //  report[i]);
      match = false;
    }
    prev_report[i] = report[i];
  }

  if (match) {
    // printf("no change in HID report\r\n");
    return;
  }

  // const char *dpad_str[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW",
  // "none"};

  // print a hexdump of the desc_report to uart
  // printf("HID Report Descriptor on callback (%d):\r\n", len);
  // for (uint32_t i = 0; i < len; i++) {
  //   printf("%02X ", report[i]);
  //   if ((i + 1) % 16 == 0) {
  //     printf("\r\n");
  //   }
  // }
  // printf("\r\n");

  toggle_led();

  my_gamepad_report_t gamepad_report;
  memcpy(&gamepad_report, report, sizeof(gamepad_report));

  // printf("A: %u, B: %u ", gamepad_report.BTN_A, gamepad_report.BTN_B);
  // printf("X: %u, Y: %u ", gamepad_report.BTN_X, gamepad_report.BTN_Y);
  // printf("Select: %u, Start: %u ", gamepad_report.BTN_Select,
  //        gamepad_report.BTN_Start);
  // printf("X: %u, Y: %u ", gamepad_report.X, gamepad_report.Y);
  // printf("\r\n");
  // }
}

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                      uint8_t const *desc_report, uint16_t desc_len) {
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  printf("tuh_hid_mount_cb / HID device address = %d, instance = %d is "
         "mounted\r\n",
         dev_addr, instance);
  printf("VID = %04x, PID = %04x\r\n", vid, pid);

  // print a hexdump of the desc_report to uart
  printf("HID Mount Report Descriptor (%d):\r\n", desc_len);
  for (uint32_t i = 0; i < desc_len; i++) {
    printf("%02X ", desc_report[i]);
    if ((i + 1) % 16 == 0) {
      printf("\r\n");
    }
  }
  printf("\r\n");

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    printf("Error: cannot request to receive report\r\n");
    return;
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr,
         instance);
}

void hid_app_task(void) {
  // nothing to do
}

void cdc_task(void) {}


void toggle_led(void) {
  static bool led_state = false;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
