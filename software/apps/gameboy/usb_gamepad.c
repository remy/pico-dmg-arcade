#include <stdio.h>
#include <stdlib.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "./usb_gamepad.h"

void toggle_led(void);

// in usb_gamepad.h
gamepad_report_t controller_state;

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
  // if (board_millis() - poll_ms < interval_ms)
  //   return; // not enough time
  poll_ms += interval_ms;



  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    printf("Error: cannot request to receive report\r\n");
    return;
  }

  // Checking if there are changes in report since the method was last called
  bool match = true;
  for (uint8_t i = 2; i < 4; i++) {
    if (prev_report[i] != report[i]) {
      // printf("change on %d (%02X <> %02X)\r\n", i, prev_report[i],
      //  report[i]);
      match = false;
    }
    prev_report[i] = report[i];
  }

  if (match) {
    // printf("no change in HID report\r\n");
    controller_state.stale = 1;
    return;
  }


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

  memcpy(&controller_state, report, sizeof(gamepad_report_t));

  // const char *dpad_str[] = {"none", "N", "S", "NS?", "W", "NW", "SW", "?", "E", "NE", "SE", "??", "WE", "NWE", "SWE", "NWSE"};

  // printf("STA: %d, SEL: %d, A: %d, B: %d, dpad: %d, %s\n", state.STA, state.SEL, state.A, state.B, state.dpad, dpad_str[state.dpad]);
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
