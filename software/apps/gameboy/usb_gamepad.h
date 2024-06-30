// typedef struct {
//   uint8_t stale : 8;
//   uint8_t : 8; // Ignored byte

//   uint8_t dpad : 4; // upper 4 bits
//   uint8_t STA : 1;  // lower 4 bits
//   uint8_t SEL : 1;
//   uint8_t : 2;

//   uint8_t : 4; // ignore upper nibble
//   uint8_t A : 1;
//   uint8_t B : 1;
//   uint8_t X : 1; // we don't use X/Y but … meh
//   uint8_t Y : 1;

// } gamepad_report_t;

typedef struct {
  uint8_t X: 8; // Ignored byte
  uint8_t Y: 8; // Ignored byte
  uint8_t stale : 8;
  uint8_t : 8; // Ignored byte
  uint8_t : 8; // Ignored byte

  uint8_t dpad : 4; // upper 4 bits
  uint8_t B : 1;
  uint8_t A : 1;
  uint8_t : 2;

  uint8_t : 4; // upper 4 bits
  uint8_t SEL : 1;
  uint8_t STA : 1;  // lower 4 bits

} gamepad_report_t;
