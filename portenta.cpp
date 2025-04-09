// Based on the k210 and esp variants of this library
// As we are on MBed for the Portenta we can utilise the timing libraries from that platform
#if ( ( defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_PORTENTA_H7_M4) ) && defined(ARDUINO_ARCH_MBED) )

#include <mbed.h>
#include <Arduino.h>

// Configuration for timing values
#define CYCLES_800_T0H (125)      // 0.3us (minus 0.15us for IO)
#define CYCLES_800_T1H (500)      // 0.7us
#define CYCLES_800 (800)          // 1.25us per bit
#define CYCLES_800_RESET (6000)   // 60ms reset pulse
#define CYCLES_400_T0H (500)      // 0.5uS
#define CYCLES_400_T1H (1200)     // 1.2us
#define CYCLES_400 (2500)         // 2.5us per bit
#define CYCLES_400_RESET (12000)  // 120ms per bit

void portentaShow(
  int16_t pin, uint8_t *pixels, uint16_t numBytes, boolean is800KHz) {

  // Pin Initialization using mbed DigitalOut
  // Note: Ensure digitalPinToPinName(pin) correctly maps to the mbed PinName.
  mbed::DigitalOut ledPin(digitalPinToPinName(pin));

  // Define timing variables
  uint32_t time0, time1, period, reset;

  // Set timing values based on frequency
  if (is800KHz) {
    time0 = CYCLES_800_T0H;
    time1 = CYCLES_800_T1H;
    period = CYCLES_800;
    reset = CYCLES_800_RESET;
  } else {
    time0 = CYCLES_400_T0H;
    time1 = CYCLES_400_T1H;
    period = CYCLES_400;
    reset = CYCLES_400_RESET;
  }

  // Send reset signal
  wait_ns(reset);

  // Transmit data
  uint8_t *p = pixels;
  uint8_t *end = p + numBytes;
  while (p < end) {
    uint8_t pix = *p++;
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
      // Determine high time based on bit value
      uint32_t th = (pix & mask) ? time1 : time0;

      // Set pin high, wait, set pin low, wait
      ledPin = 1;
      wait_ns(th);
      ledPin = 0;
      wait_ns(period - th);
    }
  }
  wait_ns(period); // Extra wait at the end to ensure last bit is sent
}

#endif  // ARDUINO_PORTENTA_H7_M7
