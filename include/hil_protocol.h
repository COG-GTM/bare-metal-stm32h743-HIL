/*==============================================================================
 * Name        : hil_protocol.h
 * Description : Byte-level UART protocol shared by the firmware, the Simulink
 *               plant model and the host software-HIL harness.
 *
 *   plant  -> controller : 4 bytes  = float32 TAS (little endian)
 *   controller -> plant  : 'H' + 4 bytes float32 thrust + '\0'
===============================================================================*/
#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#include <stdint.h>

#define HIL_HEADER      'H'
#define HIL_TERMINATOR  '\0'
#define HIL_FLOAT_BYTES 4
#define HIL_FRAME_BYTES (1 + HIL_FLOAT_BYTES + 1)

typedef union {
  float single;
  uint8_t bytes[HIL_FLOAT_BYTES];
} custom_float_t;

/* Serialise one controller -> plant frame: HIL_HEADER, float32, HIL_TERMINATOR. */
static inline void hil_frame_encode(float value, uint8_t out[HIL_FRAME_BYTES])
{
  custom_float_t f;
  f.single = value;
  out[0] = HIL_HEADER;
  for (int i = 0; i < HIL_FLOAT_BYTES; i++) {
    out[1 + i] = f.bytes[i];
  }
  out[HIL_FRAME_BYTES - 1] = HIL_TERMINATOR;
}

#endif /* HIL_PROTOCOL_H */
