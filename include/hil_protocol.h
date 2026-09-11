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

typedef union {
  float single;
  uint8_t bytes[HIL_FLOAT_BYTES];
} custom_float_t;

#endif /* HIL_PROTOCOL_H */
