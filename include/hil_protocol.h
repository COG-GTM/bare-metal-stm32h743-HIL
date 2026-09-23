/*==============================================================================
 * Name        : hil_protocol.h
 * Description : Byte-level UART protocol shared by the firmware, the Simulink
 *               plant model and the host software-HIL harness.
 *
 *   plant  -> controller : 'H' + 4 bytes float32 TAS (little endian) + '\0'
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

/* Receiver state machine. Both directions are framed so that a byte lost or
   added on the line costs a single sample: without a marker to re-anchor on,
   the 4-byte phase would slip permanently and every subsequent float would be
   assembled from parts of two different samples. */
typedef enum {
  HIL_RX_HEADER = 0,  /* discarding bytes until a header is seen */
  HIL_RX_PAYLOAD,     /* collecting the 4 payload bytes */
  HIL_RX_TERMINATOR   /* payload complete, terminator expected */
} hil_rx_stage_t;

typedef struct {
  hil_rx_stage_t stage;
  uint8_t n;
  custom_float_t frame;
} hil_rx_t;

static inline void hil_rx_init(hil_rx_t* rx)
{
  rx->stage = HIL_RX_HEADER;
  rx->n = 0;
}

/**
  * Feed one received byte to the state machine. Returns 1 and writes `value`
  * when a complete, correctly terminated frame has been assembled, 0 otherwise.
  * A frame whose terminator does not match is dropped; the offending byte is
  * itself treated as a header if it is one, so no valid frame is lost.
  */
static inline int hil_rx_push(hil_rx_t* rx, uint8_t byte, custom_float_t* value)
{
  switch (rx->stage) {
    case HIL_RX_PAYLOAD:
      rx->frame.bytes[rx->n++] = byte;
      if (rx->n == HIL_FLOAT_BYTES) { rx->stage = HIL_RX_TERMINATOR; }
      return 0;

    case HIL_RX_TERMINATOR:
      rx->n = 0;
      if (byte == HIL_TERMINATOR) {
        rx->stage = HIL_RX_HEADER;
        *value = rx->frame;
        return 1;
      }
      rx->stage = (byte == HIL_HEADER) ? HIL_RX_PAYLOAD : HIL_RX_HEADER;
      return 0;

    default:
      if (byte == HIL_HEADER) { rx->stage = HIL_RX_PAYLOAD; rx->n = 0; }
      return 0;
  }
}

#endif /* HIL_PROTOCOL_H */
