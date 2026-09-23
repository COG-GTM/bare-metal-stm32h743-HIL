/*==============================================================================
 * Name        : hil_protocol.h
 * Description : Byte-level UART protocol shared by the firmware, the Simulink
 *               plant model and the host software-HIL harness.
 *
 *   plant  -> controller : 4 bytes  = float32 TAS (little endian)
 *   controller -> plant  : 'H' + 4 bytes float32 thrust + CRC-8 + '\0'
 *
 * Wire format is defined independently of the host byte order: floats are
 * serialised with explicit shifts, least significant byte first, so the same
 * bytes go on the wire on a little- or big-endian build. IEEE-754 binary32 is
 * still assumed for the float representation itself (static-asserted below).
 *
 * The CRC-8 byte covers the header and the 4 payload bytes (CRC-8/ATM:
 * polynomial 0x07, init 0x00, no reflection, no final xor). A receiver must
 * drop any frame whose CRC or terminator does not match: without it a single
 * corrupted byte on the UART link is accepted as a valid (wrong) float.
===============================================================================*/
#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#define HIL_HEADER      'H'
#define HIL_TERMINATOR  '\0'
#define HIL_FLOAT_BYTES 4
#define HIL_CRC_BYTES   1
#define HIL_FRAME_BYTES (1 + HIL_FLOAT_BYTES + HIL_CRC_BYTES + 1)

/* Offsets of the fields inside a controller -> plant frame. */
#define HIL_PAYLOAD_OFFSET 1
#define HIL_CRC_OFFSET     (HIL_PAYLOAD_OFFSET + HIL_FLOAT_BYTES)

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(float) == HIL_FLOAT_BYTES, "wire format assumes 32-bit float");
#endif

/* CRC-8/ATM over len bytes. */
static inline uint8_t hil_crc8(const uint8_t *data, int len)
{
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (uint8_t)((crc & 0x80u) ? ((uint8_t)(crc << 1) ^ 0x07u) : (uint8_t)(crc << 1));
    }
  }
  return crc;
}

/* float32 -> 4 little-endian wire bytes, independent of host byte order. */
static inline void hil_float_encode(float value, uint8_t out[HIL_FLOAT_BYTES])
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof bits);
  for (int i = 0; i < HIL_FLOAT_BYTES; i++) {
    out[i] = (uint8_t)((bits >> (8 * i)) & 0xFFu);
  }
}

/* 4 little-endian wire bytes -> float32, independent of host byte order. */
static inline float hil_float_decode(const uint8_t in[HIL_FLOAT_BYTES])
{
  uint32_t bits = 0;
  float value;
  for (int i = 0; i < HIL_FLOAT_BYTES; i++) {
    bits |= (uint32_t)in[i] << (8 * i);
  }
  memcpy(&value, &bits, sizeof value);
  return value;
}

/* Serialise one controller -> plant frame: header, float32, CRC-8, terminator. */
static inline void hil_frame_encode(float value, uint8_t out[HIL_FRAME_BYTES])
{
  out[0] = HIL_HEADER;
  hil_float_encode(value, &out[HIL_PAYLOAD_OFFSET]);
  out[HIL_CRC_OFFSET] = hil_crc8(out, HIL_CRC_OFFSET);
  out[HIL_FRAME_BYTES - 1] = HIL_TERMINATOR;
}

/* Validate a frame and extract its payload. Returns 1 on success, 0 when the
   header, CRC or terminator does not match (the frame must then be dropped). */
static inline int hil_frame_decode(const uint8_t in[HIL_FRAME_BYTES], float *value)
{
  if (in[0] != HIL_HEADER || in[HIL_FRAME_BYTES - 1] != HIL_TERMINATOR) {
    return 0;
  }
  if (in[HIL_CRC_OFFSET] != hil_crc8(in, HIL_CRC_OFFSET)) {
    return 0;
  }
  *value = hil_float_decode(&in[HIL_PAYLOAD_OFFSET]);
  return 1;
}

#endif /* HIL_PROTOCOL_H */
