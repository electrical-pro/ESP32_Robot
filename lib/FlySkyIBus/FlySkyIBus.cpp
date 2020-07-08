/*
 * Simple interface to the Fly Sky IBus RC system.
 * Obtained from https://gitlab.com/timwilkinson/FlySkyIBus
 * Slightly modified for a bit more robustness
 */

#include <Arduino.h>
#include "FlySkyIBus.h"
#include <Streaming.h>

FlySkyIBus IBus;

void FlySkyIBus::begin(HardwareSerial& serial)
{
  serial.begin(115200);
  begin((Stream&)serial);
}

void FlySkyIBus::begin(Stream& stream)
{
  this->stream = &stream;
  this->state = DISCARD;
  this->last = millis();
  this->ptr = 0;
  this->len = 0;
  this->chksum = 0;
  this->lchksum = 0;
  this->active = 0;
}

void FlySkyIBus::loop(void)
{
  while (stream->available() > 0)
  {
    active = 1;
    uint32_t now = millis();
    if (state==DISCARD && now - last >= PROTOCOL_TIMEGAP)
    {
      state = GET_LENGTH;
    }
    last = now;

    uint8_t v = stream->read();
    switch (state)
    {
      case GET_LENGTH:
        if (v == PROTOCOL_LENGTH)
        {
          ptr = 0;
          len = v - PROTOCOL_OVERHEAD;
          chksum = 0xFFFF - v;
          state = GET_DATA;
          // tStart = now;
        }
        else
        {
          state = GET_LENGTH;
          while(Serial.available() > 0) {
            Serial.read();
          }
        }
        break;

      case GET_DATA:
        buffer[ptr++] = v;
        chksum -= v;
        if (ptr == len)
        {
          state = GET_CHKSUML;
        }
        break;

      case GET_CHKSUML:
        lchksum = v;
        state = GET_CHKSUMH;
        break;

      case GET_CHKSUMH:
        // Validate checksum
        if (chksum == (v << 8) + lchksum)
        {
          // Execute command - we only know command 0x40
          switch (buffer[0])
          {
            case PROTOCOL_COMMAND40:
              // Valid - extract channel data
              for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)
              {
                channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
              }
              break;

            default:
              break;
          }
          state = GET_LENGTH;
        } else {
        state = DISCARD;
      }
        break;

      case DISCARD:
      default:
        break;
    }
  }

  if (active) {
    uint32_t now = millis();
    if (now-last > 100) {
      active = 0;
    }
  }
}

uint16_t FlySkyIBus::readChannel(uint8_t channelNr)
{
  if (channelNr < PROTOCOL_CHANNELS)
  {
    return channel[channelNr];
  }
  else
  {
    return 0;
  }
}

boolean FlySkyIBus::isActive(void) {
  return active;
}
