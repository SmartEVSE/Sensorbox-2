#include <Arduino.h>
//
// Convert a one character hex string to an uint8
//
uint8_t HexDec1(uint8_t c1) {
  if (c1>='A') c1-=7;
  c1-='0';
  return c1;
}


//
// Convert a two character hex string to an uint8
//
uint8_t HexDec2(uint8_t c1, uint8_t c2) {
  return HexDec1(c1)*16 + HexDec1(c2);
}


//
// Convert a four character hex string to an uint16
//
uint16_t HexDec4(uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4) {
  return HexDec2(c1,c2)*256 + HexDec2(c3,c4);
}
