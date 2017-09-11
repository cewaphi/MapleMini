// vim: expandtab tabstop=2 shiftwidth=2 softtabstop=2

#include <stdint.h>
#include <stdlib.h>
#include "base64.h"

static const unsigned char encodingTable[] = {
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
  'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
  'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
  'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
  'w', 'x', 'y', 'z', '0', '1', '2', '3',
  '4', '5', '6', '7', '8', '9', '+', '/'};

static const unsigned char decodingTable[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 62, 0, 0, 0, 63, 52, 53, 54, 55,
  56, 57, 58, 59, 60, 61, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
  12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 0, 0, 0, 0, 0, 0, 26, 27,
  28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
  49, 50, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static unsigned int modTable[] = {0, 2, 1};


void base64_encode(const unsigned char *inputData, size_t inputLength, unsigned char *outputData, size_t *outputLength) {

  unsigned int i, j;

  *outputLength = 4 * ((inputLength + 2) / 3);

  for (i = 0, j = 0; i < inputLength;) {

    uint32_t octet_a = i < inputLength ? (unsigned char)inputData[i++] : 0;
    uint32_t octet_b = i < inputLength ? (unsigned char)inputData[i++] : 0;
    uint32_t octet_c = i < inputLength ? (unsigned char)inputData[i++] : 0;

    uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

    outputData[j++] = encodingTable[(triple >> 3 * 6) & 0x3F];
    outputData[j++] = encodingTable[(triple >> 2 * 6) & 0x3F];
    outputData[j++] = encodingTable[(triple >> 1 * 6) & 0x3F];
    outputData[j++] = encodingTable[(triple >> 0 * 6) & 0x3F];
  }

  for (i = 0; i < modTable[inputLength % 3]; i++)
    outputData[*outputLength - 1 - i] = '=';
}


void base64_decode(const unsigned char *inputData, size_t inputLength, unsigned char *outputData, size_t *outputLength) {

  unsigned int i, j;

  if (inputLength % 4 != 0) return;

  *outputLength = inputLength / 4 * 3;
  if (inputData[inputLength - 1] == '=') (*outputLength)--;
  if (inputData[inputLength - 2] == '=') (*outputLength)--;


  for (i = 0, j = 0; i < inputLength;) {

    uint32_t sextet_a = inputData[i] == '=' ? 0 & i++ : decodingTable[inputData[i++]];
    uint32_t sextet_b = inputData[i] == '=' ? 0 & i++ : decodingTable[inputData[i++]];
    uint32_t sextet_c = inputData[i] == '=' ? 0 & i++ : decodingTable[inputData[i++]];
    uint32_t sextet_d = inputData[i] == '=' ? 0 & i++ : decodingTable[inputData[i++]];

    uint32_t triple = (sextet_a << 3 * 6)
      + (sextet_b << 2 * 6)
      + (sextet_c << 1 * 6)
      + (sextet_d << 0 * 6);

    if (j < *outputLength) outputData[j++] = (triple >> 2 * 8) & 0xFF;
    if (j < *outputLength) outputData[j++] = (triple >> 1 * 8) & 0xFF;
    if (j < *outputLength) outputData[j++] = (triple >> 0 * 8) & 0xFF;
  }
}
