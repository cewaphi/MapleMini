// vim: expandtab tabstop=2 shiftwidth=2 softtabstop=2

#ifndef _BASE64_H_
#define _BASE64_H_

void base64_encode(const unsigned char *inputData, size_t inputLength, unsigned char *outputData, size_t *outputLength);
void base64_decode(const unsigned char *inputData, size_t inputLength, unsigned char *outputData, size_t *outputLength);

#endif /* _BASE64_H_ */
