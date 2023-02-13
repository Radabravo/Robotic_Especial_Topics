#include "Arduino.h"

void GetDecodeData(uint8_t *dataReceived,uint8_t *dataDecoded, int sizeDecoded, uint8_t Header1,uint8_t Header2, uint8_t Tail1,uint8_t Tail2 );
bool CheckHeader(uint8_t *dataReceived, int sizeReceived,uint8_t Header1,uint8_t Header2 );
bool CheckTail(uint8_t *dataReceived, int sizeReceived,uint8_t Tail1,uint8_t Tail2);
