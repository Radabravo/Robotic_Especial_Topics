#include "Arduino.h"
#include "Decode.h"


void GetDecodeData(uint8_t *dataReceived,uint8_t *dataDecoded,int sizeDecoded,uint8_t Header1,uint8_t Header2, uint8_t Tail1,uint8_t Tail2)
{
    if(CheckHeader(dataReceived, sizeDecoded+4, Header1, Header2))
    {
        
        if(CheckTail(dataReceived, sizeDecoded+4, Tail1, Tail2))
        {
            
            int j = 0;
            for (int i = 0; i < sizeDecoded+4; i++)
            {   
                if (i>1 && i<sizeDecoded+2)
                {
                    dataDecoded[j] = dataReceived[i];
                    j++;
                    
                }
                
            }
            
        }
    }
}
bool CheckHeader(uint8_t *dataReceived, int sizeReceived,uint8_t Header1,uint8_t Header2 )
{
    bool check=(dataReceived[0]==Header1 && dataReceived[1]==Header2);
    return check; 
}
bool CheckTail(uint8_t *dataReceived, int sizeReceived,uint8_t Tail1,uint8_t Tail2){
    bool check=(dataReceived[sizeReceived-2]==Tail1 && dataReceived[sizeReceived-1]==Tail2);
    return check; 
}