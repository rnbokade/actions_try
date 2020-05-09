#include "/src/fec.h"
#include "/src/rs8.h"
uint8_t packet[200];
uint8_t data[100];
void setup()
{
	for(int i=4;i<68;i++)
    {data[i]=0x01+i;}
    rs_encode(&data[68],&data[4],64);
    uint32_t syncword=0x1ACFFC1D;
    memcpy(&data[0],&syncword,sizeof(uint32_t));
    conv_encoder_1_2_7(packet,data,100);
    rs_decode(&data[4],64);
}

void loop()
{
	
}
