// * Function List:
// *    1. void MeSerial::begin(long baudrate)
// *    2. size_t MeSerial::write(uint8_t byte)
// *    3. int16_t MeSerial::read(void)
// *    4. int16_t MeSerial::available(void)
// *    5. int16_t MeSerial::poll(void)
// *    6. void MeSerial::print(char *fmt,...)
// *    7. void MeSerial::println(char *fmt,...)

#include "MeOrion.h"
#include <SoftwareSerial.h>

char table[128] = {0};
SoftwareSerial softuart(13,12);

char iicData[20];//IIC传送的字符数组
float recData[3]; 

void setup() 
{
   Serial.begin(57600);
   softuart.begin(57600); 
}

void loop()
{
    int readdata = 0,i = 0,count = 0;
    if (softuart.available())
    {
        Serial.print("   readdata:");
        while((readdata = softuart.read()) != (int)-1)
        {
            table[count] = readdata;
            count++;
            delay(1);
        }
        for(i = 0;i<20;i++)
        {
            //Serial.print(" 0x");
            //Serial.print( table[i],HEX);
            //softuart.write(table[i]);
            iicData[i] = table[i];
            //Serial.print( char(table[i]));
        }
        char2floatFunc(iicData,20);
        for(int j=0;j<3;j++){
          Serial.print(recData[j]);
          Serial.print(" ");
      }
        Serial.println("   stop rev");
    }
}

//字符数组转浮点数组
void char2floatFunc(char* iicdata,char iic_size){
  char intervalArr[]="";
  int n=0;
  int cn=0;
  for(int i=0;i<iic_size;i++){
    if(n>3)
    {
      n=0;
      break;
      }
     
    if(iicData[i] == 'X'){
      recData[n]=atof(intervalArr);
      //Serial.print(recData[n]);
      n++;
      cn=i;
      //Serial.println(cn); 
    }
    else
    {
      if(n==0)
      {
        intervalArr[i]=iicData[i];
        }
      else
      {
        intervalArr[i-cn-1]=iicData[i];
        //Serial.print(intervalArr[i-cn]);
        }
      }
  }
//  Serial.println(recData[0]); 
}

