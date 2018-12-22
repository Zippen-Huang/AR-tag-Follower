#include <Wire.h>

bool tf =false;
extern char iicData[20];//IIC传送的字符数组
extern float recData[3]; 
char Device_Address = 8;
#define I2C_ERROR                  (-1)

void IIC_VelDataReader_Init(){
  Wire.begin();
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

//读取IIC缓存区数据
char read_Data(char start, char *buffer, char size)
{
  int16_t i = 0;
  int8_t return_value = 0;
  Wire.beginTransmission(Device_Address);
  return_value = Wire.write(start);
  if(return_value != 1)
  {
    return(I2C_ERROR);
  }
  return_value = Wire.endTransmission(false);
  if(return_value != 0)
  {
    return(return_value);
  }
  delayMicroseconds(1);
  /* Third parameter is true: relase I2C-bus after data is read. */
  Wire.requestFrom(Device_Address, size, (uint8_t)true);
  while(Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  delayMicroseconds(1);
  if(i != size)
  {
    return(I2C_ERROR);
  }
  return(0); //return: no error 
}
