#define nrf_len 29
#define chanel 0x6f

#include <stdio.h>
#include <string.h>

#include "nRF24L01.h"
#include "RF24.h"


//RF24 radio(9,10);
RF24 radio(7,8);

uint64_t address =  0x8a00431136; //8a00431136;//361143008a;//ff000000ff;//0xf3f3f3f3f3;//0xff000000ff;//0xf1f1f1f1f1;//0xf3f3f3f3f3;  //возможные номера труб


uint8_t pipeNo, receive_data[nrf_len],buf1[4],buf2[4],buf3[4];


void setup(){
  Serial.begin(9600); //открываем порт для связи с ПК
  radio.begin(); //активировать модуль
  radio.setAutoAck(0);         //режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0,15);     //(время между попыткой достучаться, число попыток)
  radio.setPayloadSize(nrf_len);     //размер пакета, в байтах

  radio.openReadingPipe(0,address);      //хотим слушать трубу 0
  radio.setChannel(chanel);  //выбираем канал (в котором нет шумов!)

  radio.setPALevel (RF24_PA_MIN); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  
  radio.powerUp(); //начать работу
  radio.startListening();  //начинаем слушать эфир, мы приёмный модуль

  Serial.println("Init");


  }


int j = 0;
char buf[1];

uint32_t time_from_stm = 0;
uint16_t temp_raw = 0;
float temp_bmp = 0;
uint32_t press_bmp = 0;

int16_t AX, AY, AZ;
float AX_float, AY_float, AZ_float;
uint8_t CRC_local = 0;
uint8_t msg_type = 0;
uint8_t foto = 0

uint16_t DS_buf = 0;
uint8_t DS_temp_int = 0;
float DS_temp = 0;

int16_t GX, GY, GZ;
float GX_float, GY_float, GZ_float;

int16_t MX, MY, MZ;
float MX_float, MY_float, MZ_float;

void loop() {

                             
    while( radio.available(&pipeNo)){    // слушаем эфир со всех труб
      radio.read( &receive_data, sizeof(receive_data) );         // чиатем входящий сигнал

      for(int i=0; i< nrf_len; i++)
      {
        Serial.print(receive_data[i], HEX);
        Serial.print(", ");
      }
      Serial.println();
      Serial.print("Packet start mark : ");
      Serial.print(receive_data[0], HEX);
      Serial.println(receive_data[1], HEX);
      Serial.print("Team ID : ");
      Serial.print(receive_data[2], HEX);
      Serial.println(receive_data[3], HEX);
      time_from_stm = receive_data[4];
      time_from_stm |= ((uint32_t)receive_data[5] << 8);
      time_from_stm |= ((uint32_t)receive_data[6] << 16);
      time_from_stm |= ((uint32_t)receive_data[7] << 24);

      Serial.print("Time from stm = ");
      Serial.println(time_from_stm);

      foto = (receive_data[21] >> 6) & 0x01;
      Serial.print("photoresistor: ");

      temp_raw = receive_data[8];
      temp_raw |= ((uint16_t)receive_data[9] ) << 8;
      temp_bmp = (float)temp_raw / 100.0;
      Serial.print("Temp from BMP : ");
      Serial.println(temp_bmp);

      press_bmp = receive_data[10];
      press_bmp |= ((uint32_t)receive_data[11] << 8);
      press_bmp |= ((uint32_t)receive_data[12] << 16);
      press_bmp |= ((uint32_t)receive_data[13] << 24);
      Serial.print("Press from BMP : ");
      Serial.println(press_bmp);

      AX = ((int16_t)receive_data[15] << 8) | receive_data[14];
      AY = ((int16_t)receive_data[17] << 8) | receive_data[16];
      AZ = ((int16_t)receive_data[19] << 8) | receive_data[18];

      AX_float = (float)AX/32768.0*16.0;
      AY_float = (float)AY/32768.0*16.0;
      AZ_float = (float)AZ/32768.0*16.0;

      Serial.println("Axelerometr: ");
      Serial.print("AX : ");
      Serial.print(AX_float);
      Serial.print(" AY : ");
      Serial.print(AY_float);
      Serial.print(" AZ : ");
      Serial.print(AZ_float);
      Serial.println();

      Serial.print("Check sum : ");
      Serial.println(receive_data[20], HEX);

      CRC_local = receive_data[0];
      for(int i = 1; i < 20; i++)
        CRC_local = CRC_local ^ receive_data[i];
      Serial.print("Local check sum  = ");
      Serial.println(CRC_local, HEX);
      if(CRC_local == receive_data[20])
        Serial.println("Data correct CRC = CRC_local");

      DS_buf = ((uint16_t)receive_data[21] << 8) | receive_data[22];
      DS_temp_int = (DS_buf >> 4) & 0b01111111;
      DS_temp = (float)DS_temp_int + (0.5 * ((receive_data[22]>>3)&0x01)) + (0.25 * ((receive_data[22]>>2)&0x01)) + (0.125 * ((receive_data[22]>>1)&0x01)) + (0.0625 * ((receive_data[22]>>0)&0x01));
      DS_temp = DS_temp * (1-2*((receive_data[21]>>3)&0x01));
      Serial.print("Temp from DS : ");
      Serial.println(DS_temp);

      msg_type = (receive_data[21] >> 7) & 0x01;
      Serial.print("Msg type = ");
      Serial.println(msg_type);
      Serial.println("If Msg type is 1 then the data comes from magnitometr; else the data comes from gyroscope.");

      if(msg_type)
      {
        MX = ((int16_t)receive_data[24] << 8) | receive_data[23];
        MY = ((int16_t)receive_data[26] << 8) | receive_data[25];
        MZ = ((int16_t)receive_data[28] << 8) | receive_data[27];
  
        MX_float = (float)MX/1711.0;
        MY_float = (float)MY/1711.0;
        MZ_float = (float)MZ/1711.0;
  
        Serial.println("Magnitometr: ");
        Serial.print("MX : ");
        Serial.print(MX_float);
        Serial.print(" MY : ");
        Serial.print(MY_float);
        Serial.print(" MZ : ");
        Serial.print(MZ_float);
        Serial.println();
      }
      else
      {
        GX = ((int16_t)receive_data[24] << 8) | receive_data[23];
        GY = ((int16_t)receive_data[26] << 8) | receive_data[25];
        GZ = ((int16_t)receive_data[28] << 8) | receive_data[27];
  
        GX_float = (float)GX/32768.0*2000.0;
        GY_float = (float)GY/32768.0*2000.0;
        GZ_float = (float)GZ/32768.0*2000.0;
  
        Serial.println("Gyroscope: ");
        Serial.print("GX : ");
        Serial.print(GX_float);
        Serial.print(" GY : ");
        Serial.print(GY_float);
        Serial.print(" GZ : ");
        Serial.print(GZ_float);
        Serial.println();
      }
      Serial.println();
      //Serial.println(receive_data[0]);
    } 
}
