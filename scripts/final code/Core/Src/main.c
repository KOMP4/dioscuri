/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "shiftreg.h"
#include "nrf24l01.h"
#include "nrf_regs.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define debug 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//================================ДАННЫЕ======================================================

//--------------------------------BMP280------------------------------------------------------

 uint8_t data_for_srs[2] = {0b00011111, 0b00000000}; //массив данных для сдвиговых регистров;
 uint8_t bmp_cal_data[24]; //массив данных для колибровочных данных
 uint8_t bmp_raw_data[6]; //массив данных о температуре и давении с BMP280
 uint8_t bmp_prev_status = 0;  //переменная для хранения предыдущего состояния регистра статус датчика bmp.


 unsigned short dig_T1;
 signed short dig_T2;
 signed short dig_T3;
 unsigned short dig_P1;
 signed short dig_P2;
 signed short dig_P3;
 signed short dig_P4;
 signed short dig_P5;
 signed short dig_P6;
 signed short dig_P7;
 signed short dig_P8;
 signed short dig_P9;

 signed long int t_fine = 0;




 //--------------------------------DS18B20-------------------------------------------------------------------

uint32_t milis = 0, micros = 0, milis_delay = 0, micros_delay = 0, milis_d = 0;  //переменные для времени

uint8_t ds_delay_type = 0;  //пусть 0-ой бит - это задержка на 485 мкс, 1-ый бит - на 65 мкс
uint32_t ds_micros=0;
uint32_t ds_wait_temp=0;
int ds_convert_delay = 760;

uint8_t ds_online = 0;

uint8_t ds_started = 0;
float ds_temp=0;
uint8_t ds_data[2];

//-----------------------------------LIS------------------------------------------------
uint8_t lis_raw_data[6];
//-----------------------------------LSM------------------------------------------------
uint8_t data_lsm[14];
int16_t out_data_lsm[6];
float data_lsm_float[6];

//-----------------------------------NRF-------------------------------------------------
uint8_t data_nrf[data_length];

int16_t temp_nrf;
unsigned long int press_nrf;
uint8_t NRF_msg_type = 0;
uint8_t new_data = 0;  // флаг того, что есть новые данные для отправки

//---------------------------карта памяти--------------------------------------------------
FRESULT res;  // FatFs function common result code
uint32_t byteswritten;  ///File write/read counts
FIL MyFile;     //File object

uint8_t sd_data_bmp[4096];
uint8_t sd_data_ds[4096];
uint8_t sd_data_lis[4096];
uint8_t sd_data_lsm[4096];
uint8_t sd_data_adc[4096];

uint sd_pos_bmp = 0;
uint sd_pos_ds = 0;
uint sd_pos_lis = 0;
uint sd_pos_lsm = 0;
uint sd_pos_adc = 0;

//----------------ADC-----------------
uint16_t adc = 0;
uint32_t adc_time = 0;
//--------биппер и фоторезистор----------
uint32_t start_time = 5*60*1000;
uint32_t start_spare_time = 10*60*1000;
uint8_t flag_beep = 0;
uint16_t is_highlighted = 2000;

uint32_t beep_time= 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) //переопределение функции вывода информации через порт swo
{
  int i = 0;
  for(i = 0; i < len; i++)
  {
    ITM_SendChar((*ptr++));  //отправка 1-го значения char через swo
  }
  return len;
}

//=============================ФУНКЦ�?�?=======================================================

//-------------------------------BMP---------------------------------------------------------
void bmp_get_call_data()
{
	uint8_t buf = 0x88 | (1 << 7); //адрес регистра с которог начинаем читать данные

	bmp_cs(0); //отключаем ножку CS у BMP для того чтобы сказать, что мы начали общение с BMP

	HAL_SPI_Transmit(&hspi2, &buf, 1, 1000); //отправляем начальный регистор для чтения
	HAL_SPI_Receive(&hspi2, bmp_cal_data, 24, 1000); //считываем 24 байта информациии в массив

	bmp_cs(1); // включаем ножку

	//совмещаем данные
	dig_T1 = ((unsigned short)bmp_cal_data[1]) << 8 | bmp_cal_data[0];
	dig_T2 = ((signed short)bmp_cal_data[3]) << 8 | bmp_cal_data[2];
	dig_T3 = ((signed short)bmp_cal_data[5]) << 8 | bmp_cal_data[4];
	dig_P1 = ((unsigned short)bmp_cal_data[7]) << 8 | bmp_cal_data[6];
	dig_P2 = ((signed short)bmp_cal_data[9]) << 8 | bmp_cal_data[8];
	dig_P3 = ((signed short)bmp_cal_data[11]) << 8 | bmp_cal_data[10];
	dig_P4 = ((signed short)bmp_cal_data[13]) << 8 | bmp_cal_data[12];
	dig_P5 = ((signed short)bmp_cal_data[15]) << 8 | bmp_cal_data[14];
	dig_P6 = ((signed short)bmp_cal_data[17]) << 8 | bmp_cal_data[16];
	dig_P7 = ((signed short)bmp_cal_data[19]) << 8 | bmp_cal_data[18];
	dig_P8 = ((signed short)bmp_cal_data[21]) << 8 | bmp_cal_data[20];
	dig_P9 = ((signed short)bmp_cal_data[23]) << 8 | bmp_cal_data[22];
}

float bmp_get_temp(signed long int adc_T)
{
  signed long var1, var2, T;  //буферные переменные
  float T_back;

  var1 = ((((adc_T>>3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2))>>11;
  var2 = (((((adc_T>>4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;

  T_back = T;
  temp_nrf = T;
  T_back = T_back / 100;

  return T_back;
}

unsigned long int bmp_get_press(signed long int adc_P)
{
  signed long long int var1, var2, p;
  var1 = ((signed long long int)t_fine) - 128000;
  var2 = var1 * var1 * (signed long long int)dig_P6;
  var2 = var2 + ((var1*(signed long long int)dig_P5)<<17);
  var2 = var2 + (((signed long long int)dig_P4)<<35);
  var1 = ((var1 * var1 * (signed long long int)dig_P3)>>8) + ((var1 * (signed long long int)dig_P2)<<12);
  var1 = (((((signed long long int)1)<<47)+var1))*((signed long long int)dig_P1)>>33;
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576-adc_P;
  p = (((p<<31)-var2)*3125)/var1;
  var1 = (((signed long long int)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((signed long long int)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((signed long long int)dig_P7)<<4);
  p = p  / 256;

  unsigned long int return_p = (unsigned long int)p;
  press_nrf = return_p;

  return return_p;
}

void bmp_setup()
{
	uint8_t buf = 0xF5 & (~(1 << 7));

	bmp_cs(0); //отключаем ножку CS у BMP для того чтобы сказать, что мы начали общение с BMP
	HAL_SPI_Transmit(&hspi2, &buf, 1, 1000); //отправляем регистор, который пишем

	buf = 0b01000000; //отправляем в регистор, который пишем

	HAL_SPI_Transmit(&hspi2, &buf, 1, 1000);
	bmp_cs(1);
	bmp_cs(0);

	buf = 0xF4 & (~(1 << 7));

	HAL_SPI_Transmit(&hspi2, &buf, 1, 1000);

	buf = 0b10010111;

	HAL_SPI_Transmit(&hspi2, &buf, 1, 1000);
	bmp_cs(1);
}

void bmp_get_data()
{
	uint8_t buf = 0xF3 | (1 << 7); //адрес регистра с которог начинаем читать данные

	bmp_cs(0); //отключаем ножку CS у BMP для того чтобы сказать, что мы начали общение с BMP

	HAL_SPI_Transmit(&hspi2, &buf, 1, 1000); //отправляем начальный регистор для чтения
	HAL_SPI_Receive(&hspi2, &buf, 1, 1000); //считываем 24 байта информациии в массив

	bmp_cs(1); // включаем ножку

	buf = (buf & 0b00001000) >> 3;  //получаем бит, отвечающий за то, идёт ли обработка данных с датчика или нет (бит measuring)


	if((buf == 0)&(bmp_prev_status == 1))  //проверяем бит считанный из регстра статус, что данные сейчас не обрабатываются и проверяем, была ли до этого обработка данных
	{
		bmp_prev_status = buf;
		buf = 0xF7 | (1<<7);
		bmp_cs(0); //отключаем ножку CS у BMP для того чтобы сказать, что мы начали общение с BMP
		HAL_SPI_Transmit(&hspi2, &buf, 1, 1000); //отправляем начальный регистор для чтения
		HAL_SPI_Receive(&hspi2, bmp_raw_data, 6, 1000); //считываем 24 байта информациии в массив
		bmp_cs(1); // включаем ножку

		new_data++;  // говорим, что есть новые данные для отправки

		//карта памяти
		uint32_t buf_milis = milis; //время
		for(int i = 0; i < 4; i++)
		{
			uint8_t buf = buf_milis >> (i*8);
			sd_data_bmp[sd_pos_bmp] = buf;
			sd_pos_bmp++;
		}

		for(int i = 0; i < 6; i++) //данные
		{
			sd_data_bmp[sd_pos_bmp] = bmp_raw_data[i];
			sd_pos_bmp++;
		}

		for (int i = 0; i < 6; ++i)
		{
			sd_data_bmp[sd_pos_bmp] = 0;
			sd_pos_bmp++;
		}

		if(sd_pos_bmp == sizeof(sd_data_bmp)) //запись в файл
		{
			res = f_open(&MyFile, "BMP.hex",FA_WRITE);
			res = f_lseek(&MyFile, f_size(&MyFile));
			res = f_write(&MyFile, sd_data_bmp, sizeof(sd_data_bmp), (void*)&byteswritten);
			res = f_close(&MyFile);
			sd_pos_bmp = 0;
		}

		#ifdef debug
			signed long int raw_press;
			signed long int raw_temp;

			raw_press = ((signed long int)bmp_raw_data[0])<<12 | ((signed long int)bmp_raw_data[1])<<4 | bmp_raw_data[2]>>4;
			raw_temp = ((signed long int)bmp_raw_data[3])<<12 | ((signed long int)bmp_raw_data[4])<<4 | bmp_raw_data[5]>>4;

			float bmp_temp = 0;
			int bmp_press = 0;

			bmp_temp = bmp_get_temp(raw_temp);
			bmp_press = bmp_get_press(raw_press);

			printf("time = %lu, temp = %f, press = %d\n", milis, bmp_temp, bmp_press);
		#endif
	}
	else
	{
		bmp_prev_status = buf;
	}


}

void bmp_calibr()
{
	  uint8_t buf;

	  buf = 0xD0 | (1<<7);

	  bmp_cs(0);

	  HAL_SPI_Transmit(&hspi2, &buf, 1, 1000);
	  HAL_SPI_Receive(&hspi2, &buf, 1, 1000);

	  bmp_cs(1);

	  printf("ID = %x = %d\n", buf, buf);

	  bmp_get_call_data();

	  for(int i = 0; i < 24; ++i)
	  {
		  printf("cal[%d] = %x\n", i, bmp_cal_data[i]);
	  }

	  printf("dig_T1 = %x\n", dig_T1);
	  printf("dig_T2 = %x\n", dig_T2);
	  printf("dig_T3 = %x\n", dig_T3);
	  printf("dig_P1 = %x\n", dig_P1);
	  printf("dig_P2 = %x\n", dig_P2);
	  printf("dig_P3 = %x\n", dig_P3);
	  printf("dig_P4 = %x\n", dig_P4);
	  printf("dig_P5 = %x\n", dig_P5);
	  printf("dig_P6 = %x\n", dig_P6);
	  printf("dig_P7 = %x\n", dig_P7);
	  printf("dig_P8 = %x\n", dig_P8);
	  printf("dig_P9 = %x\n", dig_P9);

	  bmp_setup();
}
//-----------------------------DS18B20------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //вектор прерываний для таймеров
{
  if(htim->Instance == TIM3) //check if the interrupt comes from TIM3
    {
      micros += 15;
    }
  if(htim->Instance == TIM4) //check if the interrupt comes from TIM4 (если прерывание пришло от 4таймера)
    {
      milis++;
    }
}

static void DS_INIT(int a)  //переинициализация пина протокола 1-WIRE
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin : DS_Pin */
    GPIO_InitStruct.Pin = DS_Pin;
    if(a == 0)
     GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //перестраивается в режим входа
    else
     {
     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
     }
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS_GPIO_Port, &GPIO_InitStruct);

  }

uint8_t DS_find()  //поиск датчика на шине 1-WIRE
{
	uint8_t buf = 0;
	DS_INIT(1);
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
	ds_micros = micros + 480;
	while( ds_micros > micros);
	DS_INIT(0);

	ds_micros = micros + 60;
	while( ds_micros > micros );
	buf = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);

	ds_micros = micros + 420;
	while( ds_micros > micros);

	if(buf == 0)
		return 1;
	else
		return 0;

}

 void DS_send(uint8_t data)  //отправка байта данных по 1-WIRE
 {
   for(int i = 0; i < 8; i++)
   {
	   if( (data & (1<<i)) == (1<<i) )
	   {
		   DS_INIT(1);
		   HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
		   HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
		   HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
		   HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
		   HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
		   DS_INIT(0);

		   ds_micros = micros + 65;
		   while( ds_micros > micros );
	   }
	   else
	   {
		   DS_INIT(1);
		   HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);

		   ds_micros = micros + 65;
		   while( ds_micros > micros);

		   DS_INIT(0);
	   }
   }
 }

uint8_t DS_read()  //чтение байта данных с 1-WIRE
{
   uint8_t data = 0;
   for(int i = 0; i < 8; i++)
   {
		DS_INIT(1);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, 0);
		DS_INIT(0);

		ds_micros = micros + 15;
		while( ds_micros > micros );
		if(HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin) ==  0)
			data &= ~(1<<i);
		else
			data |= (1<<i);
   }
   return data;
}

 float DS_get_data()  //получение данных о температуре с датчика DS18B20
 {
	float temp = 0;
	HAL_TIM_Base_Start_IT(&htim3);  //запускаем таймер для микросекунд

	if(ds_started == 0)
	{
		if(DS_find() == 1)
		{
			DS_send(0xcc);
			DS_send(0x44);
			ds_started = 1;
			ds_wait_temp = milis + ds_convert_delay;
		}
	}
	else if (ds_wait_temp <= milis)
	{
		if(DS_find() == 1)
		{
			DS_send(0xcc);
			DS_send(0xbe);
			ds_data[0] = DS_read();
			ds_data[1] = DS_read();
			HAL_TIM_Base_Stop_IT(&htim3);

			new_data++;  // говорим, что есть новые данные для отправки

			//карта памяти
			uint32_t buf_milis = milis; //время
			for(int i = 0; i < 4; i++)
			{
				uint8_t buf = buf_milis >> (i*8);
				sd_data_ds[sd_pos_ds] = buf;
				sd_pos_ds++;
			}

			for(int i = 0; i < 2; i++) //данные
			{
				sd_data_ds[sd_pos_ds] = ds_data[i];
				sd_pos_ds++;
			}

			for(int i = 0; i < 2; i++) //доп данные
			{
				sd_data_ds[sd_pos_ds] = 0;
				sd_pos_ds++;
			}

			if(sd_pos_ds == sizeof(sd_data_ds)) //запись в файл
			{
				res = f_open(&MyFile, "DS.hex",FA_WRITE);
				res = f_lseek(&MyFile, f_size(&MyFile));
				res = f_write(&MyFile, sd_data_ds, sizeof(sd_data_ds), (void*)&byteswritten);
				res = f_close(&MyFile);
				sd_pos_ds = 0;
			}

			#ifdef debug
				uint16_t buf_ds=0;
				buf_ds = (ds_data[1]<<8) | ds_data[0];

				temp = (buf_ds>>4)&0b01111111;
				temp += 0.5*((buf_ds>>3)&0x01);
				temp += 0.25*((buf_ds>>2)&0x01);
				temp += 0.125*((buf_ds>>1)&0x01);
				temp += 0.0625*(buf_ds&0x01);
				if (((buf_ds>>15)&0x01) > 0)
					temp = temp * (-1);
			#endif

			ds_started = 0;
		}
	}

	HAL_TIM_Base_Stop_IT(&htim3);
	return temp;
}

  void DS_init(uint8_t res)  //инициализация датчика DS18B20
  {
   /*
    * Данная функция нужна для настройки датчика ds18b20
    * В данню функцию передаётся значение разрешения температуры
    * 0 - 9 бит
    * 1 - 10 бит
    * 2 - 11 бит
    * остальные - 12 бит
  */
   HAL_TIM_Base_Start_IT(&htim3);

   if(DS_find())
   {
    DS_send(0xcc);
    DS_send(0x4E);
    DS_send(0xff);
    DS_send(0x00);

    switch(res)
    {
    case 0:
    {
     DS_send(0b00011111);
     ds_convert_delay = 100;
     break;
    }
    case 1:
    {
     DS_send(0b00111111);
     ds_convert_delay = 200;
     break;
    }
    case 2:
    {
     DS_send(0b01011111);
     ds_convert_delay = 400;
     break;
    }
    default:
     DS_send(0b01111111);
     ds_convert_delay = 800;
    }
   }
   HAL_TIM_Base_Stop_IT(&htim3);

  }

//-----------------------------LIS------------------------------------------
void lis_init()
   {
     // настройка lis3mdl. Особенность в том, что надо сразу все настройки загружать в память датчика, иначе работать не будет. Почему такое происходит неизвестно
     uint8_t lis_config[6]={(0x20|0b01000000),0b01111100,0b01100000,0b00000000,0b00001100,0b01000000};
     lis3mdl_cs(0);
     HAL_SPI_Transmit(&hspi2, (uint8_t *)lis_config,6,5000);
     lis3mdl_cs(1);
   }

void lis_get_data()
{
	uint8_t buf;

	buf = (0x27) | 0b11000000;
	lis3mdl_cs(0);
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	HAL_SPI_Receive(&hspi2, &buf, 1, 500);
	lis3mdl_cs(1);

	buf = buf & 0x0f;

	if(buf == 15)
	{

		lis3mdl_cs(0);
		buf = (0x28) | 0b11000000;
		HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
		HAL_SPI_Receive(&hspi2, lis_raw_data, 6, 500);  //читывание данных с датчика
		lis3mdl_cs(1);
		new_data++;  // говорим, что есть новые данные для отправки
#ifdef debug
		int16_t lis_data[3];
		float lis_data_float[3];
		for(int i=0; i<3; i++)  //преобразование данных с датчика и вывод
		{
		  lis_data[i] = lis_raw_data[i*2+1];
		  lis_data[i] = lis_data[i]<<8 | lis_raw_data[i*2];
		  lis_data_float[i] = (float)lis_data[i]/1711.0;
//		  printf("%d, ", lis_data[i]);
		}
//		printf("\n");

		printf("time = %lu, Mag X = %f, Mag Y = %f, Mag Z = %f\n",milis, lis_data_float[0],lis_data_float[1],lis_data_float[2]);
#endif
	//карта памяти
	uint32_t buf_milis = milis; //время
	for(int i = 0; i < 4; i++)
	{
		uint8_t buf = buf_milis >> (i*8);
		sd_data_lis[sd_pos_lis] = buf;
		sd_pos_lis++;
	}

	for(int i = 0; i < 6; i++) //данные
	{
		sd_data_lis[sd_pos_lis] = lis_raw_data[i];
		sd_pos_lis++;
	}

	for(int i = 0; i < 6; i++) //данные
	{
		sd_data_lis[sd_pos_lis] = 0;
		sd_pos_lis++;
	}

	if(sd_pos_lis == sizeof(sd_data_lis)) //запись в файл
	{
		res = f_open(&MyFile, "LIS.hex",FA_WRITE);
		res = f_lseek(&MyFile, f_size(&MyFile));
		res = f_write(&MyFile, sd_data_lis, sizeof(sd_data_lis), (void*)&byteswritten);
		res = f_close(&MyFile);
		sd_pos_lis = 0;
	}
	}

}

 //----------------------------LSM-------------------------------------------
void lsm_init() //настройка
{
	uint8_t buf;

	lsm6ds_cs(0);
	buf = 0x10;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	buf = 0b01000100;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	lsm6ds_cs(1);

	lsm6ds_cs(0);
	buf = 0x11;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	buf = 0b01001100;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	lsm6ds_cs(1);

	lsm6ds_cs(0);
	buf = 0x12;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	buf = 0b01000100;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	lsm6ds_cs(1);

	lsm6ds_cs(0);
	buf = 0x13;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	buf = 0b00000100;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	lsm6ds_cs(1);

	lsm6ds_cs(0);
	buf = 0x14;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	buf = 0b00000000;
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	lsm6ds_cs(1);

}

void lsm_get_data()
{
	uint8_t buf;
	buf = 0x1E | 0b10000000;

	lsm6ds_cs(0);
	HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
	HAL_SPI_Receive(&hspi2, &buf, 1, 500);
	lsm6ds_cs(1);
	buf = buf & 0b00000011;  //вычленяем только 2 последних бита

	if(buf > 0)
	{
		  buf = 0x22 | 0b10000000;
		  lsm6ds_cs(0);
		  HAL_SPI_Transmit(&hspi2, &buf, 1, 500);
		  HAL_SPI_Receive(&hspi2, &data_lsm[0], 12, 500);
		  lsm6ds_cs(1);

		  new_data++;  // говорим, что есть новые данные для отправки

		  //карта памяти
		  	uint32_t buf_milis = milis; //время
		  	for(int i = 0; i < 4; i++)
		  	{
		  		uint8_t buf = buf_milis >> (i*8);
		  		sd_data_lsm[sd_pos_lsm] = buf;
		  		sd_pos_lsm++;
		  	}

		  	for(int i = 0; i < 12; i++) //данные
		  	{
		  		sd_data_lsm[sd_pos_lsm] = data_lsm[i];
		  		sd_pos_lsm++;
		  	}

		  	if(sd_pos_lsm == sizeof(sd_data_lsm)) //запись в файл
		  	{
		  		res = f_open(&MyFile, "LSM.hex",FA_WRITE);
		  		res = f_lseek(&MyFile, f_size(&MyFile));
		  		res = f_write(&MyFile, sd_data_lsm, sizeof(sd_data_lsm), (void*)&byteswritten);
		  		res = f_close(&MyFile);
		  		sd_pos_lsm = 0;
		  	}

			#ifdef debug
				  for(int i=0; i<6; i++)
				  {
					  out_data_lsm[i] = data_lsm[i*2+1];
					  out_data_lsm[i] = out_data_lsm[i]<<8 | data_lsm[i*2];
			//		  printf("%d, ", out_data_lsm[i]);
				  }
			//	  printf("\n");

				  for(int i=0; i<3; i++)
				  {
					  data_lsm_float[i] = (float)out_data_lsm[i]/32768.0*2000.0;
				  }
				  for(int i=3; i<6; i++)
				  {
					  data_lsm_float[i] = (float)out_data_lsm[i]/32768.0*16.0;
				  }

				  printf("time = %lu, GX = %f, GY = %f, GZ = %f, AX = %f, AY = %f, AZ = %f\n",milis, data_lsm_float[0],data_lsm_float[1],data_lsm_float[2],data_lsm_float[3],data_lsm_float[4],data_lsm_float[5]);
			#endif
	}


}

//-----------------------------NRF----------------------------------------
void nrf_identifier()  //функция для настройки заголовка пакета данных для отправки
{
  // задаём заголовок начала пакета
	data_nrf[0] = 0xA4;
	data_nrf[1] = 0xB3;
  // задаём идентификатор команды
	data_nrf[2] = 0xBB;
	data_nrf[3] = 0xBB;
}

void nrf_time()  //записываем время в пакет данных
{
  uint32_t local_milis = milis;  //для того, чтобы избежать проблем с внезапной сменой времени во время записи, делаем локальную переменную и работаем с ней

  data_nrf[4] = local_milis & 0xff;
  data_nrf[5] = (local_milis >> 8) & 0xff;
  data_nrf[6] = (local_milis >> 16) & 0xff;
  data_nrf[7] = (local_milis >> 24) & 0xff;
}

void nrf_bmp() //данные c bmp
{
	data_nrf[8] = temp_nrf & 0xFF;
	data_nrf[9] = (temp_nrf >> 8) & 0xFF;

	data_nrf[10] = press_nrf & 0xFF;
	data_nrf[11] = (press_nrf >> 8) & 0xFF;
	data_nrf[12] = (press_nrf >> 16) & 0xFF;
	data_nrf[13] = (press_nrf >> 24) & 0xFF;
}

void nrf_aks() //данные от акселерометра
{
	for (int i = 14; i <= 19; ++i)
	{
		data_nrf[i] = data_lsm[i - 8];
	}
}

void nrf_gyr() //данные от гирокопа
{
	for (int i = 23; i <= 28; ++i)
	{
		data_nrf[i] = data_lsm[i - 23];
	}
}

void nrf_lis() //данные от магнитометра
{
	for (int i = 23; i <= 28; ++i)
	{
		data_nrf[i] = lis_raw_data[i - 23];
	}
}

void nrf_check_sum() //функция для расчёта контрольной суммы
{
	data_nrf[20] = data_nrf[0]; //задаём начальное значение контрольной суммы
	for (int i = 1; i < 20; ++i) //цикл для выполнения логической операции "исключающее �?Л�?" из всех байт информации
	{
		data_nrf[20] ^= data_nrf[i];
	}
}

void nrf_transmit()  //функция для подготовки данных к отправке и сама отправка данных
{
  nrf_identifier();
  nrf_time();
  nrf_bmp();
  nrf_aks();

  nrf_check_sum();  //считаем контрольную сумму

  // Записываем данные о температуре с DS18B20 вне зависимости от того, было считывание температуры с него или нет
  data_nrf[21] = ds_data[1] & 0x0f;  // Первым пишем старший байт с DS, т.к. в нём мы можем взять 4 старших бита под служебную инфу
  data_nrf[22] = ds_data[0];

  if (adc >= is_highlighted)
  {
	  data_nrf[21] = data_nrf[21] | 0b01000000; //фоторезистор засвечен
  }
  else
  {
	   data_nrf[21] = data_nrf[21] & 0b10111111; //фоторезистор НЕ засвечен
  }

  if(NRF_msg_type)  //Проверяем на тип сообщения, если тип сообщения 1, то отправляем в конце пакета магнитометр
  {
    data_nrf[21] = data_nrf[21] | 0b10000000;  // выставляем тип сообщения в старший бит 21-го элемента массива

    // Записываем данные от магнитометра
    nrf_lis();

    NRF_msg_type = 0;  //меняем для следующей отправки тип сообщения
  }
  else  // Если тип сообщения 0, то отправляем гироскоп
  {
    data_nrf[21] = data_nrf[21] & 0b01111111;  // выставляем тип сообщения в старший бит 21-го элемента массива

    // Записываем данные от гироскопа
    nrf_gyr();

    NRF_msg_type = 1;  //меняем для следующей отправки тип сообщения
  }
  if (new_data > 0)
  {
	  NRF_Send_data(data_nrf,data_length);  //передаём данные в NRF для отправки
	  HAL_UART_Transmit(&huart2, data_nrf, data_length, 500);
  }
}

//-------------------------ADC------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
        adc = HAL_ADC_GetValue(&hadc1);
    }
}

void sd_adc()
{
	uint32_t buf_milis = milis; //время
	for(int i = 0; i < 4; ++i)
	{
		uint8_t buf = buf_milis >> (i*8);
		sd_data_adc[sd_pos_adc] = buf;
		sd_pos_adc++;
	}

	if (milis > adc_time)
	{
		adc_time = milis + 100;

		sd_data_adc[sd_pos_adc] = adc;
		sd_pos_adc++;
		sd_data_adc[sd_pos_adc] = (adc >> 8);
		sd_pos_adc++;

		sd_data_adc[sd_pos_adc] = 0;
		sd_pos_adc++;
		sd_data_adc[sd_pos_adc] = 0;
		sd_pos_adc++;

		if(sd_pos_adc == sizeof(sd_data_adc)) //запись в файл
		{
		    res = f_open(&MyFile, "ADC.hex",FA_WRITE);
		    res = f_lseek(&MyFile, f_size(&MyFile));
		    res = f_write(&MyFile, sd_data_adc, sizeof(sd_data_adc), (void*)&byteswritten);
		    res = f_close(&MyFile);
		    sd_pos_adc = 0;
		}
	}
}

void beeper()
{
	if (((milis >= start_time) & (adc >= is_highlighted)) | (milis >= start_spare_time))
	{
		flag_beep = 1;
	}

	if ((flag_beep == 1) & (milis > beep_time))
	{
		beep_time = milis + 500;
		HAL_GPIO_TogglePin(Beep_GPIO_Port, Beep_Pin);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);  //стартуем таймер 4 для милисекунд

  ShiftRegs_init();

//------------------карта памяти-------------
    res = disk_initialize(0);
    res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);

    res = f_open(&MyFile, "BMP.hex",FA_CREATE_NEW | FA_WRITE);
    res = f_close(&MyFile);

    res = f_open(&MyFile, "LIS.hex",FA_CREATE_NEW | FA_WRITE);
    res = f_close(&MyFile);

    res = f_open(&MyFile, "LSM.hex",FA_CREATE_NEW | FA_WRITE);
    res = f_close(&MyFile);

    res = f_open(&MyFile, "DS.hex",FA_CREATE_NEW | FA_WRITE);
    res = f_close(&MyFile);

    res = f_open(&MyFile, "ADC.hex",FA_CREATE_NEW | FA_WRITE);
    res = f_close(&MyFile);


//--------------NRF-------------------
  NRF_Init();
  NRF_Set_TX();
//-------------BMP280-----------------
  bmp_calibr();
//--------------LIS------------------
  lis_init();
//--------------LSM------------------
  lsm_init();
//------------DS18B20----------------
  DS_init(4);

//--------------ADC------------------
  HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ds_temp = DS_get_data();
	  bmp_get_data();
  	  lis_get_data();
  	  lsm_get_data();

  	  nrf_transmit();

  	  sd_adc();
  	  beeper();
	  HAL_ADC_Start_IT(&hadc1);
  	  //printf("adc = %d\n",adc);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 46;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1259;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SR0E_Pin|SRL_Pin|RF_SRL_Pin|RF_SRE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DS_Pin|Beep_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SR0E_Pin SRL_Pin RF_SRL_Pin RF_SRE_Pin */
  GPIO_InitStruct.Pin = SR0E_Pin|SRL_Pin|RF_SRL_Pin|RF_SRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DS_Pin Beep_Pin */
  GPIO_InitStruct.Pin = DS_Pin|Beep_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_Insert_Pin */
  GPIO_InitStruct.Pin = SD_Insert_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_Insert_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
