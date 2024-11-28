/*
 * nrf24l01.c
 *
 *  Created on: 1 апр. 2023 г.
 *      Author: ilya0
 */

#include <nrf24l01.h>
#include <shiftreg.h>
#include <nrf_regs.h>

uint8_t rx_addr[5] = {0x01,0x02,0x03,0x04,0x05};  //адрес для приёма данных
uint8_t tx_addr[5] = {0x36,0x11,0x43,0x00,0x8a};  //адрес для отправки данных (этот адрес на ардуинке в обратном порядке)

uint8_t NRF_Read_reg(uint8_t data)
{
	/*
	 * Данная функция нужна для чтения значения регистра с радиомодуля NRF24L01
	 * На вход надо подать адрес регистра
	 * На выходе будет значение, которое на данный момент находится в регистре
	 */

	uint8_t receive;

	NRF_cs(0);

	HAL_SPI_TransmitReceive(&hspi2, &data, &receive, 1, 500);

	if(data != STATUS)
	{
		data=NOP;

		HAL_SPI_TransmitReceive(&hspi2, &data, &receive, 1, 500);
	}

	NRF_cs(1);

	return receive;
}

void NRF_Send_reg(uint8_t reg, uint8_t data)
{
	/*
	 * Данная функция нужна для записи значения в регистр
	 * На вход надо подать адрес регистра и потом значение для записи
	 * Данная функция ничего не возвращает
	 */

	//reg &= 0b00111111;
	reg |= 0x20;
	uint8_t buf;

	NRF_cs(0);
	HAL_SPI_TransmitReceive(&hspi2, &reg, &buf, 1, 500);
	HAL_SPI_TransmitReceive(&hspi2, &data, &buf, 1, 500);
	NRF_cs(1);

}

void NRF_Send_data(uint8_t* data, int n)
{
	/*
	 * Данная функция нужна для отправки данных в буфер отправки
	 * На вход подаётся адрес масива с данными и количество элементов в массиве
	 */
	uint8_t transmit[1];
	transmit[0]=W_TX_PAYLOAD;
	NRF_cs(0);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)transmit, 1, 5000);
	for(int i=0;i<n;i++)
	{
		transmit[0]=data[i];
		HAL_SPI_Transmit(&hspi2, (uint8_t*)transmit, 1, 5000);
	}
	NRF_cs(1);
	NRF_ce(0);
	for(int i=0;i<10;i++)
	{
		HAL_GPIO_WritePin(RF_SRE_GPIO_Port, RF_SRE_Pin, 0);
	}
	NRF_ce(1);
}

void NRF_Set_RX_addres(uint8_t addr)
{
	/*
	 * Данная функция нужна для установки RX адреса NRF
	 * На вход подаётся адрес трубы, в которую будем записывать адрес
	 */
	uint8_t transmit[6]={addr|W_REGISTER,rx_addr[0],rx_addr[1],rx_addr[2],rx_addr[3],rx_addr[4]};
	NRF_cs(0);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)transmit, 6, 5000);
	NRF_cs(1);
}

void NRF_Set_TX_addres(uint8_t addr)
{
	/*
	 * Данная функция нужна для установки TX адреса NRF
	 * На вход подаётся адрес трубы, в которую будем записывать адрес
	 */
	uint8_t transmit[6]={addr|W_REGISTER,tx_addr[0],tx_addr[1],tx_addr[2],tx_addr[3],tx_addr[4]};
	NRF_cs(0);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)transmit, 6, 5000);
	NRF_cs(1);
}

void NRF_Set_RX ()  //установка nrf на прием данных
{

	NRF_ce(0);  //опускаем ножку CE

	NRF_Set_RX_addres(TX_ADDR);  //настраиваем наш адрес
	NRF_Set_TX_addres(RX_ADDR_P0);  //настраиваем адрес удаленного датчика

	NRF_Send_reg(CONFIG, ((0<<MASK_RX_DR)|(1<<MASK_TX_DS)|(1<<MASK_MAX_RT)|(1<<EN_CRC)|(1<<CRCO)|(1<<PWR_UP)|(1<<PRIM_RX)));
	/*
	Настрауваем регистр CONFIG:
	MASK_RX_DR - разрешаем прерывание по получению пакета;
	MASK_TX_DS - запрещаем прерывание по удачной отправке;
	MASK_MAX_RT - запрещаем прерывание если достигнуто максимальное кол-во попыток отправить данные;
	EN_CRC - включаем рассчет контрольной суммы;
	CRCO - выставляем контрольную сумму в 2 байта (если поставить 0, то будет 1 байт);
	PWR_UP - включаем модуль на работу, переходим в Standby-1;
	PRIM_RX - говорим nrf, что он сейчас работает на приём. Он не переходит в RX Mode, пока мы не поднимем ножку CE;
	*/
	NRF_ce(1);  //подтягиваем ножку CE к питанию, для того чтобы модуль перешел в RX Mode и не отпускаем, пока нам нужен прием. Если надо вернуться в Standby-1, то просто ее подтягиваем к земле
	HAL_Delay(135);  //делаем обязательную паузу для настройки модуля на прием

}

void NRF_Set_TX()  //переводим модуль в режим standby-2. В этом режиме мы можем постоянно писать в него данные для отправки
{
	NRF_ce(0);  //опускаем ножку CE

	NRF_Set_TX_addres(TX_ADDR);  //настраиваем адрес удаленного датчика
	NRF_Set_RX_addres(RX_ADDR_P0);  //настраиваем наш адрес

	NRF_Send_reg(CONFIG, 0b01001110);//((1<<MASK_RX_DR)|(0<<MASK_TX_DS)|(0<<MASK_MAX_RT)|(1<<EN_CRC)|(1<<CRCO)|(1<<PWR_UP)|(0<<PRIM_RX)));
	/*
	Настрауваем регистр CONFIG:
	MASK_RX_DR - запрещаем прерывание по получению пакета;
	MASK_TX_DS - разрешаем прерывание по удачной отправке;
	MASK_MAX_RT - разрешаем прерывание если достигнуто максимальное кол-во попыток отправить данные;
	EN_CRC - включаем рассчет контрольной суммы;
	CRCO - выставляем контрольную сумму в 2 байта (если поставить 0, то будет 1 байт);
	PWR_UP - включаем модуль на работу, переходим в Standby-1;
	PRIM_RX - говорим nrf, что он сейчас работает на передачу. Он не переходит в TX Mode, пока мы не поднимем ножку CE;
	*/
	NRF_ce(1);  //подтягиваем ножку CE к питанию, для того чтобы модуль перешел в Standby-2 и не отпускаем. Если надо вернуться в Standby-1, то просто ее подтягиваем к земле

}

void NRF_Init ()  //инициализация NRF, установка регистров
{

	NRF_Send_reg(RF_CH, chanel);  //настраиваем канал, на котором будут передаваться данные
	NRF_Send_reg(EN_AA, pipes);  //настраиваем какие трубы будут работать с автоподтверждением
	NRF_Send_reg(EN_RXADDR, RXpipes);  //настраиваем какие трубы будут принимать данные, если nrf будет работать на прием
	NRF_Send_reg(SETUP_AW, 0b00000011); //задаем длину поля для адреса
	NRF_Send_reg(SETUP_RETR, retr);  //настраиваем автоповтор отправки
	NRF_Send_reg(RF_SETUP, setup);  //настраиваем скорость и мощность
	NRF_Send_reg(FEATURE, 0b00000001);  //разрешаем команду W_TX_PAYLOAD_NOACK
	NRF_Send_reg(RX_PW_P0, data_length);  //устанавливаем длину пакета данных

}



