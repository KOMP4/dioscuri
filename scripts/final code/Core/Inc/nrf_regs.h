#ifndef nrf_regs_H
#define nrf_regs_H


#define R_REGISTER                      0x00 //������ �������
#define W_REGISTER                      0x20 //����� � �������
#define R_RX_PAYLOAD                    0x61 //���������� �� ������ �������� ������ �� �������
#define W_TX_PAYLOAD                    0xA0 //������ ������ � ����� ��� �������� � ������
#define FLUSH_TX                        0xE1 //������� ������ ��������
#define FLUSH_RX                        0xE2 //������� ������ ������
#define REUSE_TX_PL                     0xE3
#define ACTIVATE                        0x50
#define R_RX_PL_WID                     0x60
#define W_ACK_PAYLOAD                   0xA8
#define W_TX_PAYLOAD_NOACK              0x58
#define NOP                             0xFF //������� ��������, ������ �� ������.

/*������� CONFIG*/    //���������������� �������
#define CONFIG          0x00
#define MASK_RX_DR  6 //���/���� ���������� �� ���� RX_DR � ���. STATUS. 0-���, 1-����.
#define MASK_TX_DS  5 //���/���� ���������� �� ���� TX_DS � ���. STATUS. 0-���, 1-����.
#define MASK_MAX_RT 4 //���/���� ���������� �� ���� MAX_RT � ���. STATUS. 0-���, 1-����.
#define EN_CRC      3 //��������� CRC. �� ��������� ���. ���� ���� �� ����� �������� EN_AA �������.
#define CRCO        2 //����� CRC. 0-1 ����, 1-2 �����.
#define PWR_UP      1 //1-POWER UP, 0-POWER DOWN, �� ��������� 0.
#define PRIM_RX     0 //0-����� ��������, 1-����� ������.

/*������� STATUS*/
#define STATUS          0x07
#define RX_DR           6 /*����������: ������ ��������. ��� ������ �������� 1.*/
#define TX_DS           5 /*����������: ������ ��������. ��� ������ �������� 1.*/
#define MAX_RT          4 /*����������: ������ �� ��������. ��� ������ �������� 1.*/
#define RX_P_NO2        3
#define RX_P_NO1        2
#define RX_P_NO0        1
#define TX_FULL0        0 /*���� ������������ TX FIFO ������ ��������. 1-����������, 0-���� ��� �����.*/

//������� EN_AA
#define EN_AA 0x01
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0

//������� EN_RXADDR
#define EN_RXADDR 0x02
#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0

//������� SETUP_AW
#define SETUP_AW 0x03
#define AW1 1
#define AW0 0

//������� SETUP_RETR
#define SETUP_RETR 0x04
#define ARD7 7
#define ARD6 6
#define ARD5 5
#define ARD4 4
#define ARC3 3
#define ARC2 2
#define ARC1 1
#define ARC0 0

//������� RF_CH
#define RF_CH 0x05
#define CH6 6
#define CH5 5
#define CH4 4
#define CH3 3
#define CH2 2
#define CH1 1
#define CH0 0

//������� RF_SETUP
#define RF_SETUP 0x06
#define CONT_WAVE 7
#define RF_DR_LOW 5
#define PLL_LOCK 4
#define RF_DR_HIGH 3
#define RF_PWR2 2
#define RF_PWR1 1

//������� OBSERVE_TX
#define OBSERVE_TX 0x08
#define PLOS_CNT7 7
#define PLOS_CNT6 6
#define PLOS_CNT5 5
#define PLOS_CNT4 4
#define ARC_CNT3 3
#define ARC_CNT2 2
#define ARC_CNT1 1
#define ARC_CNT0 0

//������� RPD
#define RPD 0x09
#define RPD0 0

//������� RX_ADDR_P0
#define RX_ADDR_P0 0x0A

//������� RX_ADDR_P1
#define RX_ADDR_P1 0x0B

//������� RX_ADDR_P2
#define RX_ADDR_P2 0x0C

//������� RX_ADDR_P3
#define RX_ADDR_P3 0x0D

//������� RX_ADDR_P4
#define RX_ADDR_P4 0x0E

//������� RX_ADDR_P5
#define RX_ADDR_P5 0x0F

//������� TX_ADDR
#define TX_ADDR 0x10

//������� RX_PW_P0
#define RX_PW_P0 0x11
#define RX_PW_P0_5 5
#define RX_PW_P0_4 4
#define RX_PW_P0_3 3
#define RX_PW_P0_2 2
#define RX_PW_P0_1 1
#define RX_PW_P0_0 0

//������� RX_PW_P1
#define RX_PW_P1 0x12
#define RX_PW_P1_5 5
#define RX_PW_P1_4 4
#define RX_PW_P1_3 3
#define RX_PW_P1_2 2
#define RX_PW_P1_1 1
#define RX_PW_P1_0 0

//������� RX_PW_P2
#define RX_PW_P2 0x13
#define RX_PW_P2_5 5
#define RX_PW_P2_4 4
#define RX_PW_P2_3 3
#define RX_PW_P2_2 2
#define RX_PW_P2_1 1
#define RX_PW_P2_0 0

//������� RX_PW_P3
#define RX_PW_P3 0x14
#define RX_PW_P3_5 5
#define RX_PW_P3_4 4
#define RX_PW_P3_3 3
#define RX_PW_P3_2 2
#define RX_PW_P3_1 1
#define RX_PW_P3_0 0

//������� RX_PW_P4
#define RX_PW_P4 0x15
#define RX_PW_P4_5 5
#define RX_PW_P4_4 4
#define RX_PW_P4_3 3
#define RX_PW_P4_2 2
#define RX_PW_P4_1 1
#define RX_PW_P4_0 0

//������� RX_PW_P5
#define RX_PW_P5 0x16
#define RX_PW_P5_5 5
#define RX_PW_P5_4 4
#define RX_PW_P5_3 3
#define RX_PW_P5_2 2
#define RX_PW_P5_1 1
#define RX_PW_P5_0 0

//������� FIFO_STATUS
#define FIFO_STATUS 0x17
#define TX_REUSE 6
#define TX_FULL 5
#define TX_EMPTY 4
#define RX_FULL 1
#define RX_EMPTY 0

//������� DYNPD
#define DYNPD 0x1C
#define DPL_P5 5
#define DPL_P4 4
#define DPL_P3 3
#define DPL_P2 2
#define DPL_P1 1
#define DPL_P0 0

//������� FEATURE
#define FEATURE 0x1D
#define EN_DPL 2
#define EN_ACK_PAY 1
#define EN_DYN_ACK 0

#endif