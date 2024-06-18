#from PySide6 import QtWidgets, uic  #подключаем библиотеки для создания программы и для создания интерфейса
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo  #подключаем библиотеку для работы с UART
from PySide6.QtCore import QIODevice  #подключаем библиотеку для работы библиотеки UART

#import socket...

from time import sleep
import binascii

#-----------------Settings---------------#
UDP_IP = "0.0.0.0"
UDP_PORT = 22222

GPS_UART_PORT = 'COM11'
GPS_UART_SPEED = 115200


#-----------------UDP-------------------#
#sock = socket.socket(socket.AF_INET, # Internet
#                socket.SOCK_DGRAM) # UDP
#sock.bind((UDP_IP, UDP_PORT))

#-----------------UART------------------#
uart = QSerialPort()

def uart_mounting(port, speed):
    uart.setBaudRate(speed)
    uart.setPortName(port)
    uart.open(QIODevice.ReadWrite)
    informating(port + " - is open")

def uart_unmounting(port):
    uart.close()  #закрываем COM порт
    informating(port  + " - is close")

def uart_read():  #функция для чтения принятых данных с COM порта
    rx = uart.read(1)  #читаем 1 байт в hex виде
    rx_ascii = binascii.hexlify(rx)  #переводим из hex в ascii для правильного отображения
    rxs = str(rx_ascii, 'utf-8')
    informating("<< " + rxs)
    return rxs
def uart_send(msg):
    informating(">> " + msg)
    uart.write(msg)




#Дублируем информацию во все каналы
def informating(msg):
    msg = str(msg)
    print(msg)
    #sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
    
#-----------------GPS------------------#
def gps_init():
    uart_mounting(GPS_UART_PORT, GPS_UART_SPEED)

def gps_check():
    #пингуем модуль
    informating("module check start")
    gps.write(b"AT\n\r")
    informating("AT sended")
    response1 = gps.readline().decode().strip()
    informating("get response")
    if response1 == "OK":
        informating("module ready...")
        return 1
    else:
        informating("module start error: " + response1)
        return 0
def gps_on(status):
    #Включаем gps
    gps.write(b"AT+CGPS=1\n\r")
    #Спрашиваем состояние
    gps.write(b"AT+CGPS?\n\r")
    response2 = gps.readline().decode().strip()
    if response2 == "+CGPS: 1,1":
        informating("gps ready")
        status = 1
    else:
        informating("gps not started, error: " + response2)
        status = 0


def gps_info():
    gps.write(b"AT+CGPSINFO\n\r")
    return gps.readline().decode()
    ## не доделано

if __name__ == "__main__":
    pass





# gps_on(status)
# if status:
#     informating("gps start...")
#     switch = 1
#     while switch:
#         informating(f"location: {gps_info()}")
#         sleep(2)
# else:
#     informating("error")

#sock.close()