#from PySide6 import QtWidgets, uic  #подключаем библиотеки для создания программы и для создания интерфейса
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo  #подключаем библиотеку для работы с UART
from PySide6.QtCore import QIODevice  #подключаем библиотеку для работы библиотеки UART

#import socket...

from time import sleep


#-----------------Settings---------------#
UDP_IP = "0.0.0.0"
UDP_PORT = 22222

UART_PORT = 'COM11'
UART_SPEED = 115200


#-----------------UDP initialisation---------------#
#sock = socket.socket(socket.AF_INET, # Internet
#                socket.SOCK_DGRAM) # UDP
#sock.bind((UDP_IP, UDP_PORT))

#-----------------UART------------------#
#функция для открытия выбранного порта
class Uart:

    def __init__(self, port, speed, name):
        self.port = port
        self.speed = speed
        self.name = name

    def mounting(self, port, speed):
        self.name = QSerialPort()
        self.name.setBaudRate(speed)
        self.name.setPortName(port)
        self.name.open(QIODevice.ReadWrite)
        informating(self.port + " - is open")

    def unmounting(self):
        self.name.close()  #закрываем COM порт
        informating(self.port + " - is close")
    def read_from_COM(self):  #функция для чтения принятых данных с COM порта
        rx = self.name.read(1)  #читаем 1 байт в hex виде
        rx_ascii=self.name.hexlify(rx)  #переводим из hex в ascii для правильного отображения
        print(rx_ascii)
        rxs = str(rx_ascii, 'utf-8')
    



#Дублируем информацию во все каналы
def informating(msg):
    msg = str(msg)
    print(msg)
    #sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
    

def module_check(status):
    #пингуем модуль
    print("module_check start")
    gps.write(b"AT\n\r")
    print("AT sended")
    response1 = gps.readline().decode().strip()
    print("get response")
    if response1 == "OK":
        informating("module ready...")
        status = 1
    else:
        informating("module start error: " + response1)
        status = 0
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
