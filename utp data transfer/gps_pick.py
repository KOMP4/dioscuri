import serial
import socket
from time import sleep


###Settings
UDP_IP = "0.0.0.0"
UDP_PORT = 22222
UART_PORT = 'COM7'

#-----------------UDP initialisation---------------#
#sock = socket.socket(socket.AF_INET, # Internet
#                socket.SOCK_DGRAM) # UDP
#sock.bind((UDP_IP, UDP_PORT))

#-----------------UART initialisation--------------#
gps = serial.Serial(
    port=UART_PORT,
    baudrate=115200,
    #parity=serial.PARITY_ODD,
    #stopbits=serial.STOPBITS_TWO,
    #bytesize=serial.SEVENBITS
)
sleep(2)#без этого уарт не робит!!!!

#Дублируем инфу во все каналы
def informating(msg):
    msg = str(msg)
    print(msg)
    #sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
    

def module_check(status):
    #пингуем модуль
    gps.write(b"AT\n\r")
    response1 = gps.readline().decode().strip()
    if response1 == "OK":
        informating("module ready...")
        status = 1
    else:
        informating("module start error" + response1)
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

status = 0
module_check(status)
gps_on(status)
if status:
    informating("gps start...")
    switch = 1
    while switch:
        informating(f"location: {gps_info()}")
        sleep(2)
else:
    informating("error")

#sock.close()
gps.close()