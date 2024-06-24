import subprocess
import _testimportmultiple
from socket import *
from time import *

import gps

#-----------------UART SETTINGS------------------#
GPS_UART_PORT = "COM7"


#-----------------UDP SETTINGS------------------#
host = '192.168.0.102'
port = 22222


if __name__ == "__main__":

    addr = (host,port)
    udp_socket = socket(AF_INET, SOCK_DGRAM)

    gps.check_state(GPS_UART_PORT)

    while True:

        data = gps.get_coords(GPS_UART_PORT)
        print(data)
        udp_socket.sendto(str(data).encode() , addr)

        sleep(2)