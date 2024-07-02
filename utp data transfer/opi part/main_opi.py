import subprocess
import _testimportmultiple
from socket import *
from time import *
import datetime
 

import gps

#-----------------UART SETTINGS------------------#
GPS_UART_PORT = "/dev/ttyS2"


#-----------------UDP SETTINGS------------------#
host = '91.240.218.98'
port = 22222

if __name__ == "__main__":

    addr = (host,port)
    udp_socket = socket(AF_INET, SOCK_DGRAM)

    start_time = datetime.datetime.now().time()

    gps.check_state(GPS_UART_PORT)

    while True:
        
        log = open(f"log_from_{start_time}", "a")

        data = gps.get_coords(GPS_UART_PORT)
        
        print(data)
        log.write(f"[{datetime.datetime.now().time()}]: {data} \n")
        udp_socket.sendto(str(data).encode() , addr)
        log.close()
        sleep(2)