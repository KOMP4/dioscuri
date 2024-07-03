import subprocess
import _testimportmultiple
from socket import *
from time import *
import datetime
 
import stm_uart
import gps

#-----------------UART SETTINGS------------------#
GPS_UART_PORT = "/dev/ttyS2"
STM_UART_PORT = "/dev/ttyS4"

#-----------------UDP SETTINGS------------------#
host = '91.240.218.98'
port = 22222

if __name__ == "__main__":

    addr = (host,port)
    udp_socket = socket(AF_INET, SOCK_DGRAM)

    start_time = str(datetime.datetime.now().time())
    # start_time_fix = str()
    # for i in start_time:
    #     if i == ':':
    #         start_time_fix = start_time_fix + ','
    #     else:
    #         start_time_fix = start_time_fix + str(i)
    # start_time = start_time_fix
    # #print(f"log_from_{str(start_time_fix)[0:8]}.txt")


    gps.check_state(GPS_UART_PORT)
    log = open(f"log_from_{str(start_time)[0:8]}.txt", "w")
    while True:
        
        log = open(f"log_from_{str(start_time)[0:8]}.txt", "a")


        data = str(gps.get_coords(GPS_UART_PORT)) + "    " + str(stm_uart.get_hex(STM_UART_PORT))
        print(data)
        log.write(f"[{datetime.datetime.now().time()}]: {data} \n")
        udp_socket.sendto(str(data).encode() , addr)
        log.close()
        #sleep(0.5)