import subprocess
from time import sleep
from socket import *


GPS_UART_PORT = "/dev/ttyS0"

host = '91.240.218.98'
port = 22222
addr = (host,port)


def check_state(port):
    responce1 = subprocess.run(["atcom","-p", f"{GPS_UART_PORT}", "AT"], stdout=subprocess.PIPE, stderr = subprocess.PIPE, text=True,)
    responce1 = str(responce1)
    if responce1 == "OK":
        print("module work, responce: " + responce1)
        return 1
    else: 
        print("module not work, response: : " + responce1)
        return 0

def get_data(port):
    text = subprocess.run(["atcom","--port", f"{GPS_UART_PORT}", "AT+CGPSINFO"], stdout=subprocess.PIPE, text=True,)
    text = str(text.stdout)
    #text = text.split("\n")[3]
    #text = text.strip("+CGPSINFO: ").split(',')
    return text


if __name__ == "__main__":

    udp_socket = socket(AF_INET, SOCK_DGRAM)
    
    while True:
        data = get_data(GPS_UART_PORT)
        print(data)
        #coord = int(data[])

        udp_socket.sendto(str(data).encode() , addr)
        sleep(2)

    print(get_data(GPS_UART_PORT))
