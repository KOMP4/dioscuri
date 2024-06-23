import subprocess
import _testimportmultiple
from socket import *
from time import *


GPS_UART_PORT = "/dev/ttyS0"

host = '192.168.0.102'
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
    text = subprocess.run(["atcom","--port", "/dev/ttyS0", "AT+CGPSINFO"], stdout=subprocess.PIPE, text=True,)
    text = str(text.stdout)
    text = list(text.split("\n"))
    for i in range(0, len(text)):
        if "+CGPSINFO: " in text[i]:
            cgps = text[i].strip("+CGPSINFO: ").split(',')

            lat = float(cgps[0]) / 100
            min_lat = lat % 1
            lat = int(lat) + min_lat / 60 * 100

            log = float(cgps[2]) / 100
            min_log = log % 1
            log = int(log) + min_log / 60 * 100

            return lat, log
    return text
    #text = text.strip("+CGPSINFO: ").split(',')
    


if __name__ == "__main__":

    udp_socket = socket(AF_INET, SOCK_DGRAM)
    
    while True:
        data = get_data(GPS_UART_PORT)
        print(data)
        #coord = int(data[])

        udp_socket.sendto(str(data).encode() , addr)
        #sleep(2)
        #udp_socket.sendto(f"time from start: {round(time()*10000000)}".encode() , addr)
        sleep(2)

    print(get_data(GPS_UART_PORT))
