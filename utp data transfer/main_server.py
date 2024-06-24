import socket
import datetime



UDP_IP = "192.168.0.102"  
UDP_PORT = 22222

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

def get_data():
    current_time = datetime.datetime.now().time()
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print(f"[{current_time}] received message: %s" %data.decode())
    return f"[{current_time}] received message: %s" %data.decode()

if __name__ == "__main__":

    while True: 
        get_data()