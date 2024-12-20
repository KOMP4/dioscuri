import socket
import datetime



UDP_IP = "0.0.0.0"  
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
    
    start_time = str(datetime.datetime.now().time())
    start_time_fix = str()
    for i in start_time:
        if i == ':':
            start_time_fix = start_time_fix + ','
        else:
            start_time_fix = start_time_fix + str(i)
    start_time = start_time_fix

    log = open(f"server_log_from_{start_time}.txt", "a")


    print("server start")
    while True: 
        log = open(f"server_log_from_{start_time}.txt", "a")
        log.write(get_data())
        log.close()