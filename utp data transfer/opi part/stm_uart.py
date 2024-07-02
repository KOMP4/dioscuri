import codecs
import serial


#STM_UART_PORT = "COM5"


def get_hex(port):    
    ser = serial.Serial(port, 115200)
    if 'a4b3' in ser.read(2).hex():
        print("packet start")
        hex_values = ser.read(27).hex()
        print(hex_values)
        #outp = f"ID bytes: {hex_values[0:4]}"   
        #outp = outp + f"    Time from stm = {hex_values[3:4]}"
        return hex_values

# if __name__ == "__main__":
#     while True:
#         print(get_stm())
