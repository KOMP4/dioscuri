import subprocess
from socket import *
from time import *




def check_state(port):
    responce1 = subprocess.run(["atcom","--port", f"{port}", "--timeout", "2" , "AT"], stdout=subprocess.PIPE, stderr = subprocess.PIPE, text=True,)
    responce1 = str(responce1.stdout)
    if "OK" in responce1:
        print("module work")
        return 1
    else: 
        print("module not work, response: : " + responce1)
        return 0

def get_coords(port):
    
    text = subprocess.run(["atcom","--port", f"{port}", "--timeout", "2", "AT+CGPSINFO"], stdout=subprocess.PIPE, text=True,)
    text = str(text.stdout)
    text = list(text.split("\n"))
    
    for i in range(0, len(text)):
        if "+CGPSINFO: " in text[i] and "AT" not in text[i]:

            cgps = text[i].strip("+CGPSINFO: ").split(',')

            if cgps[0] == '':
                return "no signal"
            
            lat = float(cgps[0]) / 100
            min_lat = lat % 1
            lat = int(lat) + min_lat / 60 * 100

            log = float(cgps[2]) / 100
            min_log = log % 1
            log = int(log) + min_log / 60 * 100

            return lat, log
    return f"incorect output: {text}"
    #text = text.strip("+CGPSINFO: ").split(',')
    



