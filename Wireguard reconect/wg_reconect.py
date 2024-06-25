from time import *
import subprocess

def ping(adr):
   responce = subprocess.run(["ping", "-c", "4", "-w", "5" , f"{adr}"], stdout=subprocess.PIPE, stderr = subprocess.PIPE, text=True)
   #print(responce.stdout)
   if f"64 bytes from {adr}" in str(responce.stdout):
      return 1
   else:
      return 0



if __name__ == "__main__":
    sleep(60)
    tryes = 1
    while(tryes <= 20):
    #Test Ping Sucessful
        if ping('8.8.8.8'):
            print('Google ping sucessful!')
            proc = subprocess.Popen(['sudo','wg-quick','up', 'wg0'])
        
        #sleep(5)
            if ping('10.66.66.1'):
                print("ALL done")
                exit()
            else:
                print("Fail to connect Wg server")
                tryes = tryes + 1
        else:
            tryes = tryes + 1
    print("After 20 tryes nothing work :(")