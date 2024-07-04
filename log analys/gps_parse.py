f = open('server_log.txt', "r")
original = f.read()

for i in range(0, len(original)):
    print(i)