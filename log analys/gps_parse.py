f = open('server_log.txt', "r")

original = f.read()
fix = str()

for i in original:
    if i == "[":
        fix = fix + '\n'
    else:
        fix = fix + str(i)
original = fix
original = original.split('\n')

for i in range(0, len(original)):
    if "incorect output" in original[i]:
        original[i] = ''
        original[i+1] = ''
        original[i+2] = ''

fixed = list()

for i in range(0, len(original)):
    if original[i] != '' and "received message: no signal    None" not in original[i] :
        fixed.append('[' + original[i])
        



fin = open('clear_server_log.txt', 'w')
for i in range(0, len(fixed)):
    fixed[i] = fixed[i][:9] + fixed[i][13:]
    fin.write(fixed[i][1:12] + fixed[i][13:] + '\n')

fin.close()
fin = open('clear_server_log.txt', 'r')

hex_udp = open('hex_from_udp.hex', 'w')

for_hex = fin.read().split('\n')
for i in range(0, len(for_hex)):
    if "dcba" in for_hex[i]:
        hex_udp.write("a4b3" + for_hex[i][for_hex[i].find("dcba"):])

gps_udp = open('gps_from_udp.csv', 'w')
gps_udp.write("id,lat,long\n")
for i in range(0, len(for_hex)):
    
    if "no signal" not in for_hex[i]:
        cords = for_hex[i][for_hex[i].find("(")+1:for_hex[i].find(")")]
        to_write = f"{340+ i}, {cords}\n"
        # gps_udp.write(for_hex[i][for_hex[i].find("(")+1:for_hex[i].find(")")] + ", 0.000" + "\n") 
        gps_udp.write(to_write)


clear_log_time = open('clear_server_log.txt', 'r')
clear_time = clear_log_time.read().split('\n')

avg_ping = int()
for i in range(0, len(clear_time)-2):
    avg_ping += int(clear_time[i+1][6:11]) - int(clear_time[i][6:11])
avg_ping = avg_ping / len(clear_time)
print(avg_ping*10)
# print(clear_time[2][6:11])

bad_log = open('server_log.txt', 'r')
log_new_line = bad_log.read()
fixx = str()

for i in log_new_line:
    if i == "[":
        fixx = fixx + '\n'
    else:
        fixx = fixx + str(i)

fixx = fixx.split('\n')
new = list()

for i in range(1, len(fixx)):
    if "08" in fixx[i]:
        new.append(str(int(fixx[i][:2]) + 3) + fixx[i][2:] + '\n')
    else:
        new.append(fixx[i])
for i in range(1, len(new)):
    print(new[i])