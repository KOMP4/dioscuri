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
    fin.write(fixed[i][1:9] + fixed[i][13:] + '\n')

fin.close()
fin = open('clear_server_log.txt', 'r')

hex_udp = open('hex_from_udp.hex', 'w')

for_hex = fin.read().split('\n')
for i in range(0, len(for_hex)):
    if "dcba" in for_hex[i]:
        hex_udp.write("a4b3" + for_hex[i][for_hex[i].find("dcba"):])

gps_udp = open('gps_from_udp.txt', 'w')
for i in range(0, len(for_hex)):
    if "no signal" not in for_hex[i]:
        cords = for_hex[i][for_hex[i].find("(")+1:for_hex[i].find(")")] + ", 0.000"
        to_write = f"""<Placemark id="{i+100}">""" + "\n" + f"<name>{i}</name>" + "\n" + "<styleUrl>#__managed_style_05AF603B4A31B35A2E60</styleUrl>" + f"""<Point>
			    <extrude>1</extrude>
			    <coordinates>{cords}</coordinates>
		</Point>""" + "\n" + "</Placemark>\n\n";
        # gps_udp.write(for_hex[i][for_hex[i].find("(")+1:for_hex[i].find(")")] + ", 0.000" + "\n") 
        gps_udp.write(to_write)


    # <Placemark id="ffd">
	# 	<name>56°24&apos;42.1&quot;N 40°58&apos;31.4&quot;E</name>
	# 	<styleUrl>#__managed_style_05AF603B4A31B35A2E60</styleUrl>
		# <Point>
		# 	<extrude>1</extrude>
		# 	<coordinates>40.9753962,56.411689,0</coordinates>
		# </Point>
	# </Placemark>