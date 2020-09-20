# import libraries
import time
import os

# global variables
existing_hosh_ip_address = ""
polling_time = 2 # seconds

# monitor hosh ip address
while True:
	time.sleep(polling_time)
	f = open('/etc/hosts', 'r')
	while True:
		line = f.readline()
		if not line: break
		if "hosh" in line:
			cur_hosh_ip = line.split()[0]
			if not cur_hosh_ip == existing_hosh_ip_address:
				# save new hosh ip address
				existing_hosh_ip_address = cur_hosh_ip

				# kill all existing consoles
				command = 'killall bash'
				os.system(command)

				# start wizard consoles
				command = 'bash $HOME/robot-slang/scripts/open-wizard-nodes.sh'
				os.system(command)

