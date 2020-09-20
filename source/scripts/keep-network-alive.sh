#!/bin/bash
IP='128.46.214.183'
PASSWORD='yourpassword'

host_found_message_printed=0
num_net_restarts=0
timeout=8
t1=$(date +"%s")

# loop infinitely
while :
do
	url='http://www.google.com'
	status=$(curl --head --location --connect-timeout 1 --write-out %{http_code} --silent --output /dev/null ${url})

	# check whether we are connected to the internet
	if [[ $status == 200 ]] || [[ $status == 204 ]]
	then
		# restart our counter
		num_net_restarts=0
		# restart our timer
		t1=$(date +"%s")
		# if not printed out yet, indicate that we are connected to internet
		if [ $host_found_message_printed == 0 ]
		then
			echo "Connected to internet."
			host_found_message_printed=1
		fi
	# check whether curl has failed (i.e. internet is down)
	elif [[ $status == 500 ]] || [[ $status == 000 ]]
	then
		# reset flag for print host found
		host_found_message_printed=0
		
		# compute how much time has elapsed since internet has been down (or we restarted networking) 
		t2=$(date +"%s")
		timeelapsed=`expr $t2 - $t1`
		echo "Host not found for" $timeelapsed "seconds"

		# if the timeout has elapsed, restart the networking
		if (( $timeelapsed > $timeout )); then
			echo "Restarting networking because timeout reached"
			nmcli networking off 
			nmcli networking on
			# keep track of how many times we restarted the networking
			num_net_restarts=$((num_net_restarts+1))
			# restart our timer
			t1=$(date +"%s")
		fi

		# if we have restarted the networking 3x, then restart the entire networking manager
		if [[ $num_net_restarts == 3 ]]
		then
			echo "Restarting network manager because it had been reestablished"
			echo $PASSWORD | sudo -S systemctl restart network-manager
			# restart our counter
			num_net_restarts=0
			# restart our timer
			t1=$(date +"%s")
		fi
	# print whatever other http status there is
	else
		echo "Some other http status:" $status
	fi
	sleep 0.5
done

