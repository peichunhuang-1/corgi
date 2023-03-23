# /bin/bash
trap "kill -- -$$" INT # add so ctrl c can kill all process
echo Master IP:        # print user guide to input master ip
read master_ip         # read master ip
echo Master port:      # print user guide to input master port
read master_port       # read master port
echo Local IP:         # print user guide to input local ip
read local_ip          # read local ip
echo path:             # print user guide to input path of executable files
read path              # read path
NodeCore -m "$master_ip" -l "$local_ip" -p "$master_port" & 
sleep .1   

./"$path"/cpg -m "$master_ip" -l "$local_ip" -p "$master_port" -f 500 & 
sleep .1

./"$path"/odometry -m "$master_ip" -l "$local_ip" -p "$master_port" -f 500 & 
sleep .1

./"$path"/force -m "$master_ip" -l "$local_ip" -p "$master_port" -f 500 & 
sleep .1

./"$path"/foothold -m "$master_ip" -l "$local_ip" -p "$master_port" -f 500 & 
sleep .1

./"$path"/legs -m "$master_ip" -l "$local_ip" -p "$master_port" -f 500 & 
sleep .1

wait