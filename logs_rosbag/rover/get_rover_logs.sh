#!/bin/bash



# if [[ $1 == "-h" ]]; then 
#     echo "./get_rover_logs.sh [log_name] -t [topic1] [topic2]";
# else
topic_names=("/rover_pose" "/dock_pose" "/cmd_vel")
filename=""
while getopts "hf:t:" opt; do
    case $opt in
        h)
        echo "YEAH";
        ;;
        f)
        echo "oh no";
        filename="$OPTARG"
        ;;
        t) 
        echo "Yeah";
        topic_names+=("$OPTARG")
        ;;
        \?)
        echo "test"
        ;;
    esac
    shift
done
scp aion@10.0.1.128:guerlelogs/$filename ./

echo $filename
command="rosbags-convert $filename";
for value in "${topic_names[@]}"; do
    command="$command --include-topic ${value}"
done

echo "Running : $command"
eval "${command}"
echo "Sauvegarde et conversion du fichier réussies!"



