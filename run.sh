#!/bin/bash

if [[ $# -eq 0 ]]; then
    echo "Usage: $0 <nb_runs>"
    exit 1
fi

re='^[0-9]+$'
if ! [[ $1 =~ $re ]]; then
    echo "First argument should be an integer (number of runs)"
    exit 2
fi

for i in $(seq 1 $1)
do
    label=$(date +%h-%d-%H%M%S-%N)
    echo "Run $i / $1 ($label)..."
    time python main.py $label 2> benchmark/logs/$label.log 1> benchmark/logs/$label-rave.log
    echo ""  # manucure ;p
    sleep 1
done
