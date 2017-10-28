#!/usr/bin/env bash

if [ $# -ne "3" ]
then
    echo "$0 <num-jobs> <config-file> <output-prefix>"
    exit
fi

let job_count=0

for i in `seq 1 100`
do
    if [ $job_count = $1 ]
    then
        wait -n
        let job_count--
    fi
    argos3 -c $2 2>/dev/null | perl -pe 's/\e\[?.*?[\@-~]//g' | grep "^beacon reached" | cut -d ":" -f 2 >> $3.dat &
    let job_count++
    echo -n '.'
done

echo

wait
