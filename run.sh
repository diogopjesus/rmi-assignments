#!/bin/bash

envname=".venv"
challenge="0"
host="localhost"
robname="pClient"
pos="0"
outfile="solution"

if ! command -v python3 >/dev/null 2>&1 ; then
    echo "Error: python3 not found on system!"
    return 1
fi

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR: unknown parameter"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        source $envname/bin/activate
        python3 mainC1.py -h "$host" -p "$pos" -r "$robname"
        deactivate
        ;;
    2)
        source $envname/bin/activate
        python3 mainC2.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv solution.map $outfile.map 2> /dev/null || true # if needed
        deactivate
        ;;
    3)
        source $envname/bin/activate
        python3 mainC3.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        mv solution.path $outfile.path 2> /dev/null || true # if needed
        deactivate
        ;;
esac
