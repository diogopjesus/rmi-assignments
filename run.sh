#!/bin/bash

envname=".venv"
challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

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
        python3 mainC2.py -h "$host" -p "$pos" -r "$robname"
        # mv your_mapfile $outfile.map # if needed
        deactivate
        ;;
    3)
        source $envname/bin/activate
        python3 mainC3.py -h "$host" -p "$pos" -r "$robname"
        # mv your_mapfile $outfile.path # if needed
        deactivate
        ;;
esac

