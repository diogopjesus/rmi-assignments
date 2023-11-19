#!/bin/bash

set -e

envname=".venv"
challenge="0"
host="localhost"
robname="pClient"
pos="0"
outfile="solution"

if ! command -v python3 >/dev/null 2>&1 ; then
    echo "Error: python3 not found on system!"
    exit 1
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
    4)
        source $envname/bin/activate
        python3 mainC4.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        deactivate
        ;;
esac
