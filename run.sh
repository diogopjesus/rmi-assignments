#!/bin/bash

set -eu

source .env

if ! command -v python3 >/dev/null 2>&1 ; then
    echo "Error: python3 not found on system!" 1>&2
    exit 1
fi

challenge="0"
host="localhost"
robname="pClient"
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
        source $VENV_DIR/bin/activate
        python3 $BIN_DIR/pyMainRob.py -c $challenge -h "$host" -p "$pos" -r "$robname"
        deactivate
        ;;
    2)
        source $VENV_DIR/bin/activate
        python3 $BIN_DIR/pyMainRob.py -c $challenge -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        deactivate
        ;;
    3)
        source $VENV_DIR/bin/activate
        python3 $BIN_DIR/pyMainRob.py -c $challenge -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        deactivate
        ;;
    4)
        $BIN_DIR/mainRob -c $challenge -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        ;;
esac
