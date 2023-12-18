#!/bin/bash

set -e

bindir="bin"
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
    4)
        $bindir/mainRob -c $challenge -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        ;;
esac
