#!/bin/bash

challenge="0"
outfile="solution"

while getopts "c:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
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

if ! command -v gawk >/dev/null 2>&1 ; then
    echo "Error: gawk not found on system!"
    return 1
fi

case $challenge in
    1)
        ;;
    2)
        # check if outfile exists
        if [ ! -f $outfile ]; then
            echo 'Error: $outfile not found!'
            return 1
        fi
        gawk -f ../simulator/mapping_score.awk ../simulator/mapping.out $outfile
        ;;
    3)
        # check if outfile exists
        if [ ! -f $outfile ]; then
            echo 'Error: $outfile not found!'
            return 1
        fi
        gawk -f ../simulator/planning_score.awk ../simulator/planning.out $outfile
        ;;
esac
