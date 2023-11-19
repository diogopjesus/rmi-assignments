#!/bin/bash

set -e

challenge="0"
outfile="solution"

if ! command -v gawk >/dev/null 2>&1 ; then
    echo "Error: gawk not found on system!"
    exit 1
fi

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

case $challenge in
    4)
        # check if mapfile exists
        if [ ! -f "$outfile.map" ]; then
            echo "Error: $outfile.map file not found!"
            exit 1
        fi

        # check if pathfile exists
        if [ ! -f "outfile.path" ]; then
            echo "Error: $outfile.path file not found!"
            exit 1
        fi
        
        # test map
        # TODO: compare results with the best case
        #       best case obtained by -> gawk -f ../simulator/planning_score.awk  ../simulator/planning.out ../simulator/planning.out
        echo "TEST 1: testing generated map..."
        gawk -f ../simulator/planning_score.awk ../simulator/planning.out "$outfile.map"

        # TODO: compare results with the best case
        #       best case obtained is 1 (check if true)
        echo -e "\nTEST 2: testing generated path..."
        gawk -f ../simulator/mapping_score.awk ../simulator/planning.out "$outfile.path"
        ;;
esac
