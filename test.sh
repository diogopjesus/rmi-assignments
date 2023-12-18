#!/bin/bash

set -e

challenge="0"
outfile="solution"
failed_dir="failed_runs"
failed_suffix=".failed$(date +%s%N)"
tmpfile=".tmp$(date +%s%N)"
scofile="scores.txt"

mkdir -p $failed_dir

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
        if [ ! -f "$outfile.path" ]; then
            echo "Error: $outfile.path file not found!"
            exit 1
        fi
        
        # get best case
        gawk -f ../simulator/mapping_score.awk ../simulator/planning.out ../simulator/planning.out > $tmpfile
        while IFS= read -r line
        do
            best_case=$line
        done < $tmpfile

        # check if best case matches the pattern
        pattern="mapping score:[0-9]+"
        if [[ ! "$best_case" =~ $pattern ]]; then
            echo "Error: best case not found!"
            exit 1
        fi
        best_case=$(echo $best_case | sed -e "s/mapping score://gi")
        rm $tmpfile

        # test map
        echo "TEST 1: testing generated map..."
        gawk -f ../simulator/mapping_score.awk ../simulator/planning.out $outfile.map > $tmpfile
        while IFS= read -r line
        do
            map_score=$line
        done < $tmpfile

        # get map score
        map_score=$(echo $map_score | sed -e "s/mapping score://gi")
        
        # check if map score matches the best_case
        if [[ "$map_score" != "$best_case" ]]; then
            echo "TEST 1: failed to obtain best case!"
            cp $outfile.map $failed_dir/$outfile.map$failed_suffix
        else
            echo "TEST 1: passed!"
        fi
        rm $tmpfile

        # test path
        echo "TEST 2: testing generated path..."
        gawk -f ../simulator/planning_score.awk ../simulator/planning.out $outfile.path > $tmpfile || true

        while IFS= read -r line
        do
            path_score=$line
        done < $tmpfile

        # get path score
        path_score=$(echo $path_score | sed -e "s/planning score: //gi")

        # check if path score matches the best_case
        if [[ "$path_score" != "1" ]]; then
            echo "TEST 2: failed to obtain best case!"
            cp $outfile.path $failed_dir/$outfile.path$failed_suffix
        else
            echo "TEST 2: passed!"
        fi
        rm $tmpfile

        # store scores
        echo "$map_score $path_score" >> $scofile
        ;;
esac
