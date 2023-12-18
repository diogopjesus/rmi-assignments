#!/bin/bash

set -eu

source .env

if ! command -v gawk >/dev/null 2>&1 ; then
    echo "Error: gawk not found on system!" 1>&2
    exit 1
fi

mappingScore() {
    m_simulator_outfile=$1
    m_outfile=$2

    m_tmpfile=".tmp$(date +%s%N)"

    gawk -f $MAPPING_SCORE_SCRIPT $m_simulator_outfile $m_outfile > $m_tmpfile
    
    m_map_score=$(tail -n 1 $m_tmpfile) # last line
    m_map_score=$(echo $m_map_score | sed -e "s/mapping score://gi") # isolate score
    
    rm $m_tmpfile

    echo $m_map_score
}

planningScore() {
    m_simulator_outfile=$1
    m_outfile=$2

    m_tmpfile=".tmp$(date +%s%N)"

    gawk -f $PLANNING_SCORE_SCRIPT $m_simulator_outfile $m_outfile > $m_tmpfile
    
    m_path_score=$(tail -n 1 $m_tmpfile) # last line
    m_path_score=$(echo $m_path_score | sed -e "s/planning score: //gi") # isolate score
    
    rm $m_tmpfile

    echo $m_path_score
}

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

case $challenge in
    2)
        # check if mapfile exists
        if [ ! -f "$outfile.map" ]; then
            echo "Error: $outfile.map file not found!"
            exit 1
        fi

        echo "TEST: testing generated map..."
        
        best_map_score=$(mappingScore $SIMULATOR_MAPPING_OUTFILE $SIMULATOR_MAPPING_OUTFILE) 

        map_score=$(mappingScore $SIMULATOR_MAPPING_OUTFILE $outfile.map)

        # check if map score matches the best_case
        if [[ "$map_score" != "$best_map_score" ]]; then
            echo "TEST: failed to obtain exact map!"
            echo "TEST: map score is $map_score"
        else
            echo "TEST: passed!"
        fi
        ;;
    
    3)
        # check if pathfile exists
        if [ ! -f "$outfile.path" ]; then
            echo "Error: $outfile.path file not found!"
            exit 1
        fi

        echo "TEST: testing generated path..."
        
        best_path_score="1"
        
        path_score=$(planningScore $SIMULATOR_PLANNING_OUTFILE "$outfile.path")

        if [[ $path_score != $best_path_score ]]; then
            echo "TEST: failed to obtain best path!"
            echo "TEST: path score is $path_score"
        else
            echo "TEST: passed!"
        fi
        ;;

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

        # suffix for failed files (to identify map and path files of the same run)
        failname="fail_$(date +%s)"

        echo "TEST 1: testing generated map..."
        
        best_map_score=$(mappingScore $SIMULATOR_PLANNING_OUTFILE $SIMULATOR_PLANNING_OUTFILE) # use planning file to get best case
        
        map_score=$(mappingScore $SIMULATOR_PLANNING_OUTFILE "$outfile.map")

        # check if map score matches the best_case
        if [[ "$map_score" != "$best_map_score" ]]; then
            echo "TEST 1: failed to obtain exact map!"
            echo "TEST: map score is $map_score"
            
            # store failed map
            mkdir -p "$FAILED_DIR"
            cp "$outfile.map" "$FAILED_DIR/$failname.map"
        else
            echo "TEST 1: passed!"
        fi

        echo "TEST 2: testing generated path..."

        best_path_score="1"

        path_score=$(planningScore $SIMULATOR_PLANNING_OUTFILE "$outfile.path")

        # check if path score matches the best_case
        if [[ $path_score != $best_path_score ]]; then
            echo "TEST 2: failed to obtain best path!"
            echo "TEST: path score is $path_score"

            # store failed path
            mkdir -p "$FAILED_DIR"
            cp "$outfile.path" "$FAILED_DIR/$failname.path"
        else
            echo "TEST 2: passed!"
        fi

        # store both scores
        echo "$map_score $path_score" >> $SCORES_FILE
        ;;
esac
