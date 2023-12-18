#!/bin/bash

set -e

source .env

run() {
    challenge=$1
    outfile=$2

    args="SIMULATOR_ARGS_C"$challenge
    args=${!args}
    echo "Running challenge $challenge with args: $args"

    (cd ../simulator; ./simulator $args) >/dev/null 2>&1 &
    sleep 1

    if [ "$ENABLE_VIEWER" != 0 ]; then
        (cd ../Viewer; ./Viewer --autoconnect) >/dev/null 2>&1 &
        sleep 1
    fi
    
    ./run.sh -c$challenge -f $outfile >/dev/null

    killall simulator >/dev/null 2>&1 || true
    killall Viewer >/dev/null 2>&1 || true
}

silent_mode=0 # whether to run in silent mode

while getopts "c:s" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "s")
            silent_mode=1
            ;;
        default)
            echo "ERROR: unknown parameter"
            ;;
    esac
done

shift $(($OPTIND-1))

outfile="solution"

case $challenge in
    1)
        for n in $(seq 1 $NUM_RUNS)
        do
            if [ "$silent_mode" = 0 ]; then
                run $challenge $outfile >/dev/null 2>&1
            else
                echo "Run $n:"
                run $challenge $outfile
            fi
        done
        ;;
    2|3)
        for n in $(seq 1 $NUM_RUNS)
        do
            if [ "$silent_mode" = 0 ]; then
                run $challenge $outfile >/dev/null 2>&1
                ./test.sh -c$challenge -f$outfile >/dev/null 2>&1
            else
                echo "Run $n:"
                run $challenge $outfile
                ./test.sh -c$challenge -f$outfile
            fi
        done
        ;;
    4)
        # remove old scores and failed runs
        rm $SCORES_FILE >/dev/null 2>&1 || true
        rm -r $FAILED_DIR >/dev/null 2>&1 || true 

        for n in $(seq 1 $NUM_RUNS)
        do
            if [ "$silent_mode" = 0 ]; then
                run $challenge $outfile >/dev/null 2>&1
                ./test.sh -c$challenge -f$outfile >/dev/null 2>&1
            else
                echo "Run $n:"
                run $challenge $outfile
                ./test.sh -c$challenge -f$outfile
            fi
        done

        # calculate the average score for both map and path
        total_map_score=0
        total_path_score=0

        while IFS= read -r line
        do
            score=$(echo $line | cut -d' ' -f1)
            total_map_score=$(echo "scale=2;$total_map_score + $score" | bc)
            
            score=$(echo $line | cut -d' ' -f2)
            total_path_score=$(echo "scale=2;$total_path_score + $score" | bc)
        done < scores.txt

        map_score=$(echo "scale=2;$map_score / $NUM_ITER" | bc -l)
        path_score=$(echo "scale=2;$path_score / $NUM_ITER" | bc -l)

        echo "Average map score: $map_score"
        echo "Average path score: $path_score"
        ;;
esac
