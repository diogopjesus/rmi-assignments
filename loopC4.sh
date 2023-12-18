#!/bin/bash

ARGS="--param ../Labs/rmi-2324/C4-config.xml"
ARGS+=" --lab ../Labs/rmi-2324/C4-lab.xml"
ARGS+=" --grid ../Labs/rmi-2324/C4-grid.xml"
ARGS+=" --scoring 6"

NUM_ITER=100

ENABLE_VIEWER=1

set -e

rm -r failed_runs >/dev/null 2>&1 || true 
rm scores.txt >/dev/null 2>&1 || true

for iter in $(seq 1 $NUM_ITER)
do
    echo "Run $iter:"

    (cd ../simulator; ./simulator $ARGS) >/dev/null 2>&1 &
    sleep 1

    if [ "$ENABLE_VIEWER" != 0 ]; then
        (cd ../Viewer; ./Viewer --autoconnect) >/dev/null 2>&1 &
        sleep 1
    fi
    
    ./run.sh -c4

    killall simulator >/dev/null 2>&1 || true
    killall Viewer >/dev/null 2>&1 || true

    ./test.sh -c4

    echo;
done

# calculate the average score for both map and path
map_score=0
path_score=0

while IFS= read -r line
do
    score=$(echo $line | cut -d' ' -f1)
    map_score=$(echo "scale=2;$map_score + $score" | bc)
    score=$(echo $line | cut -d' ' -f2)
    path_score=$(echo "scale=2;$path_score + $score" | bc)
done < scores.txt

map_score=$(echo "scale=2;$map_score / $NUM_ITER" | bc -l)
path_score=$(echo "scale=2;$path_score / $NUM_ITER" | bc -l)

echo "Average map score: $map_score"
echo "Average path score: $path_score"
