#!/bin/bash

set -eu

source .env

if ! command -v zip >/dev/null 2>&1 ; then
    echo "Error: zip not found on system!" 1>&2
    exit 1
fi

if ! command -v sed >/dev/null 2>&1 ; then
    echo "Error: sed not found on system!" 1>&2
    exit 1
fi

assignment="0"

while getopts "a:" op
do
    case $op in
        "a")
            assignment=$OPTARG
            ;;
        default)
            echo "ERROR: unknown parameter"
            ;;
    esac
done

shift $(($OPTIND-1))

tmpdir=".tmp$(date +%s%N)"
mkdir -p $tmpdir/agent

case $assignment in
    1)
        cp build.sh \
           run.sh \
           requirements.txt \
           bin/agentC1.py \
           bin/agentC2.py \
           bin/agentC3.py \
           bin/croblink.py \
           bin/pyMainRob.py \
           $tmpdir/agent
        
        cd $tmpdir

        zip -r agent_$NMEC1\_$NMEC2.zip agent
        mv agent_$NMEC1\_$NMEC2.zip ../
        cd ..; rm -r $tmpdir
        
        ;;
    2)
        cp -r build.sh \
            run.sh \
            CMakeLists.txt \
            include/ \
            src/ \
            $tmpdir/agent

        cd $tmpdir

        # remove test references in CMakeLists.txt
        sed -i '/add_subdirectory(test)/d' agent/CMakeLists.txt

        zip -r agent_$NMEC1\_$NMEC2.zip agent
        mv agent_$NMEC1\_$NMEC2.zip ../
        cd ..; rm -r $tmpdir

        ;;
esac
