#!/bin/bash

tmpdir=".tmp9233897596"
nmec1=92338
nmec2=97596

if command -v zip >/dev/null 2>&1 ; then
    mkdir -p $tmpdir/agent
    cp build.sh run.sh mainC4.py croblink.py requirements.txt $tmpdir/agent
    cd $tmpdir
    zip -r agent_$nmec1\_$nmec2.zip agent
    mv agent_$nmec1\_$nmec2.zip ../
    cd ..; rm -r $tmpdir
else
    echo "zip not found"
    return 1
fi
