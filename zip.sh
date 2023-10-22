#!/bin/bash

if command -v zip >/dev/null 2>&1 ; then
    mkdir -p .tmp/agent
    cp build.sh run.sh mainC1.py mainC2.py mainC3.py croblink.py requirements.txt .tmp/agent
    cd .tmp
    zip -r agent_92338_97596.zip agent
    mv agent_92338_97596.zip ../
    cd ..; rm -r .tmp
else
    echo "zip not found"
    return 1
fi
