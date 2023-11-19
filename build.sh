#!/bin/bash

set -e

envname=".venv"

if ! command -v python3 >/dev/null 2>&1 ; then
    echo "Error: python3 not found on system!"
    exit 1
fi

if [ ! -d $envname ]; then
    echo "Creating and activating virtual environment"
    python3 -m venv $envname --prompt="ass2-env"
    if [ $? -ne  0 ]; then
        echo "Error: Could not create virtual environment!"
        exit 1
    fi    
fi

source $envname/bin/activate

echo "Install requirements"
python3 -m pip install -r requirements.txt > /dev/null

echo "Exiting virtual environment"
deactivate

