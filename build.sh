#!/bin/bash

envname=".venv"

if ! command -v python3 >/dev/null 2>&1 ; then
    echo "Error: python3 not found on system!"
    return 1
fi

if [ -d $envname ]; then
    read -p ".venv already exists. Replace directory? [y/N]: " -r
    if [[ ! $REPLY =~ ^[Yy]$ ]]
    then
        [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1
    fi
fi


echo "Creating and activating virtual environment"
python3 -m venv $envname --prompt="ass1-env"
if [ $? -ne  0 ]; then
    echo "Error: Could not create virtual environment!"
    exit 1
fi

source $envname/bin/activate

echo "Install requirements"
python3 -m pip install --upgrade pip > /dev/null
python3 -m pip install -r requirements.txt > /dev/null

echo "Exiting virtual environment"
deactivate

