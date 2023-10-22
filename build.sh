#!/bin/bash

envname=".venv"

if [ -d $envname ]; then
    read -p ".venv already exists. Replace directory? [y/N]: " -r
    if [[ ! $REPLY =~ ^[Yy]$ ]]
    then
        [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1
    fi
fi

echo "Creating and activating virtual environment"
python -m venv $envname --prompt="ass1-env"
source $envname/bin/activate

echo "Install requirements"
python -m pip install --upgrade pip > /dev/null
python -m pip install -r requirements.txt > /dev/null

echo "Exiting virtual environment"
deactivate

