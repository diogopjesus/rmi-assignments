#!/bin/bash

set -e

source .env

if ! command -v cmake >/dev/null 2>&1 ; then
    echo "Error: cmake not found on system!" 1>&2
    exit 1
fi

if ! command -v python3 >/dev/null 2>&1 ; then
    echo "Error: python3 not found on system!" 1>&2
    exit 1
fi

python3 -c "import venv" >/dev/null 2>&1 >/dev/null \
    || (echo "Error: python3 venv module not found on system!" 1>&2; exit 1)

echo "Status: Building agents..."

# create virtual environment for challenges 1,2 and 3
python3 -m venv $VENV_DIR

# install dependencies
source $VENV_DIR/bin/activate
pip3 install -r requirements.txt
deactivate

# compile agent for challenge 4
rm -rf build
mkdir -p build
cd build

cmake ..
if ! command -v nproc >/dev/null 2>&1 ; then
    make
else
    make -j$(nproc)
fi
