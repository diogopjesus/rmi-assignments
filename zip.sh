#!/bin/bash

set -e

tmpdir=".tmp9233897596"
nmec1=92338
nmec2=97596

if ! command -v zip >/dev/null 2>&1 ; then
    echo "Error: zip not found on system!" 1>&2
    exit 1
fi

mkdir -p $tmpdir/agent

cp -r build.sh \
      run.sh \
      CMakeLists.txt \
      include/ \
      src/ \
      $tmpdir/agent

cd $tmpdir

# remove test references in CMakeLists.txt
sed -i '/add_subdirectory(test)/d' agent/CMakeLists.txt

zip -r agent_$nmec1\_$nmec2.zip agent
mv agent_$nmec1\_$nmec2.zip ../
cd ..; rm -r $tmpdir
