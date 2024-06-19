#!/usr/bin/env bash
cp -r data/ ..
cd ..
if [ ! -d "build" ]; then
    mkdir build
else
    echo "build directory already exists, skipping mkdir"
fi
mv data build/