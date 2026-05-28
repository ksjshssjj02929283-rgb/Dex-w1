#!/bin/bash

CURDIR=$(pwd)
PROJECT_DIR=$(basename "$PWD")
PACKAGE_NAME=${PROJECT_DIR//_/-}
echo "Running file installation for $PACKAGE_NAME..."


mkdir -p "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PROJECT_DIR"
mkdir -p "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/install"

if [ ! -d "$CURDIR/../install" ]; then
    echo "Error: install directory not found!"
    exit 1
fi

find "$CURDIR/run_w1.sh" -maxdepth 1 -type f -exec cp {} "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PROJECT_DIR/" \;
find "$CURDIR/run_sim.sh" -maxdepth 1 -type f -exec cp {} "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PROJECT_DIR/" \;
find "$CURDIR/set_w1.sh" -maxdepth 1 -type f -exec cp {} "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PROJECT_DIR/" \;
find "$CURDIR/../install" -maxdepth 1 -mindepth 1 -type d -exec cp -r {} "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/install/" \;