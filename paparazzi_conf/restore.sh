#!/bin/bash

# copy all files from custom conf to paparazzi
for f in $(find files/ -type f -printf '%P\n')
do
  mkdir -p $(dirname "$PAPARAZZI_HOME/$f")
  cp "files/$f" "$PAPARAZZI_HOME/$f"
done
