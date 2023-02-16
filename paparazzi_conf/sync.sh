#!/bin/bash

# update all files from paparazzi repo if they still exists, or delete them if they no longer exists
for f in $(find files/ -type f -printf '%P\n')
do
#  echo "files/$f"
  if ! cmp "files/$f" "$PAPARAZZI_HOME/$f" >/dev/null 2>&1
  then
    if test -f "$PAPARAZZI_HOME/$f"
    then
      cp "$PAPARAZZI_HOME/$f" "files/$f"
    else
      rm "files/$f"
    fi
  fi
done

# copy all untracked files from paparazzi
for f in $(git -C $PAPARAZZI_HOME ls-files --others --exclude-standard)
do
  mkdir -p $(dirname "files/$f") && cp "$PAPARAZZI_HOME/$f" "files/$f"
done

