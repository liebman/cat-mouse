#!/bin/sh
#set -x
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

if [ -e ./flashfs.var ]; then
    . flashfs.var
else
    echo 1>&2 "'flashfs.var' does not exist.  'Did you run cargo build'?"
    exit 1
fi

./spiffsgen.py $spiffs_size ./data ../target/spiffs.img
esptool.py $@ write_flash $spiffs_address ../target/spiffs.img
#espflash  write-bin $@ $spiffs_address ../target/spiffs.img
