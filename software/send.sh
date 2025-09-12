#!/bin/bash

if [ "$#" -gt 2 ]; then
  echo "Too many arguments. Specify ";
fi

SERIAL_PORT=$1
BAUDRATE=115200
if [ "$#" -eq 2 ]; then
  BAUDRATE=$2
fi

stty -F "$SERIAL_PORT" "$BAUDRATE" line 0 min 1 time 0 \
  -brkint -icrnl -imaxbel \
  -opost \
  -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke;
