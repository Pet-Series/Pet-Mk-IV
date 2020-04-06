#!/bin/bash

DATE=$(date +"%Y-%m-%d_%H%M")
FILE=/home/pi/Pictures/$DATE.jpg

raspistill -rot 180 -v -f -o $FILE
echo Sparar bild: $FILE
gpicview $FILE
