#!/bin/bash

DATE=$(date +"%Y-%m-%d_%H%M")
FILE=/home/pi/camera/$DATE.jpg

raspistill -rot 180 -v -f -o $FILE
echo Sparar bild: $FILE
gpicview $FILE
