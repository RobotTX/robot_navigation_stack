#!/bin/bash
var=$(ps -a | grep roscore)
if [ $? -ne 0 ] 
then
echo "y"
fi
