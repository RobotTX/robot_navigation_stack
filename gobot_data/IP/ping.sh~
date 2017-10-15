#!/bin/bash
fping -g 192.168.4.0/24 2>/dev/null | grep alive > alive.txt
cut -f 1 alive.txt --delimiter=' ' >isAlive.txt
echo 192.168.7.1 >> isAlive.txt
echo 192.168.7.2 >> isAlive.txt
echo 192.168.7.3 >> isAlive.txt
rm alive.txt
