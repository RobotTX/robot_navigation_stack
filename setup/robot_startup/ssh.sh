#!/usr/bin/expect
#sudo apt install expect
set timeout 30
spawn sudo ssh gtdollar@192.168.100.48
expect {
"*yes/no" { send "yes\r"; exp_continue }
"*password:" { send "gtdollar123\r" }
}
interact
