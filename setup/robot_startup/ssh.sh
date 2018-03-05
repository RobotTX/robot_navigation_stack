#!/usr/bin/expect
#sudo apt install expect
set timeout 30
spawn sudo ssh username@192.168.100.48
expect {
"*yes/no" { send "yes\r"; exp_continue }
"*password:" { send "user_password\r" }
}
interact
