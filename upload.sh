#!/usr/bin/expect
cd /home/pmx/git/ev3dev-lang/cpp/
spawn scp drive-test ev3dev-lang-demo ev3dev-lang-test button-test root@192.168.7.9:/pmx/
expect {
password: {
send "pmx\r";
exp_continue
}
}
