#/bin/bash

dfu-util -d vid:pid,0x0483:0xDF11 -a 0 -s 0x08000000:leave -D "$1"
