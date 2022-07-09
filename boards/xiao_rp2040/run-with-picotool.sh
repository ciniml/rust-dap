#!/bin/sh

elf2uf2-rs "$1" && picotool load "$1.uf2" && picotool reboot
exit $?