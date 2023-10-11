#!/bin/sh

sleep 0.1
/usr/bin/gpioset gpiochip0 36=1

sleep 0.5

/usr/bin/gpioset gpiochip1 8=1
/usr/bin/gpioset gpiochip0 36=0
sleep 0.5
/usr/bin/gpioset gpiochip0 36=1
sleep 0.5
/usr/bin/gpioset gpiochip1 8=0
