from gpiozero import LED
from time import sleep

#Setup
led = LED(4)

#Output logic
for i in range(0,4):
	led.on()
	sleep(1)
	led.off()
	sleep(1)
