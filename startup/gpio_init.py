import Jetson.GPIO as GPIO

gpio_in = [16, 18]
GPIO.setmode(GPIO.BOARD)
GPIO.setup(gpio_in, GPIO.IN)

gpio_out = [29, 31]
GPIO.setup(gpio_out, GPIO.OUT, initial = GPIO.LOW)
