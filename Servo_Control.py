import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)

x_gimb = GPIO.PWM(11,50)
y_gimb = GPIO.PWM(12,50)

x_gimb.start(9.5)
y_gimb.start(4)

x = 0
y = 0

try:
    while True:
        x_gimb.ChangeDutyCycle(8.2)
        y_gimb.ChangeDutyCycle(4)
        print '2.5'
        time.sleep(0.1)
##        x_gimb.ChangeDutyCycle(7.5)
##        y_gimb.ChangeDutyCycle(7.5)
##        print '7.5'
##        time.sleep(1.5)
##        x_gimb.ChangeDutyCycle(12.5)
##        y_gimb.ChangeDutyCycle(12.5)
##        print '12.5'
##        time.sleep(1.5)
        pass
        
except KeyboardInterrupt:
    x_gimb.stop()
    y_gimb.stop()
    GPIO.cleanup()
