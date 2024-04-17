import time
import pigpio
import rotary_encoder
from Dynamixel_Turret_Lib import *

pos = 0
def callback(way):
    global pos
    pos += way
    print("pos={}".format(pos))

decoder = rotary_encoder.decoder(pi_gpio, 7, 8, callback)

# Spin motor for 1 sec
pi_gpio.write(GPIO_MOTOR_IN1, MODE_DISABLE)
pi_gpio.write(GPIO_MOTOR_IN2, MODE_ENABLE)
time.sleep(1)
pi_gpio.write(GPIO_MOTOR_IN1, MODE_DISABLE)
pi_gpio.write(GPIO_MOTOR_IN2, MODE_DISABLE)

decoder.cancel()
pi_gpio.stop()
