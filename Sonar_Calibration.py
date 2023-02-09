from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

# Constants
BP = brickpi3.BrickPi3()
SONAR_PORT = BP.PORT_1

# BP Setting
BP.set_sensor_type(SONAR_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)

def continuous_output():
    try:
        while True:
            # read and display the sensor value
            # BP.get_sensor retrieves a sensor value.
            # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
            # BP.get_sensor returns the sensor value (what we want to display).
            try:
                value = BP.get_sensor(BP.PORT_1)
                print("Distance: ", value)                         # print the distance in CM
            except brickpi3.SensorError as error:
                print(error)
            
            time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

        except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.


continuous_output()