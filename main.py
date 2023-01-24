import time
from enum import Enum
from dataclasses import dataclass

from brickpi3 import BrickPi3

class STATUS: pass
class ALL_GOOD(STATUS): pass
class TO_TURN(STATUS): pass
@dataclass
class LEFT_SHIFTED(STATUS):
    value: float
@dataclass
class RIGHT_SHIFTED(STATUS):
    value: float


BP = BrickPi3()

LEFT_MOTOR_PORT = BP.PORT_A
RIGHT_MOTOR_PORT = BP.PORT_B


def setWalkStraight(*, speed: int = 30) -> None:
    if speed > 70:
        print("Speed is too high")
        speed = 70
    BP.set_motor_power(LEFT_MOTOR_PORT, speed)
    BP.set_motor_power(RIGHT_MOTOR_PORT, speed)
    

def setTurnLeftSlightly(*, speed: int = 30) -> None:
    pass

def setTurnRightSlightly(*, speed: int = 30) -> None:
    pass

def setTurnLeft90Degrees(*, speed: int = 30) -> None:
    pass

def checkStatus(leftMotorCurrentStatus, rightMotorCurrentStatus, history) -> STATUS:
    pass

def main():

    history = []

    try:
        while True:
            time.sleep(0.2)

            leftMotorCurrentStatus  = BP.get_motor_status(LEFT_MOTOR_PORT)
            rightMotorCurrentStatus = BP.get_motor_status(RIGHT_MOTOR_PORT)
            running_status          = checkStatus(leftMotorCurrentStatus, rightMotorCurrentStatus, history)

            # probably we need history in future tasks
            history.append((leftMotorCurrentStatus, rightMotorCurrentStatus))

            if   isinstance(running_status, ALL_GOOD):      setWalkStraight()
            elif isinstance(running_status, LEFT_SHIFTED):  setTurnLeftSlightly()
            elif isinstance(running_status, RIGHT_SHIFTED): setTurnRightSlightly()
            elif isinstance(running_status, TO_TURN):       setTurnLeft90Degrees()
            else:                                           raise Exception("I don't know what to do")

    except KeyboardInterrupt:
        BP.reset_all()
       
        
        