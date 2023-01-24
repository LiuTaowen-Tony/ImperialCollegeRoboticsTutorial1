import time
from enum import Enum

from brickpi3 import BrickPi3
class STATUS(Enum):
    ALL_GOOD = 1
    LEFT_SHIFTED = 2
    RIGHT_SHIFTED = 3
    TO_TURN = 4

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

    while True:
        time.sleep(0.2)

        leftMotorCurrentStatus = BP.get_motor_status(LEFT_MOTOR_PORT)
        rightMotorCurrentStatus = BP.get_motor_status(RIGHT_MOTOR_PORT)
        running_status = checkStatus(leftMotorCurrentStatus, rightMotorCurrentStatus, history)

        # probably we need history in future tasks
        history.append((leftMotorCurrentStatus, rightMotorCurrentStatus))

        if running_status == STATUS.ALL_GOOD:
            setWalkStraight()
        elif running_status == STATUS.LEFT_SHIFTED:
            setTurnLeftSlightly()
        elif running_status == STATUS.RIGHT_SHIFTED:
            setTurnRightSlightly()
        elif running_status == STATUS.TO_TURN:
            setTurnLeft90Degrees()
        else:
            raise Exception("I don't know what to do")
        
        
        