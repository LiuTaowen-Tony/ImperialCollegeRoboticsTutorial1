from brickpi3 import BrickPi3
import time

BP = BrickPi3()


LEFT_MOTOR_PORT = BP.PORT_A
RIGHT_MOTOR_PORT = BP.PORT_B

def setWalkStraight(speed):
    pass

def setTurnLeftSlightly(speed):
    pass

def setTurnRightSlightly(speed):
    pass

def setTurnLeft90Degrees(speed):
    pass

def isAllGood(leftMotorCurrentStatus, rightMotorCurrentStatus, history):
    return True

def isleftShifted(leftMotorCurrentStatus, rightMotorCurrentStatus, history):
    return True

def isRightShifted(leftMotorCurrentStatus, rightMotorCurrentStatus, history):
    return True

def needToTurn(leftMotorCurrentStatus, rightMotorCurrentStatus, history):
    return True


def main():

    history = []

    while True:
        time.sleep(0.2)

        leftMotorCurrentStatus = BP.get_motor_status(LEFT_MOTOR_PORT)
        rightMotorCurrentStatus = BP.get_motor_status(RIGHT_MOTOR_PORT)

        # probably we need history in future tasks

        allGood = isAllGood(leftMotorCurrentStatus, rightMotorCurrentStatus, history)
        leftShifted = isleftShifted(leftMotorCurrentStatus, rightMotorCurrentStatus, history)
        rightShifted = isRightShifted(leftMotorCurrentStatus, rightMotorCurrentStatus, history)
        toTurn = needToTurn(leftMotorCurrentStatus, rightMotorCurrentStatus, history)

        history.append((leftMotorCurrentStatus, rightMotorCurrentStatus))

        # sanity check
        if (allGood + leftShifted + rightShifted + toTurn) != 1:
            raise Exception("I don't know what to do")
        
        if allGood:
            setWalkStraight()
        elif leftShifted:
            setTurnLeftSlightly()
        elif rightShifted:
            setTurnRightSlightly()
        elif toTurn:
            setTurnLeft90Degrees()
        else:
            raise Exception("I don't know what to do")
        
        
        