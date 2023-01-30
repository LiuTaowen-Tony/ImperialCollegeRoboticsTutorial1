import time
from dataclasses import dataclass
from enum import Enum

from brickpi3 import BrickPi3

#class Instruction(Enum):
#    WALK_STRAIGHT = 0
#    TURN_LEFT = 1
#    STOP = 2



BP = BrickPi3()

LEFT_MOTOR_PORT = BP.PORT_C
RIGHT_MOTOR_PORT = BP.PORT_D

class MoveStatus(Enum):
    WALK_STRAIGHT = 0
    TURN_LEFT = 1
    STOP = 2

class InternalState:
    def __init__(self, *, l, r, cms):
        # cms : current move status
        self.l_mileage_cms = l
        self.r_mileage_cms = r
        self.cms = cms
        self.segment = 0
        self.round = 0


def setWalkStraight() -> None:
    if speed > 70:
        print("Speed is too high")
        speed = 70
    BP.set_motor_dps(LEFT_MOTOR_PORT, 360)
    BP.set_motor_dps(RIGHT_MOTOR_PORT, 360)
    

def setTurnLeft90Degrees() -> None:
    BP.set_motor_dps(LEFT_MOTOR_PORT, 180)
    BP.set_motor_dps(RIGHT_MOTOR_PORT, -180)

def setStop() -> None:
    BP.set_motor_power(LEFT_MOTOR_PORT, 0)
    BP.set_motor_power(RIGHT_MOTOR_PORT, 0)



def main():

    current_status = InternalState(
        l = BP.get_motor_status(LEFT_MOTOR_PORT), 
        r = BP.get_motor_status(RIGHT_MOTOR_PORT),
        cms = MoveStatus.WALK_STRAIGHT)

    try:
        while True:
            time.sleep(0.05)

            l_status = BP.get_motor_status(LEFT_MOTOR_PORT)
            r_status = BP.get_motor_status(RIGHT_MOTOR_PORT)
            _, _, l_mileage, _ = l_status
            _, _, r_mileage, _ = r_status
            print(l_status, r_status)

            # probably we need history in future tasks
            if current_status.cms == MoveStatus.WALK_STRAIGHT:
                if l_mileage - current_status.l_mileage_cms >= 762.5:
                    current_status.cms = MoveStatus.TURN_LEFT
                    current_status.l_mileage_cms = l_mileage
                    current_status.r_mileage_cms = r_mileage
                    setTurnLeft90Degrees()
            elif current_status.cms == MoveStatus.TURN_LEFT:
                if l_mileage - current_status.l_mileage_cms >= 180:
                    current_status.cms = MoveStatus.WALK_STRAIGHT
                    current_status.l_mileage_cms = l_mileage
                    current_status.r_mileage_cms = r_mileage
                    setWalkStraight()
            else:                                          
                raise Exception("I don't know what to do")

    except Exception as e:
        print(e)
        BP.reset_all()
        setStop()
    except KeyboardInterrupt:
        setStop()
        BP.reset_all()
       
        
if __name__ == "__main__":
    main() 
