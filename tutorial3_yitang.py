import time
from dataclasses import dataclass
from enum import Enum
import random
import math

from brickpi3 import BrickPi3



BP = BrickPi3()

LEFT_MOTOR_PORT = BP.PORT_C
RIGHT_MOTOR_PORT = BP.PORT_D
ONE_CM_DIST = 20.5
PI_TURN = 530
SAMPLE_SIZE = 100
ES = 0.002
FS = 0.0005
GS = 0.02

nav_points = [(10, 0), (20, 0), (30, 0), (40, 0), (40, 10), (40, 20), (40, 30), (40, 40), (30, 40), (20, 40), (10, 40), (0, 40), (0, 30), (0, 20), (0, 10), (0, 0)]

def gausses(sigma):
    return [random.gauss(0, sigma) for _ in range(SAMPLE_SIZE)]


def trans_coord(pos):
    x, y, theta = pos
    return (x * 10 + 100, 500 - y * 10, theta)

def distance(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def new_sample_go_straight(cur_pos, distance_error_weight, angle_error_weight, dist):
    x, y, theta, w = cur_pos
    nx = x + dist * (1 + distance_error_weight) * math.cos(theta)
    ny = y + dist * (1 + distance_error_weight) * math.sin(theta)
    ntheta = (theta + dist * angle_error_weight) % (2 * math.pi)
    return (nx, ny, ntheta, w)

def new_samples_turn(pos, angle_error, angle):
    x, y, theta, w = pos
    return (x, y, (theta + (math.pi + angle_error) * angle) % (2 * math.pi), w)

def setWalkStraight() -> None:
    BP.set_motor_dps(LEFT_MOTOR_PORT, 360)
    BP.set_motor_dps(RIGHT_MOTOR_PORT, 360)
    

def setTurnLeft() -> None:
    BP.set_motor_dps(LEFT_MOTOR_PORT, 180)
    BP.set_motor_dps(RIGHT_MOTOR_PORT, -180)
    
def setTurnRight() -> None:
    BP.set_motor_dps(LEFT_MOTOR_PORT, -180)
    BP.set_motor_dps(RIGHT_MOTOR_PORT, 180)

def setStop() -> None:
    BP.reset_all()
    BP.set_motor_power(LEFT_MOTOR_PORT, 0)
    BP.set_motor_power(RIGHT_MOTOR_PORT, 0)

class MoveStatus(Enum):
    WALK_STRAIGHT = 0
    TURN_LEFT = 1
    STOP = 2

class Robot:
    def __init__(self, *, l, r, cms):
        # cms : current move status
        self.l_mileage_cms = l
        self.r_mileage_cms = r
        self.cms = cms
        self.segment = 0
        self.round = 0
        self.step = 0
        self.samples = [(0, 0, 0, 1 / SAMPLE_SIZE) for _ in range(SAMPLE_SIZE)]
        self.history = self.samples
        
    def cur_pos(self):
        x = 0
        y = 0
        theta_x = 0
        theta_y = 0
        for (xx, yy, tt, ww) in self.samples:
            x += xx * ww
            y += yy * ww
            theta_x += math.cos(tt) * ww
            theta_y += math.sin(tt) * ww 
        return [x, y, math.atan2(theta_y, theta_x)]
        
    def navigateToWaypoint(self, x, y):
        cur_x, cur_y, theta = self.cur_pos()
        angle = math.atan2(y - cur_y, x - cur_x)
        dtheta = ((angle - theta) / math.pi) % 2
        self.turn(dtheta)
        time.sleep(0.5)
        self.go_straight_dist(distance((cur_x, cur_y), (x, y)))
        print(theta, angle, dtheta)

    def go_straight_dist(self, dist=10):
        try:
            while True:
                l_status = BP.get_motor_status(LEFT_MOTOR_PORT)
                r_status = BP.get_motor_status(RIGHT_MOTOR_PORT)
                _, _, l_mileage, _ = l_status
                _, _, r_mileage, _ = r_status
                print(l_status, r_status)
                
                if l_mileage - self.l_mileage_cms >= ONE_CM_DIST * dist:
                    setStop()
                    self.l_mileage_cms = l_mileage
                    self.r_mileage_cms = r_mileage
                    break
                else:
                    setWalkStraight()
                
                time.sleep(0.02)
            
            dist_error_by_xy = gausses(ES)
            dist_error_by_theta = gausses(FS)
            self.samples = [
                    new_sample_go_straight(s, d, a, dist) 
                    for s, d, a in 
                    zip(self.samples, dist_error_by_xy, dist_error_by_theta)
                ]
            self.history += self.samples
            self.draw()
        except Exception as e:
            print(e)
            setStop()
        except KeyboardInterrupt:
            setStop()
    
    def turn(self, angle_rad=0.5):
        try:
            while True:
                l_status = BP.get_motor_status(LEFT_MOTOR_PORT)
                r_status = BP.get_motor_status(RIGHT_MOTOR_PORT)
                _, _, l_mileage, _ = l_status
                _, _, r_mileage, _ = r_status
                print(l_status, r_status)
                
                if angle_rad < 1:
                    if l_mileage - self.l_mileage_cms >= PI_TURN * angle_rad:
                        setStop()
                        self.l_mileage_cms = l_mileage
                        self.r_mileage_cms = r_mileage
                        break
                    else:
                        setTurnLeft()
                else:
                    if r_mileage - self.r_mileage_cms >= PI_TURN * (2 - angle_rad):
                        setStop()
                        self.l_mileage_cms = l_mileage
                        self.r_mileage_cms = r_mileage
                        break
                    else:
                        setTurnRight()

                time.sleep(0.02)
            
            # better version of turn_one
            angle_errors = gausses(GS)
            self.samples = [
                new_samples_turn(p, g, angle_rad) 
                for p, g in 
                zip(self.samples, angle_errors)
            ]
            self.history += self.samples
        except Exception as e:
            print(e)
            setStop()
        except KeyboardInterrupt:
            setStop()

    def draw(self):
        print("drawParticles:" + str([trans_coord(pos) for pos in self.history]))





def main():
    current_status = Robot(
        l = BP.get_motor_status(LEFT_MOTOR_PORT)[2], 
        r = BP.get_motor_status(RIGHT_MOTOR_PORT)[2],
        cms = MoveStatus.WALK_STRAIGHT)
    for pos in nav_points:
        current_status.navigateToWaypoint(pos[0], pos[1])
        time.sleep(0.5)


if __name__ == "__main__":
    main()