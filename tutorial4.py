import time
from dataclasses import dataclass
from enum import Enum
import random
import math
from typing import *

from brickpi3 import BrickPi3
import brickpi3  # import the BrickPi3 drivers

from typing import *


BP = BrickPi3()

LEFT_MOTOR_PORT = BP.PORT_C
RIGHT_MOTOR_PORT = BP.PORT_D
SONAR_PORT = BP.PORT_4
BP.reset_all()
ONE_CM_DIST = 21
PI_TURN = 170
SAMPLE_SIZE = 114
ES = 0.1
FS = 0.001
GS = 0.08
SONAR_SIGMA = 2
SONAR_GAIN = 0.05

SX = 84
SY = 30

BP.set_sensor_type(SONAR_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)

nav_points = [(180, 30), (180, 54), (138, 54), (138, 168),
              (114, 168), (114, 84), (84, 84), (84, 30)]
# nav_points = [(180, 30)]

def my_print(*args, **kwargs):
    print(*args, **kwargs, flush=True)


class Line:
    def __init__(self, is_horizontal, anchor, range_min, range_max):
        self.is_horizontal = is_horizontal
        self.anchor = anchor
        self.range = (min(range_min, range_max), max(range_min, range_max))

    def __str__(self):
        return f"{'y' if self.is_horizontal else 'x'} = {self.anchor}"

    def distance(self, x, y, theta):
        if self.is_horizontal:
            if theta == 0.0 or theta == 1.0:
                return None
            t = (self.anchor - y) / math.sin(theta)
            if t < 0:
                return None
            x_inter = x + t * math.cos(theta)
            if self.range[0] <= x_inter <= self.range[1]:
                coord = (x_inter, self.anchor)
            else:
                return None
        else:
            if theta == 0.5 or theta == 1.5:
                return None
            t = (self.anchor - x) / math.cos(theta)
            if t < 0:
                return None
            y_inter = y + t * math.sin(theta)
            if self.range[0] <= y_inter <= self.range[1]:
                coord = (self.anchor, y_inter)
            else:
                return None

        (new_x, new_y) = coord
        return math.sqrt((new_y - y) ** 2 + (new_x - x) ** 2)


real_walls = [Line(False, 0, 0, 168), Line(True, 168, 0, 84), Line(False, 84, 126, 210), Line(
    True, 210, 84, 168), Line(False, 168, 84, 210), Line(True, 84, 168, 210), Line(False, 210, 0, 84), Line(True, 0, 0, 210)]
# real_walls = [Line(False, 184, 0, 168)]


def calculate_likelihood(x, y, theta, sonar, walls=real_walls):
    ground_val = sonar_ground_truth(walls, x, y, theta) # If None, the particles should be outside the walls
    if ground_val is None:
        return None
    return math.exp(-0.5 * ((ground_val - sonar) / SONAR_SIGMA) ** 2) + SONAR_GAIN

# Return the sonar depth by choosing distance to the nearest wall
def sonar_ground_truth(walls: List[Line], x: float, y: float, theta: float):
    try:
        return min(filter(lambda x: x is not None, [wall.distance(x, y, theta) for wall in walls]))
    except:
        return None

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print("drawLine:" + str((x1,y1,x2,y2)))

    def draw(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print("drawParticles:" + str(display));

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

def gausses(sigma):
    return [random.gauss(0, sigma) for _ in range(SAMPLE_SIZE)]

def trans_coord(pos):
    x, y, theta, _ = pos
    return (x * 10 + 100, 500 - y * 10, theta)

canvas = Canvas();

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);

def distance(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)




# def new_sample_turn(pos, angle_error, angle):
#     x, y, theta, w = pos
#     if angle < 1:
#         return (x, y, (theta + (math.pi + angle_error) * angle) % (2 * math.pi), w)
#     return (x, y, (theta - (math.pi + angle_error) * (2 - angle)) % (2 * math.pi), w)



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
        self.samples = [(SX, SY, 0, 1 / SAMPLE_SIZE)
                        for _ in range(SAMPLE_SIZE)]
        self.history = self.samples
        self.map = Map()
        BP.set_motor_limits(LEFT_MOTOR_PORT, 100, 300)
        BP.set_motor_limits(RIGHT_MOTOR_PORT, 100, 300)

    def drawWall(self):
        self.map.add_wall((0,0,0,168));        # a
        self.map.add_wall((0,168,84,168));     # b
        self.map.add_wall((84,126,84,210));    # c
        self.map.add_wall((84,210,168,210));   # d
        self.map.add_wall((168,210,168,84));   # e
        self.map.add_wall((168,84,210,84));    # f
        self.map.add_wall((210,84,210,0));     # g
        self.map.add_wall((210,0,0,0));        # h
        self.map.draw()
        
    def inc_motor_positions(self, l_radian, r_radian):
        _, _, old_l_mileage, _  = BP.get_motor_status(LEFT_MOTOR_PORT)
        BP.set_motor_position(LEFT_MOTOR_PORT, old_l_mileage + l_radian)
        _, _, old_r_mileage, _  = BP.get_motor_status(RIGHT_MOTOR_PORT)
        BP.set_motor_position(RIGHT_MOTOR_PORT, old_r_mileage + r_radian)
        pre_l_mileage = old_l_mileage
        pre_r_mileage = old_r_mileage
        while True:
            time.sleep(0.01)
            _, _, l_mileage, _  = BP.get_motor_status(LEFT_MOTOR_PORT)
            _, _, r_mileage, _  = BP.get_motor_status(RIGHT_MOTOR_PORT)
            if (pre_l_mileage == l_mileage and pre_r_mileage == r_mileage):
                break
            pre_l_mileage = l_mileage
            pre_r_mileage = r_mileage

    def move_forward_by_dist(self, dist_cm):
        radii = ONE_CM_DIST * dist_cm
        self.inc_motor_positions(radii, radii)

    def turn_by_radian(self, turn_left_radian):
        """
        turn_left_radian: 0 ~ 2pi
        """
        assert 0 <= turn_left_radian <= 2 * math.pi
        if turn_left_radian < math.pi:
            self.inc_motor_positions(PI_TURN * turn_left_radian, -PI_TURN * turn_left_radian)
        else:
            self.inc_motor_positions(
                -PI_TURN * (2 * math.pi - turn_left_radian), 
                PI_TURN * (2 * math.pi - turn_left_radian))
    
    def draw(self, samples):
        canvas.draw(samples)

    def resample_cur_pos(self):
        self.resample()
        return self.cur_pos_no_resample()

    # Return the current position by taking weighted average from all particles.
    def cur_pos_no_resample(self):
        x = 0
        y = 0
        theta_x = 0
        theta_y = 0
        for (xx, yy, tt, ww) in self.samples:
            x += xx * ww
            y += yy * ww
            theta_x += math.cos(tt) * ww
            theta_y += math.sin(tt) * ww
        return [x, y, math.atan2(theta_y, theta_x) % (2 * math.pi)]

    def to_point(self, x, y):
        while True:
            # turn stage: adjust robot's angle
            ####################
            self.localization_and_draw()

            cur_x, cur_y, theta = self.cur_pos_no_resample()
            target_angle = math.atan2(y - cur_y, x - cur_x)
            dtheta = (target_angle - theta) % (2 * math.pi)
            
            my_print(f"pos: ({cur_x}, {cur_y}), target: ({x}, {y})")
            my_print(f"target_angle: {target_angle}, theta: {theta}, dtheta: {dtheta}")

            if 0.05 * math.pi < dtheta < 1.95 * math.pi:
                # turn
                self.turn_by_radian(dtheta)
                new_samples = self.calc_turn_error(dtheta)
                self.samples = new_samples
                self.localization_and_draw()


            # forward stage
            ####################
            cur_x, cur_y, theta = self.cur_pos_no_resample()
            dist = distance((cur_x, cur_y), (x, y))

            print(f"Remaining distance: {dist}")
            if dist <= 1:
                # we are at the point, thus done
                return
            elif dist <= 10:
                self.move_forward_by_dist_update_samples(dist)
                return
            elif dist <= 40:
                # go without resampling
                self.move_forward_by_dist_update_samples(dist / 2)
            else:
                # After going forward 20cm, loop back to adjust robot's angle.
                self.move_forward_by_dist_update_samples(20)

    def localization_and_draw(self):
        new_samples = self.read_sonar_calc_new_samples()
        self.draw(new_samples)
        # show for 0.5 sec
        time.sleep(0.5)
        new_samples = self.select_survived_samples(new_samples)
        self.draw(new_samples)
        self.samples = new_samples

    def move_forward_by_dist_update_samples(self, dist):
        self.move_forward_by_dist(dist)
        new_samples = self.calc_move_forward_error(dist)
        self.samples = new_samples


    def calc_move_forward_error(self, dist):
        def new_sample_go_straight(cur_pos, distance_error_weight, angle_error_weight, dist):
            x, y, theta, w = cur_pos
            nx = x + dist * (1 + distance_error_weight) * math.cos(theta)
            ny = y + dist * (1 + distance_error_weight) * math.sin(theta)
            ntheta = (theta + dist * angle_error_weight) % (2 * math.pi)
            return (nx, ny, ntheta, w)
        dist_error_by_xy = gausses(ES)
        dist_error_by_theta = gausses(FS)
        new_samples = []
        # Update particles stages after moving forward
        for s, d, a in zip(self.samples, dist_error_by_xy, dist_error_by_theta):
            new_samples.append(new_sample_go_straight(s, d, a, dist))
        return new_samples

    def calc_turn_error(self, turn_left_radian):
        def new_sample_turn(cur_pos, angle_error_weight):
            x, y, theta, w = cur_pos
            ntheta = (theta + turn_left_radian * (1 + angle_error_weight)) % (2 * math.pi)
            return (x, y, ntheta, w)
        angle_error_by_theta = gausses(FS)
        new_samples = []
        # Update particles stages after turning
        for s, a in zip(self.samples, angle_error_by_theta):
            new_samples.append(new_sample_turn(s, a))
        return new_samples

    def read_sonar_calc_new_samples(self):
        sonar = self.accurate_sonar_read()
        new_samples = []
        for s in self.samples:
            x, y, theta, weight = s
            likelihood = calculate_likelihood(x, y, theta, sonar)
            if likelihood is not None:
                new_samples.append((x, y, theta, weight * likelihood))

        # Normalisation
        normaliser = sum([w for _, _, _, w in new_samples])
        new_samples = [(x, y, theta, w / normaliser) for x, y, theta, w in new_samples]
        return new_samples

    def select_survived_samples(self, new_samples):
        weights = [w for _, _, _, w in new_samples]
        survived_samples = random.choices(new_samples, weights=weights, k=SAMPLE_SIZE)
        assert len(survived_samples) == SAMPLE_SIZE
        return [(x, y, theta, 1 / SAMPLE_SIZE) for (x, y, theta, _) in survived_samples]

    
    # Return the median of 11 sonar readings to reduce effect of garbage readings.
    def accurate_sonar_read(self):
        time.sleep(1.5)
        readings = []
        ix = 1
        while ix <= 11:
            # print(f'iteration {ix}')
            try:
                value = BP.get_sensor(SONAR_PORT)
                readings.append(value)
                ix += 1
            except brickpi3.SensorError as error:
                print(error)
            time.sleep(0.1)

        readings.sort()
        sonar = readings[5]
        return sonar

def main():
    current_status = Robot(
        l=BP.get_motor_status(LEFT_MOTOR_PORT)[2],
        r=BP.get_motor_status(RIGHT_MOTOR_PORT)[2],
        cms=MoveStatus.WALK_STRAIGHT)
    current_status.drawWall()
    time.sleep(2)
    for pos in nav_points:
        current_status.to_point(pos[0], pos[1])
        print(f"Expected location: {pos[0]}, {pos[1]}")
        print(f"Actual position: {current_status.cur_pos_no_resample()}")
        time.sleep(0.5)


if __name__ == "__main__":
    main()
