import math
import copy
from dataclasses import dataclass
from typing import List, Dict
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from configs import config as cfg
from cybernetic_core.geometry.angles import (
    TettasException,
    RobotPosition, 
    calculate_leg_angles, 
    turn_on_angle, 
    convert_legs_angles_C, 
    calculate_C_point,
    tettas_ok
)
from cybernetic_core.geometry.lines import Point, LinearFunc, calculate_intersection, move_on_a_line
import configs.code_config as code_config
import logging.config
from cybernetic_core.cybernetic_utils.moves import MoveSnapshot


def get_turn_coords(between_legs, angle, x0=0, y0=0, x_delta=0, y_delta=0):

    x3, y3 = x0 + x_delta, y0 + y_delta

    a = between_legs
    alpha = math.radians(angle)
    beta = math.radians(90 - angle)

    x4 = x3 + a*math.cos(alpha)
    y4 = y3 + a*math.sin(alpha)

    x2 = x3 - a*math.cos(beta)
    y2 = y3 + a*math.sin(beta)

    x1 = x4 - a*math.sin(alpha)
    y1 = y4 + a*math.cos(alpha)

    print(f'{between_legs}, {x0}, {y0}, ({x1, y1}), ({x2, y2}), ({x3, y3}), ({x4, y4})')

    return x1, y1, x2, y2, x3, y3, x4, y4

import numpy as np

def rotate_point(x, y, angle):
    # Convert angle from degrees to radians
    angle_rad = np.radians(angle)
    
    # Rotation matrix
    rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                 [np.sin(angle_rad), np.cos(angle_rad)]])
    
    # Coordinates after rotation
    new_x, new_y = np.dot(rotation_matrix, np.array([x, y]))
    
    return new_x, new_y

def move_forward(x, y, angle, move_distance):
    # Calculate the new coordinates after moving forward
    new_x = x + move_distance * np.cos(np.radians(angle))
    new_y = y + move_distance * np.sin(np.radians(angle))
    
    return new_x, new_y

def move_robot_legs(leg1_x, leg1_y, leg2_x, leg2_y, leg3_x, leg3_y, leg4_x, leg4_y, angle, move_distance):
    # Rotate the legs by the given angle
    print(leg1_x, leg1_y, leg2_x, leg2_y, leg3_x, leg3_y, leg4_x, leg4_y, angle, move_distance)
    leg1_x, leg1_y = rotate_point(leg1_x, leg1_y, angle)
    leg2_x, leg2_y = rotate_point(leg2_x, leg2_y, angle)
    leg3_x, leg3_y = rotate_point(leg3_x, leg3_y, angle)
    leg4_x, leg4_y = rotate_point(leg4_x, leg4_y, angle)

    # Move the legs forward
    leg1_x, leg1_y = move_forward(leg1_x, leg1_y, angle, move_distance)
    leg2_x, leg2_y = move_forward(leg2_x, leg2_y, angle, move_distance)
    leg3_x, leg3_y = move_forward(leg3_x, leg3_y, angle, move_distance)
    leg4_x, leg4_y = move_forward(leg4_x, leg4_y, angle, move_distance)
    print(leg1_x, leg1_y, leg2_x, leg2_y, leg3_x, leg3_y, leg4_x, leg4_y)
    return leg1_x, leg1_y, leg2_x, leg2_y, leg3_x, leg3_y, leg4_x, leg4_y

class Leg:
    def __init__(self, C: Point):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')
        self.C = C
        self.update_angles()

    def update_angles(self):
        # tetta is not fully correct, because it uses atan2
        # tetta is corrected via convert_tetta function
        #print(self.O, self.D)
        self.tetta, self.alpha, self.beta = calculate_leg_angles(self.C, self.logger)
        self.logger.info(f'Leg angles: {math.degrees(self.tetta), math.degrees(self.alpha), math.degrees(self.beta)}')

    def move_mount_point(self, delta_x, delta_y, delta_z):
        self.C.move(-delta_x, -delta_y, -delta_z)
        self.update_angles()
    
    def move_end_point(self, delta_x, delta_y, delta_z):
        #print(f'C before: {self.C}')
        self.C.move(delta_x, delta_y, delta_z)
        #print(f'C after: {self.C}')
        self.update_angles()
    
    def __repr__(self):
        return f't: {round(math.degrees(self.tetta), 2)}, a: {round(math.degrees(self.alpha), 2)}, b: {round(math.degrees(self.beta), 2)}'

class Kinematics:
    """
    Either take initial position from config
    or provide horizontal x, y and vertical v
    or provide exact angles to create a kinematic model
    """
    def __init__(self, robot_position: RobotPosition = None, init_snapshot=True):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')

        if robot_position is None:
            self.legs = self.initiate_legs()
        else:
            self.legs = self.build_legs_from_angles(robot_position)
            
        self.angles_history = []
        if init_snapshot:
            self.add_angles_snapshot('init')
    
    def check_tettas(self):
        if not tettas_ok(
            self.legs[1].tetta,
            self.legs[2].tetta,
            self.legs[3].tetta,
            self.legs[4].tetta,
            self.legs[5].tetta,
            self.legs[6].tetta,
            self.logger
        ):
            raise TettasException('Bad tettas')

    #def move_leg_endpoint(self, legnum, delta_x, delta_y, delta_z):
    #    self.legs[legnum].move_end_point(delta_x, delta_y, delta_z)

    def move_leg_endpoint(self, leg_num, delta_x, delta_y, delta_z, snapshot_type='endpoint', add_snapshot=False):        
        self.legs[leg_num].move_end_point(delta_x, delta_y, delta_z)
        if add_snapshot:
            self.add_angles_snapshot(snapshot_type)
        self.check_tettas()
    
    def move_leg_mountpoint(self, legnum, delta_x, delta_y, delta_z):
        self.logger.info(f'Leg {legnum}. {self.legs[legnum]}')
        self.legs[legnum].move_mount_point(delta_x, delta_y, delta_z)
        self.logger.info(f'Leg {legnum}. {self.legs[legnum]}')
        

    def reset_history(self):
        self.angles_history = []

    def add_angles_snapshot(self, move_type: str = 'unknown'):
        rp = RobotPosition(
            l1t=self.legs[1].tetta,
            l1a=self.legs[1].alpha,
            l1b=self.legs[1].beta,

            l2t=self.legs[2].tetta,
            l2a=self.legs[2].alpha,
            l2b=self.legs[2].beta,

            l3t=self.legs[3].tetta,
            l3a=self.legs[3].alpha,
            l3b=self.legs[3].beta,

            l4t=self.legs[4].tetta,
            l4a=self.legs[4].alpha,
            l4b=self.legs[4].beta,

            l5t=self.legs[5].tetta,
            l5a=self.legs[5].alpha,
            l5b=self.legs[5].beta,

            l6t=self.legs[6].tetta,
            l6a=self.legs[6].alpha,
            l6b=self.legs[6].beta,
        )

        #new_move = MoveSnapshot(move_type, convert_legs_angles(angles_in))
        new_move = MoveSnapshot(move_type, rp)
        self.angles_history.append(new_move)

    @property
    def height(self):
        return sum([(leg.O.z - leg.C.z) for leg in self.legs.values()])/4
        #return self.current_legs_offset_v

    @property
    def sequence(self):
        sequence = []
        for move in self.angles_history:
            sequence.append(MoveSnapshot(move.move_type, convert_legs_angles_C(move.angles_snapshot, self.logger)))
        return sequence
        #return self.angles_history
    
    def build_legs_from_angles(self, rp: RobotPosition):
        C1 = calculate_C_point(rp.l1t, rp.l1a, rp.l1b)
        self.logger.info('[Init] Building leg 1')
        #print(f'Building leg 1: {math.degrees(alpha1), math.degrees(beta1), math.degrees(gamma1)}')
        Leg1 = Leg(C1)

        C2 = calculate_C_point(rp.l2t, rp.l2a, rp.l2b)
        self.logger.info('[Init] Building leg 2')
        Leg2 = Leg(C2)
        #print(f'Leg2.2:{[round(math.degrees(x), 2) for x in [Leg2.tetta, Leg2.alpha, Leg2.beta, Leg2.gamma]]}')

        C3 = calculate_C_point(rp.l3t, rp.l3a, rp.l3b)
        self.logger.info('[Init] Building leg 3')
        Leg3 = Leg(C3)

        C4 = calculate_C_point(rp.l4t, rp.l4a, rp.l4b)
        self.logger.info('[Init] Building leg 4')
        Leg4 = Leg(C4)

        C5 = calculate_C_point(rp.l5t, rp.l5a, rp.l5b)
        self.logger.info('[Init] Building leg 5')
        Leg5 = Leg(C5)

        C6 = calculate_C_point(rp.l6t, rp.l6a, rp.l6b)
        self.logger.info('[Init] Building leg 6')
        Leg6 = Leg(C6)

        self.logger.info('[Init] Build successful')

        return {1: Leg1, 2: Leg2, 3: Leg3, 4: Leg4, 5: Leg5, 6: Leg6}

    @property
    def current_position(self):
        #return convert_legs_angles_back(self.sequence[-1].angles_snapshot)
        return self.angles_history[-1].angles_snapshot

    def initiate_legs(self):
        C1 = Point(cfg.robot.horizontal_x - cfg.robot.x_offset,
                   cfg.robot.horizontal_y - cfg.robot.y_offset,
                   -cfg.robot.vertical)
        self.logger.info('[Init] Initiating leg 1')
        Leg1 = Leg(C1)

        C2 = Point(cfg.robot.x_offset,
                   cfg.robot.horizontal_y - cfg.robot.y_offset + cfg.robot.middle_leg_offset,
                   -cfg.robot.vertical)
        self.logger.info('[Init] Initiating leg 2')
        Leg2 = Leg(C2)

        C3 = Point(-cfg.robot.horizontal_x - cfg.robot.x_offset,
                   cfg.robot.horizontal_y - cfg.robot.y_offset,
                   -cfg.robot.vertical)
        self.logger.info('[Init] Initiating leg 3')
        Leg3 = Leg(C3)

        C4 = Point(-cfg.robot.horizontal_x - cfg.robot.x_offset,
                   -cfg.robot.horizontal_y - cfg.robot.y_offset,
                   -cfg.robot.vertical)
        self.logger.info('[Init] Initiating leg 4')
        Leg4 = Leg(C4)

        C5 = Point(- cfg.robot.x_offset,
                   -cfg.robot.horizontal_y - cfg.robot.y_offset - cfg.robot.middle_leg_offset,
                   -cfg.robot.vertical)
        self.logger.info('[Init] Initiating leg 5')
        Leg5 = Leg(C5)

        C6 = Point(cfg.robot.horizontal_x - cfg.robot.x_offset,
                   -cfg.robot.horizontal_y - cfg.robot.y_offset,
                   -cfg.robot.vertical)
        self.logger.info('[Init] Initiating leg 6')
        Leg6 = Leg(C6)

        self.logger.info('[Init] Initialization successful')

        return {1: Leg1, 2: Leg2, 3: Leg3, 4: Leg4, 5: Leg5, 6: Leg6}
    
    ################## MOVEMENTS START HERE ##################
    def leg_movement(self, leg_num, leg_delta, snapshot=True):
        self.logger.info(f'Leg move {leg_num}: {leg_delta}')
        #leg = self.legs[leg_num]

        self.move_leg_endpoint(leg_num, leg_delta[0], leg_delta[1], leg_delta[2])
        if snapshot:
            self.add_angles_snapshot('endpoint')

    def body_movement(self, delta_x, delta_y, delta_z, snapshot=True):
        self.logger.info(f'Body movement [{delta_x}, {delta_y}, {delta_z}]')
        if delta_x == delta_y == delta_z == 0:
            return

        for leg_num in self.legs.keys():
            self.move_leg_mountpoint(leg_num, delta_x, delta_y, delta_z)
        self.check_tettas()
        if snapshot:
            self.add_angles_snapshot('body')

    def reset(self):
        self.logger.info('Processing reset command')
        self.body_to_center()
        delta_z = -self.legs[1].C.z - cfg.robot.vertical
        self.body_movement(0, 0, -delta_z)

    # ?
    def end(self):
        self.reset()
        self.body_movement(0, 0, -cfg.robot.vertical)
            
    def legs_D_offsets(self):
        x_offset = abs(round((self.legs[1].C.x - self.legs[3].C.x)/2))
        y_offset = abs(round((self.legs[1].C.y - self.legs[6].C.y)/2))
        return {"x": x_offset, "y": y_offset}
    
    def reposition_legs(self, target_x, target_y):
        self.logger.info(f'reposition_legs target:({target_x}, {target_y})')

        target_x_1 = target_x - cfg.robot.x_offset
        target_y_1 = target_y - cfg.robot.y_offset

        target_x_2 = cfg.robot.x_offset
        target_y_2 = target_y - cfg.robot.y_offset + cfg.robot.middle_leg_offset

        target_x_3 = -target_x - cfg.robot.x_offset
        target_y_3 = target_y - cfg.robot.y_offset

        target_x_4 = -target_x - cfg.robot.x_offset
        target_y_4 = -target_y - cfg.robot.y_offset

        target_x_5 = -cfg.robot.x_offset
        target_y_5 = -target_y - cfg.robot.y_offset - cfg.robot.middle_leg_offset

        target_x_6 = target_x - cfg.robot.x_offset
        target_y_6 = -target_y - cfg.robot.y_offset

        delta_x_1 = target_x_1 - self.legs[1].C.x
        delta_y_1 = target_y_1 - self.legs[1].C.y

        delta_x_2 = target_x_2 - self.legs[2].C.x
        delta_y_2 = target_y_2 - self.legs[2].C.y

        delta_x_3 = target_x_3 - self.legs[3].C.x
        delta_y_3 = target_y_3 - self.legs[3].C.y

        delta_x_4 = target_x_4 - self.legs[4].C.x
        delta_y_4 = target_y_4 - self.legs[4].C.y

        delta_x_5 = target_x_5 - self.legs[5].C.x
        delta_y_5 = target_y_5 - self.legs[5].C.y

        delta_x_6 = target_x_6 - self.legs[6].C.x
        delta_y_6 = target_y_6 - self.legs[6].C.y

        #print(f'Deltas1: {delta_x_1, delta_y_1}')
        #print(f'Deltas2: {delta_x_2, delta_y_2}')
        #print(f'Deltas3: {delta_x_3, delta_y_3}')
        #print(f'Deltas4: {delta_x_4, delta_y_4}')
        #print(f'Deltas5: {delta_x_5, delta_y_5}')
        #print(f'Deltas6: {delta_x_6, delta_y_6}')

        self.move_leg_endpoint(2, delta_x_2, delta_y_2, cfg.robot.leg_up)
        self.move_leg_endpoint(4, delta_x_4, delta_y_4, cfg.robot.leg_up)
        self.move_leg_endpoint(6, delta_x_6, delta_y_6, cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

        self.move_leg_endpoint(2, 0, 0, -cfg.robot.leg_up)
        self.move_leg_endpoint(4, 0, 0, -cfg.robot.leg_up)
        self.move_leg_endpoint(6, 0, 0, -cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

        self.move_leg_endpoint(1, delta_x_1, delta_y_1, cfg.robot.leg_up)
        self.move_leg_endpoint(3, delta_x_3, delta_y_3, cfg.robot.leg_up)
        self.move_leg_endpoint(5, delta_x_5, delta_y_5, cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

        self.move_leg_endpoint(1, 0, 0, -cfg.robot.leg_up)
        self.move_leg_endpoint(3, 0, 0, -cfg.robot.leg_up)
        self.move_leg_endpoint(5, 0, 0, -cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

    def switch_mode(self, mode: str):
        self.logger.info(f'Switching mode to {mode}')
        self.reset()
        required_xy = cfg.modes.__getattribute__(cfg.modes, mode)
        current_xy = self.legs_D_offsets()

        print(f'Current_xy: {current_xy}. Required: {required_xy}')
        delta_x = round(required_xy.x - current_xy["x"], 1)
        delta_y = round(required_xy.y - current_xy["y"], 1)
        if abs(delta_x) + abs(delta_y) != 0:
            self.reposition_legs(required_xy.x, required_xy.y)
    
    def body_delta_xy(self, delta_y=cfg.robot.y_offset, delta_x=cfg.robot.x_offset):
        # move body to center
        avg_c_x, avg_c_y = 0, 0
        for leg in self.legs.values():
            avg_c_x += leg.C.x
            avg_c_y += leg.C.y

        avg_c_x /= 6
        avg_c_y /= 6

        return [round(-avg_c_x - delta_x, 2),
                round(-avg_c_y - delta_y, 2)]
                           

    def body_to_center(self, delta_y=cfg.robot.y_offset, delta_x=cfg.robot.x_offset, snapshot=True):
        # move body to center
        
        body_delta_xy = self.body_delta_xy(delta_y, delta_x)
        self.logger.info(f'Moving body: {body_delta_xy}')
        self.body_movement(-body_delta_xy[0],
                           -body_delta_xy[1],
                           0, 
                           snapshot)

    def turn_move(self, angle_deg, mode=None):
        self.turn(-angle_deg, mode=mode)
        self.turn(angle_deg, only_body=True)

    def turn(self, angle_deg, only_body=False, mode=None):
        angle = math.radians(angle_deg)

        center_x = 0
        center_y = 0

        leg_up = cfg.robot.leg_up
        if mode == "fb":
            leg_up *= 2

        for leg_num in [2, 4, 6]:
            leg = self.legs[leg_num]
            x_new, y_new = turn_on_angle(center_x, center_y, leg.C.x, leg.C.y, angle)
            delta_x = x_new - leg.C.x
            delta_y = y_new - leg.C.y

            self.move_leg_endpoint(leg_num, delta_x, delta_y, leg_up)

        if not only_body:
            self.add_angles_snapshot()

        for leg_num in [2, 4, 6]:
            self.move_leg_endpoint(leg_num, 0, 0, -leg_up)

        if not only_body:
            self.add_angles_snapshot()

        for leg_num in [1, 3, 5]:
            leg = self.legs[leg_num]
            x_new, y_new = turn_on_angle(center_x, center_y, leg.C.x, leg.C.y, angle)
            delta_x = x_new - leg.C.x
            delta_y = y_new - leg.C.y

            self.move_leg_endpoint(leg_num, delta_x, delta_y, leg_up)

        if not only_body:
            self.add_angles_snapshot()

        for leg_num in [1, 3, 5]:
            self.move_leg_endpoint(leg_num, 0, 0, -leg_up)

        self.add_angles_snapshot('endpoint')

        self.body_to_center()

    """
    Two phased moves
    """
    # phased 2-legged movement
    def move_2_legs_phased_13(self, delta_x: int = 0, delta_y: int = 0) -> None:
        self.body_movement(round(delta_x / 2, 1), round(delta_y / 2, 1), 0)
        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg_num in [1, 3, 5]:
            self.move_leg_endpoint(leg_num, delta_x, delta_y, cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg_num in [1, 3, 5]:
            self.move_leg_endpoint(leg_num, 0, 0, -cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')
        
    def move_2_legs_phased_24(self, delta_x: int = 0, delta_y: int = 0) -> None:
        self.body_movement(round(delta_x / 2, 1), round(delta_y / 2, 1), 0)
        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg_num in [2, 4, 6]:
            self.move_leg_endpoint(leg_num, delta_x, delta_y, cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg_num in [2, 4, 6]:
            self.move_leg_endpoint(leg_num, 0, 0, -cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')      
    
    
    """
    Two phased moves end
    """

    def wave_gait(self):
        self.move_leg_endpoint(1, 9, 0, cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

        for leg_num, legs_prev in zip([2, 3, 6, 5, 4], [1, 2, 3, 6, 5]):
            self.move_leg_endpoint(legs_prev, 0, 0, -cfg.robot.leg_up)
            self.move_leg_endpoint(leg_num, 9, 0, cfg.robot.leg_up)
            #self.add_angles_snapshot('endpoints')
            #leg.move_end_point(0, 0, -cfg.robot.leg_up)
            #self.add_angles_snapshot('endpoints')
            self.body_movement(1.5, 0, 0)
        self.move_leg_endpoint(4, 0, 0, -cfg.robot.leg_up)
        self.body_movement(1.5, 0, 0)
    
    def ripple_gait(self):
        self.move_leg_endpoint(1, 9, 0, cfg.robot.leg_up)
        self.move_leg_endpoint(4, 9, 0, cfg.robot.leg_up)
        self.body_movement(1.5, 0, 0, False)
        self.add_angles_snapshot('endpoint')

        self.move_leg_endpoint(1, 0, 0, -cfg.robot.leg_up)
        self.move_leg_endpoint(4, 0, 0, -cfg.robot.leg_up)
        self.move_leg_endpoint(2, 9, 0, cfg.robot.leg_up)
        self.move_leg_endpoint(6, 9, 0, cfg.robot.leg_up)
        self.body_movement(3, 0, 0, False)
        self.add_angles_snapshot('endpoint')

        self.move_leg_endpoint(2, 0, 0, -cfg.robot.leg_up)
        self.move_leg_endpoint(6, 0, 0, -cfg.robot.leg_up)
        self.move_leg_endpoint(3, 9, 0, cfg.robot.leg_up)
        self.move_leg_endpoint(5, 9, 0, cfg.robot.leg_up)
        self.body_movement(3, 0, 0, False)
        self.add_angles_snapshot('endpoint')

        self.move_leg_endpoint(3, 0, 0, -cfg.robot.leg_up)
        self.move_leg_endpoint(5, 0, 0, -cfg.robot.leg_up)
        self.body_movement(1.5, 0, 0, False)
        self.add_angles_snapshot('endpoint')

    
    def hit(self):
        self.body_movement(-7, 0, 0)
        for leg_num in [1, 6]:
            self.move_leg_endpoint(leg_num, 0, 0, 7 + cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

        self.move_leg_endpoint(1, 0, -10, 0)
        self.move_leg_endpoint(6, 0, 10, 0)
        self.add_angles_snapshot('endpoints')

        forward_move = 8
        self.move_leg_endpoint(1, forward_move, 0, 0)
        self.add_angles_snapshot('endpoints')

        self.move_leg_endpoint(1, -forward_move, 0, 0)
        self.move_leg_endpoint(6, forward_move, 0, 0)
        self.add_angles_snapshot('endpoints')

        self.move_leg_endpoint(6, -forward_move, 0, 0)
        self.add_angles_snapshot('endpoints')

        self.move_leg_endpoint(1, 0, 10, 0)
        self.move_leg_endpoint(6, 0, -10, 0)
        self.add_angles_snapshot('endpoints')

        for leg_num in [1, 6]:
            self.move_leg_endpoint(leg_num, 0, 0, -7 - cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')
        self.body_movement(7, 0, 0)
    
    """
    def play1(self):
        self.body_movement(0, 0, 5)
        for leg in [self.legs[1], self.legs[2], self.legs[3]]:
            leg.move_end_point(0, 0, 10)
        for leg in [self.legs[4], self.legs[5], self.legs[6]]:
            leg.move_end_point(0, 0, -10)
        self.add_angles_snapshot('endpoints')
        for leg in [self.legs[1], self.legs[2], self.legs[3]]:
            leg.move_end_point(0, 0, -20)
        for leg in [self.legs[4], self.legs[5], self.legs[6]]:
            leg.move_end_point(0, 0, 20)
        self.add_angles_snapshot('endpoints')
        for leg in [self.legs[1], self.legs[2], self.legs[3]]:
            leg.move_end_point(0, 0, 10)
        for leg in [self.legs[4], self.legs[5], self.legs[6]]:
            leg.move_end_point(0, 0, -10)
        self.add_angles_snapshot('endpoints')
        
        self.body_movement(7, 0, 0)
        self.body_movement(0, 5, 0)
        self.body_movement(-14, 0, 0)
        self.body_movement(0, -10, 0)
        self.body_movement(14, 0, 0)
        self.body_movement(-7, 5, -5)
    """
    # feedback moves
    def leg_move_custom(self, leg_num, mode, leg_delta=[0, 0, 0], add_snapshot=True):
        if mode == 'touch':
            iterations = 3
        else:
            iterations = 1
        for i in range(iterations):
            self.move_leg_endpoint(
                leg_num, 
                round(leg_delta[0]/iterations, 1), 
                round(leg_delta[1]/iterations, 1),
                round(leg_delta[2]/iterations, 1), 
                mode,
                add_snapshot=add_snapshot
            )
    
    def move_body_abs(self, z):
        min_z = min([leg.C.z for leg in self.legs.values()])
        print(f'Min_z, z, z + min_z: {min_z, z, z + min_z}')
        self.body_movement(0, 0, z + min_z)

    def move_leg_endpoint_abs(self, leg_num, leg_delta, snapshot_type='endpoint', add_snapshot=True):
        min_z = min([leg.C.z for leg in self.legs.values()])
        
        leg = self.legs[leg_num]
        target_x = leg_delta[0]
        if leg_delta[0] is None:
            target_x = leg.C.x
        
        target_y = leg_delta[1]
        if leg_delta[1] is None:
            target_y = leg.C.y

        target_z = leg_delta[2]
        if leg_delta[2] is None:
            target_z = leg.C.z - min_z
        print(f'target_z, leg.C.z, min_z: {(target_z, leg.C.z, min_z)}')
        new_delta = [round(target_x - leg.C.x, 1), round(target_y - leg.C.y, 1), round(target_z - leg.C.z + min_z, 1)]
        print(f'Legnum: {leg_num}.\nOriginal delta: {leg_delta}\nNew delta: {new_delta}')
        self.logger.info(f'move_leg_endpoint_abs. Legnum: {leg_num}.\nOriginal delta: {leg_delta}\nNew delta: {new_delta}')
        self.move_leg_endpoint(leg_num, *new_delta)
        #self.legs_deltas[leg_num] = [x + y for x, y in zip(self.legs_deltas[leg_num], leg_delta)]        
        if add_snapshot:
            self.add_angles_snapshot(snapshot_type)

if __name__ == '__main__':
    rk = Kinematics()
    #rk.body_movement(0, 0, -5)
    print(rk.sequence[-1].angles_snapshot)
    # C before: Point(x=20.0, y=-10.0, z=-10.0)
    # C after: Point(x=10.0, y=-10.0, z=-10.0)

    # C before: Point(x=10.0, y=10.0, z=-10.0)
    # C before: Point(x=20.0, y=10.0, z=-10.0)
    # C after: Point(x=10.0, y=10.0, z=-10.0)