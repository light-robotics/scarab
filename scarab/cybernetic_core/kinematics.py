import math
import copy
from dataclasses import dataclass
from typing import List, Dict
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from configs import config as cfg
from cybernetic_core.geometry.angles import RobotPosition, calculate_leg_angles, turn_on_angle, convert_legs_angles_C, calculate_C_point
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

    def move_mount_point(self, delta_x, delta_y, delta_z):
        self.C.move(-delta_x, -delta_y, -delta_z)
        self.update_angles()
    
    def move_end_point(self, delta_x, delta_y, delta_z):
        self.C.move(delta_x, delta_y, delta_z)
        #print(f'D Move: {delta_x, delta_y, delta_z}')
        self.update_angles()

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
        return sum([(leg.O.z - leg.D.z) for leg in self.legs.values()])/4
        #return self.current_legs_offset_v

    @property
    def sequence(self):
        sequence = []
        for move in self.angles_history:
            sequence.append(MoveSnapshot(move.move_type, convert_legs_angles_C(move.angles_snapshot, self.logger)))
        return sequence
        #return self.angles_history
    
    def build_legs_from_angles(self, rp: RobotPosition):
        C1 = calculate_C_point(rp.legs[1])
        self.logger.info('[Init] Building leg 1')
        #print(f'Building leg 1: {math.degrees(alpha1), math.degrees(beta1), math.degrees(gamma1)}')
        Leg1 = Leg(C1)

        C2 = calculate_C_point(rp.legs[2])
        self.logger.info('[Init] Building leg 2')
        Leg2 = Leg(C2)
        #print(f'Leg2.2:{[round(math.degrees(x), 2) for x in [Leg2.tetta, Leg2.alpha, Leg2.beta, Leg2.gamma]]}')

        C3 = calculate_C_point(rp.legs[3])
        self.logger.info('[Init] Building leg 3')
        Leg3 = Leg(C3)

        C4 = calculate_C_point(rp.legs[4])
        self.logger.info('[Init] Building leg 4')
        Leg4 = Leg(C4)

        C5 = calculate_C_point(rp.legs[5])
        self.logger.info('[Init] Building leg 5')
        Leg5 = Leg(C5)

        C6 = calculate_C_point(rp.legs[6])
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
                   cfg.robot.horizontal_y - cfg.robot.y_offset,
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
                   -cfg.robot.horizontal_y - cfg.robot.y_offset,
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
        leg = self.legs[leg_num]

        leg.move_end_point(leg_delta[0], leg_delta[1], leg_delta[2])
        if snapshot:
            self.add_angles_snapshot('endpoint')

    def body_movement(self, delta_x, delta_y, delta_z, snapshot=True):
        self.logger.info(f'Body movement [{delta_x}, {delta_y}, {delta_z}]')
        if delta_x == delta_y == delta_z == 0:
            return

        for leg in self.legs.values():
            leg.move_mount_point(delta_x, delta_y, delta_z)

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
    
    def switch_mode(self, mode: str):
        self.logger.info(f'Switching mode to {mode}')
        self.reset()
        required_xy = cfg.modes[mode]
        current_xy = self.legs_D_offsets()

        print(f'Current_xy: {current_xy}. Required: {required_xy}')
        delta_x = required_xy["x"] - current_xy["x"]
        delta_y = required_xy["y"] - current_xy["y"]
        if abs(delta_x) + abs(delta_y) != 0:
            self.reposition_legs(delta_x, delta_y)
    
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

    def move_leg_endpoint(self, leg_num, leg_delta, snapshot_type='endpoint', add_snapshot=True):        
        self.legs[leg_num].move_end_point(*leg_delta)
        if add_snapshot:
            self.add_angles_snapshot(snapshot_type)
        print(f'move_leg_endpoint. Leg {leg_num}. C: {self.legs[leg_num].C}')

    """
    Two phased moves
    """
    # phased 2-legged movement
    def move_2_legs_phased_13(self, delta_x: int = 0, delta_y: int = 0) -> None:
        self.body_movement(round(delta_x / 2, 1), round(delta_y / 2, 1), 0)
        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[1], self.legs[3], self.legs[5]]:
            leg.move_end_point(delta_x, delta_y, cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[1], self.legs[3], self.legs[5]]:
            leg.move_end_point(0, 0, -cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')
        
    def move_2_legs_phased_24(self, delta_x: int = 0, delta_y: int = 0) -> None:
        self.body_movement(round(delta_x / 2, 1), round(delta_y / 2, 1), 0)
        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[2], self.legs[4], self.legs[6]]:
            leg.move_end_point(delta_x, delta_y, cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')

        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[2], self.legs[4], self.legs[6]]:
            leg.move_end_point(0, 0, -cfg.robot.leg_up)
        self.add_angles_snapshot('endpoints')      
    
    
    """
    Two phased moves end
    """
    
if __name__ == '__main__':
    rk = Kinematics()
    rk.body_movement(0, 0, -5)
    #print(rk.sequence[-1].angles_snapshot)
