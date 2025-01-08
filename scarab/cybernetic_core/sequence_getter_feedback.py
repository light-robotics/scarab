from typing import List, Tuple
import sys
import os
import math
from joblib import Memory
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from cybernetic_core.kinematics import Kinematics
from configs import config as cfg
from configs import code_config
from cybernetic_core.geometry.angles import RobotPosition

#from functools import cache
memory = Memory(code_config.cache_dir, verbose=0)

UP_OR_DOWN_CM   = cfg.moves.up_or_down_cm
FORWARD_BODY_CM = cfg.moves.move_body_cm
FORWARD_LEGS_1LEG_CM = cfg.moves.forward_body_1_leg_cm
FORWARD_LEGS_2LEG_CM = cfg.moves.forward_body_2_leg_cm
REPOSITION_CM   = cfg.moves.reposition_cm


class Move:
    def __init__(self, move_type, values):
        self.move_type = move_type
        self.values = values
    
    def __repr__(self):
        return f'Move({self.move_type}, {self.values})'

#@cache
#@memory.cache
def get_sequence_for_command(command: str, kwargs=None):
    sequence = []
    
    if command == 'up':
        sequence.append(Move('body_movement', {'deltas': [0, 0, UP_OR_DOWN_CM]}))
    elif command == 'down':
        sequence.append(Move('body_movement', {'deltas': [0, 0, -UP_OR_DOWN_CM]}))
    elif command == 'balance':
        sequence.append(Move('balance', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('balance', {}))
    
    #elif command == 'forward_two_legged':
    elif command == 'forward_32':
    #elif command == 'forward_one_legged':        
        sequence.append(Move('endpoints', {'legs': [1, 3], 'deltas': [8, 0, 20]}))
        sequence.append(Move('down_2_legs', {'legs': [1, 3]}))
        sequence.append(Move('down_2_legs', {'legs': [1, 3]}))
        sequence.append(Move('down_2_legs', {'legs': [1, 3]}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('down_leg_up', {}))
        #sequence.append(Move('body_to_center', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('body_movement', {'deltas': [4, 0, 0]}))
        sequence.append(Move('endpoints', {'legs': [2, 4], 'deltas': [8, 0, 20]}))
        sequence.append(Move('down_2_legs', {'legs': [2, 4]}))
        sequence.append(Move('down_2_legs', {'legs': [2, 4]}))
        sequence.append(Move('down_2_legs', {'legs': [2, 4]}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('body_movement', {'deltas': [4, 0, 0]}))
    elif command == 'wave_gait':
        for leg in [1, 2, 3, 6, 5, 4]:
            sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [None, None, 12]}))
            sequence.append(Move('endpoint', {'leg': leg, 'deltas': [FORWARD_LEGS_1LEG_CM, 0, 0]}))            
            sequence.append(Move('touch', {'leg': leg}))
            sequence.append(Move('body_movement', {'deltas': [FORWARD_LEGS_1LEG_CM/6, 0, 0]}))
        
    elif command in ['battle_mode', 'sentry_mode', 'walking_mode', 'run_mode']:
        sequence.append(Move('switch_mode', {"mode": command}))
    else:
        print(f'Unknown command')
    
    #print(f'[SG]. Sequence commands: {sequence}')
    return sequence

def get_angles_for_sequence(move: Move, robot_position: RobotPosition):
    rk = Kinematics(robot_position=robot_position, init_snapshot=False)
    print(f'Move: {move.move_type}. {move.values}')
    # print(f'robot_position: {robot_position}')

    if move.move_type == 'body_movement':
        rk.body_movement(*move.values['deltas'])
    elif move.move_type == 'body_to_center':
        rk.body_to_center()
    elif move.move_type == 'endpoint':
        rk.move_leg_endpoint(move.values['leg'], move.values['deltas'])
    elif move.move_type == 'endpoint_absolute':
        rk.move_leg_endpoint_abs(move.values['leg'], move.values['deltas'])
    elif move.move_type == 'endpoints':
        rk.move_leg_endpoint(move.values['legs'][0], move.values['deltas'], add_snapshot=False)
        rk.move_leg_endpoint(move.values['legs'][1], move.values['deltas'])
    elif move.move_type == 'endpoint_normalized':
        leg_num = move.values['leg']
        leg = rk.legs[leg_num]
          
        delta = move.values['deltas']
        rk.move_leg_endpoint(move.values['leg'], delta)
    elif move.move_type == 'touch':
        rk.leg_move_custom(move.values['leg'], 'touch', [0, 0, -12])
        #rk.move_leg_endpoint(move.values['leg'], [0, 0, -2])
    
    elif move.move_type == 'switch_mode':
        rk.switch_mode(move.values['mode'])

    #print(f'[SG]. Sequence: {rk.sequence[-1]}')
    return rk.sequence
