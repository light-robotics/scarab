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
    def __init__(self, move_type, values, snapshot=True):
        self.move_type = move_type
        self.values = values
        self.snapshot = snapshot
    
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
    elif command in [
        'forward_32', 
        'forward_22', 
        'forward_1', 
        'forward_2', 
        'forward_3',
        'backward_1',
        'backward_2',
        'backward_22',
        'backward_3',
        'backward_32',
        'strafe_right_1',
        'strafe_right_2',
        'strafe_right_22',
        'strafe_right_3',
        'strafe_right_32',
        'strafe_left_1',
        'strafe_left_2',
        'strafe_left_22',
        'strafe_left_3',
        'strafe_left_32',
        'diagonal_forward_right',
        'diagonal_forward_left',
        'diagonal_backward_right',
        'diagonal_backward_left',
        'turn_left_fb',
        'turn_right_fb'
        ]:
        sequence.append(Move(command, {}))
    elif command == 'forward_two_legged_fb':
        for _ in range(cfg.robot.balance_iterations):
            sequence.append(Move('balance', {}))
        
        #sequence.append(Move('body_movement', {'deltas': [0, 0, 3*UP_OR_DOWN_CM]}))
        sequence.append(Move('body_absolute', {'z': cfg.robot.feedback_body_up}))
        
        sequence.append(Move('endpoint_absolute', {
                'leg': [1, 3, 5], 
                'deltas': {
                    1: [None, None, cfg.robot.touch_up],
                    3: [None, None, cfg.robot.touch_up],
                    5: [None, None, cfg.robot.touch_up]
                }
                }))
    
        sequence.append(Move('endpoint_absolute', {
                'leg': [1, 3, 5], 
                'deltas': {
                    1: [
                        cfg.modes.walking_mode.x + cfg.moves.forward_body_2_leg_cm, 
                        cfg.modes.walking_mode.y, 
                        None
                    ],
                    3: [
                        -cfg.modes.walking_mode.x + cfg.moves.forward_body_2_leg_cm, 
                        cfg.modes.walking_mode.y, 
                        None
                    ],
                    5: [
                        cfg.moves.forward_body_2_leg_cm, 
                        -cfg.modes.walking_mode.y - cfg.robot.middle_leg_offset, 
                        None
                    ]
                }
                }))

        for _ in range(cfg.robot.touch_down_iterations):
            sequence.append(Move('touch', {}))

        for _ in range(cfg.robot.balance_iterations):
            sequence.append(Move('balance', {}))
        
        sequence.append(
            Move('body_movement', 
                {'deltas': [round(cfg.moves.forward_body_2_leg_cm, 1), 0, 0]}
            ))

        for _ in range(cfg.robot.balance_iterations):
            sequence.append(Move('balance', {}))
    
        sequence.append(Move('body_absolute', {'z': cfg.robot.feedback_body_up}))

        sequence.append(Move('endpoint_absolute', 
                             {'leg': [2, 4, 6],
                            'deltas': {
                                2: [None, None, cfg.robot.touch_up],
                                4: [None, None, cfg.robot.touch_up],
                                6: [None, None, cfg.robot.touch_up]
                            }
                            }))
        
        sequence.append(Move('endpoint_absolute',
                    {'leg': [2, 4, 6], 
                     'deltas': { 
                        2: [
                             0, 
                             cfg.modes.walking_mode.y + cfg.robot.middle_leg_offset, 
                             None
                        ],
                        4: [
                            -cfg.modes.walking_mode.x, 
                            -cfg.modes.walking_mode.y, 
                            None
                        ],
                        6: [
                            cfg.modes.walking_mode.x, 
                            -cfg.modes.walking_mode.y, 
                            None
                        ]
                     }
                    }))        
        
        for _ in range(cfg.robot.touch_down_iterations):
            sequence.append(Move('touch', {}))

        for _ in range(cfg.robot.balance_iterations):
            sequence.append(Move('balance', {}))
        
        #sequence.append(
        #    Move('body_movement', 
        #        {'deltas': [round(cfg.moves.forward_body_2_leg_cm / 2, 1), 0, 0]}
        #    ))
    elif command == 'backward_two_legged_fb':
        # moving forward down, not backwards
        for _ in range(cfg.robot.balance_iterations):
            sequence.append(Move('balance', {}))
        
        #sequence.append(Move('body_movement', {'deltas': [0, 0, 3*UP_OR_DOWN_CM]}))
        sequence.append(Move('body_absolute', {'z': cfg.robot.feedback_body_up/2}))
        
        sequence.append(Move('endpoint_absolute', {
                'leg': [1, 3, 5], 
                'deltas': {
                    1: [None, None, cfg.robot.touch_up/2],
                    3: [None, None, cfg.robot.touch_up/2],
                    5: [None, None, cfg.robot.touch_up/2]
                }
                }))
    
        sequence.append(Move('endpoint_absolute', {
                'leg': [1, 3, 5], 
                'deltas': {
                    1: [
                        cfg.modes.walking_mode.x + cfg.moves.forward_body_2_leg_cm, 
                        cfg.modes.walking_mode.y, 
                        None
                    ],
                    3: [
                        -cfg.modes.walking_mode.x + cfg.moves.forward_body_2_leg_cm, 
                        cfg.modes.walking_mode.y, 
                        None
                    ],
                    5: [
                        cfg.moves.forward_body_2_leg_cm, 
                        -cfg.modes.walking_mode.y - cfg.robot.middle_leg_offset, 
                        None
                    ]
                }
                }))

        for _ in range(cfg.robot.touch_down_iterations):
            sequence.append(Move('touch', {}))

        for _ in range(cfg.robot.balance_iterations):
            sequence.append(Move('balance', {}))
        
        sequence.append(
            Move('body_movement', 
                {'deltas': [round(cfg.moves.forward_body_2_leg_cm, 1), 0, 0]}
            ))

        sequence.append(Move('endpoint_absolute', 
                             {'leg': [2, 4, 6],
                            'deltas': {
                                2: [None, None, cfg.robot.touch_up/2],
                                4: [None, None, cfg.robot.touch_up/2],
                                6: [None, None, cfg.robot.touch_up/2]
                            }
                            }))
        
        sequence.append(Move('endpoint_absolute',
                    {'leg': [2, 4, 6], 
                     'deltas': { 
                        2: [
                             0, 
                             cfg.modes.walking_mode.y + cfg.robot.middle_leg_offset, 
                             None
                        ],
                        4: [
                            -cfg.modes.walking_mode.x, 
                            -cfg.modes.walking_mode.y, 
                            None
                        ],
                        6: [
                            cfg.modes.walking_mode.x, 
                            -cfg.modes.walking_mode.y, 
                            None
                        ]
                     }
                    }))        
        
        for _ in range(cfg.robot.touch_down_iterations):
            sequence.append(Move('touch', {}))

        for _ in range(cfg.robot.balance_iterations):
            sequence.append(Move('balance', {}))
        
        #sequence.append(
        #    Move('body_movement', 
        #        {'deltas': [round(cfg.moves.forward_body_2_leg_cm / 2, 1), 0, 0]}
        #    ))

    elif command == 'reset':
        #for _ in range(cfg.robot.balance_iterations):
        #    sequence.append(Move('balance', {}))
        
        #sequence.append(Move('body_movement', {'deltas': [0, 0, 3*UP_OR_DOWN_CM]}))
        #sequence.append(Move('body_absolute', {'z': cfg.robot.reset_feedback_body_up}))
        
        sequence.append(Move('endpoint_absolute', {
                'leg': [1, 3, 5], 
                'deltas': {
                    1: [cfg.modes.walking_mode.x, cfg.modes.walking_mode.y, cfg.robot.reset_touch_up],
                    3: [-cfg.modes.walking_mode.x, cfg.modes.walking_mode.y, cfg.robot.reset_touch_up],
                    5: [0, -cfg.modes.walking_mode.y - cfg.robot.middle_leg_offset, cfg.robot.reset_touch_up]
                }
                }))
    
        for _ in range(cfg.robot.touch_down_iterations):
            sequence.append(Move('touch', {}))

        #for _ in range(cfg.robot.balance_iterations):
        #    sequence.append(Move('balance', {}))
        
        sequence.append(Move('endpoint_absolute', 
                             {'leg': [2, 4, 6],
                            'deltas': {
                                2: [0, cfg.modes.walking_mode.y + cfg.robot.middle_leg_offset, cfg.robot.reset_touch_up],
                                4: [-cfg.modes.walking_mode.x, -cfg.modes.walking_mode.y, cfg.robot.reset_touch_up],
                                6: [cfg.modes.walking_mode.x, -cfg.modes.walking_mode.y, cfg.robot.reset_touch_up]
                            }
                            }))
        
        for _ in range(cfg.robot.touch_down_iterations):
            sequence.append(Move('touch', {}))

        #for _ in range(cfg.robot.balance_iterations):
        #    sequence.append(Move('balance', {}))
    elif command == 'reset_onelegged':
        #for _ in range(cfg.robot.balance_iterations):
        #    sequence.append(Move('balance', {}))

        leg_deltas = {
            1: [cfg.modes.walking_mode.x, cfg.modes.walking_mode.y, cfg.robot.reset_touch_up],
            2: [0, cfg.modes.walking_mode.y + cfg.robot.middle_leg_offset, cfg.robot.reset_touch_up],
            3: [-cfg.modes.walking_mode.x, cfg.modes.walking_mode.y, cfg.robot.reset_touch_up],
            4: [-cfg.modes.walking_mode.x, -cfg.modes.walking_mode.y, cfg.robot.reset_touch_up],
            5: [0, -cfg.modes.walking_mode.y - cfg.robot.middle_leg_offset, cfg.robot.reset_touch_up],
            6: [cfg.modes.walking_mode.x, -cfg.modes.walking_mode.y, cfg.robot.reset_touch_up]
        }        
        for leg in [1, 2, 3, 4, 5, 6]:
            sequence.append(Move('endpoint_absolute', {
                    'leg': [leg], 
                    'deltas': {
                        leg: leg_deltas[leg]
                    }
                    }))
        
            for _ in range(cfg.robot.touch_down_iterations):
                sequence.append(Move('touch', {}))

            #for _ in range(cfg.robot.balance_iterations):
            #    sequence.append(Move('balance', {}))

    elif command == 'wave_gait':
        #sequence.append(Move('body_movement', {'deltas': [cfg.moves.forward_body_1_leg_cm/2, 0, 0]}))
        for leg in [1, 2, 3, 6, 5, 4]:            
            sequence.append(Move('endpoint_absolute', {'leg': [leg], 'deltas': [None, None, cfg.robot.touch_up]}))
            #sequence.append(Move('endpoint', {'leg': leg, 'deltas': [FORWARD_LEGS_1LEG_CM, 0, 0]}))
            if leg == 1:
                sequence.append(Move('endpoint_absolute', 
                    {'leg': [1], 'deltas': 
                        [cfg.modes.walking_mode.x + cfg.moves.forward_body_1_leg_cm, # - cfg.moves.forward_body_1_leg_cm/6, 
                         cfg.modes.walking_mode.y, None]}))
            elif leg == 2:
                sequence.append(Move('endpoint_absolute', 
                    {'leg': [2], 'deltas': [
                        cfg.moves.forward_body_1_leg_cm, # - 2*cfg.moves.forward_body_1_leg_cm/6, 
                        cfg.modes.walking_mode.y + cfg.robot.middle_leg_offset, None]}))
            elif leg == 3:
                sequence.append(Move('endpoint_absolute', 
                    {'leg': [3], 'deltas': [
                        -cfg.modes.walking_mode.x + cfg.moves.forward_body_1_leg_cm, # - 3*cfg.moves.forward_body_1_leg_cm/6, 
                        cfg.modes.walking_mode.y, None]}))
            elif leg == 6:
                sequence.append(Move('endpoint_absolute', 
                    {'leg': [6], 'deltas': [
                        cfg.modes.walking_mode.x + cfg.moves.forward_body_1_leg_cm, # - 4*cfg.moves.forward_body_1_leg_cm/6, 
                        -cfg.modes.walking_mode.y, None]}))
            elif leg == 5:
                sequence.append(Move('endpoint_absolute', 
                    {'leg': [5], 'deltas': [
                        cfg.moves.forward_body_1_leg_cm, # - 5*cfg.moves.forward_body_1_leg_cm/6, 
                        -cfg.modes.walking_mode.y - cfg.robot.middle_leg_offset, None]}))
            elif leg == 4:
                sequence.append(Move('endpoint_absolute', 
                    {'leg': [4], 'deltas': [
                        -cfg.modes.walking_mode.x + cfg.moves.forward_body_1_leg_cm, # - cfg.moves.forward_body_1_leg_cm, 
                        -cfg.modes.walking_mode.y, None]}))

            for _ in range(cfg.robot.touch_down_iterations):
                sequence.append(Move('touch', {}))
            
            sequence.append(Move('endpoint', {'leg': [leg], 'deltas': [0, 0, -3]}))
        sequence.append(Move('body_movement', {'deltas': [cfg.moves.forward_body_1_leg_cm, 0, 0]}))
        
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
    if move.move_type == 'add_snapshot':
        rk.add_angles_snapshot('endpoint')
    elif move.move_type == 'body_movement':
        rk.body_movement(*move.values['deltas'])
    elif move.move_type == 'forward_1':
        # Legs 1 and 3 moved x1
        rk.move_2_legs_phased_13(FORWARD_LEGS_2LEG_CM, 0)
    elif move.move_type == 'forward_2':
        # Legs 2 and 4 moved x2
        rk.move_2_legs_phased_24(2 * FORWARD_LEGS_2LEG_CM, 0)
    elif move.move_type == 'forward_22':
        # Legs 2 and 4 moved x1
        rk.move_2_legs_phased_24(FORWARD_LEGS_2LEG_CM, 0)
    elif move.move_type == 'forward_3':
        # Legs 1 and 3 moved x2
        for _ in range(3):
            rk.move_2_legs_phased_13(2 * FORWARD_LEGS_2LEG_CM, 0)
            rk.move_2_legs_phased_24(2 * FORWARD_LEGS_2LEG_CM, 0)
        rk.move_2_legs_phased_13(2 * FORWARD_LEGS_2LEG_CM, 0)
    elif move.move_type == 'forward_32':
        # Legs 1 and 3 moved x1
        rk.move_2_legs_phased_13(FORWARD_LEGS_2LEG_CM, 0)
    elif move.move_type == 'backward_1':
        # Legs 1 and 3 moved x1
        rk.move_2_legs_phased_13(-FORWARD_LEGS_2LEG_CM, 0)
    elif move.move_type == 'backward_2':
        # Legs 2 and 4 moved x2
        rk.move_2_legs_phased_24(-2 * FORWARD_LEGS_2LEG_CM, 0)
    elif move.move_type == 'backward_22':
        # Legs 2 and 4 moved x1
        rk.move_2_legs_phased_24(-FORWARD_LEGS_2LEG_CM, 0)
    elif move.move_type == 'backward_3':
        # Legs 1 and 3 moved x2
        for _ in range(3):
            rk.move_2_legs_phased_13(-2 * FORWARD_LEGS_2LEG_CM, 0)
            rk.move_2_legs_phased_24(-2 * FORWARD_LEGS_2LEG_CM, 0)
        rk.move_2_legs_phased_13(-2 * FORWARD_LEGS_2LEG_CM, 0)
    elif move.move_type == 'backward_32':
        # Legs 1 and 3 moved x1
        rk.move_2_legs_phased_13(-FORWARD_LEGS_2LEG_CM, 0)
    
    elif move.move_type == 'strafe_right_1':
        # Legs 1 and 3 moved x1
        rk.move_2_legs_phased_13(0, -FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'strafe_right_2':
        # Legs 2 and 4 moved x2
        rk.move_2_legs_phased_24(0, -2 * FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'strafe_right_22':
        # Legs 2 and 4 moved x1
        rk.move_2_legs_phased_24(0, -FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'strafe_right_3':
        # Legs 1 and 3 moved x2
        for _ in range(3):
            rk.move_2_legs_phased_13(0, -2 * FORWARD_LEGS_2LEG_CM)
            rk.move_2_legs_phased_24(0, -2 * FORWARD_LEGS_2LEG_CM)
        rk.move_2_legs_phased_13(0, -2 * FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'strafe_right_32':
        # Legs 1 and 3 moved x1
        rk.move_2_legs_phased_13(0, -FORWARD_LEGS_2LEG_CM)

    elif move.move_type == 'strafe_left_1':
        # Legs 1 and 3 moved x1
        rk.move_2_legs_phased_13(0, FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'strafe_left_2':
        # Legs 2 and 4 moved x2
        rk.move_2_legs_phased_24(0, 2 * FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'strafe_left_22':
        # Legs 2 and 4 moved x1
        rk.move_2_legs_phased_24(0, FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'strafe_left_3':
        # Legs 1 and 3 moved x2
        for _ in range(3):
            rk.move_2_legs_phased_13(0, 2 * FORWARD_LEGS_2LEG_CM)
            rk.move_2_legs_phased_24(0, 2 * FORWARD_LEGS_2LEG_CM)
        rk.move_2_legs_phased_13(0, 2 * FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'strafe_left_32':
        # Legs 1 and 3 moved x1
        rk.move_2_legs_phased_13(0, FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'diagonal_forward_right':
        rk.move_2_legs_phased_13(FORWARD_LEGS_2LEG_CM, -FORWARD_LEGS_2LEG_CM)
        rk.move_2_legs_phased_24(FORWARD_LEGS_2LEG_CM, -FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'diagonal_forward_left':
        rk.move_2_legs_phased_13(FORWARD_LEGS_2LEG_CM, FORWARD_LEGS_2LEG_CM)
        rk.move_2_legs_phased_24(FORWARD_LEGS_2LEG_CM, FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'diagonal_backward_right':
        rk.move_2_legs_phased_13(-FORWARD_LEGS_2LEG_CM, -FORWARD_LEGS_2LEG_CM)
        rk.move_2_legs_phased_24(-FORWARD_LEGS_2LEG_CM, -FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'diagonal_backward_left':
        rk.move_2_legs_phased_13(-FORWARD_LEGS_2LEG_CM, FORWARD_LEGS_2LEG_CM)
        rk.move_2_legs_phased_24(-FORWARD_LEGS_2LEG_CM, FORWARD_LEGS_2LEG_CM)
    elif move.move_type == 'turn_left_fb':
        rk.turn_move(30, mode='fb')
    elif move.move_type == 'turn_right_fb':
        rk.turn_move(-30, mode='fb')
    elif move.move_type == 'body_to_center':
        rk.body_to_center()
    elif move.move_type == 'endpoint':
        for leg in move.values['leg']:
            rk.move_leg_endpoint(leg, move.values['deltas'], add_snapshot=False)
        rk.add_angles_snapshot('endpoint')
    elif move.move_type == 'endpoint_absolute':
        for leg in move.values['leg']:
            rk.move_leg_endpoint_abs(leg, move.values['deltas'][leg], add_snapshot=False)
        if move.snapshot:
            rk.add_angles_snapshot('endpoint')
    elif move.move_type == 'body_absolute':
        rk.move_body_abs(move.values['z'])
    elif move.move_type == 'endpoints':
        rk.move_leg_endpoint(move.values['legs'][0], move.values['deltas'], add_snapshot=False)
        rk.move_leg_endpoint(move.values['legs'][1], move.values['deltas'])
    elif move.move_type == 'endpoint_normalized':
        leg_num = move.values['leg']
        leg = rk.legs[leg_num]
          
        delta = move.values['deltas']
        rk.move_leg_endpoint(move.values['leg'], delta)
    elif move.move_type == 'touch':
        for i in range(10):
            with open(cfg.files.neopixel, "r") as f:
                legs_down = f.readline().split(',')[0]
            if legs_down:
                break
        print(f"touch. legs_down: {legs_down}")
        if sum(int(x) for x in legs_down) == 6:
            pass
        else:
            for legnum in range(6):
                if legs_down[legnum] == '0':
                    rk.leg_move_custom(legnum + 1, 'touch', [0, 0, cfg.robot.touch_down], add_snapshot=False)
        rk.add_angles_snapshot('endpoint')
    
    elif move.move_type == 'switch_mode':
        rk.switch_mode(move.values['mode'])

    elif move.move_type == 'balance':
        attempts = 0
        ga_read = False
        while not ga_read and attempts < 10:
            try:
                attempts += 1
                with open(cfg.files.gyroaccel, "r") as f:
                    pitch, roll = f.readline().split(',')
                ga_read = True
            except ValueError:
                print('Value error reading pitch and roll')
                continue
            
        pitch, roll = float(pitch), float(roll)
        pre_balance_value = cfg.robot.fb_pre_balance_value
        one_leg_balance_value = cfg.robot.fb_one_leg_balance_value
        balance_value = cfg.robot.fb_balance_value
        inward_delta = cfg.robot.fb_inward_delta

        with open(cfg.files.neopixel, "r") as f:
            legs_down = f.readline().split(',')[0]
        print(f"balance. legs_down: {legs_down}")
        if len(legs_down) != 6:
            legs_down = '111111'
            
        leg1_down, leg2_down, leg3_down, leg4_down, leg5_down, leg6_down = legs_down
        
        if leg1_down == '0':
            rk.leg_movement(1, [-inward_delta, -inward_delta, pre_balance_value])
            rk.leg_move_custom(1, 'balance1', [0, 0, one_leg_balance_value])
            print(f'{pitch, roll}. Branch 17. Balance [1] {one_leg_balance_value}')
        elif leg2_down == '0':
            rk.leg_movement(2, [0, -inward_delta, pre_balance_value])
            rk.leg_move_custom(2, 'balance1', [0, 0, one_leg_balance_value])
            print(f'{pitch, roll}. Branch 18. Balance [2] {one_leg_balance_value}')
        elif leg3_down == '0':
            rk.leg_movement(3, [inward_delta, -inward_delta, pre_balance_value])
            rk.leg_move_custom(3, 'balance1', [0, 0, one_leg_balance_value])
            print(f'{pitch, roll}. Branch 19. Balance [3] {one_leg_balance_value}')
        elif leg4_down == '0':
            rk.leg_movement(4, [-inward_delta, inward_delta, pre_balance_value])
            rk.leg_move_custom(4, 'balance1', [0, 0, one_leg_balance_value])
            print(f'{pitch, roll}. Branch 20. Balance [4] {one_leg_balance_value}')
        elif leg5_down == '0':
            rk.leg_movement(5, [0, inward_delta, pre_balance_value])
            rk.leg_move_custom(5, 'balance1', [0, 0, one_leg_balance_value])
            print(f'{pitch, roll}. Branch 21. Balance [5] {one_leg_balance_value}')
        elif leg6_down == '0':
            rk.leg_movement(6, [inward_delta, inward_delta, pre_balance_value])
            rk.leg_move_custom(6, 'balance1', [0, 0, one_leg_balance_value])
            print(f'{pitch, roll}. Branch 22. Balance [6] {one_leg_balance_value}')

        elif pitch < -cfg.robot.balance_offset and abs(roll) < cfg.robot.balance_offset:
            #if rk.legs[1].D.z + rk.legs[4].D.z > rk.legs[2].D.z + rk.legs[3].D.z:
                rk.leg_move_custom(4, 'balance2', [0, 0, balance_value], add_snapshot=False)
                rk.leg_move_custom(5, 'balance2', [0, 0, balance_value], add_snapshot=False)
                rk.leg_move_custom(6, 'balance2', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 1. Balance [4, 5, 6] {balance_value}')
            #else:
            #    rk.leg_move_custom(2, 'balance2', [0, 0, -balance_value], add_snapshot=False)
            #    rk.leg_move_custom(3, 'balance2', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 2. Balance [2, 3] {-balance_value}')
        elif pitch > cfg.robot.balance_offset and abs(roll) < cfg.robot.balance_offset:
            #if rk.legs[2].D.z + rk.legs[3].D.z > rk.legs[1].D.z + rk.legs[4].D.z:
                rk.leg_move_custom(1, 'balance2', [0, 0, balance_value], add_snapshot=False)
                rk.leg_move_custom(2, 'balance2', [0, 0, balance_value], add_snapshot=False)
                rk.leg_move_custom(3, 'balance2', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 3. Balance [1, 2, 3] {balance_value}')
            #else:
            #    rk.leg_move_custom(1, 'balance2', [0, 0, -balance_value], add_snapshot=False)
            #    rk.leg_move_custom(4, 'balance2', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 4. Balance [1, 4] {-balance_value}')
        elif abs(pitch) < cfg.robot.balance_offset and roll < -cfg.robot.balance_offset:
            #if rk.legs[3].D.z + rk.legs[4].D.z > rk.legs[1].D.z + rk.legs[2].D.z:
                rk.leg_move_custom(2, 'balance2', [0, 0, balance_value/2], add_snapshot=False)
                rk.leg_move_custom(5, 'balance2', [0, 0, balance_value/2], add_snapshot=False)
                rk.leg_move_custom(3, 'balance2', [0, 0, balance_value], add_snapshot=False)
                rk.leg_move_custom(4, 'balance2', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 5. Balance [3, 4] {balance_value} / [2, 5] {balance_value/2}')
            #else:
            #    rk.leg_move_custom(1, 'balance2', [0, 0, -balance_value], add_snapshot=False)
            #    rk.leg_move_custom(2, 'balance2', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 6. Balance [1, 2] {-balance_value}')
        elif abs(pitch) < cfg.robot.balance_offset and roll > cfg.robot.balance_offset:
            #if rk.legs[1].D.z + rk.legs[2].D.z > rk.legs[3].D.z + rk.legs[4].D.z:
                rk.leg_move_custom(2, 'balance2', [0, 0, balance_value/2], add_snapshot=False)
                rk.leg_move_custom(5, 'balance2', [0, 0, balance_value/2], add_snapshot=False)
                rk.leg_move_custom(1, 'balance2', [0, 0, balance_value], add_snapshot=False)
                rk.leg_move_custom(6, 'balance2', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 7. Balance [1, 6] {balance_value} / [2, 5] {balance_value/2}')
            #else:
            #    rk.leg_move_custom(3, 'balance2', [0, 0, -balance_value], add_snapshot=False)
            #    rk.leg_move_custom(4, 'balance2', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 8. Balance [3, 4] {-balance_value}')
        elif pitch < -cfg.robot.balance_offset and roll > cfg.robot.balance_offset:
            #if rk.legs[1].D.z > rk.legs[3].D.z:
                rk.leg_move_custom(6, 'balance1', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 9. Balance [6] {balance_value}')
            #else:
            #    rk.leg_move_custom(3, 'balance1', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 10. Balance [3] {-balance_value}')
        elif pitch > cfg.robot.balance_offset and roll > cfg.robot.balance_offset:
            #if rk.legs[2].D.z > rk.legs[4].D.z:
                rk.leg_move_custom(1, 'balance1', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 11. Balance [1] {balance_value}')
            #else:
            #    rk.leg_move_custom(4, 'balance1', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 12. Balance [4] {-balance_value}')
        elif pitch > cfg.robot.balance_offset and roll < -cfg.robot.balance_offset:
            #if rk.legs[3].D.z > rk.legs[1].D.z:
                rk.leg_move_custom(3, 'balance1', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 13. Balance [3] {balance_value}')
            #else:
            #    rk.leg_move_custom(1, 'balance1', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 14. Balance [1] {-balance_value}')
        elif pitch < -cfg.robot.balance_offset and roll < -cfg.robot.balance_offset:
            #if rk.legs[4].D.z > rk.legs[2].D.z:
                rk.leg_move_custom(4, 'balance1', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 15. Balance [4] {balance_value}')
            #else:
            #    rk.leg_move_custom(2, 'balance1', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 16. Balance [2] {-balance_value}')

    return rk.sequence
