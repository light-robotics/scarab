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


class VirtualRobot():
    """
    An intermediate level of abstraction to apply constraints
    Actually this is only for look angles, cuz other constraints it is easy to check on the fly
    This looks really bad, I need some better decision some day
    """
    def __init__(self, logger):
        self.logger = logger
        self.side_look_angle = 0
        self.vertical_look_angle = 0

    def get_sequence(self, command: str, robot_position: RobotPosition, kwargs=None):
        sequence, new_position = get_sequence_for_command_cached(command, robot_position, kwargs)
        return sequence, new_position

    def get_height(self, robot_position: RobotPosition):
        fk = Kinematics(robot_position=robot_position)
        return fk.height + 13

    def get_leg_angle_to_surface(self, robot_position: RobotPosition, leg_num: int):
        fk = Kinematics(robot_position=robot_position)
        print(math.degrees(fk.legs[leg_num].alpha), math.degrees(fk.legs[leg_num].beta))
        return math.degrees(fk.legs[leg_num].alpha - fk.legs[leg_num].beta)

    def get_legs_zs(self, robot_position: RobotPosition):
        fk = Kinematics(robot_position=robot_position)
        return [leg.O.z - leg.C.z for leg in fk.legs.values()]

#@cache
#@memory.cache
def get_sequence_for_command_cached(command: str, robot_position: RobotPosition, kwargs=None):
    fk = Kinematics(robot_position=robot_position)
    
    if command == 'forward_1':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_24(FORWARD_LEGS_2LEG_CM, 0)
    elif command == 'forward_2':
        # Legs 2 and 4 moved x2
        fk.move_2_legs_phased_13(2 * FORWARD_LEGS_2LEG_CM, 0)
    elif command == 'forward_22':
        # Legs 2 and 4 moved x1
        fk.move_2_legs_phased_13(FORWARD_LEGS_2LEG_CM, 0)
    elif command == 'forward_3':
        # Legs 1 and 3 moved x2
        #for _ in range(3):
        #    fk.move_2_legs_phased_24(2 * FORWARD_LEGS_2LEG_CM, 0)
        #    fk.move_2_legs_phased_13(2 * FORWARD_LEGS_2LEG_CM, 0)
        fk.move_2_legs_phased_24(2 * FORWARD_LEGS_2LEG_CM, 0)
    elif command == 'forward_32':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_24(FORWARD_LEGS_2LEG_CM, 0)
    elif command == 'backward_1':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_13(-FORWARD_LEGS_2LEG_CM, 0)
    elif command == 'backward_2':
        # Legs 2 and 4 moved x2
        fk.move_2_legs_phased_24(-2 * FORWARD_LEGS_2LEG_CM, 0)
    elif command == 'backward_22':
        # Legs 2 and 4 moved x1
        fk.move_2_legs_phased_24(-FORWARD_LEGS_2LEG_CM, 0)
    elif command == 'backward_3':
        # Legs 1 and 3 moved x2
        for _ in range(3):
            fk.move_2_legs_phased_13(-2 * FORWARD_LEGS_2LEG_CM, 0)
            fk.move_2_legs_phased_24(-2 * FORWARD_LEGS_2LEG_CM, 0)
        fk.move_2_legs_phased_13(-2 * FORWARD_LEGS_2LEG_CM, 0)
    elif command == 'backward_32':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_13(-FORWARD_LEGS_2LEG_CM, 0)
    
    elif command == 'strafe_right_1':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_13(0, FORWARD_LEGS_2LEG_CM)
    elif command == 'strafe_right_2':
        # Legs 2 and 4 moved x2
        fk.move_2_legs_phased_24(0, 2 * FORWARD_LEGS_2LEG_CM)
    elif command == 'strafe_right_22':
        # Legs 2 and 4 moved x1
        fk.move_2_legs_phased_24(0, FORWARD_LEGS_2LEG_CM)
    elif command == 'strafe_right_3':
        # Legs 1 and 3 moved x2
        for _ in range(3):
            fk.move_2_legs_phased_13(0, 2 * FORWARD_LEGS_2LEG_CM)
            fk.move_2_legs_phased_24(0, 2 * FORWARD_LEGS_2LEG_CM)
        fk.move_2_legs_phased_13(0, 2 * FORWARD_LEGS_2LEG_CM)
    elif command == 'strafe_right_32':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_13(0, FORWARD_LEGS_2LEG_CM)

    elif command == 'strafe_left_1':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_13(0, -FORWARD_LEGS_2LEG_CM)
    elif command == 'strafe_left_2':
        # Legs 2 and 4 moved x2
        fk.move_2_legs_phased_24(0, -2 * FORWARD_LEGS_2LEG_CM)
    elif command == 'strafe_left_22':
        # Legs 2 and 4 moved x1
        fk.move_2_legs_phased_24(0, -FORWARD_LEGS_2LEG_CM)
    elif command == 'strafe_left_3':
        # Legs 1 and 3 moved x2
        for _ in range(3):
            fk.move_2_legs_phased_13(0, -2 * FORWARD_LEGS_2LEG_CM)
            fk.move_2_legs_phased_24(0, -2 * FORWARD_LEGS_2LEG_CM)
        fk.move_2_legs_phased_13(0, -2 * FORWARD_LEGS_2LEG_CM)
    elif command == 'strafe_left_32':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_13(0, -FORWARD_LEGS_2LEG_CM)
    elif command == 'diagonal_forward_right':
        fk.move_2_legs_phased_13(FORWARD_LEGS_2LEG_CM, -FORWARD_LEGS_2LEG_CM)
        fk.move_2_legs_phased_24(FORWARD_LEGS_2LEG_CM, -FORWARD_LEGS_2LEG_CM)
    elif command == 'diagonal_forward_left':
        fk.move_2_legs_phased_13(FORWARD_LEGS_2LEG_CM, FORWARD_LEGS_2LEG_CM)
        fk.move_2_legs_phased_24(FORWARD_LEGS_2LEG_CM, FORWARD_LEGS_2LEG_CM)
    elif command == 'diagonal_backward_right':
        fk.move_2_legs_phased_13(-FORWARD_LEGS_2LEG_CM, -FORWARD_LEGS_2LEG_CM)
        fk.move_2_legs_phased_24(-FORWARD_LEGS_2LEG_CM, -FORWARD_LEGS_2LEG_CM)
    elif command == 'diagonal_backward_left':
        fk.move_2_legs_phased_13(-FORWARD_LEGS_2LEG_CM, FORWARD_LEGS_2LEG_CM)
        fk.move_2_legs_phased_24(-FORWARD_LEGS_2LEG_CM, FORWARD_LEGS_2LEG_CM)
    elif command == 'wave_gait':
        #fk.move_body_straight(FORWARD_LEGS_1LEG_CM, 0)
        fk.wave_gait()
    elif command == 'ripple_gait':
        fk.ripple_gait()
    #elif command == 'touching':
    #    fk.leg_move_with_touching()
    elif command in ['battle_mode', 'sentry_mode', 'walking_mode', 'run_mode']:
        fk.switch_mode(command)
    elif command == "left_turn_in_move":
        fk.turn_in_move(20)
    elif command == "right_turn_in_move":
        fk.turn_in_move(-20)
    elif command == 'body_forward_8':
        fk.body_movement(8, 0, 0)
    elif command == 'body_backward_8':
        fk.body_movement(-8, 0, 0)
    elif command == 'body_forward':
        if fk.body_delta_xy()[1] > cfg.limits["body_forward"]:
            print('Forward body limit reached')
        else:
            fk.body_movement(0, FORWARD_BODY_CM, 0)
    elif command == 'body_backward':
        if fk.body_delta_xy()[1] < -cfg.limits["body_forward"]:
            print('Backward body limit reached')
        else:
            fk.body_movement(0, -FORWARD_BODY_CM, 0)
    elif command == 'body_left':
        if fk.body_delta_xy()[0] < -cfg.limits["body_sideways"]:
            print('Body left limit reached')
        else:
            fk.body_movement(-FORWARD_BODY_CM, 0, 0)
    elif command == 'body_right':
        if fk.body_delta_xy()[0] > cfg.limits["body_sideways"]:
            print('Body right limit reached')
        else:
            fk.body_movement(FORWARD_BODY_CM, 0, 0)
    elif command == 'body_to_center':
        fk.body_to_center()
    elif command == 'up':
        fk.body_movement(0, 0, UP_OR_DOWN_CM)
    elif command == 'down':
        fk.body_movement(0, 0, -UP_OR_DOWN_CM)
    elif command == 'up_4':
        fk.body_movement(0, 0, 4)
    elif command == 'up_16':
        fk.body_movement(0, 0, 16)
    elif command == 'down_16':
        fk.body_movement(0, 0, -16)
    elif command == 'play1':
        fk.play1()
    elif command == 'climb_2':
        fk.climb_20()
    elif command == 'backward_one_legged':
        pass
    
    elif command == 'strafe_left_two_legged':
        pass
    elif command == 'strafe_left_one_legged':
        pass
    
    elif command == 'strafe_right_two_legged':
        pass
    elif command == 'strafe_right_one_legged':
        pass    
    elif command == 'look_up':
        fk.look_on_angle_new(up=True)
    elif command == 'look_down':
        fk.look_on_angle_new(up=False)
    elif command == 'look_left':
        fk.turn(-24, only_body=True)
    elif command == 'look_right':
        fk.turn(24, only_body=True)
    elif command == 'sight_to_normal':
        fk.look_on_angle(0)
        fk.turn(-fk.side_look_angle, only_body=True)
    elif command == 'turn_left_two_legged':
        fk.turn_move(40)
    elif command == 'turn_right_two_legged':
        fk.turn_move(-40)
    elif command == 'reposition_x_up':
        fk.reposition_legs(REPOSITION_CM, 0)
    elif command == 'reposition_x_down':
        fk.reposition_legs(-REPOSITION_CM, 0)
    elif command == 'reposition_y_up':
        fk.reposition_legs(0, REPOSITION_CM)
    elif command == 'reposition_y_down':
        fk.reposition_legs(0, -REPOSITION_CM)
    elif command == 'start':
        fk.start()
    elif command == 'end':
        fk.end()
    elif command == 'reset':
        fk.reset()
    elif command == 'hit':
        fk.hit()
    elif command == 'jump':
        fk.jump()
    elif command == 'right_hook':
        fk.hook(1)
    elif command == 'left_hook':
        fk.hook(6) 
    elif command == 'climb_2_legs':
        fk.climb_2_legs(10)
    elif command == 'descend_2_legs':
        fk.descend_2_legs(10)
    elif command == 'back_8':
        fk.body_movement(-8, 0, 0)
    elif command == 'leg_up_adjusted':
        leg_num = kwargs["leg_num"]
        leg_up = kwargs["leg_up"]
        #fk.body_compensation_for_a_leg(leg_num)
        val = 0
        if leg_num == 1:
            x_value = -val
        else:
            x_value = val
        fk.move_leg_endpoint(leg_num, [-val, x_value, leg_up])
        print(f'{leg_num, [-val, x_value, leg_up]}')
    elif command == 'leg_down_adjusted':
        leg_num = kwargs["leg_num"]
        leg_down = kwargs["leg_down"]
        val = 0
        if leg_num == 1:
            x_value = val
        else:
            x_value = -val
        fk.move_leg_endpoint(leg_num, [8 + val, x_value, -leg_down])
        #fk.body_to_center()
    elif command == "back_legs":
        fk.body_movement(8, 0, 0)
        for leg_num in [3, 4]:
            fk.body_compensation_for_a_leg(leg_num)
            fk.move_leg_endpoint(leg_num, [8, 0, 7])
            fk.move_leg_endpoint(leg_num, [0, 0, -7])
        fk.body_to_center()
    elif command == 'body_to_center':
        fk.body_to_center()
    else:
        print(f'Unknown command')
    
    return fk.sequence, fk.current_position
